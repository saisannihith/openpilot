"""Native 3D visualization with bounded reusable meshes and display-only input."""
import math
import time
from functools import lru_cache
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.onroad.world_scene import WorldScene

CAPACITY = 4095
# Existing onroad instruments use light text. Keep their contrast intact.
BACKGROUND = rl.Color(0, 0, 0, 255)
GROUND = rl.Color(0, 0, 0, 255)
WHITE = rl.Color(255, 255, 255, 255)
UP = rl.Vector3(0, 1, 0)
ONE = rl.Vector3(1, 1, 1)
ORIGIN = rl.Vector3(0, 0, 0)


class GpuMesh:
  """Own native heap buffers and one GPU mesh; release them together."""
  def __init__(self, vertices, colors, dynamic=False):
    vertices = np.ascontiguousarray(vertices, dtype=np.float32).reshape(-1, 3)
    colors = np.ascontiguousarray(colors, dtype=np.uint8).reshape(-1, 4)
    mesh = rl.Mesh()
    mesh.vertexCount, mesh.triangleCount = len(vertices), len(vertices) // 3
    mesh.vertices = rl.ffi.cast('float *', rl.mem_alloc(vertices.nbytes))
    mesh.colors = rl.ffi.cast('unsigned char *', rl.mem_alloc(colors.nbytes))
    if mesh.vertices == rl.ffi.NULL or mesh.colors == rl.ffi.NULL:
      rl.unload_mesh(mesh)
      raise MemoryError('3D scene mesh allocation failed')
    rl.ffi.memmove(mesh.vertices, rl.ffi.cast('void *', vertices.ctypes.data), vertices.nbytes)
    rl.ffi.memmove(mesh.colors, rl.ffi.cast('void *', colors.ctypes.data), colors.nbytes)
    rl.upload_mesh(mesh, dynamic)
    self.model = rl.load_model_from_mesh(mesh)
    self.closed = False

  def update(self, vertices, colors, count):
    rl.update_mesh_buffer(self.model.meshes[0], 0, rl.ffi.cast('void *', vertices.ctypes.data), count * 12, 0)
    rl.update_mesh_buffer(self.model.meshes[0], 3, rl.ffi.cast('void *', colors.ctypes.data), count * 4, 0)
    self.model.meshes[0].triangleCount = count // 3
    self.model.meshes[0].vertexCount = count

  def draw(self, position=ORIGIN, yaw=0.0):
    if self.model.meshes[0].triangleCount:
      rl.draw_model_ex(self.model, position, UP, yaw, ONE, WHITE)

  def close(self):
    if not self.closed:
      rl.unload_model(self.model)
      self.closed = True


@lru_cache(maxsize=1)
def vehicle_mesh():
  # Author-modeled CC0 mesh, baked offline: no OBJ parsing or textures onroad.
  from openpilot.common.basedir import BASEDIR
  from pathlib import Path
  with np.load(Path(BASEDIR)/'selfdrive/assets/world/sedan.npz',allow_pickle=False) as asset:
    vertices,colors = asset['vertices'],asset['colors']
  if (vertices.dtype != np.float32 or colors.dtype != np.uint8 or vertices.shape != (len(colors),3)
      or colors.shape != (len(vertices),4) or not 0 < len(vertices) <= 20000 or len(vertices)%3
      or not np.isfinite(vertices).all()):
    raise ValueError('Invalid world vehicle asset')
  return vertices,colors

class TeslaRoadRenderer:
  def __init__(self):
    self.scene = WorldScene()
    self._meshes = []
    self._target = None
    self._target_size = None
    self._geometry_key = None
    self._vertices = np.zeros((CAPACITY, 3), dtype=np.float32)
    self._colors = np.zeros((CAPACITY, 4), dtype=np.uint8)
    self._count = 0
    self.overlay_exclusions = []
    self._camera = rl.Camera3D(rl.Vector3(0, 10.5, 20), rl.Vector3(0, 0, -10), UP, 46,
                              rl.CameraProjection.CAMERA_PERSPECTIVE)
    eye,target,up = (np.array([v.x,v.y,v.z],dtype=np.float64) for v in (self._camera.position,self._camera.target,self._camera.up))
    forward = target-eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward,up)
    right /= np.linalg.norm(right)
    self._view_basis = np.array([right,np.cross(right,forward),forward])
    self._eye = eye
    self._focal_factor = .5/math.tan(math.radians(self._camera.fovy)/2)

  def _initialize(self):
    if self._meshes:
      return
    try:
      for _ in range(2):
        self._meshes.append(GpuMesh(*vehicle_mesh()))
      self._meshes.append(GpuMesh(self._vertices, self._colors, dynamic=True))
    except Exception:
      self.close()
      raise

  def _triangle(self, a, b, c, color):
    if self._count + 3 > CAPACITY:
      raise ValueError('3D scene exceeded fixed geometry budget')
    self._vertices[self._count:self._count+3] = (a,b,c)
    self._colors[self._count:self._count+3] = color
    self._count += 3

  def _ribbon(self, points, half_width, height, color, path=False):
    if not points:
      return
    sides = []
    for i, (x, y) in enumerate(points):
      before, after = points[max(0,i-1)], points[min(len(points)-1,i+1)]
      dx, dy = after[0]-before[0], after[1]-before[1]
      length = max(1e-6, math.hypot(dx,dy))
      nx, ny = -dy/length, dx/length
      width = self.scene.path_half_width(x,y) if path else half_width
      sides.append(((y-ny*width, height, -(x-nx*width)), (y+ny*width, height, -(x+nx*width))))
    for (a,b),(c,d) in zip(sides,sides[1:], strict=False):
      self._triangle(a,b,c,color)
      self._triangle(b,d,c,color)

  def _update_geometry(self, engaged):
    key = (self.scene.revision, engaged)
    if key == self._geometry_key:
      return
    self._count = 0
    for edge in self.scene.edges:
      self._ribbon(edge,.06,.008,(230,48,48,255))
    for lane in self.scene.lanes:
      self._ribbon(lane,.045,.014,(246,247,249,255))
    fill = (40,149,246,255) if engaged else (166,179,189,255)
    self._ribbon(self.scene.path,.85,.018,fill,path=True)
    self._meshes[2].update(self._vertices,self._colors,self._count)
    self._geometry_key = key

  def render(self, rect, sm, started_frame, engaged, now=None, parent_target=None, road_overlay=None):
    now = time.monotonic() if now is None else now
    self.scene.update(sm, started_frame, now)
    self.overlay_exclusions.clear()
    # One color/depth target reused every frame, bounded independently of DPI.
    scale = min(1.0, 1440.0/max(1,rect.width), 810.0/max(1,rect.height))
    size = (max(2,int(rect.width*scale)), max(2,int(rect.height*scale)))
    # Raylib texture modes are not a stack. Preserve the app's transform and
    # explicitly rebind its scaled/burn-in framebuffer before drawing the HUD.
    rl.rl_draw_render_batch_active()
    parent_modelview = rl.rl_get_matrix_modelview()
    parent_projection = rl.rl_get_matrix_projection()
    rl.rl_push_matrix()
    rl.rl_load_identity()
    rl.end_scissor_mode()
    try:
      self._initialize()
      if road_overlay is None:
        self._update_geometry(engaged)
      if size != self._target_size:
        if self._target is not None:
          rl.unload_render_texture(self._target)
        self._target = None
        self._target_size = None
        target = rl.load_render_texture(*size)
        if not target.id:
          raise RuntimeError('3D scene render target unavailable')
        self._target, self._target_size = target, size
        rl.set_texture_filter(target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      rl.begin_texture_mode(self._target)
      rl.clear_background(BACKGROUND)
      rl.begin_mode_3d(self._camera)
      rl.rl_disable_backface_culling()
      try:
        rl.draw_plane(rl.Vector3(0,-.02,-40), rl.Vector2(200,260), GROUND)
        if road_overlay is None:
          self._meshes[2].draw()
        else:
          # Shared path styles render on the ground before the vehicle pass;
          # drawing them over the completed world would paint through cars.
          rl.rl_enable_backface_culling()
          rl.end_mode_3d()
          try:
            road_overlay(self,rl.Rectangle(0,0,*size))
          finally:
            rl.begin_mode_3d(self._camera)
            rl.rl_disable_backface_culling()
        for obj in self.scene.objects:
          position = rl.Vector3(obj.right,0,-obj.forward-2.4)
          if obj.vehicle:
            self._meshes[1].draw(position)
          else:
            # A radar return has no verified body shape or vehicle class.
            rl.draw_sphere_ex(rl.Vector3(obj.right,.12,-obj.forward),.12,4,6,rl.Color(125,133,140,255))
        yaw = self.scene.ego_yaw
        angle = math.radians(yaw)
        # Pivot at the nose/path origin so the avatar cannot drift sideways
        # when showing near-path heading. This is intent, not measured yaw.
        self._meshes[0].draw(rl.Vector3(2.4*math.sin(angle),0,2.4*math.cos(angle)), yaw)
      finally:
        rl.rl_enable_backface_culling()
        rl.end_mode_3d()
    finally:
      rl.end_texture_mode()
      if parent_target is not None:
        rl.begin_texture_mode(parent_target)
      # Texture mode selected MODELVIEW rather than the saved transform stack.
      # Re-select that stack before restoring our caller's transform.
      rl.rl_push_matrix()
      rl.rl_pop_matrix()
      rl.rl_pop_matrix()
      rl.rl_set_matrix_modelview(parent_modelview)
      rl.rl_set_matrix_projection(parent_projection)
      rl.begin_scissor_mode(int(rect.x),int(rect.y),int(rect.width),int(rect.height))
    rl.draw_texture_pro(self._target.texture, rl.Rectangle(0,0,size[0],-size[1]), rect, rl.Vector2(0,0),0,WHITE)

  def lead_anchor(self, index, rect):
    """Project the same displayed lead, not a camera-calibrated coordinate."""
    obj = next((obj for obj in self.scene.objects if obj.key == ('lead', index)), None)
    if obj is None or self._target_size is None:
      return None
    return self.project(obj.forward+2.4,obj.right,rect,height=1.8)

  def project(self, forward, right, rect, height=0.02):
    if self._target_size is None or not all(math.isfinite(v) for v in (forward,right,height)):
      return None
    dx,dy,dz = right-self._eye[0],height-self._eye[1],-forward-self._eye[2]
    h,v,d = self._view_basis
    depth = d[0]*dx+d[1]*dy+d[2]*dz
    if depth <= .01:
      return None
    focal = self._target_size[1]*self._focal_factor
    x = rect.x+rect.width*(.5+(h[0]*dx+h[1]*dy+h[2]*dz)*focal/depth/self._target_size[0])
    y = rect.y+rect.height*(.5-(v[0]*dx+v[1]*dy+v[2]*dz)*focal/depth/self._target_size[1])
    return (x,y) if rect.x <= x <= rect.x+rect.width and rect.y <= y <= rect.y+rect.height else None

  def project_ribbon(self, points, width, rect):
    if not points or not math.isfinite(width) or width < 0:
      return np.empty((0,2),np.float32)
    if len(points) < 2:
      return np.empty((0,2),np.float32)
    p = np.asarray(points,dtype=np.float64)
    delta = np.empty_like(p)
    delta[1:-1] = p[2:]-p[:-2]
    delta[0],delta[-1] = p[1]-p[0],p[-1]-p[-2]
    norm = np.maximum(1e-6,np.hypot(delta[:,0],delta[:,1]))
    offset = np.column_stack((-delta[:,1],delta[:,0]))*(width/norm[:,None])
    both = np.concatenate((p-offset,p+offset))
    xyz = np.column_stack((both[:,1],np.full(len(both),.02),-both[:,0]))
    view = (xyz-self._eye) @ self._view_basis.T
    depth = np.maximum(.01,view[:,2])
    focal = self._target_size[1]*self._focal_factor
    screen = np.column_stack((rect.x+rect.width*(.5+view[:,0]*focal/depth/self._target_size[0]),
                              rect.y+rect.height*(.5-view[:,1]*focal/depth/self._target_size[1])))
    valid = (view[:,2] > .01) & np.isfinite(screen).all(axis=1)
    valid &= (screen[:,0] >= rect.x) & (screen[:,0] <= rect.x+rect.width)
    valid &= (screen[:,1] >= rect.y) & (screen[:,1] <= rect.y+rect.height)
    n = len(p)
    paired = valid[:n] & valid[n:]
    return np.concatenate((screen[:n][paired],screen[n:][paired][::-1])).astype(np.float32)

  def close(self, parent_target=None):
    has_resources = bool(self._meshes) or self._target is not None
    parent_framebuffer = parent_target.id if parent_target is not None else (rl.rl_get_active_framebuffer() if has_resources else None)
    released_framebuffer = self._target.id if self._target is not None else None
    if has_resources:
      rl.rl_draw_render_batch_active()
    for mesh in self._meshes:
      mesh.close()
    self._meshes.clear()
    if self._target is not None:
      rl.unload_render_texture(self._target)
      self._target = None
    # UnloadRenderTexture can bind framebuffer 0. View changes must not steal
    # the application's active target while it continues drawing this frame.
    if parent_framebuffer is not None and parent_framebuffer != released_framebuffer:
      rl.rl_enable_framebuffer(parent_framebuffer)
    self._target_size = None
    self._geometry_key = None
    self.scene.reset()
