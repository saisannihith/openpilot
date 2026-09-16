"""Native 3D visualization with bounded reusable meshes and display-only input."""
import math
import time
from functools import lru_cache
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.onroad.world_scene import WorldScene

CAPACITY = 4095
# Existing onroad instruments use light text. Keep their contrast intact.
BACKGROUND = rl.Color(18, 20, 22, 255)
GROUND = rl.Color(31, 33, 35, 255)
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


@lru_cache(maxsize=2)
def vehicle_mesh(body):
  vertices, colors = [], []
  for i in range(24):
    a, b = i * math.tau / 24, (i + 1) * math.tau / 24
    vertices.extend(((0,.025,0), (1.2*math.cos(a),.025,2.8*math.sin(a)), (1.2*math.cos(b),.025,2.8*math.sin(b))))
    colors.extend(((0,0,0,100),(0,0,0,0),(0,0,0,0)))

  def face(points, color):
    a, b, c = (np.asarray(p, dtype=float) for p in points[:3])
    normal = np.cross(b - a, c - a)
    normal /= max(1e-9, float(np.linalg.norm(normal)))
    light = 0.72 + 0.24 * abs(normal[1]) + 0.04 * normal[0]
    shade = tuple(int(max(0, min(255, value * light))) for value in color[:3]) + (255,)
    for i in range(1, len(points) - 1):
      vertices.extend((points[0], points[i], points[i + 1]))
      colors.extend((shade,) * 3)

  rings = []
  for z, width, top in ((-2.4, .73, .66), (-2.18, .93, .85), (-1.35, .98, .98),
                        (1.42, .98, 1.02), (2.18, .93, .93), (2.4, .78, .75)):
    rings.append([(x, y, z) for x, y in ((-width*.83,.3), (width*.83,.3), (width,.47),
                  (width,top-.14), (width*.79,top), (-width*.79,top), (-width,top-.14), (-width,.47))])
  for front, rear in zip(rings, rings[1:], strict=False):
    for i in range(8):
      face((front[i], front[(i+1)%8], rear[(i+1)%8], rear[i]), body)
  face(tuple(reversed(rings[0])), body)
  face(rings[-1], body)
  low = [(-.82,.92,-1.47),(.82,.92,-1.47),(.85,.98,1.76),(-.85,.98,1.76)]
  high = [(-.66,1.62,-.75),(.66,1.62,-.75),(.7,1.62,1.15),(-.7,1.62,1.15)]
  face(high, body)
  for i in range(4):
    j = (i+1)%4
    face((low[i], low[j], high[j], high[i]), (40, 55, 68))
  # Static tail lamps, never inferred brake state from relative velocity.
  face(((-.7,.49,2.405),(.7,.49,2.405),(.7,.64,2.405),(-.7,.64,2.405)), (70,77,85))
  for side in (-1, 1):
    x0, x1 = sorted((side*.52, side*.8))
    face(((x0,.65,2.405),(x1,.65,2.405),(x1,.73,2.405),(x0,.73,2.405)), (150,35,41))
    for z in (-1.5, 1.5):
      for radius, x, color in ((.34, side*.99, (24,27,31)), (.18, side*1.01, (112,121,129))):
        center = (x,.34,z)
        ring = [(x,.34+radius*math.cos(i*math.tau/12),z+radius*math.sin(i*math.tau/12)) for i in range(12)]
        for i in range(12):
          face((center,ring[i],ring[(i+1)%12]), color)
  return np.array(vertices, np.float32), np.array(colors, np.uint8)


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
    self._camera = rl.Camera3D(rl.Vector3(0, 10.5, 20), rl.Vector3(0, 0, -10), UP, 46,
                              rl.CameraProjection.CAMERA_PERSPECTIVE)

  def _initialize(self):
    if self._meshes:
      return
    try:
      for color in ((28,134,246), (181,190,199)):
        self._meshes.append(GpuMesh(*vehicle_mesh(color)))
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
      self._ribbon(edge,.06,.008,(87,93,99,255))
    for lane in self.scene.lanes:
      self._ribbon(lane,.045,.014,(246,247,249,255))
    fill = (40,149,246,255) if engaged else (166,179,189,255)
    self._ribbon(self.scene.path,.85,.018,fill,path=True)
    self._meshes[2].update(self._vertices,self._colors,self._count)
    self._geometry_key = key

  def render(self, rect, sm, started_frame, engaged, now=None, parent_target=None):
    now = time.monotonic() if now is None else now
    self.scene.update(sm, started_frame, now)
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
        self._meshes[2].draw()
        for obj in self.scene.objects:
          position = rl.Vector3(obj.right,0,-obj.forward-2.4)
          if obj.vehicle:
            self._meshes[1].draw(position)
          else:
            rl.draw_cube_wires(rl.Vector3(obj.right,.2,-obj.forward),.55,.4,.55,rl.Color(109,122,136,255))
        self._meshes[0].draw(rl.Vector3(0,0,2.4))
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
