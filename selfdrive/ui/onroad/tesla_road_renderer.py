"""Native 3D visualization with bounded reusable meshes and display-only input."""
import colorsys
import math
import time
from functools import lru_cache
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.onroad.world_scene import MAX_POINTS, WorldScene, elevation_at, lateral_at, road_surface_geometry
from openpilot.selfdrive.ui.onroad.world_presentation import WorldPresentation, RenderQuality

CAPACITY = 4095
FLOW_CAPACITY = 768
WORLD_CAMERA_FOVY = 42.0
EGO_CAMERA_OFFSET_M = 4.6
# Median left/right coordinates from the baked Carnival rear-lens clusters.
# These remain in the ego mesh frame and are rotated only with the avatar.
EGO_REAR_SIGNAL_OFFSETS = ((-.715, 1.13, 2.377), (.702, 1.13, 2.378))
# Existing onroad instruments use light text. Keep their contrast intact.
BACKGROUND = rl.Color(0, 0, 0, 255)
WHITE = rl.Color(255, 255, 255, 255)
RADAR_AVATAR = rl.Color(185, 192, 200, 255)
ROAD_EDGE_HALO = (116, 21, 37, 138)
ROAD_EDGE_MID = (198, 48, 61, 210)
ROAD_EDGE = (255, 96, 98, 255)
LANE_GLOW = (108, 148, 190, 76)
LANE_MARKING = (242, 247, 252, 255)
ROAD_FLOW = (92, 178, 255, 150)
ROAD_SHEEN = (30, 78, 145, 64)
ROAD_SHEEN_HOT = (64, 124, 208, 84)
PATH_EDGE = (26, 225, 128, 220)
SIGNAL_GLOW = rl.Color(255, 213, 0, 120)
SIGNAL_YELLOW = rl.Color(255, 239, 16, 255)
SIGNAL_HOTSPOT = rl.Color(255, 255, 208, 255)
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

  def draw(self, position=ORIGIN, yaw=0.0, tint=WHITE):
    if self.model.meshes[0].triangleCount:
      rl.draw_model_ex(self.model, position, UP, yaw, ONE, tint)

  def close(self):
    if not self.closed:
      rl.unload_model(self.model)
      self.closed = True


def _world_asset(name):
  from os import getenv
  from openpilot.common.basedir import BASEDIR
  from pathlib import Path
  # Isolated asset roots are useful for offroad renderer validation. Nothing
  # outside this display-only loader observes the override.
  root = getenv('TESLA_ROAD_ASSET_ROOT')
  return (Path(root) if root else Path(BASEDIR)/'selfdrive/assets/world')/name


def _world_mesh(name, max_vertices=20_000):
  # Offline-baked mesh only: no model parsing, textures, allocations, or I/O onroad.
  with np.load(_world_asset(name),allow_pickle=False) as asset:
    vertices,colors = asset['vertices'],asset['colors']
  if (vertices.dtype != np.float32 or colors.dtype != np.uint8 or vertices.shape != (len(colors),3)
      or colors.shape != (len(vertices),4) or not 0 < len(vertices) <= max_vertices or len(vertices)%3
      or not np.isfinite(vertices).all()):
    raise ValueError('Invalid world vehicle asset')
  return vertices,colors


@lru_cache(maxsize=4)
def vehicle_mesh(distant=False, lead=False):
  # Generic traffic never claims a make/model. Model-associated lead objects get
  # a compact, neutralized high-detail proxy only while they are nearby.
  if lead:
    asset = 'lead_vehicle_lod.npz' if distant else 'lead_vehicle.npz'
    return _world_mesh(asset, max_vertices=100_000)
  return _world_mesh('sedan_lod.npz' if distant else 'sedan.npz')


@lru_cache(maxsize=1)
def ego_vehicle_mesh():
  # CC-BY Carnival asset is display-only and applies to the known ego car only.
  return _world_mesh('carnival.npz', max_vertices=1_500_000)


def ego_signal_flash(left, right, now):
  """Match StarPilot's existing 500 ms turn-signal cadence."""
  visible = int(now * 2.0) % 2 == 0
  return bool(left) and visible, bool(right) and visible


def _ego_display_point(anchor, yaw, x, y, z):
  angle = math.radians(yaw)
  sin, cos = math.sin(angle), math.cos(angle)
  return rl.Vector3(anchor.x+x*cos+z*sin, anchor.y+y, anchor.z+z*cos-x*sin)


def ego_rear_signal_positions(anchor, yaw):
  """Transform the known Carnival rear lamp positions with its display yaw."""
  return tuple(_ego_display_point(anchor,yaw,x,y,z) for x,y,z in EGO_REAR_SIGNAL_OFFSETS)


class TeslaRoadRenderer:
  def __init__(self):
    self.scene = WorldScene()
    self.presentation = WorldPresentation()
    self.quality = RenderQuality()
    self._far_ids = set()
    self._lead_ids = frozenset()
    self._lead_object_key = None
    self._meshes = []
    self._target = None
    self._target_size = None
    self._geometry_key = None
    self._vertices = np.zeros((CAPACITY, 3), dtype=np.float32)
    self._colors = np.zeros((CAPACITY, 4), dtype=np.uint8)
    self._count = 0
    self._flow_vertices = np.zeros((FLOW_CAPACITY, 3), dtype=np.float32)
    self._flow_colors = np.zeros((FLOW_CAPACITY, 4), dtype=np.uint8)
    self._flow_count = 0
    self._flow_key = None
    self._motion_time = None
    self._motion_distance = 0.0
    self._ambient_texture = None
    self.overlay_exclusions = []
    self._intent_path = ()
    self._intent_width = .85
    self._intent_edge = 0.
    self._intent_rainbow = False
    self._intent_acceleration = ()
    self._intent_key = None
    # Frame the road from just behind the ego vehicle, not a distant overview.
    # Projection helpers share this camera so labels remain attached to cars.
    # A slightly tighter third-person lens makes nearby measured traffic readable while
    # retaining its untouched physical world coordinates and true lead label.
    # A lower eye line puts the model-road horizon at the landscape horizon
    # rather than projecting distant vehicles into the sky.
    self._camera = rl.Camera3D(rl.Vector3(0, 4, 12), rl.Vector3(0, 0, -2.8), UP, WORLD_CAMERA_FOVY,
                               rl.CameraProjection.CAMERA_PERSPECTIVE)
    self._camera_time = None
    self._camera_fovy = WORLD_CAMERA_FOVY
    self._set_camera((0., 4., 12.), (0., 0., -2.8), WORLD_CAMERA_FOVY)

  def _set_camera(self, eye, target, fovy):
    self._camera.position = rl.Vector3(*eye)
    self._camera.target = rl.Vector3(*target)
    self._camera.fovy = fovy
    eye,target,up = (np.asarray(value,dtype=np.float64) for value in (eye,target,(0.,1.,0.)))
    forward = target-eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward,up)
    right /= np.linalg.norm(right)
    self._view_basis = np.array([right,np.cross(right,forward),forward])
    self._eye = eye
    self._focal_factor = .5/math.tan(math.radians(fovy)/2)

  def _update_camera(self, now):
    """Smoothly frame measured traffic and model grade without camera jumps."""
    nearby = [(obj.forward, abs(obj.right)) for obj in self.scene.objects if obj.vehicle or obj.radar_avatar]
    lead = min((forward for forward, right in nearby if right <= 2.4), default=80.)
    side = min((forward for forward, right in nearby if right > 2.4), default=80.)
    # Zoom gently toward a verified near lead; preserve side context when a
    # real adjacent actor is close. No actor is created solely for framing.
    near_lead = max(0., min(1., (28.-lead)/18.))
    near_side = max(0., min(1., (18.-side)/12.))
    desired_fovy = max(38., min(46., WORLD_CAMERA_FOVY-near_lead*3.+near_side*2.))
    if self._camera_time is None or not 0. <= now-self._camera_time <= 1.:
      blend = 1.
    else:
      blend = 1.-math.exp(-(now-self._camera_time)/.35)
    self._camera_time = now
    self._camera_fovy += (desired_fovy-self._camera_fovy)*blend
    eye_height = 4.0+self.scene.road_height(0.)
    target_height = self.scene.road_height(8.)
    self._set_camera((0., eye_height, 12.), (0., target_height, -2.8), self._camera_fovy)

  def configure_road_intent(self, path, width, edge, rainbow, acceleration):
    """Receive existing StarPilot path policy as display metadata only."""
    path = tuple(path)
    values = []
    for value in acceleration[:MAX_POINTS]:
      try:
        value = float(value)
      except (TypeError, ValueError):
        value = 0.
      values.append(value if math.isfinite(value) else 0.)
    acceleration = tuple(values)
    width = max(.08, min(.95, float(width)))
    edge = max(0., min(1., float(edge)))
    key = (path, width, edge, bool(rainbow), acceleration)
    if key != self._intent_key:
      self._intent_path, self._intent_width, self._intent_edge = path, width, edge
      self._intent_rainbow, self._intent_acceleration, self._intent_key = bool(rainbow), acceleration, key
      self._geometry_key = None

  def _initialize(self):
    if self._meshes:
      return
    try:
      for distant in (False, True):
        self._meshes.append(GpuMesh(*vehicle_mesh(distant)))
      self._meshes.append(GpuMesh(self._vertices, self._colors, dynamic=True))
      self._meshes.append(GpuMesh(*ego_vehicle_mesh()))
      self._meshes.append(GpuMesh(self._flow_vertices, self._flow_colors, dynamic=True))
      for distant in (False, True):
        self._meshes.append(GpuMesh(*vehicle_mesh(distant, lead=True)))
    except Exception:
      self.close()
      raise

  def _triangle(self, a, b, c, color, color_b=None, color_c=None):
    if self._count + 3 > CAPACITY:
      raise ValueError('3D scene exceeded fixed geometry budget')
    self._vertices[self._count:self._count+3] = (a,b,c)
    self._colors[self._count:self._count+3] = (color, color if color_b is None else color_b,
                                                color if color_c is None else color_c)
    self._count += 3

  def _ribbon(self, points, half_width, height, color, path=False):
    if not points:
      return
    sides = []
    for i, point in enumerate(points):
      x, y = point[:2]
      z = point[2] if len(point) > 2 else 0.
      before, after = points[max(0,i-1)], points[min(len(points)-1,i+1)]
      dx, dy = after[0]-before[0], after[1]-before[1]
      length = max(1e-6, math.hypot(dx,dy))
      nx, ny = -dy/length, dx/length
      width = min(self.scene.path_half_width(x,y), half_width) if path else half_width
      sides.append(((y-ny*width, z+height, -(x-nx*width)), (y+ny*width, z+height, -(x+nx*width))))
    for index, ((a,b),(c,d)) in enumerate(zip(sides,sides[1:], strict=False)):
      segment_color = color(index, len(sides)-1) if callable(color) else color
      self._triangle(a,b,c,segment_color)
      self._triangle(b,d,c,segment_color)

  @staticmethod
  def _asphalt_color(forward, lateral, height):
    # A depth-varying road material makes only the measured edge polygon read
    # as asphalt. It does not add map-derived shoulders or topology.
    horizon = max(0., min(1., forward / 96.))
    grade = max(0., min(1., abs(height) / 2.5))
    # This is subtle material variation, not a lane or road classification.
    grain = .5 + .5 * math.sin(forward*.21 + lateral*1.37) * math.sin(forward*.07-lateral*.83)
    return (7+int(horizon*8+grain*2), 17+int(horizon*19+grain*3),
            36+int(horizon*35+grade*6+grain*5), 255)

  def _road_surface(self, points):
    for (x0, left0, left_z0, right0, right_z0), (x1, left1, left_z1, right1, right_z1) in zip(points, points[1:], strict=False):
      a, b = (left0, left_z0+.001, -x0), (right0, right_z0+.001, -x0)
      c, d = (left1, left_z1+.001, -x1), (right1, right_z1+.001, -x1)
      ca, cb = self._asphalt_color(x0, left0, left_z0), self._asphalt_color(x0, right0, right_z0)
      cc, cd = self._asphalt_color(x1, left1, left_z1), self._asphalt_color(x1, right1, right_z1)
      self._triangle(a,b,c,ca,cb,cc)
      self._triangle(b,d,c,cb,cd,cc)

  def _flow_triangle(self, a, b, c, color):
    if self._flow_count + 3 > FLOW_CAPACITY:
      return
    self._flow_vertices[self._flow_count:self._flow_count+3] = (a,b,c)
    self._flow_colors[self._flow_count:self._flow_count+3] = color
    self._flow_count += 3

  def _flow_ribbon(self, points, half_width, height, color):
    if len(points) != 2:
      return
    (x0,y0,z0),(x1,y1,z1) = points
    dx,dy = x1-x0,y1-y0
    length = math.hypot(dx,dy)
    if length <= 1e-6:
      return
    nx,ny = -dy/length,dx/length
    a,b = (y0-ny*half_width,z0+height,-(x0-nx*half_width)),(y0+ny*half_width,z0+height,-(x0+nx*half_width))
    c,d = (y1-ny*half_width,z1+height,-(x1-nx*half_width)),(y1+ny*half_width,z1+height,-(x1+nx*half_width))
    self._flow_triangle(a,b,c,color)
    self._flow_triangle(b,d,c,color)

  @staticmethod
  def _line_sample(line, distance):
    if len(line) < 2:
      return None
    xy = tuple((x, y) for x, y, _ in line)
    heights = tuple(z for _, _, z in line)
    lateral = lateral_at(xy, distance)
    height = elevation_at(xy, heights, distance)
    return None if lateral is None or height is None else (lateral, height)

  def _flow_road_reflection(self, left, right, start, end, center, half_width, color):
    samples = (self._line_sample(left, start), self._line_sample(right, start),
               self._line_sample(left, end), self._line_sample(right, end))
    if any(sample is None for sample in samples):
      return
    left_start, right_start, left_end, right_end = samples
    # Long, uneven highlights read as wet-asphalt reflections rather than
    # cross-road stripes. They remain strictly inside the measured edge
    # polygon and do not assert a lane, object, or drivable-space boundary.
    def bounds(first, second):
      span = second[0]-first[0]
      margin = .14
      width = min(.15, half_width)
      low = max(margin, center-width)
      high = min(1.-margin, center+width)
      return (first[0]+span*low, first[1]+(second[1]-first[1])*low), \
             (first[0]+span*high, first[1]+(second[1]-first[1])*high)
    a, b = bounds(left_start,right_start)
    c, d = bounds(left_end,right_end)
    self._flow_triangle((a[0],a[1]+.003,-start), (b[0],b[1]+.003,-start),
                        (c[0],c[1]+.003,-end), color)
    self._flow_triangle((b[0],b[1]+.003,-start), (d[0],d[1]+.003,-end),
                        (c[0],c[1]+.003,-end), color)

  @staticmethod
  def _ego_speed(sm):
    try:
      speed = float(sm['carState'].vEgo)
    except (AttributeError, KeyError, TypeError, ValueError):
      return 0.0
    return min(40.0,max(0.0,speed)) if math.isfinite(speed) else 0.0

  def _advance_motion(self, now, speed, enabled):
    if self._motion_time is None or not 0.0 <= now-self._motion_time <= 1.0:
      self._motion_time = now
      return
    elapsed = now-self._motion_time
    self._motion_time = now
    if enabled:
      # This is literal travelled distance for visual flow only. It never moves
      # model lanes, traffic positions, paths, or their safety labels.
      self._motion_distance = (self._motion_distance+elapsed*speed) % 10.0

  def _update_flow(self, enabled, speed):
    key = (self.scene.revision, enabled, int(self._motion_distance*8), round(speed,1))
    if key == self._flow_key:
      return
    self._flow_count = 0
    display_edges = tuple(self.scene.edge_geometry(index) for index in range(2))
    if all(len(edge) >= 2 for edge in display_edges):
      near = max(3., display_edges[0][0][0])
      far = min(88., display_edges[0][-1][0], display_edges[1][-1][0])
      if far - near > 6.:
        phase = self._motion_distance if enabled else 0.
        for index in range(4):
          start = near + (index*23.-phase*1.15) % (far-near)
          end = min(far, start + 5.5 + start*.04)
          center = .31 if index % 2 else .68
          self._flow_road_reflection(*display_edges, start, end, center, .075,
                                     ROAD_SHEEN_HOT if index == 0 else ROAD_SHEEN)
    if enabled and speed >= .5 and len(self.scene.path) >= 2:
      horizon = min(72.0,self.scene.path[-1][0])
      distance = max(3.0,10.0-self._motion_distance)
      length = min(1.25,.36+speed*.026)
      while distance < horizon:
        end = min(horizon,distance+length)
        start_y = lateral_at(self.scene.path,distance)
        end_y = lateral_at(self.scene.path,end)
        if start_y is not None and end_y is not None:
          self._flow_ribbon(((distance,start_y,self.scene.road_height(distance)),
                             (end,end_y,self.scene.road_height(end))),.065,.023,ROAD_FLOW)
        distance += 10.0
    self._meshes[4].update(self._flow_vertices,self._flow_colors,self._flow_count)
    self._flow_key = key

  def _draw_ego_signals(self, sm, started_frame, now, anchor, yaw):
    if not self.scene.fresh(sm,'carState',started_frame,now):
      return
    car_state = sm['carState']
    active = ego_signal_flash(getattr(car_state, 'leftBlinker', False), getattr(car_state, 'rightBlinker', False), now)
    if any(active):
      # The independently baked mesh has no separate tail-lamp material. Its
      # depth surface would otherwise hide a coplanar light, so thin cuboids
      # overlay only the known lamp band and then restore depth.
      rl.rl_disable_depth_test()
      rl.rl_push_matrix()
      try:
        rl.rl_translatef(anchor.x,anchor.y,anchor.z)
        rl.rl_rotatef(yaw,0,1,0)
        for enabled,(x,y,z) in zip(active,EGO_REAR_SIGNAL_OFFSETS,strict=True):
          if enabled:
            # A thin, bright rectangular strip: never a circle or floating dot.
            rl.draw_cube(rl.Vector3(x,y,z),.22,.060,.024,SIGNAL_GLOW)
            rl.draw_cube(rl.Vector3(x,y,z+.014),.16,.028,.028,SIGNAL_YELLOW)
            rl.draw_cube(rl.Vector3(x,y,z+.030),.070,.009,.032,SIGNAL_HOTSPOT)
      finally:
        rl.rl_pop_matrix()
        rl.rl_enable_depth_test()

  def _ambient(self, size, motion):
    if self._ambient_texture is None:
      texture = rl.load_texture(str(_world_asset('carnival_aurora_ambient_v2.png')))
      if not texture.id:
        raise RuntimeError('Ambient world texture unavailable')
      self._ambient_texture = texture
      rl.set_texture_filter(texture,rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    texture = self._ambient_texture
    # The horizon only drifts a few pixels. It gives OLED pixels a gentle
    # refresh without becoming a competing animation while driving.
    offset = math.sin(self._motion_distance*.18)*size[0]*.008 if motion else 0.0
    source = rl.Rectangle(0,0,texture.width,texture.height)
    destination = rl.Rectangle(-size[0]*.02+offset,-size[1]*.01,size[0]*1.04,size[1]*1.02)
    rl.draw_texture_pro(texture,source,destination,rl.Vector2(0,0),0,WHITE)

  def _path_color(self, index, count, engaged):
    if self._intent_rainbow:
      hue = (.58 + index / max(1, count) * .42) % 1.
      red, green, blue = colorsys.hsv_to_rgb(hue, .72, 1.)
      return (int(red*255), int(green*255), int(blue*255), 235)
    if self._intent_acceleration:
      value = self._intent_acceleration[min(index, len(self._intent_acceleration)-1)]
      if value > .15:
        return (42, 185, 125, 235)
      if value < -.15:
        return (232, 108, 86, 235)
    return (40,149,246,235) if engaged else (166,179,189,230)

  def _update_geometry(self, engaged):
    key = (self.scene.revision, engaged, self._intent_key)
    if key == self._geometry_key:
      return
    self._count = 0
    # Keep the world OLED-black except for a road area bounded by two fresh,
    # plausible model edges. No map, lane-count, or road-type inference here.
    display_edges = tuple(self.scene.edge_geometry(index) for index in range(2))
    for surface in road_surface_geometry(*display_edges):
      self._road_surface(surface)
    for display in display_edges:
      self._ribbon(display,.18,.004,ROAD_EDGE_HALO)
      self._ribbon(display,.08,.009,ROAD_EDGE_MID)
      self._ribbon(display,.028,.014,ROAD_EDGE)
    for index in range(4):
      self._ribbon(self.scene.lane_geometry(index),.090,.010,LANE_GLOW)
      self._ribbon(self.scene.lane_geometry(index),.026,.016,LANE_MARKING)
    path = self._intent_path or self.scene.path
    path_geometry = self.scene.path_geometry(path)
    if self._intent_edge > 0:
      self._ribbon(path_geometry,self._intent_width,.017,PATH_EDGE,path=True)
    self._ribbon(path_geometry,self._intent_width*(1.-self._intent_edge),.020,
                 lambda index,count: self._path_color(index,count,engaged),path=True)
    self._meshes[2].update(self._vertices,self._colors,self._count)
    self._geometry_key = key

  def render(self, rect, sm, started_frame, engaged, now=None, parent_target=None, road_overlay=None,
             ambient=False, motion=False):
    now = time.monotonic() if now is None else now
    self.scene.update(sm, started_frame, now)
    if self.scene.object_key != self._lead_object_key:
      self._lead_object_key = self.scene.object_key
      self._lead_ids = frozenset(obj.identity for obj in self.scene.objects if obj.key[0] == 'lead')
    self.presentation.update(self.scene.objects, started_frame, now, self.scene.revision)
    speed = self._ego_speed(sm)
    self._advance_motion(now,speed,motion)
    self.overlay_exclusions.clear()
    # One color/depth target reused every frame, bounded independently of DPI.
    scale = min(1.0, 1440.0/max(1,rect.width), 810.0/max(1,rect.height))*self.quality.scale
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
      if road_overlay is not None:
        # Existing StarPilot controls configure display style here. The callback
        # deliberately issues no drawing commands in Tesla Road mode.
        road_overlay(self,rl.Rectangle(0,0,*size))
      self._update_geometry(engaged)
      self._update_flow(motion,speed)
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
      self._update_camera(now)
      rl.begin_texture_mode(self._target)
      rl.clear_background(BACKGROUND)
      if ambient:
        self._ambient(size,motion)
      rl.begin_mode_3d(self._camera)
      rl.rl_disable_backface_culling()
      try:
        self._meshes[2].draw()
        self._meshes[4].draw()
        far_ids = set()
        for identity, pose in self.presentation.poses.items():
          vehicle, radar_avatar = self.presentation.styles.get(identity, (False, False))
          if vehicle or radar_avatar:
            if pose.alpha <= .01:
              continue
            lead_proxy = vehicle and identity in self._lead_ids
            if lead_proxy:
              switch_distance = 30. if identity in self._far_ids else 34.
            else:
              switch_distance = 50. if identity in self._far_ids else 60.
            distant = pose.forward > switch_distance
            if distant:
              far_ids.add(identity)
            base = RADAR_AVATAR if radar_avatar else WHITE
            tint = rl.Color(base.r,base.g,base.b,round(base.a*pose.alpha))
            self._meshes[5+int(distant) if lead_proxy else int(distant)].draw(
              rl.Vector3(pose.right,self.scene.road_height(pose.forward),-pose.forward),pose.yaw,tint)
        for obj in self.scene.objects:
          if not (obj.vehicle or obj.radar_avatar):
            # A radar return has no verified body shape or vehicle class.
            rl.draw_sphere_ex(rl.Vector3(obj.right,self.scene.road_height(obj.forward)+.12,-obj.forward),.12,4,6,
                              rl.Color(125,133,140,255))
        self._far_ids = far_ids
        yaw = self.scene.ego_yaw
        angle = math.radians(yaw)
        # Pivot at the nose/path origin so the avatar cannot drift sideways
        # when showing near-path heading. This is intent, not measured yaw.
        speed_ratio = speed/40.0 if motion else 0.0
        # The camera, model road, and detected traffic are deliberately fixed.
        # Motion belongs exclusively to the on-screen ego chassis and scales
        # with ego speed, never with any sensor position.
        chassis_lateral = math.sin(now*(2.2+speed*.32))*speed_ratio*.008
        chassis_lift = math.sin(now*(2.8+speed*.21))*speed_ratio*.016
        anchor = rl.Vector3(EGO_CAMERA_OFFSET_M*math.sin(angle)+chassis_lateral,self.scene.road_height(0.)+chassis_lift,
                            EGO_CAMERA_OFFSET_M*math.cos(angle))
        display_yaw = yaw+math.sin(now*(1.8+speed*.12))*speed_ratio*.10
        self._meshes[3].draw(anchor,display_yaw)
        self._draw_ego_signals(sm,started_frame,now,anchor,display_yaw)
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
    pose = self.presentation.pose(obj)
    forward,right = (pose.forward,pose.right) if obj.vehicle or obj.radar_avatar else (obj.forward,obj.right)
    return self.project(forward,right,rect,height=self.scene.road_height(forward)+1.8)

  def project(self, forward, right, rect, height=0.02, clip=True):
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
    return (x,y) if not clip or (rect.x <= x <= rect.x+rect.width and rect.y <= y <= rect.y+rect.height) else None

  def project_ribbon(self, points, width, rect):
    if not points or not math.isfinite(width) or width < 0:
      return np.empty((0,2),np.float32)
    if len(points) < 2:
      return np.empty((0,2),np.float32)
    p = np.asarray(points,dtype=np.float64)
    xy = p[:,:2]
    heights = p[:,2] if p.shape[1] >= 3 else np.zeros(len(p),dtype=np.float64)
    delta = np.empty_like(xy)
    delta[1:-1] = xy[2:]-xy[:-2]
    delta[0],delta[-1] = xy[1]-xy[0],xy[-1]-xy[-2]
    norm = np.maximum(1e-6,np.hypot(delta[:,0],delta[:,1]))
    offset = np.column_stack((-delta[:,1],delta[:,0]))*(width/norm[:,None])
    both = np.concatenate((xy-offset,xy+offset))
    both_heights = np.concatenate((heights,heights))
    xyz = np.column_stack((both[:,1],both_heights+.02,-both[:,0]))
    view = (xyz-self._eye) @ self._view_basis.T
    depth = np.maximum(.01,view[:,2])
    focal = self._target_size[1]*self._focal_factor
    screen = np.column_stack((rect.x+rect.width*(.5+view[:,0]*focal/depth/self._target_size[0]),
                              rect.y+rect.height*(.5-view[:,1]*focal/depth/self._target_size[1])))
    valid = (view[:,2] > .01) & np.isfinite(screen).all(axis=1)
    # Keep offscreen endpoints: the GPU clips the connecting strip at the
    # viewport. Dropping pairs here truncated lanes before the screen edge.
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
    self._flow_key = None
    self._intent_key = None
    self._intent_path = ()
    self._intent_acceleration = ()
    self._motion_time = None
    self._motion_distance = 0.0
    if self._ambient_texture is not None:
      rl.unload_texture(self._ambient_texture)
      self._ambient_texture = None
    self.scene.reset()
    self.presentation.reset()
    self._far_ids.clear()
    self._lead_ids = frozenset()
    self._lead_object_key = None
    self.quality = RenderQuality()
