"""Bounded, display-only snapshots in model coordinates (forward, right).

Radar yRel is left-positive, unlike model y. Convert it exactly once here.
No track classification, brake-light state, or road topology is inferred.
"""
from dataclasses import dataclass
from itertools import islice
import math
from openpilot.selfdrive.ui.onroad.radar_visual_tracks import RadarVisualTracks

MAX_POINTS = 33
MAX_OBJECTS = 16
MAX_RADAR_INPUTS = 128
MAX_DISTANCE = 120.0
MAX_AGE = 0.35


def polyline(line):
  points = []
  previous = -math.inf
  for x, y in islice(zip(line.x, line.y, strict=False), MAX_POINTS):
    x, y = float(x), float(y)
    if not (math.isfinite(x) and math.isfinite(y)):
      return ()
    # At standstill, initial samples can wobble by micrometers around x=0.
    # Ignore only a sub-millimeter duplicate of the accepted origin; preserve
    # real reversals and every later forward-horizon truncation below.
    if len(points) == 1 and abs(points[0][0]) <= .001 and abs(x-points[0][0]) <= .001 and abs(y-points[0][1]) <= .001:
      continue
    # Stopped/turning trajectories can double back at the far horizon.
    # Keep their usable forward prefix instead of blanking the entire road.
    if x <= previous or abs(y) > 45.0 or x > MAX_DISTANCE:
      break
    previous = x
    if 0 <= x <= MAX_DISTANCE:
      points.append((x, y))
  return tuple(points) if len(points) > 1 else ()


def lateral_at(points, distance):
  if not points or distance < points[0][0] or distance > points[-1][0]:
    return None
  for (x0, y0), (x1, y1) in zip(points, points[1:], strict=False):
    if x0 <= distance <= x1:
      return y0 + (y1 - y0) * (distance - x0) / (x1 - x0)
  return points[-1][1]


def display_lane_continuation(points):
  """Illustrative near-field tail only; never change the detected scene data."""
  if len(points) < 2 or not 0 <= points[0][0] <= 2.:
    return points
  x,y = points[0]
  end = min(x+6.,points[-1][0])
  if end-x < 1.:
    return points
  slope = (lateral_at(points,end)-y)/(end-x)
  # No invented topology or unseen objects: only continue the current tangent
  # behind the avatar so visible lane ribbons can reach the viewport boundary.
  slope = max(-.4,min(.4,slope))
  return ((-8.,y+(-8.-x)*slope),*points)


def path_yaw(points):
  """Near-field intent heading in Raylib's right-handed Y-up coordinates."""
  if not points or points[0][0] > 2.0:
    return 0.0
  start = points[0][0]
  end = min(start + 6.0, points[-1][0])
  if end - start < 1.0:
    return 0.0
  slope = (lateral_at(points, end) - points[0][1]) / (end - start)
  return max(-35.0, min(35.0, -math.degrees(math.atan(slope))))


def road_yaw(lanes, forward, right):
  """Illustrative road tangent, not measured object heading or lane snapping."""
  def sample(line):
    if not line:
      return None
    lo,hi = max(line[0][0],forward-4.),min(line[-1][0],forward+4.)
    y = lateral_at(line,forward)
    if y is None or hi-lo < 2.:
      return None
    return y,(lateral_at(line,hi)-lateral_at(line,lo))/(hi-lo)
  values = [sample(line) for line in lanes]
  for left,right_line in zip(values,values[1:],strict=False):
    if left is None or right_line is None:
      continue
    width = right_line[0]-left[0]
    if 2. <= width <= 5.5 and left[0] <= right <= right_line[0]:
      blend = (right-left[0])/width
      slope = left[1]+blend*(right_line[1]-left[1])
      return max(-75.,min(75.,-math.degrees(math.atan(slope))))
  return None


def path_yaw_at(points, forward):
  """Model-path tangent for display-only radar avatars when lanes are weak."""
  if not points:
    return None
  lo, hi = max(points[0][0], forward - 4.), min(points[-1][0], forward + 4.)
  if hi - lo < 2.:
    return None
  y0, y1 = lateral_at(points, lo), lateral_at(points, hi)
  if y0 is None or y1 is None:
    return None
  return max(-75., min(75., -math.degrees(math.atan((y1-y0)/(hi-lo)))))


def road_surface_segments(first, second):
  """Paired, measured road-edge spans for display-only asphalt geometry."""
  if len(first) < 2 or len(second) < 2:
    return ()
  segments, current = [], []
  for x, y in first:
    other = lateral_at(second, x)
    # Do not bridge unknown edge sections or paint implausibly wide topology.
    if other is None or not 2.5 <= abs(other-y) <= 24.:
      if len(current) >= 2:
        segments.append(tuple(current))
      current = []
      continue
    current.append((x, y, other))
  if len(current) >= 2:
    segments.append(tuple(current))
  return tuple(segments)


def in_lane_corridor(lanes, forward, right):
  """Same lane bounds as road_yaw, without computing four unused tangents."""
  values = [lateral_at(line,forward) if line and min(line[-1][0],forward+4.)-max(line[0][0],forward-4.) >= 2.
            else None for line in lanes]
  return any(left is not None and other is not None and 2. <= other-left <= 5.5 and left <= right <= other
             for left,other in zip(values,values[1:],strict=False))


def vehicle_center(obj):
  # RadarState dRel is expressed from the ego front bumper. It provides no
  # validated target-bumper reference, so both model and raw-radar avatars use
  # the reported world coordinate directly. The sedan asset is already centered.
  return obj.forward,obj.right


@dataclass(slots=True)
class SceneObject:
  key: tuple
  forward: float
  right: float
  vehicle: bool
  received: float
  identity: tuple = ()
  yaw: float = 0.
  heading_time: float = 0.
  radar_avatar: bool = False
  direction: int = 1


class WorldScene:
  def __init__(self):
    self.reset()

  def reset(self):
    self.path = ()
    self.lanes = ((), (), (), ())
    self.edges = ((), ())
    self.objects = ()
    self.model_key = None
    self.object_key = None
    self.revision = 0
    self.ego_yaw = 0.0
    self.heading_time = None
    self.radar_visuals = RadarVisualTracks()
    self.radar_visual_key = None
    self.radar_avatars = {}

  @staticmethod
  def fresh(sm, name, started_frame, now):
    return (sm.valid.get(name, False) and sm.alive.get(name, False)
            and sm.recv_frame.get(name, -1) > started_frame
            and 0.0 <= now - sm.recv_time.get(name, 0.0) <= MAX_AGE)

  def update(self, sm, started_frame, now):
    model_ok = self.fresh(sm, 'modelV2', started_frame, now)
    model_key = (started_frame, sm.recv_frame['modelV2']) if model_ok else None
    model_changed = model_key != self.model_key
    if model_key != self.model_key:
      self.model_key = model_key
      self.path, self.lanes, self.edges = (), ((), (), (), ()), ((), ())
      if model_ok:
        model = sm['modelV2']
        self.path = polyline(model.position)
        self.lanes = tuple(polyline(model.laneLines[i]) if i < len(model.laneLines)
                           and i < len(model.laneLineProbs) and math.isfinite(model.laneLineProbs[i])
                           and model.laneLineProbs[i] >= 0.5 else () for i in range(4))
        self.edges = tuple(polyline(model.roadEdges[i]) if i < len(model.roadEdges)
                           and i < len(model.roadEdgeStds) and math.isfinite(model.roadEdgeStds[i])
                           and 0 <= model.roadEdgeStds[i] < 0.7 else () for i in range(2))
      received = sm.recv_time['modelV2'] if model_ok else None
      target_yaw = path_yaw(self.path)
      dt = received - self.heading_time if received is not None and self.heading_time is not None else 0.0
      blend = 1.0 - math.exp(-dt / .15) if 0 < dt < MAX_AGE else 1.0
      self.ego_yaw += blend * (target_yaw - self.ego_yaw)
      self.heading_time = received
      self.revision += 1

    sources = ('radarState', 'starpilotRadarState', 'liveTracks')
    available = tuple(self.fresh(sm, key, started_frame, now) for key in sources)
    if available[2]:
      errors = sm['liveTracks'].errors
      available = (*available[:2], not (errors.canError or errors.radarFault or errors.wrongConfig or
                                        errors.radarUnavailableTemporary))
    radar_key = (started_frame,sm.recv_frame['liveTracks']) if available[2] else None
    if not available[2] or not model_ok or not self.fresh(sm,'carState',started_frame,now):
      self.radar_visuals.reset()
      self.radar_avatars = {}
      self.radar_visual_key = None
    elif radar_key != self.radar_visual_key:
      if self.radar_visual_key is None or self.radar_visual_key[0] != started_frame:
        self.radar_visuals.reset()
      self.radar_avatars = self.radar_visuals.update(
        sm['liveTracks'].points, sm.recv_time['liveTracks'], float(sm['carState'].vEgo))
      self.radar_visual_key = radar_key
    object_key = (started_frame, bool(self.radar_avatars),
                  *(sm.recv_frame[key] if ok else -1 for key, ok in zip(sources, available, strict=False)))
    if object_key == self.object_key:
      if model_changed:
        self._orient_objects(sm.recv_time['modelV2'] if model_ok else now)
      return
    same_drive = self.object_key is not None and self.object_key[0] == started_frame
    self.object_key = object_key
    candidates = []

    def add(key, obj, vehicle, received):
      d, y = float(obj.dRel), -float(obj.yRel)
      if not (math.isfinite(d) and math.isfinite(y)) or not 0.5 <= d <= MAX_DISTANCE or abs(y) > 18:
        return
      track_id = int(getattr(obj,'trackId',-1)) if key[0] == 'radar' else int(getattr(obj,'radarTrackId',-1))
      tracked = key[0] == 'radar' or bool(getattr(obj,'radar',False))
      identity = ('track',track_id) if tracked and track_id >= 0 else ('slot',*key)
      if any(other.identity == identity or
             (not (other.identity[0] == identity[0] == 'track') and
              abs(other.forward-d) < 2.5 and abs(other.right-y) < 1.0) for other in candidates):
        return
      candidate = SceneObject(key, d, y, vehicle, received, identity)
      if not vehicle and tracked and track_id in self.radar_avatars:
        candidate.radar_avatar = True
        candidate.direction = self.radar_avatars[track_id]
      candidates.append(candidate)

    if available[0]:
      rs = sm['radarState']
      for index, lead in enumerate((rs.leadOne, rs.leadTwo)):
        if lead.status:
          add(('lead', index), lead, lead.modelProb >= 0.5, sm.recv_time['radarState'])
    if available[1]:
      rs = sm['starpilotRadarState']
      for index, lead in enumerate((rs.leadLeft, rs.leadRight)):
        # Adjacent radar-only targets have no verified vehicle class.
        if lead.status:
          add(('adjacent', index), lead, getattr(lead, 'modelProb', 0.0) >= 0.5,
              sm.recv_time['starpilotRadarState'])
    if available[2]:
      radar = sm['liveTracks']
      errors = radar.errors
      if not (errors.canError or errors.radarFault or errors.wrongConfig or errors.radarUnavailableTemporary):
        for point in islice(radar.points, MAX_RADAR_INPUTS):
          add(('radar', int(point.trackId)), point, False, sm.recv_time['liveTracks'])
    candidates.sort(key=lambda item: (not item.vehicle, not item.radar_avatar, item.forward))
    previous = {obj.identity: obj for obj in self.objects} if same_drive else {}
    for obj in candidates[:MAX_OBJECTS]:
      old = previous.get(obj.identity)
      if old is not None and 0 <= obj.received - old.received < MAX_AGE:
        if abs(obj.forward - old.forward) < 4.0 and abs(obj.right - old.right) < 1.0:
          blend = 1.0 - math.exp(-(obj.received - old.received) / 0.045)
          obj.forward = old.forward + blend * (obj.forward - old.forward)
          obj.right = old.right + blend * (obj.right - old.right)
          obj.yaw,obj.heading_time = old.yaw,old.heading_time
    self.objects = tuple(candidates[:MAX_OBJECTS])
    self._orient_objects(sm.recv_time['modelV2'] if model_ok else now)

  def _orient_objects(self, model_time):
    for obj in self.objects:
      if not (obj.vehicle or obj.radar_avatar):
        continue
      if obj.radar_avatar and obj.identity[1] not in self.radar_avatars:
        obj.radar_avatar = False
        continue
      target = road_yaw(self.lanes,obj.forward,obj.right)
      # The radar provides no vehicle yaw. When lane pairs are unavailable,
      # align its generic avatar to the model road path instead of discarding
      # a confirmed moving track as a dot.
      if target is None and obj.radar_avatar:
        target = path_yaw_at(self.path,obj.forward)
      received = max(model_time,obj.received)
      if target is None:
        if obj.radar_avatar:
          target = 0.
        else:
          obj.yaw,obj.heading_time = 0.,0.
          continue
      if obj.radar_avatar and obj.direction < 0:
        target += 180.
      dt = received-obj.heading_time
      blend = 1.0-math.exp(-dt/.12) if obj.heading_time and 0 <= dt < MAX_AGE else 1.
      obj.yaw += blend*((target-obj.yaw+180.)%360.-180.)
      obj.heading_time = received

  def path_half_width(self, forward, right):
    left = lateral_at(self.lanes[1], forward)
    other = lateral_at(self.lanes[2], forward)
    if left is not None and other is not None and left < right < other:
      return max(0.08, min(0.85, right - left - 0.12, other - right - 0.12))
    return 0.3
