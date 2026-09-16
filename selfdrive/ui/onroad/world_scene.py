"""Bounded, display-only snapshots in model coordinates (forward, right).

Radar yRel is left-positive, unlike model y. Convert it exactly once here.
No track classification, brake-light state, or road topology is inferred.
"""
from dataclasses import dataclass
from itertools import islice
import math

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


@dataclass(slots=True)
class SceneObject:
  key: tuple
  forward: float
  right: float
  vehicle: bool
  received: float


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

  @staticmethod
  def fresh(sm, name, started_frame, now):
    return (sm.valid.get(name, False) and sm.alive.get(name, False)
            and sm.recv_frame.get(name, -1) > started_frame
            and 0.0 <= now - sm.recv_time.get(name, 0.0) <= MAX_AGE)

  def update(self, sm, started_frame, now):
    model_ok = self.fresh(sm, 'modelV2', started_frame, now)
    model_key = (started_frame, sm.recv_frame['modelV2']) if model_ok else None
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
    object_key = (started_frame, *(sm.recv_frame[key] if ok else -1 for key, ok in zip(sources, available, strict=False)))
    if object_key == self.object_key:
      return
    self.object_key = object_key
    candidates = []

    def add(key, obj, vehicle, received):
      d, y = float(obj.dRel), -float(obj.yRel)
      if not (math.isfinite(d) and math.isfinite(y)) or not 0.5 <= d <= MAX_DISTANCE or abs(y) > 18:
        return
      if any(abs(other.forward - d) < 2.5 and abs(other.right - y) < 1.0 for other in candidates):
        return
      candidates.append(SceneObject(key, d, y, vehicle, received))

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
    candidates.sort(key=lambda item: (not item.vehicle, item.forward))
    previous = {obj.key: obj for obj in self.objects}
    for obj in candidates[:MAX_OBJECTS]:
      old = previous.get(obj.key)
      if old is not None and 0 < obj.received - old.received < MAX_AGE:
        if abs(obj.forward - old.forward) < 4.0 and abs(obj.right - old.right) < 1.0:
          blend = 1.0 - math.exp(-(obj.received - old.received) / 0.045)
          obj.forward = old.forward + blend * (obj.forward - old.forward)
          obj.right = old.right + blend * (obj.right - old.right)
    self.objects = tuple(candidates[:MAX_OBJECTS])

  def path_half_width(self, forward, right):
    left = lateral_at(self.lanes[1], forward)
    other = lateral_at(self.lanes[2], forward)
    if left is not None and other is not None and left < right < other:
      return max(0.08, min(0.85, right - left - 0.12, other - right - 0.12))
    return 0.3
