"""Bounded display interpolation and world availability; no control authority."""
from dataclasses import dataclass
import math

from openpilot.selfdrive.ui.onroad.world_scene import MAX_AGE, MAX_OBJECTS, WorldScene, polyline, vehicle_center


@dataclass(slots=True)
class Pose:
  forward: float
  right: float
  yaw: float
  alpha: float = 1.0


@dataclass(slots=True)
class Transition:
  start: Pose
  target: Pose
  stamp: float
  received: float
  source: tuple
  duration: float = .05


def interpolate(transition, now, output=None):
  amount = min(1., max(0., (now-transition.stamp)/transition.duration))
  a, b = transition.start, transition.target
  yaw_delta = (b.yaw-a.yaw+180.) % 360.-180.
  output = Pose(0.,0.,0.) if output is None else output
  output.forward = a.forward+(b.forward-a.forward)*amount
  output.right = a.right+(b.right-a.right)*amount
  output.yaw = a.yaw+yaw_delta*amount
  output.alpha = a.alpha+(b.alpha-a.alpha)*amount
  return output


class WorldPresentation:
  def __init__(self):
    self.reset()

  def reset(self):
    self.transitions = {}
    self.poses = {}
    self.styles = {}
    self.drive = None
    self.time = None
    self.object_snapshot = None
    self.snapshot_revision = None

  def update(self, objects, drive, now, revision=None):
    if drive != self.drive or self.time is None or not 0 <= now-self.time <= MAX_AGE:
      self.reset()
    self.drive, self.time = drive, now
    if (revision is not None and revision == self.snapshot_revision and objects is self.object_snapshot
        and all(0 <= now-t.received <= MAX_AGE for t in self.transitions.values())):
      expired = tuple(identity for identity, transition in self.transitions.items()
                      if transition.target.alpha == 0.0 and now-transition.received >= transition.duration)
      for identity in expired:
        self.transitions.pop(identity, None)
        self.poses.pop(identity, None)
        self.styles.pop(identity, None)
      for identity, transition in self.transitions.items():
        interpolate(transition, now, self.poses[identity])
      return
    self.object_snapshot = objects
    self.snapshot_revision = revision
    transitions, poses, styles = {}, {}, {}
    for obj in objects[:MAX_OBJECTS]:
      if not (obj.vehicle or obj.radar_avatar) or not 0 <= now-obj.received <= MAX_AGE:
        continue
      target = Pose(*vehicle_center(obj), obj.yaw, 1.0)
      old = self.transitions.get(obj.identity)
      # Radar and model publications are asynchronous. An identity-preserving
      # handoff may legitimately carry a slightly older (but still fresh) sample.
      handoff = old is not None and old.source != obj.key and obj.identity[0] == 'track'
      continuous = old is not None and (abs(obj.received-old.received) < MAX_AGE if handoff
                                        else 0 <= obj.received-old.received < MAX_AGE)
      if not continuous:
        # New model/radar avatars are visible immediately, but ease to full
        # opacity instead of popping over a raw radar dot.
        current = Pose(target.forward, target.right, target.yaw, .35) if old is None else target
      else:
        current = interpolate(old, now)
        # Recycled IDs, newly selected unrelated leads and teleports are not
        # animated through intervening traffic. No extrapolation on dropout.
        if abs(target.forward-current.forward) > 4. or abs(target.right-current.right) > 1.:
          current = target
      if old is None or target != old.target or obj.received != old.received or handoff:
        old = Transition(current, target, now, obj.received, obj.key, .12 if old is None else .05)
      transitions[obj.identity] = old
      poses[obj.identity] = interpolate(old, now)
      styles[obj.identity] = (obj.vehicle, obj.radar_avatar)

    # A target can disappear when a message is still fresh. Let the exact last
    # observation dissolve for at most 120 ms; never extrapolate it or retain a
    # visual ghost past the source's freshness budget.
    for identity, old in self.transitions.items():
      if identity in transitions:
        continue
      age = now-old.received
      if not 0. <= age < .12:
        continue
      current = interpolate(old, now)
      final = Pose(old.target.forward, old.target.right, old.target.yaw, 0.0)
      retiring = old if old.target.alpha == 0.0 else Transition(current, final, old.received, old.received, old.source, .12)
      transitions[identity] = retiring
      poses[identity] = interpolate(retiring, now)
      styles[identity] = self.styles.get(identity, (False, True))
    self.transitions, self.poses, self.styles = transitions, poses, styles

  def pose(self, obj):
    pose = self.poses.get(obj.identity)
    return pose if pose is not None else Pose(*vehicle_center(obj), obj.yaw)


class WorldAvailability:
  """Keep the camera selected after an active world view loses model data."""
  def __init__(self):
    self.drive = None
    self.since = None
    self.active = False
    self.last_time = None
    self.model_key = None
    self.geometry_valid = False
    self.fallback_reason = None

  def update(self, sm, drive, now):
    if drive != self.drive:
      self.fallback_reason = None
      self.since, self.active = None, False
    elif self.last_time is not None and not 0 <= now-self.last_time <= MAX_AGE and not self.active:
      # A slow render does not invalidate fresh model data. Only restart warmup.
      self.since = None
    self.drive, self.last_time = drive, now
    usable = WorldScene.fresh(sm, 'modelV2', drive, now)
    key = (drive, sm.recv_frame['modelV2']) if usable else None
    if key != self.model_key:
      self.model_key = key
      # No lane lines required: unmarked roads still have a model path.
      self.geometry_valid = usable and bool(polyline(sm['modelV2'].position))
    usable = usable and self.geometry_valid
    if not usable:
      if self.active:
        self.fallback_reason = 'model data unavailable'
      self.since, self.active = None, False
    elif self.fallback_reason is not None:
      return False
    elif self.since is None:
      self.since = now
    elif now-self.since >= .75:
      self.active = True
    return self.active


def world_lead_lines(distance, speed, ego, desired, distance_unit, speed_unit, dc, sc):
  """Unavailable plan/ego values must not masquerade as zero distance or gap."""
  lines = [f'{round(distance*dc)} {distance_unit}  |  {round(speed*sc)}{speed_unit}']
  details = []
  if math.isfinite(ego) and ego >= 1.:
    details.append(f'{distance/ego:.2f} s')
  if math.isfinite(desired) and desired > 0:
    details.append(f'desired {round(desired*dc)} {distance_unit}')
  if details:
    lines.append('  |  '.join(details))
  return lines


class RenderQuality:
  """Three bounded world-only resolutions, with warmup and asymmetric recovery."""
  scales = (1., .85, .7)

  def __init__(self):
    self.level = 0
    self.warmup = 20
    self.slow = self.fast = 0
    self.changed = -math.inf
    self.enabled = True

  @property
  def scale(self):
    return self.scales[self.level] if self.enabled else 1.

  def observe(self, milliseconds, now):
    if not self.enabled or not math.isfinite(milliseconds) or milliseconds < 0:
      return
    if self.warmup:
      self.warmup -= 1
      return
    self.slow = self.slow+1 if milliseconds > 35. else 0
    self.fast = self.fast+1 if milliseconds < 18. else 0
    if now-self.changed < 5.:
      return
    next_level = self.level
    if self.slow >= 12:
      next_level = min(2, self.level+1)
    elif self.fast >= 200:
      next_level = max(0, self.level-1)
    if next_level != self.level:
      self.level, self.changed = next_level, now
      self.slow = self.fast = 0
      self.warmup = 20


def degraded_world_text(sm, drive, now, active, failed):
  if failed:
    return 'World rendering unavailable - camera view'
  if not active:
    return 'World data unavailable - camera view'
  if not WorldScene.fresh(sm, 'radarState', drive, now):
    return 'Lead data unavailable'
  # Empty fresh returns are not an outage. A never-seen optional raw stream
  # does not indicate a fault on vehicles that do not publish radar tracks.
  if sm.recv_frame.get('liveTracks', -1) <= drive:
    return ''
  if not WorldScene.fresh(sm, 'liveTracks', drive, now):
    return 'Radar visualization unavailable'
  errors = sm['liveTracks'].errors
  if errors.canError or errors.radarFault or errors.wrongConfig or errors.radarUnavailableTemporary:
    return 'Radar visualization unavailable'
  return ''
