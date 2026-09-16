"""Bounded display interpolation and world availability; no control authority."""
from dataclasses import dataclass
import math

from openpilot.selfdrive.ui.onroad.world_scene import MAX_AGE, MAX_OBJECTS, WorldScene, polyline, vehicle_center


@dataclass(slots=True)
class Pose:
  forward: float
  right: float
  yaw: float


@dataclass(slots=True)
class Transition:
  start: Pose
  target: Pose
  stamp: float
  received: float
  source: tuple


def interpolate(transition, now):
  amount = min(1., max(0., (now-transition.stamp)/.05))
  a, b = transition.start, transition.target
  yaw_delta = (b.yaw-a.yaw+180.) % 360.-180.
  return Pose(a.forward+(b.forward-a.forward)*amount, a.right+(b.right-a.right)*amount,
              a.yaw+yaw_delta*amount)


class WorldPresentation:
  def __init__(self):
    self.reset()

  def reset(self):
    self.transitions = {}
    self.poses = {}
    self.drive = None
    self.time = None

  def update(self, objects, drive, now):
    if drive != self.drive or self.time is None or not 0 <= now-self.time <= MAX_AGE:
      self.reset()
    self.drive, self.time = drive, now
    transitions, poses = {}, {}
    for obj in objects[:MAX_OBJECTS]:
      if not (obj.vehicle or obj.radar_avatar) or not 0 <= now-obj.received <= MAX_AGE:
        continue
      target = Pose(*vehicle_center(obj), obj.yaw)
      old = self.transitions.get(obj.identity)
      # Radar and model publications are asynchronous. An identity-preserving
      # handoff may legitimately carry a slightly older (but still fresh) sample.
      handoff = old is not None and old.source != obj.key and obj.identity[0] == 'track'
      continuous = old is not None and (abs(obj.received-old.received) < MAX_AGE if handoff
                                        else 0 <= obj.received-old.received < MAX_AGE)
      if not continuous:
        current = target
      else:
        current = interpolate(old, now)
        # Recycled IDs, newly selected unrelated leads and teleports are not
        # animated through intervening traffic. No extrapolation on dropout.
        if abs(target.forward-current.forward) > 4. or abs(target.right-current.right) > 1.:
          current = target
      if old is None or target != old.target or obj.received != old.received or handoff:
        old = Transition(current, target, now, obj.received, obj.key)
      transitions[obj.identity] = old
      poses[obj.identity] = interpolate(old, now)
    self.transitions, self.poses = transitions, poses

  def pose(self, obj):
    pose = self.poses.get(obj.identity)
    return pose if pose is not None else Pose(*vehicle_center(obj), obj.yaw)


class WorldAvailability:
  """Fallback on stale/invalid model; require sustained recovery before return."""
  def __init__(self):
    self.drive = None
    self.since = None
    self.active = False
    self.last_time = None

  def update(self, sm, drive, now):
    if drive != self.drive or (self.last_time is not None and not 0 <= now-self.last_time <= MAX_AGE):
      self.since, self.active = None, False
    self.drive, self.last_time = drive, now
    usable = WorldScene.fresh(sm, 'modelV2', drive, now)
    if usable:
      # No lane lines required: unmarked roads still have a model path.
      usable = bool(polyline(sm['modelV2'].position))
    if not usable:
      self.since, self.active = None, False
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
