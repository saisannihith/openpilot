"""Display-only motion history. An avatar is not an OEM object classification."""
from dataclasses import dataclass
from itertools import islice
import math


@dataclass(slots=True)
class VisualHistory:
  time: float
  distance: float
  lateral: float
  velocity: float
  direction: int
  moving_since: float
  samples: int
  qualified: bool


class RadarVisualTracks:
  def __init__(self):
    self.history = {}

  def reset(self):
    self.history.clear()

  def update(self, points, now, v_ego, in_lane):
    current, avatars, seen = {}, {}, set()
    if not math.isfinite(v_ego) or not 0 <= v_ego <= 80:
      self.reset()
      return avatars
    for point in islice(points,128):
      d,y,v = float(point.dRel),-float(point.yRel),float(point.vRel)
      track = int(point.trackId)
      if track in seen:
        current.pop(track,None)
        avatars.pop(track,None)
        continue
      seen.add(track)
      if (not getattr(point,'measured',False) or track < 0 or
          not all(math.isfinite(x) for x in (d,y,v)) or not .5 <= d <= 120 or
          abs(y) > 18 or abs(v) > 100 or not in_lane(d,y)):
        continue
      old = self.history.get(track)
      dt = now-old.time if old else 0.
      continuous = (old is not None and 0 < dt <= .35 and
                    abs(d-old.distance-.5*(v+old.velocity)*dt) <= .5+4*dt and
                    abs(y-old.lateral) <= .5+3*dt)
      speed = v_ego+v
      direction = 1 if 3 < speed <= 65 else (-1 if -80 <= speed < -3 else 0)
      same_motion = continuous and direction != 0 and direction == old.direction
      # A stopped return becomes an avatar only if this same continuous target
      # was already observed moving. Do not turn static clutter into parked cars.
      held_stop = continuous and old.qualified and abs(speed) <= 3
      since = old.moving_since if same_motion or held_stop else now
      samples = old.samples+1 if same_motion or held_stop else 1
      qualified = held_stop or (direction != 0 and samples >= 4 and now-since >= .3)
      if held_stop:
        direction = old.direction
      current[track] = VisualHistory(now,d,y,v,direction,since,samples,qualified)
      if qualified:
        avatars[track] = direction
    # Missing IDs, recycled IDs with jumps, and stale tracks leave no ghosts.
    self.history = current
    return avatars
