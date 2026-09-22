"""Replay real route messages through the bounded Tesla Road scene adapter."""
import argparse
import json
import math
from pathlib import Path


class Messages(dict):
  def __init__(self):
    super().__init__()
    self.valid = {}
    self.alive = {}
    self.recv_frame = {}
    self.recv_time = {}


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--source', type=Path, required=True)
  parser.add_argument('--route', type=Path, required=True)
  parser.add_argument('--out', type=Path, required=True)
  args = parser.parse_args()
  if Path('/AGNOS').exists():
    assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'

  import openpilot.selfdrive.ui.onroad as onroad
  onroad.__path__.insert(0, str(args.source))
  from openpilot.selfdrive.ui.onroad.world_scene import MAX_DISPLAY_GRADE, MAX_OBJECTS, WorldScene
  from openpilot.selfdrive.ui.onroad.world_presentation import WorldPresentation
  from openpilot.tools.lib.logreader import LogReader

  services = {'modelV2', 'radarState', 'starpilotRadarState', 'liveTracks', 'carState'}
  sm, scene, presentation = Messages(), WorldScene(), WorldPresentation()
  models = messages = grade_frames = actor_frames = fade_frames = retired_frames = 0
  max_abs_height = max_grade = max_retired_age = 0.
  worst_grade_segment = None
  identities = set()
  for message in LogReader(str(args.route), sort_by_time=True):
    name = message.which()
    if name not in services:
      continue
    messages += 1
    now = message.logMonoTime * 1e-9
    sm[name] = getattr(message, name)
    sm.valid[name] = message.valid
    sm.alive[name] = True
    sm.recv_frame[name] = messages
    sm.recv_time[name] = now
    scene.update(sm, 0, now)
    presentation.update(scene.objects, 0, now, scene.revision)
    assert len(scene.objects) <= MAX_OBJECTS and len(presentation.poses) <= MAX_OBJECTS
    active = {obj.identity for obj in scene.objects if obj.vehicle or obj.radar_avatar}
    for identity, transition in presentation.transitions.items():
      if identity not in active:
        age = now-transition.received
        assert 0. <= age < .12, 'retired actor survived its bounded fade window'
        retired_frames += 1
        max_retired_age = max(max_retired_age, age)
    identities.update(obj.identity for obj in scene.objects)
    actor_frames += bool(presentation.poses)
    fade_frames += sum(0. < pose.alpha < 1. for pose in presentation.poses.values())
    if name != 'modelV2' or not scene.path:
      continue
    models += 1
    max_abs_height = max(max_abs_height, *(abs(value) for value in scene.path_elevations))
    for (x0, _), (x1, _) , z0, z1 in zip(scene.path, scene.path[1:], scene.path_elevations, scene.path_elevations[1:], strict=False):
      if x1 > x0:
        grade = abs((z1-z0)/(x1-x0))
        if grade > max_grade:
          max_grade = grade
          worst_grade_segment = (x0, x1, z0, z1)
    grade_frames += any(abs(value) > .01 for value in scene.path_elevations)
  assert models > 0, 'Route did not contain usable modelV2 frames'
  assert max_grade <= MAX_DISPLAY_GRADE + 1e-9, (
    f'display grade exceeded {MAX_DISPLAY_GRADE}: {max_grade} at {worst_grade_segment}')
  report = {
    'route': str(args.route), 'messages': messages, 'model_frames': models,
    'grade_frames': grade_frames, 'max_abs_height_m': max_abs_height,
    'max_grade': max_grade, 'actor_frames': actor_frames, 'fade_frames': fade_frames,
    'retired_actor_frames': retired_frames, 'max_retired_age_s': max_retired_age,
    'unique_actor_identities': len(identities), 'max_objects': MAX_OBJECTS,
  }
  args.out.write_text(json.dumps(report, indent=2))
  print(json.dumps(report), flush=True)


if __name__ == '__main__':
  main()
