"""Offroad bounded-history/per-frame CPU check of the display adapter."""
import gc
import argparse
import json
import math
from pathlib import Path
import time
import tracemalloc
from types import SimpleNamespace as NS
parser = argparse.ArgumentParser()
parser.add_argument('--source',type=Path)
parser.add_argument('--out',type=Path,default=Path('/tmp/world-radar-stress.json'))
parser.add_argument('--timing-updates',type=int,default=1000)
parser.add_argument('--memory-updates',type=int,default=1000)
args = parser.parse_args()
assert args.timing_updates >= 100 and args.memory_updates >= 100
if args.source:
  import openpilot.selfdrive.ui.onroad as package
  package.__path__.insert(0,str(args.source))
from openpilot.selfdrive.ui.onroad.world_scene import WorldScene
from openpilot.selfdrive.ui.onroad.world_presentation import WorldPresentation
from verify_world_view import fixture

assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
scene,sm = WorldScene(),fixture(.0001)
presentation = WorldPresentation()
sm['carState'] = NS(vEgo=20.)
def update(i):
  sm['liveTracks'].points = [NS(trackId=(i//100)*1000+j,dRel=10.+(j%24)*4+math.sin(i*.01),
                              yRel=-(j%3-1)*3.5,vRel=0.,measured=True) for j in range(128)]
  sm.tick(i+1,10+i*.05)
  scene.update(sm,0,10+i*.05)
  presentation.update(scene.objects,0,10+i*.05)
  presentation.update(scene.objects,0,10+i*.05+.025)
  assert len(presentation.poses) <= 16 and len(presentation.transitions) <= 16
  assert len(scene.radar_visuals.history) <= 128 and len(scene.objects) <= 16

times=[]
for i in range(args.timing_updates):
  if i%100 == 0:
    assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
  start=time.perf_counter()
  update(i)
  times.append((time.perf_counter()-start)*1000)
tracemalloc.start()
warm_end = args.timing_updates+200
for i in range(args.timing_updates,warm_end):
  update(i)
gc.collect()
baseline=tracemalloc.get_traced_memory()[0]
for i in range(warm_end,warm_end+args.memory_updates):
  if i%100 == 0:
    assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
  update(i)
gc.collect()
retained=tracemalloc.get_traced_memory()[0]-baseline
assert retained < 128000
times.sort()
report=dict(updates=warm_end+args.memory_updates,max_inputs=128,history=len(scene.radar_visuals.history),objects=len(scene.objects),
            retained_bytes=retained,median_ms=times[len(times)//2],p99_ms=times[int(len(times)*.99)])
args.out.write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
