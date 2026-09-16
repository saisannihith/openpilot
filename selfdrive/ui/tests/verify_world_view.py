"""Exercise production scene code with real Raylib, plus bounded input tests."""
import argparse
import gc
import importlib.util
import json
import math
import os
from pathlib import Path
import resource
import sys
import time
from types import SimpleNamespace as NS
import tracemalloc


def require_offroad():
  if Path('/AGNOS').exists():
    assert Path('/data/params/d/IsOnroad').read_bytes() == b'0', 'Device started driving; stopping UI benchmark'


def load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  sys.modules[name] = mod
  spec.loader.exec_module(mod)
  return mod


class Messages(dict):
  def tick(self, frame, now):
    self.recv_frame = dict.fromkeys(self, frame)
    self.recv_time = dict.fromkeys(self, now)
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)


def fixture(curve=0.0):
  def line(offset):
    x = [i*3.5 for i in range(33)]
    return NS(x=x, y=[offset + curve*t*t for t in x])
  def lead(d,y,status=True):
    return NS(status=status,dRel=d,yRel=y,modelProb=.95)
  errors = NS(canError=False,radarFault=False,wrongConfig=False,radarUnavailableTemporary=False)
  return Messages(modelV2=NS(position=line(0),laneLines=[line(x) for x in (-5.4,-1.8,1.8,5.4)],
                  laneLineProbs=[.8,.99,.99,.8],roadEdges=[line(-7.1),line(7.1)],roadEdgeStds=[.2,.2]),
                  radarState=NS(leadOne=lead(15,-curve*225),leadTwo=lead(45,-curve*2025)),
                  starpilotRadarState=NS(leadLeft=lead(10,3.5),leadRight=lead(24,-3.5)),
                  liveTracks=NS(errors=errors,points=[NS(trackId=i,dRel=8+i*5,yRel=(i%3-1)*5,vRel=0) for i in range(32)]))


def checks(module):
  scene = module.WorldScene()
  sm = fixture()
  sm.tick(10,1.0)
  scene.update(sm,0,1.0)
  assert len(scene.objects) <= 16
  left = next(obj for obj in scene.objects if obj.key == ('adjacent',0))
  assert left.right < 0
  assert scene.lanes[1][0][1] < 0
  previous = scene.objects
  scene.update(sm,0,1.1)
  assert scene.objects is previous, 'unchanged messages must not re-filter or allocate objects'
  scene.update(sm,0,1.4)
  assert not scene.objects and not scene.path, 'frozen valid messages must expire'
  sm.tick(11,2.0)
  sm['modelV2'].position.y[4] = float('nan')
  scene.update(sm,0,2.0)
  assert not scene.path
  scene.update(sm,11,2.0)
  assert not scene.objects, 'previous drive must not leak into a new drive'
  sm = fixture()
  sm.tick(12,3)
  sm['liveTracks'].errors.canError = True
  scene.update(sm,0,3)
  assert all(obj.key[0] != 'radar' for obj in scene.objects)
  assert abs(scene.path_half_width(30,0)-.85) < 1e-6
  assert scene.path_half_width(30,1.7) < .1
  return 10


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1]/'onroad')
  parser.add_argument('--out', type=Path, required=True)
  parser.add_argument('--frames', type=int, default=2000)
  parser.add_argument('--updates', type=int, default=12000)
  parser.add_argument('--progress', action='store_true')
  parser.add_argument('--cpu-only', action='store_true')
  parser.add_argument('--route', action='append', default=[])
  args = parser.parse_args()
  if args.frames <= 100 or args.updates < 1:
    parser.error('--frames must exceed 100 and --updates must be positive')
  require_offroad()
  args.out.mkdir(parents=True,exist_ok=True)
  model = load('openpilot.selfdrive.ui.onroad.world_scene', args.source/'world_scene.py')
  report = {'behavior_checks': checks(model)}
  report['replay'] = []
  snapshots = []
  if args.route:
    from openpilot.tools.lib.logreader import LogReader
    for route in args.route:
      replay_sm = Messages()
      replay_sm.tick(0, 0)
      replay_scene = model.WorldScene()
      frames = objects = paths = 0
      for message in LogReader(route, sort_by_time=True):
        name = message.which()
        if name not in ('modelV2', 'radarState', 'starpilotRadarState', 'liveTracks'):
          continue
        now = message.logMonoTime * 1e-9
        replay_sm[name] = getattr(message, name)
        replay_sm.recv_frame[name] = frames + 1
        replay_sm.recv_time[name] = now
        replay_sm.valid[name] = message.valid
        replay_sm.alive[name] = True
        replay_scene.update(replay_sm, 0, now)
        assert len(replay_scene.objects) <= model.MAX_OBJECTS
        frames += 1
        objects += len(replay_scene.objects)
        paths += bool(replay_scene.path)
        if frames % 200 == 0 and replay_scene.path and len(snapshots) < 24:
          # Offline-only capture. No retained capnp readers in the real UI.
          snapshots.append((replay_scene.path, replay_scene.lanes, replay_scene.edges, replay_scene.objects))
      report['replay'].append({'route': route, 'messages': frames, 'path_frames': paths, 'objects_total': objects})
  scene = model.WorldScene()
  sm = fixture(.0005)
  for i in range(300):
    sm.tick(i+1,100+i*.05)
    scene.update(sm,0,100+i*.05)
  tracemalloc.start()
  gc.collect()
  baseline = tracemalloc.get_traced_memory()[0]
  memory_samples = []
  for i in range(args.updates):
    if i % 500 == 0:
      require_offroad()
    sm.tick(i+400,200+i*.05)
    for j,p in enumerate(sm['liveTracks'].points):
      p.trackId = i*128+j
    scene.update(sm,0,200+i*.05)
    assert len(scene.objects) <= 16
    if i % 2000 == 1999:
      gc.collect()
      memory_samples.append(tracemalloc.get_traced_memory()[0]-baseline)
  gc.collect()
  report['retained_python_bytes'] = tracemalloc.get_traced_memory()[0]-baseline
  report['memory_updates'] = args.updates
  report['retained_python_samples'] = memory_samples
  report['peak_python_bytes'] = tracemalloc.get_traced_memory()[1]
  tracemalloc.stop()
  times = []
  for i in range(2000):
    sm.tick(i+13000,1000+i*.05)
    before = time.perf_counter()
    scene.update(sm,0,1000+i*.05)
    times.append((time.perf_counter()-before)*1000)
  times.sort()
  report['update_ms'] = {'median':times[len(times)//2],'p99':times[int(len(times)*.99)],'max':max(times)}
  if not args.cpu_only:
    if args.progress:
      print('CPU checks finished; initializing graphics', flush=True)
    import pyray as rl
    renderer_module = load('world_renderer_under_test',args.source/'tesla_road_renderer.py')
    if (args.source/'selfdrive/assets/world/sedan.npz').exists():
      from openpilot.common import basedir
      old_base = basedir.BASEDIR
      try:
        basedir.BASEDIR = str(args.source)
        renderer_module.vehicle_mesh()
      finally:
        basedir.BASEDIR = old_base
    rl.set_trace_log_level(rl.TraceLogLevel.LOG_INFO)
    rl.init_window(1440,720,'World view verification')
    if args.progress:
      print('Window initialized', flush=True)
    assert rl.is_window_ready()
    rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
    report['backend'] = os.getenv('RAYLIB_BACKEND', 'desktop')
    renderer = renderer_module.TeslaRoadRenderer()
    try:
      elapsed = []
      rss_samples = []
      current_rss = []
      cpu_elapsed = []
      sm = fixture(.0006)
      for frame in range(args.frames):
        if args.progress and frame % 100 == 0:
          print(f'Frame {frame}', flush=True)
        if frame % 50 == 0:
          require_offroad()
        now = 100 + frame*.05
        if frame % 2 == 0:
          sm.tick(frame+1,now)
          sm['radarState'].leadOne.dRel = 15 + math.sin(frame*.02)*3
        rect = rl.Rectangle(100,50,1240,620) if frame % 300 >= 290 else rl.Rectangle(0,0,1440,720)
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        rl.begin_scissor_mode(int(rect.x),int(rect.y),int(rect.width),int(rect.height))
        start = time.perf_counter()
        cpu_start = time.process_time()
        renderer.render(rect,sm,0,True,now=now)
        rl.end_scissor_mode()
        rl.end_drawing()
        # Readback fences the rendered frame using Raylib's own GL loader.
        # This is benchmark-only; the production renderer never reads pixels.
        pixel = rl.rl_read_screen_pixels(1, 1)
        rl.mem_free(pixel)
        elapsed.append((time.perf_counter()-start)*1000)
        cpu_elapsed.append((time.process_time()-cpu_start)*1000)
        if frame in (20,100,290,args.frames-1):
          screenshot = rl.load_image_from_screen()
          try:
            assert rl.export_image(screenshot, str(args.out/f'world-{frame}.png'))
            background = rl.get_image_color(screenshot, 25, 25)
            assert (background.r,background.g,background.b) == (0,0,0), 'World background is not OLED black'
            if frame == 100:
              import numpy as np
              rl.image_format(screenshot,rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
              pixels = np.frombuffer(rl.ffi.buffer(screenshot.data,screenshot.width*screenshot.height*4),dtype=np.uint8).reshape(-1,4)
              red = (pixels[:,0] > 180) & (pixels[:,1] < 90) & (pixels[:,2] < 90)
              white = (pixels[:,:3].min(axis=1) > 175) & (np.ptp(pixels[:,:3],axis=1) < 12)
              assert int(red.sum()) > 200, 'Red road edges did not render'
              assert int(white.sum()) > 1000, 'White vehicles/lanes did not render'
              report['palette_checks'] = {'black':True,'red_pixels':int(red.sum()),'white_pixels':int(white.sum())}
              anchor = renderer.lead_anchor(0,rect)
              assert anchor is not None and rect.x < anchor[0] < rect.x+rect.width and rect.y < anchor[1] < rect.y+rect.height
              assert renderer.lead_anchor(7,rect) is None
          finally:
            rl.unload_image(screenshot)
          rss_samples.append(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        if frame % 500 == 499:
          renderer.close()
          renderer.close()
          resident = int(Path('/proc/self/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE') // 1024
          current_rss.append(resident)
      warm = sorted(elapsed[100:])
      report['render_ms_including_present'] = {'median':warm[len(warm)//2],'p99':warm[int(len(warm)*.99)],'max':max(warm)}
      cpu = sorted(cpu_elapsed[100:])
      report['render_cpu_ms'] = {'median':cpu[len(cpu)//2], 'p99':cpu[int(len(cpu)*.99)]}
      report['rss_highwater_kib'] = rss_samples
      report['rss_after_cleanup_kib'] = current_rss
      report['render_frames'] = args.frames
      renderer._initialize()
      # Real route snapshots exercise the exact GPU geometry builder too.
      for path, lanes, edges, objects in snapshots:
        renderer.scene.path, renderer.scene.lanes, renderer.scene.edges = path, lanes, edges
        renderer.scene.objects = objects
        renderer.scene.revision += 1
        renderer._update_geometry(True)
        assert renderer._count <= renderer_module.CAPACITY
        assert renderer._meshes[2].model.meshes[0].vertexCount == renderer._count
      report['real_route_geometry_snapshots'] = len(snapshots)
      # Match GuiApplication's scaled render texture, then verify an overlay
      # lands in that same target with the original transform restored.
      parent = rl.load_render_texture(1440, 720)
      original_scissor = rl.begin_scissor_mode
      try:
        rl.begin_scissor_mode = lambda x,y,w,h: original_scissor(int(x*.75),int(y*.75),int(w*.75),int(h*.75))
        for aspect, width, height in [('landscape',1920,960), ('narrow',700,960)]:
          rl.begin_texture_mode(parent)
          rl.clear_background(rl.MAGENTA)
          rl.rl_push_matrix()
          rl.rl_scalef(.75,.75,1.)
          sm.tick(50000,10000.)
          renderer.render(rl.Rectangle(0,0,width,height),sm,0,True,now=10000.,parent_target=parent)
          rl.draw_rectangle(80,40,80,40,rl.RED)
          renderer.close(parent_target=parent)
          rl.draw_rectangle(200,40,80,40,rl.GREEN)
          rl.end_scissor_mode()
          rl.rl_pop_matrix()
          rl.end_texture_mode()
          screen = rl.load_image_from_texture(parent.texture)
          try:
            # Render texture image coordinates are bottom-up.
            pixel = rl.get_image_color(screen,90,720-45)
            assert rl.export_image(screen,str(args.out/f'composition-{aspect}.png'))
            assert pixel.r > 200 and pixel.g < 100 and pixel.b < 100, f'HUD transform or parent framebuffer lost: {pixel.r},{pixel.g},{pixel.b}'
            pixel = rl.get_image_color(screen,180,720-45)
            assert pixel.g > 200 and pixel.r < 100 and pixel.b < 100, 'View cleanup lost the active framebuffer'
          finally:
            rl.unload_image(screen)
        report['scaled_parent_composition_checks'] = 2
      finally:
        rl.begin_scissor_mode = original_scissor
        rl.unload_render_texture(parent)
    finally:
      renderer.close()
      rl.close_window()
  (args.out/'report.json').write_text(json.dumps(report,indent=2))
  print(json.dumps(report,indent=2))


if __name__ == '__main__':
  main()
