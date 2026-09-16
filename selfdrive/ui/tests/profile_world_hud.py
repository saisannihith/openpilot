"""Offroad full StarPilot view replay. No control publishers or parameter writes."""
import argparse
import cProfile
import gc
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace as NS

import numpy as np


def offroad():
  assert Path('/data/params/d/IsOnroad').read_bytes() == b'0', 'Stopped: device is onroad'


class ReadOnlyParams:
  def __init__(self, original, overrides=None):
    self.original = original
    self.overrides = overrides or {}

  def get_int(self, key, *args, **kwargs):
    return self.overrides[key] if key in self.overrides else self.original.get_int(key,*args,**kwargs)

  def __getattr__(self, name):
    if name.startswith(('put','remove','clear','delete')):
      raise AssertionError('Benchmark attempted a parameter write: '+name)
    return getattr(self.original,name)


def rss():
  return int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')//1024


def temperatures():
  result = {}
  for path in Path('/sys/class/thermal').glob('thermal_zone*'):
    try:
      kind = (path/'type').read_text().strip()
      value = float((path/'temp').read_text())
      if 0 < value < 200000:
        result[kind] = value/1000 if value > 1000 else value
    except (OSError,ValueError):
      pass
  return result


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--source',type=Path)
  parser.add_argument('--route',required=True)
  parser.add_argument('--out',type=Path,required=True)
  parser.add_argument('--frames',type=int,default=1200)
  parser.add_argument('--camera-check',action='store_true')
  parser.add_argument('--realtime',action='store_true')
  parser.add_argument('--profile',action='store_true')
  args = parser.parse_args()
  offroad()
  assert 200 <= args.frames <= 2400
  args.out.mkdir(parents=True,exist_ok=True)
  import openpilot.selfdrive.ui.onroad as package
  import openpilot.selfdrive.ui.onroad.starpilot as sp_package
  import openpilot.system.ui.lib as ui_lib
  if args.source:
    package.__path__.insert(0,str(args.source))
    sp_package.__path__.insert(0,str(args.source))
    ui_lib.__path__.insert(0,str(args.source))
  import pyray as rl
  from msgq.visionipc import VisionIpcServer, VisionStreamType
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.onroad import augmented_road_view as arv, tesla_road_renderer as tr
  from openpilot.selfdrive.ui.onroad.starpilot.starpilot_onroad_view import StarPilotOnroadView

  rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
  rl.init_window(2160,1080,'Isolated world HUD benchmark')
  gui_app._load_fonts()
  if args.source:
    from openpilot.common import basedir
    original_base = basedir.BASEDIR
    basedir.BASEDIR = str(args.source)
    try:
      tr.vehicle_mesh()
      tr.vehicle_mesh(True)
    finally:
      basedir.BASEDIR = original_base
  ui_state.ui_params = ReadOnlyParams(ui_state.ui_params,{'CameraView':5})
  ui_state.params = ReadOnlyParams(ui_state.params)
  ui_state.params_memory = ReadOnlyParams(ui_state.params_memory)
  arv.messaging.PubMaster = lambda *_args: NS(send=lambda *_args: None)
  ui_state.started,ui_state.started_frame,ui_state.status = True,0,UIStatus.ENGAGED
  ui_state._started_prev = True
  ui_state.started_time = 0.
  sm = ui_state.sm
  sm.frame = 0
  name = 'world_ui_test_'+str(os.getpid())
  vipc = VisionIpcServer(name)
  vipc.create_buffers(VisionStreamType.VISION_STREAM_ROAD,4,640,480)
  vipc.start_listener()
  yuv = np.full(640*480*3//2,128,np.uint8)
  yuv[:640*480] = 110
  view = StarPilotOnroadView()
  view._name = name
  rect = rl.Rectangle(0,0,2160,1080)
  clock = time.monotonic
  replay_time = 0.
  times, cpu, residents, levels, camera_frames = [],[],[],[],[]
  before_temp = temperatures()
  models = 0
  profiler = cProfile.Profile() if args.profile else None
  try:
    time.monotonic = lambda: replay_time
    for message in LogReader(args.route,sort_by_time=True):
      service = message.which()
      if service not in sm.data:
        continue
      replay_time = message.logMonoTime*1e-9
      inject_stale = args.camera_check and 300 <= models < 340 and service == 'modelV2'
      sm.frame += 1
      if not inject_stale:
        sm.data[service] = getattr(message,service)
        sm.seen[service] = sm.updated[service] = sm.alive[service] = True
        sm.valid[service] = message.valid
        sm.recv_time[service],sm.recv_frame[service] = replay_time,sm.frame
        sm.logMonoTime[service] = message.logMonoTime
      if service == 'carParams':
        ui_state.CP = sm['carParams']
        ui_state.has_longitudinal_control = ui_state.CP.openpilotLongitudinalControl
      if service != 'modelV2':
        continue
      models += 1
      if models%100 == 0:
        offroad()
        print('FRAME',models,flush=True)
      ui_state.status = UIStatus.ENGAGED if sm['selfdriveState'].enabled else UIStatus.DISENGAGED
      vipc.send(VisionStreamType.VISION_STREAM_ROAD,yuv,models,int(replay_time*1e9),int(replay_time*1e9))
      start, cpu_start = time.perf_counter(),time.process_time()
      if profiler is not None:
        profiler.enable()
      rl.begin_drawing()
      rl.clear_background(rl.BLACK)
      view.render(rect)
      gui_app._populate_render_texture_cache()
      rl.end_drawing()
      pixel = rl.rl_read_screen_pixels(1,1)
      rl.mem_free(pixel)
      if profiler is not None:
        profiler.disable()
      times.append((time.perf_counter()-start)*1000)
      cpu.append((time.process_time()-cpu_start)*1000)
      assert not view._world_failed, 'World renderer fell back due to exception'
      quality = getattr(view.tesla_road_renderer,'quality',None)
      levels.append(quality.level if quality is not None else 0)
      if args.camera_check and 310 <= models <= 340 and not view._tesla_road_view:
        if view.frame is not None:
          camera_frames.append(models)
      if models in (150,325,450,args.frames):
        image = rl.load_image_from_screen()
        try:
          assert rl.export_image(image,str(args.out/f'hud-{models}.png'))
          if args.camera_check and models == 325:
            c = rl.get_image_color(image,650,700)
            assert min(c.r,c.g,c.b) > 50 and max(c.r,c.g,c.b)-min(c.r,c.g,c.b) < 15, 'Camera pixels not restored'
        finally:
          rl.unload_image(image)
      if models == 450 and args.camera_check:
        assert view._tesla_road_view, 'World did not recover after fresh geometry'
      if models%100 == 0:
        gc.collect()
        residents.append(rss())
      sm.updated = dict.fromkeys(sm.updated,False)
      if args.realtime:
        time.sleep(max(0.,.05-(time.perf_counter()-start)))
      if models >= args.frames:
        break
    assert models >= min(1100,args.frames)
    if args.camera_check:
      assert len(camera_frames) >= 10, 'Insufficient real VisionIPC camera fallback frames'
    def stats(samples):
      return dict(zip(('median','p95','p99','max'),np.percentile(samples[100:],[50,95,99,100]).round(3).tolist(),strict=True))
    report = dict(route=args.route,frames=models,wall_ms=stats(times),cpu_ms=stats(cpu),rss_kib=residents,
                  levels=sorted(set(levels)),camera_frames=camera_frames,temperature_before=before_temp,temperature_after=temperatures())
    from openpilot.system.ui.lib import text_measure
    report['text_cache_entries'] = len(text_measure._cache)
    if profiler is not None:
      profiler.dump_stats(str(args.out/'render.prof'))
    (args.out/'report.json').write_text(json.dumps(report,indent=2))
    print('FULL_HUD_RESULT',json.dumps(report),flush=True)
  finally:
    time.monotonic = clock
    view.close()
    del vipc
    rl.close_window()


if __name__ == '__main__':
  main()
