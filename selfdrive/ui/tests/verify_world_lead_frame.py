"""Offroad-only native GPU capture of the production world AND lead overlay."""
import argparse
import json
from pathlib import Path
import time
from types import SimpleNamespace as NS


def verify_corner_cache():
  import numpy as np
  import pyray as rl
  from openpilot.selfdrive.ui.onroad.starpilot.favorite_radial_menu import FavoriteRadialMenu
  from openpilot.system.ui.lib.application import gui_app

  for pressed in (False, True):
    key = f'favorites_corner_hint_v1_{int(pressed)}'
    gui_app.cached_render_texture(key, 150, 150,
      lambda p=pressed: FavoriteRadialMenu._draw_corner_hint_vectors(0., 150., 1., p))
    gui_app._populate_render_texture_cache()
    texture = gui_app._cached_render_textures[key].texture
    rl.begin_drawing()
    rl.clear_background(rl.Color(60, 80, 100, 255))
    FavoriteRadialMenu._draw_corner_hint_vectors(0., 150., 1., pressed)
    rl.begin_blend_mode(rl.BlendMode.BLEND_ALPHA_PREMULTIPLY)
    rl.draw_texture_pro(texture, rl.Rectangle(0, 0, 150, -150),
                        rl.Rectangle(200, 0, 150, 150), rl.Vector2(0, 0), 0., rl.WHITE)
    rl.end_blend_mode()
    rl.end_drawing()
    image = rl.load_image_from_screen()
    try:
      pixels = np.frombuffer(rl.ffi.buffer(image.data, image.width * image.height * 4), dtype=np.uint8)
      pixels = pixels.reshape(image.height, image.width, 4).astype(np.int16)
      delta = np.abs(pixels[:150, :150, :3] - pixels[:150, 200:350, :3])
      # Repeated translucent alpha blends round differently on the cached pass.
      assert np.mean(delta) < 3.0 and np.percentile(delta, 99) < 15, (pressed, np.mean(delta), np.max(delta))
      print('CORNER_CACHE_PIXELS_OK', pressed, float(np.mean(delta)), flush=True)
    finally:
      rl.unload_image(image)


def replay_routes(routes, world, overlay, settings, out):
  import pyray as rl
  from cereal import messaging
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.selfdrive.ui.onroad import model_renderer, world_overlays
  from openpilot.selfdrive.ui.onroad.starpilot import path, stopping_point
  from openpilot.system.ui.lib.application import gui_app
  from verify_world_view import Messages
  services = ('modelV2','radarState','starpilotRadarState','liveTracks','carState','carParams',
              'starpilotPlan','selfdriveState','longitudinalPlan')
  clock = time.monotonic
  replay_time = 0.
  reports = []
  try:
    time.monotonic = lambda: replay_time
    for route in routes:
      world.close()
      sm = Messages({k:getattr(messaging.new_message(k),k) for k in services})
      sm.tick(0,0.)
      sm.valid = dict.fromkeys(services,False)
      sm.alive = dict.fromkeys(services,False)
      sm.updated = dict.fromkeys(services,False)
      state = NS(sm=sm,started_frame=0,is_metric=False,starpilot_toggles={},ui_params=overlay._params,
                 status=2,always_on_lateral_active=False)
      model_renderer.ui_state = path.ui_state = stopping_point.ui_state = state
      messages = models = rendered = 0
      oriented = 0
      radar_avatar_frames = oncoming_frames = 0
      max_avatars = 0
      capture_frame_id = None
      lane_counts = set()
      for message in LogReader(route,sort_by_time=True):
        name = message.which()
        if name not in services:
          continue
        if messages % 500 == 0 and Path('/AGNOS').exists():
          assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
        messages += 1
        replay_time = message.logMonoTime*1e-9
        sm[name] = getattr(message,name)
        sm.updated[name] = True
        sm.valid[name],sm.alive[name] = message.valid,True
        sm.recv_frame[name],sm.recv_time[name] = messages,replay_time
        # Preserve real message timing even when GPU captures are subsampled.
        world.scene.update(sm,0,replay_time)
        if name != 'modelV2':
          continue
        # Exercise presentation at model cadence even when GPU is subsampled.
        world.presentation.update(world.scene.objects,0,replay_time)
        assert len(world.presentation.poses) <= 16
        for obj in world.scene.objects:
          if obj.vehicle or obj.radar_avatar:
            from openpilot.selfdrive.ui.onroad.world_scene import vehicle_center
            d,y = vehicle_center(obj)
            pose = world.presentation.pose(obj)
            assert abs(pose.forward-d) <= 4.01 and abs(pose.right-y) <= 1.01
        models += 1
        if models % 10:
          continue
        # Exercise real messages through all three shared path color policies.
        settings['RainbowPath'] = rendered%3 == 1
        settings['AccelerationPath'] = rendered%3 == 2
        rect = rl.Rectangle(0,0,rl.get_screen_width(),rl.get_screen_height())
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        world.render(rect,sm,0,True,now=replay_time,road_overlay=overlay.render_world_road)
        identities = [o.identity for o in world.scene.objects]
        assert len(identities) == len(set(identities)), 'Duplicate physical target in scene'
        avatars = [o for o in world.scene.objects if o.radar_avatar]
        radar_avatar_frames += bool(avatars)
        oncoming_frames += any(o.direction < 0 for o in avatars)
        lane_counts.add(sum(bool(line) for line in world.scene.lanes))
        assert len(world.scene.radar_visuals.history) <= 128
        raw = {p.trackId:p for p in sm['liveTracks'].points}
        for obj in world.scene.objects:
          if obj.key[0] != 'radar':
            continue
          assert world.scene.fresh(sm,'liveTracks',0,replay_time)
          point = raw[obj.key[1]]
          assert abs(obj.forward-point.dRel) < 4.01 and abs(obj.right+point.yRel) < 1.01
        oriented += sum(o.vehicle and abs(o.yaw) > 1. for o in world.scene.objects)
        occupied = stopping_point.render_stopping_point(overlay,gui_app.font(),
          project_stop=lambda d, r=rect, s=state: world_overlays.stop_anchor(world,r,s,d))
        if occupied is not None:
          world.overlay_exclusions.append(occupied)
        overlay.render_world_leads(rect,world)
        assert sum(bool(v.chevron) for v in overlay._lead_vehicles) <= 1
        rl.end_scissor_mode()
        rl.end_drawing()
        if len(avatars) > max_avatars:
          max_avatars = len(avatars)
          capture_frame_id = int(sm['modelV2'].frameId)
          screenshot = rl.load_image_from_screen()
          try:
            assert rl.export_image(screenshot,str(out/f'replay-{len(reports)}-radar.png'))
          finally:
            rl.unload_image(screenshot)
        rendered += 1
        sm.updated = dict.fromkeys(services,False)
      assert rendered > 0
      report = dict(route=route,messages=messages,model_frames=models,gpu_frames=rendered,
                    oriented_vehicle_frames=oriented,radar_avatar_frames=radar_avatar_frames,
                    max_radar_avatars=max_avatars,oncoming_frames=oncoming_frames,lane_line_counts=sorted(lane_counts),
                    capture_frame_id=capture_frame_id)
      reports.append(report)
      (out/'replay-results.json').write_text(json.dumps(reports,indent=2))
      print('REAL_MESSAGE_REPLAY_OK',report,flush=True)
  finally:
    time.monotonic = clock


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--source', type=Path)
  parser.add_argument('--out', type=Path, required=True)
  parser.add_argument('--frames',type=int,default=3)
  parser.add_argument('--width',type=int,default=1440)
  parser.add_argument('--height',type=int,default=720)
  parser.add_argument('--full-bleed',action='store_true')
  parser.add_argument('--with-stop',action='store_true',help='Explicit synthetic STOP/collision test; not a detected sign')
  parser.add_argument('--rainbow',action='store_true')
  parser.add_argument('--route',action='append',default=[])
  args = parser.parse_args()
  if Path('/AGNOS').exists():
    assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
  import openpilot.selfdrive.ui.onroad as package
  if args.source:
    package.__path__.insert(0,str(args.source))
    import openpilot.selfdrive.ui.onroad.starpilot as sp_package
    sp_package.__path__.insert(0,str(args.source))
    import openpilot.system.ui.lib as ui_lib
    ui_lib.__path__.insert(0,str(args.source))
  import pyray as rl
  from openpilot.selfdrive.ui.onroad import model_renderer
  from openpilot.selfdrive.ui.onroad.tesla_road_renderer import TeslaRoadRenderer
  from openpilot.selfdrive.ui.onroad import tesla_road_renderer, world_overlays
  from openpilot.selfdrive.ui.onroad.starpilot import path, stopping_point
  from openpilot.system.ui.lib.application import gui_app
  from verify_world_view import fixture

  args.out.mkdir(parents=True,exist_ok=True)
  rl.init_window(args.width,args.height,'World lead overlay verification')
  assert rl.is_window_ready()
  gui_app._load_fonts()
  verify_corner_cache()
  if args.source and (args.source/'selfdrive/assets/world/sedan.npz').exists():
    from openpilot.common import basedir
    old_base = basedir.BASEDIR
    try:
      basedir.BASEDIR = str(args.source)
      tesla_road_renderer.vehicle_mesh()
      tesla_road_renderer.vehicle_mesh(True)
    finally:
      basedir.BASEDIR = old_base
  world = TeslaRoadRenderer()
  overlay = model_renderer.ModelRenderer()
  settings = {'LeadInfo':True,'ShowStoppingPoint':True,'ShowStoppingPointMetrics':True,
              'AdjacentPath':True,'BlindSpotPath':True,'ModelUI':True,'PathColor':'#2895f6',
              'AccelerationPath':False,'RainbowPath':args.rainbow}
  overlay._params = NS(get_bool=lambda k,**kw: bool(settings.get(k,kw.get('default',False))),
                       get_int=lambda k,**kw: 50, get_float=lambda k,**kw: 3.5,
                       get=lambda k,**kw: settings.get(k))
  overlay._longitudinal_control = True
  saved_state = model_renderer.ui_state
  render_times = []
  try:
    for name,curve in [('straight',0.),('right',.006),('left',-.006)]:
      world.close()
      sm = fixture(curve)
      sm['carState'] = NS(vEgo=22.,standstill=False,leftBlindspot=True,rightBlindspot=False)
      sm['carParams'] = NS(openpilotLongitudinalControl=True)
      sm['starpilotPlan'] = NS(desiredFollowDistance=30.,laneWidthLeft=3.5,laneWidthRight=3.5,
                              redLight=args.with_stop,forcingStopLength=28.)
      sm['longitudinalPlan'] = NS(allowThrottle=True)
      sm['selfdriveState'] = NS(experimentalMode=False)
      for lead in (sm['radarState'].leadOne,sm['radarState'].leadTwo):
        lead.vRel, lead.vLead = -2.,20.
      sm.updated = {'carParams':True}
      model_renderer.ui_state = NS(sm=sm,started_frame=0,is_metric=False,starpilot_toggles={},
                                  ui_params=overlay._params,status=2,always_on_lateral_active=False)
      path.ui_state = stopping_point.ui_state = model_renderer.ui_state
      for frame in range(args.frames):
        if frame % 50 == 0 and Path('/AGNOS').exists():
          assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
        now = time.monotonic()
        sm.tick(frame+1,now)
        rect = rl.Rectangle(0,0,args.width,args.height)
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        started = time.perf_counter()
        world.render(rect,sm,0,True,now=now,road_overlay=overlay.render_world_road)
        occupied = stopping_point.render_stopping_point(overlay,gui_app.font(),
          project_stop=lambda d, r=rect: world_overlays.stop_anchor(world,r,model_renderer.ui_state,d))
        if occupied is not None:
          world.overlay_exclusions.append(occupied)
        overlay.render_world_leads(rect,world)
        rl.end_scissor_mode()
        rl.end_drawing()
        pixel = rl.rl_read_screen_pixels(1,1)
        rl.mem_free(pixel)
        render_times.append((time.perf_counter()-started)*1000)
      assert len(overlay._lead_text_rects) > len(world.overlay_exclusions), 'Actual lead labels failed to lay out'
      assert all(not rl.check_collision_recs(a,b) for a in world.overlay_exclusions
                 for b in overlay._lead_text_rects[len(world.overlay_exclusions):]), 'Stop and lead text overlap'
      assert len(overlay._lead_vehicles[0].chevron) == 3
      assert not overlay._lead_vehicles[1].chevron
      for distance,lateral in ((0.,0.),(15.,2.),(50.,-3.)):
        expected = rl.get_world_to_screen_ex(rl.Vector3(lateral,.02,-distance),world._camera,args.width,args.height)
        actual = world.project(distance,lateral,rect)
        assert actual is not None and max(abs(actual[0]-expected.x),abs(actual[1]-expected.y)) < .001
      assert (world.scene.ego_yaw < 0) if curve > 0 else (world.scene.ego_yaw >= 0)
      screenshot = rl.load_image_from_screen()
      try:
        x,y = overlay._lead_vehicles[0].chevron[1]
        pixel = rl.get_image_color(screenshot,round(x),round(y)-6)
        expected = model_renderer.get_theme_color('LeadMarker',rl.Color(201,34,49,255))
        assert max(abs(pixel.r-expected.r),abs(pixel.g-expected.g),abs(pixel.b-expected.b)) <= 2, 'Lead icon not rasterized'
        if args.full_bleed:
          import numpy as np
          rl.image_format(screenshot,rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
          pixels = np.frombuffer(rl.ffi.buffer(screenshot.data,screenshot.width*screenshot.height*4),dtype=np.uint8)
          pixels = pixels.reshape(screenshot.height,screenshot.width,4)
          white = (pixels[-4:,:,:3].min(axis=2) > 175) & (np.ptp(pixels[-4:,:,:3],axis=2) < 12)
          assert white.sum() > 16, 'Lane lines stop before the bottom of the display'
        assert rl.export_image(screenshot,str(args.out/f'lead-{name}.png'))
      finally:
        rl.unload_image(screenshot)
    print('WORLD_AND_LEAD_GPU_OK: straight/left/right with production fonts, icons and metrics')
    # Exercise live path-style changes on the same initialized renderer.
    samples = []
    for rainbow in (False,True,False):
      settings['RainbowPath'] = rainbow
      sm.tick(4000,time.monotonic())
      rect = rl.Rectangle(0,0,1440,720)
      rl.begin_drawing()
      rl.clear_background(rl.BLACK)
      world.render(rect,sm,0,True,road_overlay=overlay.render_world_road)
      rl.end_scissor_mode()
      rl.end_drawing()
      shot = rl.load_image_from_screen()
      try:
        colors = []
        for d in (5.,8.,11.):
          y = world_overlays.lateral_at(world.scene.path,d)
          x,sy = world.project(d,y,rect)
          c = rl.get_image_color(shot,round(x),round(sy))
          colors.append((c.r,c.g,c.b))
        samples.append(colors)
      finally:
        rl.unload_image(shot)
    print('PATH_TOGGLE_PIXELS',samples,flush=True)
    assert samples[0] == samples[2], 'Turning rainbow off did not restore plain path'
    assert any(max(abs(a-b) for a,b in zip(c,d,strict=True)) > 20
               for c,d in zip(samples[0],samples[1],strict=True)), 'Rainbow toggle did not change path pixels'
    print('LIVE_RAINBOW_PIXEL_TOGGLE_OK')
    import numpy as np
    print('FULL_OVERLAY_RENDER_MS',dict(zip(('median','p99','max'),np.percentile(render_times,[50,99,100]).round(2).tolist(),strict=True)))
    parent = rl.load_render_texture(1440,720)
    original_scissor = rl.begin_scissor_mode
    try:
      rl.begin_scissor_mode = lambda x,y,w,h: original_scissor(int(x*.75),int(y*.75),int(w*.75),int(h*.75))
      for iteration,width in enumerate((1920,700)*10):
        rl.begin_texture_mode(parent)
        rl.clear_background(rl.MAGENTA)
        rl.rl_push_matrix()
        rl.rl_scalef(.75,.75,1.)
        now = time.monotonic()
        sm.tick(5000,now)
        world.quality.level = iteration%3
        world.render(rl.Rectangle(0,0,width,960),sm,0,True,now=now,parent_target=parent,
                     road_overlay=overlay.render_world_road)
        expected_scale = min(1.,1440/width,810/960)*world.quality.scale
        assert world._target_size == (max(2,int(width*expected_scale)),max(2,int(960*expected_scale)))
        rl.draw_rectangle(80,40,80,40,rl.RED)
        world.close(parent_target=parent)
        rl.draw_rectangle(200,40,80,40,rl.GREEN)
        rl.end_scissor_mode()
        rl.rl_pop_matrix()
        rl.end_texture_mode()
        screenshot = rl.load_image_from_texture(parent.texture)
        try:
          before = rl.get_image_color(screenshot,90,675)
          after = rl.get_image_color(screenshot,180,675)
          assert before.r > 200 and before.g < 100, 'Shared road callback lost HUD framebuffer'
          assert after.g > 200 and after.r < 100, 'World close lost HUD framebuffer'
        finally:
          rl.unload_image(screenshot)
      print('SHARED_ROAD_SCALED_PARENT_OK: 20 hot resource recreations, landscape and narrow')
    finally:
      rl.begin_scissor_mode = original_scissor
      rl.unload_render_texture(parent)
    replay_routes(args.route,world,overlay,settings,args.out)
  finally:
    model_renderer.ui_state = saved_state
    world.close()
    for font in gui_app._fonts.values():
      rl.unload_font(font)
    gui_app._fonts.clear()
    rl.close_window()


if __name__ == '__main__':
  main()
