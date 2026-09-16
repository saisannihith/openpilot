"""Offroad-only native GPU capture of the production world AND lead overlay."""
import argparse
from pathlib import Path
import time
from types import SimpleNamespace as NS


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--source', type=Path)
  parser.add_argument('--out', type=Path, required=True)
  args = parser.parse_args()
  if Path('/AGNOS').exists():
    assert Path('/data/params/d/IsOnroad').read_bytes() == b'0'
  import openpilot.selfdrive.ui.onroad as package
  if args.source:
    package.__path__.insert(0,str(args.source))
  import pyray as rl
  from openpilot.selfdrive.ui.onroad import model_renderer
  from openpilot.selfdrive.ui.onroad.tesla_road_renderer import TeslaRoadRenderer
  from openpilot.system.ui.lib.application import gui_app
  from verify_world_view import fixture

  args.out.mkdir(parents=True,exist_ok=True)
  rl.init_window(1440,720,'World lead overlay verification')
  assert rl.is_window_ready()
  gui_app._load_fonts()
  world = TeslaRoadRenderer()
  overlay = object.__new__(model_renderer.ModelRenderer)
  overlay._params = NS(get_bool=lambda k: k == 'LeadInfo', get_int=lambda k: 50, get=lambda k: b'0')
  overlay._longitudinal_control = True
  saved_state = model_renderer.ui_state
  try:
    for name,curve in [('straight',0.),('right',.006),('left',-.006)]:
      world.close()
      sm = fixture(curve)
      sm['carState'] = NS(vEgo=22.)
      sm['carParams'] = NS(openpilotLongitudinalControl=True)
      sm['starpilotPlan'] = NS(desiredFollowDistance=30.)
      for lead in (sm['radarState'].leadOne,sm['radarState'].leadTwo):
        lead.vRel, lead.vLead = -2.,20.
      sm.updated = {'carParams':True}
      model_renderer.ui_state = NS(sm=sm,started_frame=0,is_metric=False,starpilot_toggles={})
      for frame in range(3):
        now = time.monotonic()
        sm.tick(frame+1,now)
        rect = rl.Rectangle(0,0,1440,720)
        rl.begin_drawing()
        rl.clear_background(rl.BLACK)
        world.render(rect,sm,0,True,now=now)
        overlay.render_world_leads(rect,world)
        rl.end_scissor_mode()
        rl.end_drawing()
      assert overlay._lead_text_rects, 'Actual lead labels failed to lay out'
      assert len(overlay._lead_vehicles[0].chevron) == 3
      assert (world.scene.ego_yaw < 0) if curve > 0 else (world.scene.ego_yaw >= 0)
      screenshot = rl.load_image_from_screen()
      try:
        x,y = overlay._lead_vehicles[0].chevron[1]
        pixel = rl.get_image_color(screenshot,round(x),round(y)-6)
        expected = model_renderer.get_theme_color('LeadMarker',rl.Color(201,34,49,255))
        assert max(abs(pixel.r-expected.r),abs(pixel.g-expected.g),abs(pixel.b-expected.b)) <= 2, 'Lead icon not rasterized'
        assert rl.export_image(screenshot,str(args.out/f'lead-{name}.png'))
      finally:
        rl.unload_image(screenshot)
    print('WORLD_AND_LEAD_GPU_OK: straight/left/right with production fonts, icons and metrics')
  finally:
    model_renderer.ui_state = saved_state
    world.close()
    for font in gui_app._fonts.values():
      rl.unload_font(font)
    gui_app._fonts.clear()
    rl.close_window()


if __name__ == '__main__':
  main()
