from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.carnival_intersection_controller import CarnivalIntersectionController


def lead(**kwargs):
  values = dict(status=True, dRel=8.0, yRel=0.1, vLead=0.0, radar=True, modelProb=1.0)
  values.update(kwargs)
  return SimpleNamespace(**values)


def test_intersection_controller_holds_first_valid_stop_without_creep():
  controller = CarnivalIntersectionController("KIA_CARNIVAL_4TH_GEN", 0.05)
  output = controller.update(v_ego=0.4, lead=lead(), red_light=False, model_should_stop=False,
                             forcing_stop=False, driver_gas=False)
  assert output.state == "hold"
  assert output.should_stop
  assert output.accel_cap == -0.55


def test_intersection_controller_requires_confirmed_clear_before_release():
  controller = CarnivalIntersectionController("KIA_CARNIVAL_4TH_GEN", 0.05)
  controller.update(v_ego=0.0, lead=None, red_light=True, model_should_stop=True,
                    forcing_stop=False, driver_gas=False)
  for _ in range(15):
    output = controller.update(v_ego=0.0, lead=None, red_light=False, model_should_stop=False,
                               forcing_stop=False, driver_gas=False)
    assert output.state == "hold"
  output = controller.update(v_ego=0.0, lead=None, red_light=False, model_should_stop=False,
                             forcing_stop=False, driver_gas=False)
  assert output.state == "release"
  assert not output.should_stop


def test_intersection_controller_releases_for_confirmed_lead_departure():
  controller = CarnivalIntersectionController("KIA_CARNIVAL_4TH_GEN", 0.05)
  controller.update(v_ego=0.0, lead=lead(), red_light=False, model_should_stop=False,
                    forcing_stop=False, driver_gas=False)
  output = controller.update(v_ego=0.1, lead=lead(dRel=9.5, vLead=1.5), red_light=False,
                             model_should_stop=False, forcing_stop=False, driver_gas=False)
  assert output.state == "release"
  assert output.accel_cap is None


def test_intersection_controller_driver_accelerator_is_authoritative():
  controller = CarnivalIntersectionController("KIA_CARNIVAL_4TH_GEN", 0.05)
  controller.update(v_ego=0.0, lead=lead(), red_light=False, model_should_stop=False,
                    forcing_stop=False, driver_gas=False)
  output = controller.update(v_ego=0.0, lead=lead(), red_light=False, model_should_stop=False,
                             forcing_stop=False, driver_gas=True)
  assert output.state == "release"
  assert not output.should_stop


def test_intersection_controller_toggle_resets_latched_hold():
  controller = CarnivalIntersectionController("KIA_CARNIVAL_4TH_GEN", 0.05)
  controller.update(v_ego=0.0, lead=lead(), red_light=False, model_should_stop=False,
                    forcing_stop=False, driver_gas=False)
  output = controller.update(v_ego=0.0, lead=lead(), red_light=False, model_should_stop=False,
                             forcing_stop=False, driver_gas=False, feature_enabled=False)
  assert output.state == "idle"
  assert output.accel_cap is None
  assert not output.should_stop
