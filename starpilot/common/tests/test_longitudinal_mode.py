from openpilot.starpilot.common.longitudinal_mode import (
  reconcile_longitudinal_mode_params,
  set_alpha_longitudinal,
  set_conditional_drive_mode,
  set_experimental_mode,
  set_openpilot_long_disabled,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value):
    self.values[key] = bool(value)


def test_longitudinal_controls_are_mutually_exclusive():
  params = FakeParams({"AlphaLongitudinalEnabled": True})

  set_openpilot_long_disabled(params, True)
  assert params.get_bool("DisableOpenpilotLongitudinal")
  assert not params.get_bool("AlphaLongitudinalEnabled")

  set_alpha_longitudinal(params, True)
  assert params.get_bool("AlphaLongitudinalEnabled")
  assert not params.get_bool("DisableOpenpilotLongitudinal")


def test_conditional_modes_match_base_experimental_state():
  params = FakeParams()

  set_conditional_drive_mode(params, "chill")
  assert params.get_bool("ConditionalChill")
  assert not params.get_bool("ConditionalExperimental")
  assert params.get_bool("ExperimentalMode")

  set_conditional_drive_mode(params, "experimental")
  assert not params.get_bool("ConditionalChill")
  assert params.get_bool("ConditionalExperimental")
  assert not params.get_bool("ExperimentalMode")


def test_base_experimental_toggle_disables_conditional_modes():
  params = FakeParams({"ConditionalExperimental": True, "ConditionalChill": True})
  set_experimental_mode(params, False)
  assert not params.get_bool("ExperimentalMode")
  assert not params.get_bool("ConditionalExperimental")
  assert not params.get_bool("ConditionalChill")

  params.values["ConditionalExperimental"] = True
  params.values["ConditionalChill"] = True
  set_experimental_mode(params, True)
  assert params.get_bool("ExperimentalMode")
  assert not params.get_bool("ConditionalExperimental")
  assert not params.get_bool("ConditionalChill")


def test_reconcile_preserves_base_experimental_mode():
  params = FakeParams({"ExperimentalMode": True})

  updates = reconcile_longitudinal_mode_params(params)

  assert updates == {}
  assert params.get_bool("ExperimentalMode")
  assert not params.get_bool("ConditionalChill")


def test_reconcile_prefers_explicit_disable_and_conditional_chill():
  params = FakeParams({
    "AlphaLongitudinalEnabled": True,
    "DisableOpenpilotLongitudinal": True,
    "ConditionalExperimental": True,
    "ConditionalChill": True,
    "ExperimentalMode": False,
  })
  cache = FakeParams(dict(params.values))

  updates = reconcile_longitudinal_mode_params(params, cache)

  assert updates == {
    "AlphaLongitudinalEnabled": False,
    "ConditionalExperimental": False,
    "ExperimentalMode": True,
  }
  assert params.values == cache.values
