from openpilot.common.params import Params


def set_alpha_longitudinal(params: Params, enabled: bool) -> None:
  params.put_bool("AlphaLongitudinalEnabled", enabled)
  if enabled:
    params.put_bool("DisableOpenpilotLongitudinal", False)


def set_openpilot_long_disabled(params: Params, disabled: bool) -> None:
  params.put_bool("DisableOpenpilotLongitudinal", disabled)
  if disabled:
    params.put_bool("AlphaLongitudinalEnabled", False)


def set_experimental_mode(params: Params, enabled: bool) -> None:
  params.put_bool("ExperimentalMode", enabled)
  # The base toggle is an explicit mode selection. Conditional modes remain
  # available through their own controls, but must not silently override it.
  params.put_bool("ConditionalExperimental", False)
  params.put_bool("ConditionalChill", False)


def set_conditional_drive_mode(params: Params, mode: str) -> None:
  conditional_experimental = mode == "experimental"
  conditional_chill = mode == "chill"
  params.put_bool("ConditionalExperimental", conditional_experimental)
  params.put_bool("ConditionalChill", conditional_chill)

  if conditional_experimental:
    params.put_bool("ExperimentalMode", False)
  elif conditional_chill:
    params.put_bool("ExperimentalMode", True)


def reconcile_longitudinal_mode_params(params: Params, params_cache: Params | None = None) -> dict[str, bool]:
  updates: dict[str, bool] = {}

  def update(key: str, value: bool) -> None:
    if params.get_bool(key) != value:
      params.put_bool(key, value)
      updates[key] = value
    if params_cache is not None and params_cache.get_bool(key) != value:
      params_cache.put_bool(key, value)

  if params.get_bool("DisableOpenpilotLongitudinal") and params.get_bool("AlphaLongitudinalEnabled"):
    update("AlphaLongitudinalEnabled", False)

  conditional_experimental = params.get_bool("ConditionalExperimental")
  conditional_chill = params.get_bool("ConditionalChill")
  if conditional_chill:
    update("ConditionalExperimental", False)
    update("ExperimentalMode", True)
  elif conditional_experimental:
    update("ExperimentalMode", False)

  return updates
