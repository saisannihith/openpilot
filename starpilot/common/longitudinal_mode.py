"""Coherent mode Params reads with compatibility helpers for classic controls."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path

from openpilot.common.params import Params

MODE_KEYS = ("ExperimentalMode", "ConditionalChill", "ConditionalExperimental")


@contextmanager
def mode_lock(params, *, exclusive=False):
  directory = Path(params.get_param_path()).parent
  fd = os.open(directory / ".longitudinal_mode.lock", os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o660)
  try:
    fcntl.flock(fd, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)
    yield
  finally:
    os.close(fd)


def read_mode_values(params):
  with mode_lock(params):
    return {key: params.get_bool(key) for key in MODE_KEYS}


def request_mode_refresh(params, params_memory, toggles):
  try:
    values = read_mode_values(params)
  except OSError:
    return
  if values != getattr(toggles, "longitudinal_mode_values", None):
    params_memory.put_bool("StarPilotTogglesUpdated", True)


# Classic device and legacy Galaxy controls still use these helpers. The new
# Galaxy endpoint uses the locked transaction in system/the_galaxy directly.
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
