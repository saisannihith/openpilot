#!/usr/bin/env python3
from __future__ import annotations

import importlib
import inspect
import math
import os
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if (ROOT / "tools").is_dir() and (ROOT / "common").is_dir():
  OPENPILOT_ROOT = ROOT
else:
  OPENPILOT_ROOT = ROOT / "openpilot"
sys.path.insert(0, str(OPENPILOT_ROOT))


def install_openpilot_namespace() -> None:
  namespace = types.ModuleType("openpilot")
  namespace.__path__ = [str(OPENPILOT_ROOT)]  # type: ignore[attr-defined]
  sys.modules["openpilot"] = namespace


class Approx:
  def __init__(self, expected, rel=1e-6, abs=1e-12):
    self.expected = expected
    self.rel = rel
    self.abs = abs

  def __eq__(self, actual) -> bool:
    try:
      return math.isclose(float(actual), float(self.expected), rel_tol=self.rel, abs_tol=self.abs)
    except Exception:
      return actual == self.expected

  def __repr__(self) -> str:
    return f"approx({self.expected!r})"


class Raises:
  def __init__(self, exc_type):
    self.exc_type = exc_type

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc, _tb) -> bool:
    if exc_type is None:
      raise AssertionError(f"expected {self.exc_type}")
    return issubclass(exc_type, self.exc_type)


class Mark:
  def __getattr__(self, _name):
    def decorator(*_args, **_kwargs):
      if _args and callable(_args[0]) and len(_args) == 1 and not _kwargs:
        return _args[0]
      return lambda fn: fn
    return decorator


class MonkeyPatch:
  _missing = object()

  def __init__(self):
    self._undo: list[tuple[str, object, object, object]] = []

  def setattr(self, target, name=None, value=_missing):
    if isinstance(target, str):
      if value is not self._missing:
        raise TypeError("string target form takes exactly two arguments")
      parts = target.split(".")
      obj = None
      attr_parts: list[str] = []
      for index in range(len(parts) - 1, 0, -1):
        module_name = ".".join(parts[:index])
        try:
          obj = importlib.import_module(module_name)
          attr_parts = parts[index:]
          break
        except ModuleNotFoundError:
          continue
      if obj is None or not attr_parts:
        raise ModuleNotFoundError(target)
      for attr in attr_parts[:-1]:
        obj = getattr(obj, attr)
      attr_name = attr_parts[-1]
      new_value = name
      old_value = getattr(obj, attr_name, self._missing)
      self._undo.append(("attr", obj, attr_name, old_value))
      setattr(obj, attr_name, new_value)
      return

    if name is None or value is self._missing:
      raise TypeError("object target form requires name and value")
    old_value = getattr(target, name, self._missing)
    self._undo.append(("attr", target, name, old_value))
    setattr(target, name, value)

  def setitem(self, mapping, key, value):
    old_value = mapping.get(key, self._missing)
    self._undo.append(("item", mapping, key, old_value))
    mapping[key] = value

  def setenv(self, key, value):
    old_value = os.environ.get(key, self._missing)
    self._undo.append(("env", os.environ, key, old_value))
    os.environ[key] = str(value)

  def undo(self):
    while self._undo:
      kind, obj, key, old_value = self._undo.pop()
      if old_value is self._missing:
        if kind == "attr":
          try:
            delattr(obj, key)
          except AttributeError:
            pass
        elif kind == "item":
          obj.pop(key, None)
        elif kind == "env":
          obj.pop(key, None)
      elif kind == "attr":
        setattr(obj, key, old_value)
      else:
        obj[key] = old_value


def install_pytest_shim() -> None:
  pytest = types.ModuleType("pytest")
  pytest.approx = lambda expected, rel=1e-6, abs=1e-12, **_kwargs: Approx(expected, rel=rel, abs=abs)
  pytest.raises = lambda exc_type, *args, **_kwargs: Raises(exc_type) if not args else None
  pytest.mark = Mark()
  pytest.fixture = lambda *args, **kwargs: (args[0] if args and callable(args[0]) else (lambda fn: fn))
  pytest.param = lambda *values, **_kwargs: values[0] if len(values) == 1 else values
  sys.modules["pytest"] = pytest


def import_module(name: str):
  try:
    return importlib.import_module(name)
  except ModuleNotFoundError as exc:
    if exc.name and exc.name.startswith("openpilot"):
      install_openpilot_namespace()
      return importlib.import_module(name)
    raise


MODEL_VERSIONS = ("v11", "v12", "v13", "v14", "v15")


def call_variants(fn) -> list[dict[str, object]]:
  signature = inspect.signature(fn)
  variants: list[dict[str, object]] = [{}]
  for name, param in signature.parameters.items():
    if param.default is not inspect.Parameter.empty or param.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD):
      continue
    if name == "monkeypatch":
      for variant in variants:
        variant[name] = MonkeyPatch()
    elif name == "model_version":
      variants = [dict(variant, model_version=model_version) for variant in variants for model_version in MODEL_VERSIONS]
    else:
      return []
  return variants


def call_test(fn):
  variants = call_variants(fn)
  if not variants:
    return 0, []
  failures: list[str] = []
  passed = 0
  for kwargs in variants:
    monkeypatch = kwargs.get("monkeypatch")
    try:
      fn(**kwargs)
      passed += 1
    except Exception as exc:
      suffix = f"[{kwargs}]" if kwargs else ""
      failures.append(f"{suffix}: {type(exc).__name__}: {exc}")
    finally:
      if isinstance(monkeypatch, MonkeyPatch):
        monkeypatch.undo()
  return passed, failures


def run_selected(module_name: str, name_filter) -> tuple[int, list[str]]:
  module = import_module(module_name)
  passed = 0
  failures: list[str] = []

  for name, fn in sorted(vars(module).items()):
    if not name.startswith("test_") or not callable(fn) or not name_filter(name):
      continue
    count, call_failures = call_test(fn)
    passed += count
    failures.extend(f"{module_name}.{name}{failure}" for failure in call_failures)

  for class_name, cls in sorted(vars(module).items()):
    if not inspect.isclass(cls) or not class_name.startswith("Test"):
      continue
    instance = cls()
    for name, fn in sorted(vars(cls).items()):
      if not name.startswith("test_") or not callable(fn) or not name_filter(name):
        continue
      bound = getattr(instance, name)
      count, call_failures = call_test(bound)
      passed += count
      failures.extend(f"{module_name}.{class_name}.{name}{failure}" for failure in call_failures)

  return passed, failures


def run_named(module_name: str, names: list[str]) -> tuple[int, list[str]]:
  module = import_module(module_name)
  passed = 0
  failures: list[str] = []
  for name in names:
    count, call_failures = call_test(getattr(module, name))
    passed += count
    failures.extend(f"{module_name}.{name}{failure}" for failure in call_failures)
  return passed, failures


def main() -> None:
  install_pytest_shim()
  modules = {
    "selfdrive.controls.tests.test_carnival_radar_confirmation": lambda _name: True,
    "selfdrive.controls.tests.test_lead_behavior": lambda _name: True,
    "selfdrive.controls.tests.test_lead_follow_policy": lambda _name: True,
    "selfdrive.controls.tests.test_starpilot_vcruise": (
      lambda name: any(token in name for token in ("force_stop", "red_light", "carnival", "lead"))
    ),
    "selfdrive.controls.tests.test_conditional_experimental_mode": (
      lambda name: any(token in name for token in ("stop_light", "stopped_lead", "slow_lead", "open_road", "red_light"))
    ),
    "selfdrive.controls.tests.test_conditional_chill_mode": (
      lambda name: any(token in name for token in ("lead", "chill", "launch", "red_light"))
    ),
    "selfdrive.controls.tests.test_longcontrol": (
      lambda name: any(token in name for token in ("stop", "lead", "brake", "state", "stopping", "accel"))
    ),
  }
  named = {
    "selfdrive.controls.tests.test_longitudinal_planner": [
      "test_carnival_lone_high_speed_red_light_guard_requires_no_other_stop_evidence",
      "test_carnival_lone_high_speed_red_light_guard_latches_until_evidence_or_clear",
      "test_carnival_lone_high_speed_red_light_latch_ignores_forcing_stop_only_after_latched",
      "test_carnival_radar_confirmed_stop_hold_latches_confirmation_track",
      "test_carnival_radar_confirmed_stop_hold_rejects_false_contexts",
      "test_carnival_radar_confirmed_stop_hold_clears_on_departure",
      "test_carnival_radar_confirmed_stop_hold_releases_for_confirmed_departure_without_gas",
      "test_green_light_model_launch_boosts_no_lead_experimental_takeoff",
      "test_green_light_model_launch_survives_cem_switch_back_to_chill",
      "test_model_launch_does_not_override_stationary_lead_guard",
      "test_model_launch_boosts_only_after_lead_departure_is_confirmed",
      "test_model_launch_is_cancelled_when_departing_lead_stops_again",
    ],
  }

  total = 0
  failures: list[str] = []
  for module_name, name_filter in modules.items():
    passed, module_failures = run_selected(module_name, name_filter)
    total += passed
    failures.extend(module_failures)
    print(f"{module_name}: pass={passed} fail={len(module_failures)}")

  for module_name, names in named.items():
    passed, module_failures = run_named(module_name, names)
    total += passed
    failures.extend(module_failures)
    print(f"{module_name}: pass={passed} fail={len(module_failures)}")

  if failures:
    print("FAILURES:")
    for failure in failures:
      print(f"  {failure}")
    raise SystemExit(1)

  print(f"PASS longitudinal_unit_sweep total={total}")


if __name__ == "__main__":
  main()
