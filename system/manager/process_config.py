import operator
import os
import platform
import re
import sys
import time
from datetime import UTC, datetime
from types import SimpleNamespace

from cereal import car, messaging
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE, PC, TICI
from openpilot.system.manager.process import DaemonProcess, NativeProcess, PythonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None
UI_WATCHDOG_MAX_DT = int(os.getenv("UI_WATCHDOG_MAX_DT", "10"))
CAMERAD_WATCHDOG_MAX_DT = int(os.getenv("CAMERAD_WATCHDOG_MAX_DT", "5"))
CARNIVAL_LOG_ROOT = "/data/media/0/realdata"
CARNIVAL_LOG_NAMES = ("qlog", "qlog.zst", "qlog.bz2", "rlog", "rlog.zst", "rlog.bz2")
CARNIVAL_ROUTE_SETTLE_SECONDS = 45.0
CARNIVAL_ROUTE_SCAN_INTERVAL = 10.0
_carnival_next_route_scan = 0.0
_carnival_last_requested_route = ""

def driverview(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode") and not params.get_bool("LateralManeuverMode")

def lat_maneuver(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and params.get_bool("LateralManeuverMode") and not params.get_bool("LongitudinalManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def qcomgps(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and not ublox_available()

def always_run(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started

def is_carnival_4th_gen(params: Params, CP: car.CarParams) -> bool:
  fingerprint = str(CP.carFingerprint)
  if not fingerprint:
    try:
      persisted = params.get("CarParamsPersistent")
      if persisted:
        fingerprint = str(messaging.log_from_bytes(persisted, car.CarParams).carFingerprint)
    except Exception:
      fingerprint = ""
  return fingerprint == "KIA_CARNIVAL_4TH_GEN"


def carnival_only(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and is_carnival_4th_gen(params, CP)

def newest_carnival_route(root: str = CARNIVAL_LOG_ROOT) -> tuple[str, float] | None:
  newest_by_route: dict[str, float] = {}
  try:
    segments = os.scandir(root)
  except OSError:
    return None
  with segments:
    for segment in segments:
      if not segment.is_dir():
        continue
      match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment.name)
      route = match.group(1) if match else segment.name
      for name in CARNIVAL_LOG_NAMES:
        path = os.path.join(segment.path, name)
        try:
          modified = os.path.getmtime(path)
        except OSError:
          continue
        newest_by_route[route] = max(newest_by_route.get(route, 0.0), modified)
        break
  return max(newest_by_route.items(), key=lambda item: item[1]) if newest_by_route else None


def carnival_new_route_ready(params: Params, root: str, now: float) -> str:
  latest = newest_carnival_route(root)
  if latest is None:
    return ""
  route, modified = latest
  completed = params.get("CarnivalLastAnalysisRoute") or b""
  if isinstance(completed, bytes):
    completed = completed.decode("utf-8", errors="replace")
  return route if route != completed and now - modified >= CARNIVAL_ROUTE_SETTLE_SECONDS else ""


def carnival_auto_analysis_requested(params: Params) -> bool:
  global _carnival_last_requested_route, _carnival_next_route_scan
  if not params.get_bool("CarnivalAutoAnalyze"):
    return False
  now_monotonic = time.monotonic()
  if now_monotonic < _carnival_next_route_scan:
    return False
  _carnival_next_route_scan = now_monotonic + CARNIVAL_ROUTE_SCAN_INTERVAL
  route = carnival_new_route_ready(params, CARNIVAL_LOG_ROOT, datetime.now(UTC).timestamp())
  if route and route != _carnival_last_requested_route:
    _carnival_last_requested_route = route
    params.put_bool("CarnivalAnalyzeNow", True)
    return True
  return False

def carnival_offroad(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  if started or not is_carnival_4th_gen(params, CP):
    return False
  requested = any(params.get_bool(key) for key in (
    "CarnivalAnalyzeNow", "CarnivalAnalysisRunning",
    "CarnivalApplyProfile", "CarnivalRevertProfile",
  ))
  return requested or carnival_auto_analysis_requested(params)

def only_offroad(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return not started

def sentry_mode(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return not started and params.get_bool("SentryModeEnabled")

def sensord_run(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started or params.get_bool("SentryModeEnabled")

def camera_run(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return driverview(started, params, CP, starpilot_toggles) or (not started and params.get_bool("SentryModeCapture"))

def livestream(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return params.get_bool("IsLiveStreaming")

def or_(*fns):
  return lambda *args: operator.or_(*(fn(*args) for fn in fns))

def and_(*fns):
  return lambda *args: operator.and_(*(fn(*args) for fn in fns))

def not_(*fns):
  return lambda *args: operator.not_(*(fn(*args) for fn in fns))

# StarPilot variables
def allow_logging(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return not starpilot_toggles.no_logging

def allow_uploads(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return (params.get_bool("AlwaysAllowUploads") or not starpilot_toggles.no_uploads or
          (starpilot_toggles.no_onroad_uploads and not started))

def run_speed_limit_filler(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return starpilot_toggles.speed_limit_filler

def run_speed_limit_vision(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return starpilot_toggles.vision_speed_limit_detection

def run_navigationd(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and params.get("NavDestination") is not None


def run_v_asm(started: bool, params: Params, CP: car.CarParams, starpilot_toggles: SimpleNamespace) -> bool:
  return started and getattr(starpilot_toggles, "v_asm_enabled", False)


def big_device_ui_process() -> NativeProcess:
  return NativeProcess(
    "ui",
    ".",
    ["/usr/bin/env", "BIG=1", sys.executable, "-m", "openpilot.selfdrive.ui.ui"],
    always_run,
    watchdog_max_dt=UI_WATCHDOG_MAX_DT,
  )


procs = [
  DaemonProcess("manage_athenad", "system.athena.manage_athenad", "AthenadPid"),

  NativeProcess("loggerd", "system/loggerd", ["./loggerd"], and_(allow_logging, logging)),
  NativeProcess("encoderd", "system/loggerd", ["./encoderd"], and_(allow_logging, only_onroad)),
  NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], or_(and_(livestream, not_(iscar)), notcar)),
  PythonProcess("logmessaged", "system.logmessaged", always_run),

  NativeProcess("camerad", "system/camerad", ["./camerad"], or_(camera_run, livestream), enabled=not WEBCAM,
                watchdog_max_dt=CAMERAD_WATCHDOG_MAX_DT),
  PythonProcess("webcamerad", "tools.webcam.camerad", driverview, enabled=WEBCAM),
  PythonProcess("proclogd", "system.proclogd", and_(allow_logging, only_onroad), enabled=platform.system() != "Darwin"),
  PythonProcess("journald", "system.journald", and_(allow_logging, only_onroad), platform.system() != "Darwin"),
  PythonProcess("micd", "system.micd", iscar),
  PythonProcess("timed", "system.timed", always_run, enabled=not PC),

  PythonProcess("modeld", "selfdrive.modeld.modeld", only_onroad),
  PythonProcess("dmonitoringmodeld", "selfdrive.modeld.dmonitoringmodeld", driverview, enabled=(WEBCAM or not PC)),

  PythonProcess("sensord", "system.sensord.sensord", sensord_run, enabled=not PC),
  PythonProcess("sentryd", "system.sentryd.sentryd", sentry_mode, enabled=not PC),
  PythonProcess("soundd", "selfdrive.ui.soundd", driverview),
  PythonProcess("locationd", "selfdrive.locationd.locationd", only_onroad),
  NativeProcess("_pandad", "selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  PythonProcess("joystickd", "tools.joystick.joystickd", or_(joystick, notcar)),
  PythonProcess("selfdrived", "selfdrive.selfdrived.selfdrived", only_onroad),
  PythonProcess("card", "selfdrive.car.card", only_onroad),
  PythonProcess("deleter", "system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", driverview, enabled=(WEBCAM or not PC)),
  PythonProcess("qcomgpsd", "system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  PythonProcess("pandad", "selfdrive.pandad.pandad", always_run),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", only_onroad),
  PythonProcess("lagd", "selfdrive.locationd.lagd", only_onroad),
  PythonProcess("ubloxd", "system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "selfdrive.controls.plannerd", not_long_maneuver),
  PythonProcess("maneuversd", "tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  PythonProcess("lateral_maneuversd", "tools.lateral_maneuvers.lateral_maneuversd", lat_maneuver),
  PythonProcess("radard", "selfdrive.controls.radard", only_onroad),
  PythonProcess("carnivald", "selfdrive.controls.carnivald", carnival_only),
  PythonProcess("carnival_analyzerd", "selfdrive.controls.carnival_analyzerd", carnival_offroad, nice=19),
  PythonProcess("hardwared", "system.hardware.hardwared", always_run),
  PythonProcess("tombstoned", "system.tombstoned", always_run, enabled=not PC),
  PythonProcess("updated", "system.updated.updated", always_run, enabled=not PC),
  PythonProcess("uploader", "system.loggerd.uploader", allow_uploads, nice=19),
  PythonProcess("statsd", "system.statsd", always_run),
  PythonProcess("feedbackd", "selfdrive.ui.feedback.feedbackd", only_onroad),

  # debug procs
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "system.webrtc.webrtcd", or_(and_(livestream, not_(iscar)), notcar)),
  PythonProcess("webjoystick", "tools.bodyteleop.web", notcar),
  PythonProcess("joystick", "tools.joystick.joystick_control", and_(joystick, iscar)),
]

# StarPilot variables
procs += [
  PythonProcess("the_galaxy", "starpilot.system.the_galaxy.the_galaxy", always_run, nice=10),
  PythonProcess("galaxy", "starpilot.system.galaxy.galaxy", always_run, nice=10),
]

device_type = HARDWARE.get_device_type()
if device_type in ("tici", "tizi"):
  procs.append(big_device_ui_process())
else:
  procs.append(PythonProcess("ui", "selfdrive.ui.ui", always_run, watchdog_max_dt=UI_WATCHDOG_MAX_DT))

procs += [
  PythonProcess("device_syncd", "starpilot.system.device_syncd", always_run),
  PythonProcess("starpilot_process", "starpilot.starpilot_process", always_run),
  PythonProcess("mapd", "starpilot.navigation.mapd_wrapper", always_run, nice=19),
  PythonProcess("navigationd", "starpilot.navigation.navigationd", run_navigationd, nice=19),
  PythonProcess("speed_limit_filler", "starpilot.system.speed_limit_filler", run_speed_limit_filler, nice=19),
  PythonProcess("speed_limit_vision", "starpilot.system.speed_limit_vision", run_speed_limit_vision, nice=19),
  PythonProcess("adj_spot_monitor_vision", "starpilot.system.adj_spot_monitor_vision", run_v_asm, nice=19),
]

managed_processes = {p.name: p for p in procs}
