import os

from cereal import car
from openpilot.system.manager.process_config import carnival_new_route_ready, carnival_offroad, carnival_only, is_carnival_4th_gen


class _Params:
  def __init__(self, persistent_cp: bytes | None, enabled: set[str] | None = None, values: dict | None = None):
    self.persistent_cp = persistent_cp
    self.enabled = enabled or set()
    self.values = values or {}

  def get(self, key: str):
    return self.persistent_cp if key == "CarParamsPersistent" else self.values.get(key)

  def get_bool(self, key: str) -> bool:
    return key in self.enabled

  def put_bool(self, key: str, value: bool) -> None:
    if value:
      self.enabled.add(key)
    else:
      self.enabled.discard(key)


def _cp(fingerprint: str) -> car.CarParams:
  cp = car.CarParams.new_message()
  cp.carFingerprint = fingerprint
  return cp


def test_carnival_process_gate_uses_live_fingerprint_onroad():
  cp = _cp("KIA_CARNIVAL_4TH_GEN")
  assert is_carnival_4th_gen(_Params(None), cp)
  assert carnival_only(True, _Params(None), cp, None)
  assert not carnival_offroad(True, _Params(None, {"CarnivalAutoAnalyze"}), cp, None)


def test_carnival_process_gate_uses_persisted_fingerprint_offroad():
  persisted = _cp("KIA_CARNIVAL_4TH_GEN").to_bytes()
  blank_cp = _cp("")
  assert is_carnival_4th_gen(_Params(persisted), blank_cp)
  assert carnival_offroad(False, _Params(persisted, {"CarnivalAnalyzeNow"}), blank_cp, None)
  assert not carnival_only(False, _Params(persisted), blank_cp, None)


def test_carnival_analyzer_stays_stopped_without_a_request():
  persisted = _cp("KIA_CARNIVAL_4TH_GEN").to_bytes()
  assert not carnival_offroad(False, _Params(persisted), _cp(""), None)


def test_carnival_process_gate_rejects_other_cars():
  persisted = _cp("HYUNDAI_SONATA").to_bytes()
  assert not is_carnival_4th_gen(_Params(persisted), _cp(""))


def test_carnival_new_route_ready_uses_settled_latest_route(tmp_path):
  segment = tmp_path / "dongle|2026-08-24--abcd--0"
  segment.mkdir()
  qlog = segment / "qlog"
  qlog.write_bytes(b"compact")
  os.utime(qlog, (100.0, 100.0))
  params = _Params(None)
  assert carnival_new_route_ready(params, str(tmp_path), 200.0) == "dongle|2026-08-24--abcd"
  params.values["CarnivalLastAnalysisRoute"] = b"dongle|2026-08-24--abcd"
  assert carnival_new_route_ready(params, str(tmp_path), 200.0) == ""
