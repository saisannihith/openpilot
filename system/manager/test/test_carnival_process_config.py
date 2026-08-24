from cereal import car

from openpilot.system.manager.process_config import carnival_offroad, carnival_only, is_carnival_4th_gen


class _Params:
  def __init__(self, persistent_cp: bytes | None):
    self.persistent_cp = persistent_cp

  def get(self, key: str):
    return self.persistent_cp if key == "CarParamsPersistent" else None


def _cp(fingerprint: str) -> car.CarParams:
  cp = car.CarParams.new_message()
  cp.carFingerprint = fingerprint
  return cp


def test_carnival_process_gate_uses_live_fingerprint_onroad():
  cp = _cp("KIA_CARNIVAL_4TH_GEN")
  assert is_carnival_4th_gen(_Params(None), cp)
  assert carnival_only(True, _Params(None), cp, None)
  assert not carnival_offroad(True, _Params(None), cp, None)


def test_carnival_process_gate_uses_persisted_fingerprint_offroad():
  persisted = _cp("KIA_CARNIVAL_4TH_GEN").to_bytes()
  blank_cp = _cp("")
  assert is_carnival_4th_gen(_Params(persisted), blank_cp)
  assert carnival_offroad(False, _Params(persisted), blank_cp, None)
  assert not carnival_only(False, _Params(persisted), blank_cp, None)


def test_carnival_process_gate_rejects_other_cars():
  persisted = _cp("HYUNDAI_SONATA").to_bytes()
  assert not is_carnival_4th_gen(_Params(persisted), _cp(""))
