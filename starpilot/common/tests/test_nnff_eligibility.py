from openpilot.starpilot.common.nnff_eligibility import (
  CARNIVAL_4TH_GEN,
  enforce_nnff_driving_model_eligibility,
  nnff_driving_model_allowed,
  nnff_driving_model_allowed_for_params,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return self.values.get(key) in (True, 1, "1", b"1")

  def put_bool(self, key, value):
    self.values[key] = bool(value)


def test_carnival_cd210_v11_is_eligible():
  params = FakeParams({"Model": b"cd21023", "ModelVersion": b"v11", "NNFF": True})

  assert nnff_driving_model_allowed_for_params(params, CARNIVAL_4TH_GEN)
  assert enforce_nnff_driving_model_eligibility(params, CARNIVAL_4TH_GEN)
  assert params.get_bool("NNFF")


def test_carnival_other_model_disables_full_and_lite_nnff():
  params = FakeParams({"DrivingModel": "rdf43", "DrivingModelVersion": "v15", "NNFF": True, "NNFFLite": True})

  assert not enforce_nnff_driving_model_eligibility(params, CARNIVAL_4TH_GEN)
  assert not params.get_bool("NNFF")
  assert not params.get_bool("NNFFLite")


def test_carnival_unknown_or_stale_version_fails_closed():
  assert not nnff_driving_model_allowed(CARNIVAL_4TH_GEN, "cd21023", "")
  assert not nnff_driving_model_allowed(CARNIVAL_4TH_GEN, "cd21023", "v15")


def test_other_vehicles_keep_existing_nnff_behavior():
  params = FakeParams({"Model": "rdf43", "ModelVersion": "v15", "NNFF": True, "NNFFLite": True})

  assert enforce_nnff_driving_model_eligibility(params, "HYUNDAI_IONIQ_5")
  assert params.get_bool("NNFF")
  assert params.get_bool("NNFFLite")
