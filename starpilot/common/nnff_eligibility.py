"""Fail-closed driving-model eligibility for vehicle-specific NNFF controllers."""

from __future__ import annotations


CARNIVAL_4TH_GEN = "KIA_CARNIVAL_4TH_GEN"

# A model is added here only after its planner outputs have been audited against
# the matching vehicle-specific NNFF data. This is an eligibility boundary, not
# proof of closed-loop or actuation safety.
NNFF_DRIVING_MODEL_ALLOWLIST = {
  CARNIVAL_4TH_GEN: frozenset({("cd21023", "v11")}),
}


def _text(value) -> str:
  value = getattr(value, "value", value)
  if isinstance(value, bytes):
    return value.decode("utf-8", "ignore").strip()
  return str(value or "").strip()


def selected_driving_model(params) -> tuple[str, str]:
  model_id = _text(params.get("Model")) or _text(params.get("DrivingModel"))
  model_version = _text(params.get("ModelVersion")) or _text(params.get("DrivingModelVersion"))
  return model_id.lower(), model_version.lower()


def nnff_driving_model_allowed(car_fingerprint, model_id: str, model_version: str) -> bool:
  allowed = NNFF_DRIVING_MODEL_ALLOWLIST.get(_text(car_fingerprint))
  if allowed is None:
    return True
  return (_text(model_id).lower(), _text(model_version).lower()) in allowed


def nnff_driving_model_allowed_for_params(params, car_fingerprint) -> bool:
  return nnff_driving_model_allowed(car_fingerprint, *selected_driving_model(params))


def enforce_nnff_driving_model_eligibility(params, car_fingerprint) -> bool:
  eligible = nnff_driving_model_allowed_for_params(params, car_fingerprint)
  if not eligible:
    if params.get_bool("NNFF"):
      params.put_bool("NNFF", False)
    if params.get_bool("NNFFLite"):
      params.put_bool("NNFFLite", False)
  return eligible
