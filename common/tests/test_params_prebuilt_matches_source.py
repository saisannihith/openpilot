import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PARAM_KEYS = ROOT / "common" / "params_keys.h"
PARAMS_PREBUILT = ROOT / "common" / "params_pyx.so"


def test_tracked_params_prebuilt_contains_every_registered_key():
  source = PARAM_KEYS.read_text(encoding="utf-8")
  registered_keys = re.findall(r'^\s*\{"([^"]+)"', source, flags=re.MULTILINE)
  prebuilt = PARAMS_PREBUILT.read_bytes()
  missing = [key for key in registered_keys if key.encode() not in prebuilt]
  assert not missing, f"common/params_pyx.so is stale; missing registered keys: {missing}"
