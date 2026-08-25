from pathlib import Path


LOGGERD_PREBUILT = Path(__file__).resolve().parents[1] / "loggerd"


def test_tracked_loggerd_contains_carnival_state_schema():
  assert b"carnivalState" in LOGGERD_PREBUILT.read_bytes(), (
    "system/loggerd/loggerd is stale; rebuild it after changing the logged cereal schema"
  )
