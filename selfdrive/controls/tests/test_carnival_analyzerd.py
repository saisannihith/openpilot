import json

from openpilot.selfdrive.controls.carnival_analyzerd import apply_pending_profile, discover_routes, handle_profile_requests, prune_reports


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key, block=False, return_default=False, encoding=None, default=None):
    value = self.values.get(key, default)
    if encoding and isinstance(value, bytes):
      return value.decode(encoding)
    return value

  def get_bool(self, key, block=False, default=False):
    value = self.values.get(key, default)
    return value in (True, "1", b"1")

  def put(self, key, value):
    self.values[key] = str(value)

  def put_bool(self, key, value):
    self.values[key] = "1" if value else "0"


def test_discover_routes_groups_segments_and_orders_by_mtime(tmp_path):
  first = tmp_path / "dongle|2026-08-24--aaaa--0"
  second = tmp_path / "dongle|2026-08-24--bbbb--0"
  first.mkdir()
  second.mkdir()
  (first / "qlog").write_bytes(b"one")
  (second / "qlog").write_bytes(b"two")
  routes = discover_routes(tmp_path)
  assert [route for route, _, _ in routes] == ["dongle|2026-08-24--aaaa", "dongle|2026-08-24--bbbb"]


def test_apply_and_revert_pending_profile():
  profile = {
    "route": "route",
    "resolved": {"StandardFollow": {"before": 1.45, "after": 1.5}},
  }
  params = FakeParams({"CarnivalPendingProfile": json.dumps(profile)})
  assert apply_pending_profile(params)
  assert params.values["StandardFollow"] == "1.5"
  params.put_bool("CarnivalRevertProfile", True)
  handle_profile_requests(params)
  assert params.values["StandardFollow"] == "1.45"
  assert params.values["CarnivalProfileSnapshot"] == "{}"


def test_report_retention_removes_old_pairs(tmp_path):
  for index in range(3):
    for suffix in ("json", "md"):
      (tmp_path / f"carnival-report-20260824-00000{index}.{suffix}").write_text("report")
  prune_reports(tmp_path, keep=2)
  assert len(list(tmp_path.glob("*.json"))) == 2
  assert len(list(tmp_path.glob("*.md"))) == 2
