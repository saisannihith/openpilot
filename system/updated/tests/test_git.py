import contextlib
import pytest

from openpilot.system.updated.tests.test_base import ParamsBaseUpdateTest, run, update_release
from openpilot.system.updated import updated as updated_module
from openpilot.system.updated.updated import Updater, pinned_update_branch_for_origin


@pytest.mark.parametrize("origin", [
  "https://github.com/saisannihith/openpilot.git",
  "http://github.com/saisannihith/openpilot",
  "git@github.com:saisannihith/openpilot.git",
  "ssh://git@github.com/saisannihith/openpilot.git",
  "HTTPS://GITHUB.COM/SAISANNIHITH/OPENPILOT.GIT",
])
def test_snithpilot_origin_is_pinned(origin):
  assert pinned_update_branch_for_origin(origin) == "snithpilot"


@pytest.mark.parametrize("origin", [
  "https://github.com/firestar5683/StarPilot.git",
  "https://github.com/saisannihith/snithpilot.git",
  "https://github.com/another-user/openpilot.git",
  "",
])
def test_other_origins_are_not_pinned(origin):
  assert pinned_update_branch_for_origin(origin) is None


def test_snithpilot_updater_replaces_stale_starpilot_branch_params(monkeypatch):
  values = {
    "UpdaterTargetBranch": "StarPilot",
    "UpdaterAvailableBranches": "StarPilot,Dom,snithpilot",
  }

  class FakeParams:
    def get(self, key):
      return values.get(key)

    def put(self, key, value):
      values[key] = value

  monkeypatch.setattr(updated_module, "Params", FakeParams)
  monkeypatch.setattr(updated_module, "get_pinned_update_branch", lambda path: "snithpilot")

  updater = Updater()

  assert updater.target_branch == "snithpilot"
  assert values["UpdaterTargetBranch"] == "snithpilot"
  assert values["UpdaterAvailableBranches"] == "snithpilot"


class TestUpdateDGitStrategy(ParamsBaseUpdateTest):
  def update_remote_release(self, release):
    update_release(self.remote_dir, release, *self.MOCK_RELEASES[release])
    run(["git", "add", "."], cwd=self.remote_dir)
    run(["git", "commit", "-m", f"openpilot release {release}"], cwd=self.remote_dir)

  def setup_remote_release(self, release):
    run(["git", "init"], cwd=self.remote_dir)
    run(["git", "checkout", "-b", release], cwd=self.remote_dir)
    self.update_remote_release(release)

  def setup_basedir_release(self, release):
    super().setup_basedir_release(release)
    run(["git", "clone", "-b", release, self.remote_dir, self.basedir])

  @contextlib.contextmanager
  def additional_context(self):
    yield
