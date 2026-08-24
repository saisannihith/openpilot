#!/usr/bin/env python3
from __future__ import annotations

import sys
import types
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
sys.path.insert(0, str(ROOT))

namespace = types.ModuleType("openpilot")
namespace.__path__ = [str(ROOT)]  # type: ignore[attr-defined]
sys.modules["openpilot"] = namespace

import pytest  # noqa: E402


def main() -> int:
  args = [
    "-vv",
    "-s",
    "-o", "addopts=",
    str(ROOT / "selfdrive" / "controls" / "tests" / "test_longitudinal_planner.py"),
    "-k", "carnival or red_light or stopped_lead or radar",
    "--rootdir", str(ROOT),
  ]
  print(f"running pytest from {ROOT}", flush=True)
  print(f"args={args}", flush=True)
  result = pytest.main(args)
  print(f"pytest result={result}", flush=True)
  return result


if __name__ == "__main__":
  raise SystemExit(main())
