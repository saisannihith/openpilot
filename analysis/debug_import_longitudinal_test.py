import sys, types, importlib.util
from pathlib import Path

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root))
namespace = types.ModuleType("openpilot")
namespace.__path__ = [str(root)]
sys.modules["openpilot"] = namespace
print('before import', flush=True)
path = root / 'selfdrive' / 'controls' / 'tests' / 'test_longitudinal_planner.py'
spec = importlib.util.spec_from_file_location('test_longitudinal_planner_direct', path)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)
print('after import', flush=True)
print([n for n in dir(mod) if 'carnival' in n.lower()][:20], flush=True)
