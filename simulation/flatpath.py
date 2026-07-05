"""Compatibility shim: put simulation/ and all its role subpackages on
sys.path so the historical flat imports (`from nn_tire_model import ...`) keep
working after the framework/implementation reorganization. Prefer the package
form in new code (`from simulation.tire_models.nn_tire_model import ...`).
Idempotent; importing it (directly or via `import simulation`) is enough.
"""
import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.abspath(__file__))
_REPO = _os.path.dirname(_ROOT)
# simulation/ + its role subpackages, plus the sibling package(s) that the
# runtime imports flat: nn_training/ holds the terrain-window model class the
# estimators load (`from train_terrain_window_mlp import ...`).
_dirs = [_ROOT] + [_os.path.join(_ROOT, x) for x in sorted(_os.listdir(_ROOT))
                   if _os.path.isdir(_os.path.join(_ROOT, x)) and not x.startswith(("_", "."))]
_dirs.append(_os.path.join(_REPO, "nn_training"))
for _d in _dirs:
    if _os.path.isdir(_d) and _d not in _sys.path:
        _sys.path.insert(0, _d)
