from pathlib import Path
import sys

PYTHON_REFERENCE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PYTHON_REFERENCE_ROOT.parent

for path in (PROJECT_ROOT, PYTHON_REFERENCE_ROOT):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

__all__ = ["PROJECT_ROOT", "PYTHON_REFERENCE_ROOT"]
