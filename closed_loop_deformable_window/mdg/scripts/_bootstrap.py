"""Make the repository root and local src tree importable without installation."""

from pathlib import Path
import sys

MDG_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = MDG_ROOT.parents[1]
for path in (MDG_ROOT / "src", REPOSITORY_ROOT):
    value = str(path)
    if value not in sys.path:
        sys.path.insert(0, value)

