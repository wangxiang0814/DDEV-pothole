"""Compatibility entry point; implementation lives in the wheel-lift package."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / 'src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.strategies.wheel_lift import front_cli as _implementation

if __name__ == '__main__':
    _implementation.main()
else:
    # Preserve legacy diagnostic scripts that temporarily replace CLI collaborators.
    sys.modules[__name__] = _implementation
