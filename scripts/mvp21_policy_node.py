#!/usr/bin/env python3
"""MVP2.1 entry; policy_node.py is a compatibility name for this same backend."""
from pathlib import Path
import sys

# In the source tree the package is next to scripts/. A CMake installation
# places it next to this executable, already on Python's script search path.
root = Path(__file__).resolve().parents[1]
if (root/'mvp21_policy').is_dir():
    sys.path.insert(0, str(root))

from mvp21_policy.__main__ import main


if __name__ == '__main__':
    main()
