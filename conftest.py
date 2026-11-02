"""Repo-root pytest hook: make `agv_core` and the tests/ helpers importable.

The equivalent `pythonpath` ini option needs pytest >= 7; the vehicle image
(Ubuntu 22.04, ROS 2 Humble) ships 6.2, which only warned about it and left
the paths to whatever shell the suite was run from.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
for _p in (_ROOT, os.path.join(_ROOT, "tests")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
