"""
framework/ -- a scenario/autopilot runner built alongside the original
`Simulation Files/scenario_1.py`/`scenario_2.py` scripts, not in place of
them. Those two files are untouched; this package is a parallel,
independent way to run the same ideas with reusable, swappable pieces.

See framework/README.md for a quick start and
framework/DESIGN_GUIDELINES.md for how to add a new Scenario or Autopilot.

This __init__ does one thing: makes the repo root importable (so
`carla_runtime` and `pipeline` resolve regardless of the working
directory or where a framework/ module is imported from), the same
sys.path trick Simulation Files/scenario_1.py/scenario_2.py already use
-- done once here instead of repeated in every submodule.
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
