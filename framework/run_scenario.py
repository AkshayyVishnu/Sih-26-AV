#!/usr/bin/env python3
"""
CLI entry point for the framework/ scenario+autopilot runner.

Usage:
    python framework/run_scenario.py pedestrian_jumpout
    python framework/run_scenario.py traffic_stress --autopilot pipeline
    python framework/run_scenario.py pedestrian_jumpout --no-viz

See framework/README.md for a quick start and
framework/DESIGN_GUIDELINES.md for how to register a new scenario or
autopilot here.
"""
from __future__ import annotations

import argparse
import os
import sys

# When this file is run directly (`python framework/run_scenario.py`),
# Python puts only this file's own directory (framework/) on sys.path --
# NOT the repo root -- so `import framework` itself would fail before
# framework/__init__.py's own bootstrap ever got a chance to run. Fix the
# path here first, directly, the same way Simulation Files/scenario_1.py/
# scenario_2.py already do it for themselves.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from framework.autopilots.pipeline_autopilot import PipelineAutopilot
from framework.base import ScenarioRunner
from framework.scenarios.pedestrian_jumpout import PedestrianJumpOut
from framework.scenarios.traffic_stress import TrafficStress

SCENARIOS = {
    "pedestrian_jumpout": PedestrianJumpOut,
    "traffic_stress": TrafficStress,
}

AUTOPILOTS = {
    "pipeline": PipelineAutopilot,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", choices=sorted(SCENARIOS.keys()))
    parser.add_argument("--autopilot", default="pipeline", choices=sorted(AUTOPILOTS.keys()))
    parser.add_argument("--no-viz", action="store_true", help="disable the pygame debug dashboard")
    args = parser.parse_args()

    scenario = SCENARIOS[args.scenario]()
    autopilot = AUTOPILOTS[args.autopilot]()
    ScenarioRunner(scenario, autopilot, enable_visualization=not args.no_viz).run()


if __name__ == "__main__":
    main()
