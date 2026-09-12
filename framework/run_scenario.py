#!/usr/bin/env python3
"""
CLI entry point for the framework/ scenario+autopilot runner.

Usage:
    python framework/run_scenario.py pedestrian_jumpout
    python framework/run_scenario.py traffic_stress --autopilot pipeline
    python framework/run_scenario.py pedestrian_jumpout --no-viz
    python framework/run_scenario.py chaotic_traffic --autopilot pcla_tfv6
    python framework/run_scenario.py chaotic_traffic --autopilot own_perception_plant2

The 5 PS-required validation scenarios (framework/ps_scenarios/, kept
separate from the ad-hoc ones above -- see that package's own docstring).
Coordinate status per scenario, read before running:
    python framework/run_scenario.py unmarked_village_road        -- EGO_SPAWN/FINAL_GOAL/obstruction still
                                                                       placeholder (Town07, never captured live)
    python framework/run_scenario.py urban_intersection_no_signals -- real, reuses traffic_stress.py's own
                                                                       already-live-verified Town03 coordinates
    python framework/run_scenario.py highway_merge_slow_traffic    -- coordinate-free: derives everything from
                                                                       world.get_map() at runtime, no capture needed
    python framework/run_scenario.py dense_market_mixed_traffic    -- EGO_SPAWN/FINAL_GOAL still placeholder
                                                                       (Town10HD, never captured live)
    python framework/run_scenario.py cattle_crossing               -- EGO_SPAWN/FINAL_GOAL/hiding spot still
                                                                       placeholder (Town07, never captured live)

See framework/README.md for a quick start and
framework/DESIGN_GUIDELINES.md for how to register a new scenario or
autopilot here (including PCLA setup, needed for the last two
autopilots above).
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

from framework.base import ScenarioRunner
from framework.ps_scenarios.cattle_crossing import CattleCrossing
from framework.ps_scenarios.dense_market_mixed_traffic import DenseMarketMixedTraffic
from framework.ps_scenarios.highway_merge_slow_traffic import HighwayMergeSlowTraffic
from framework.ps_scenarios.unmarked_village_road import UnmarkedVillageRoad
from framework.ps_scenarios.urban_intersection_no_signals import UrbanIntersectionNoSignals
from framework.scenarios.chaotic_traffic import ChaoticTraffic
from framework.scenarios.pedestrian_jumpout import PedestrianJumpOut
from framework.scenarios.traffic_stress import TrafficStress

SCENARIOS = {
    "pedestrian_jumpout": PedestrianJumpOut,
    "traffic_stress": TrafficStress,
    "chaotic_traffic": ChaoticTraffic,
    # The 5 PS-required validation scenarios -- see framework/ps_scenarios/'s own docstring.
    "unmarked_village_road": UnmarkedVillageRoad,
    "urban_intersection_no_signals": UrbanIntersectionNoSignals,
    "highway_merge_slow_traffic": HighwayMergeSlowTraffic,
    "dense_market_mixed_traffic": DenseMarketMixedTraffic,
    "cattle_crossing": CattleCrossing,
}

# Autopilots are registered as (import_path, class_name) pairs and
# imported LAZILY (only the one actually selected on the command line) --
# NOT as already-imported classes. "pcla_tfv6" / "own_perception_plant2"
# both pull in external/PCLA, which needs its own conda env (py-trees,
# specific torch/timm pins -- see framework/DESIGN_GUIDELINES.md's PCLA
# section) that may not be set up in whatever environment is running
# this CLI. Eagerly importing all three here would mean `--autopilot
# pipeline` (which needs none of that) breaks too, just from PCLA not
# being installed -- lazy import keeps the two independent.
_AUTOPILOT_SOURCES = {
    "pipeline": ("framework.autopilots.pipeline_autopilot", "PipelineAutopilot"),
    "pcla_tfv6": ("framework.autopilots.pcla_transfuser_autopilot", "Transfuserv6Autopilot"),
    "own_perception_plant2": ("framework.autopilots.own_perception_plant2_autopilot", "OwnPerceptionPlanT2Autopilot"),
}


def _load_autopilot_class(name: str):
    import importlib
    module_path, class_name = _AUTOPILOT_SOURCES[name]
    try:
        module = importlib.import_module(module_path)
    except ImportError as e:
        raise ImportError(
            f"Could not import the '{name}' autopilot ({module_path}): {e}\n"
            f"If this is 'pcla_tfv6' or 'own_perception_plant2', this almost always means "
            f"external/PCLA isn't cloned/set up yet in the environment running this CLI -- "
            f"see framework/DESIGN_GUIDELINES.md's PCLA section."
        ) from e
    return getattr(module, class_name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", choices=sorted(SCENARIOS.keys()))
    parser.add_argument("--autopilot", default="pipeline", choices=sorted(_AUTOPILOT_SOURCES.keys()))
    parser.add_argument("--no-viz", action="store_true", help="disable the pygame debug dashboard")
    args = parser.parse_args()

    scenario = SCENARIOS[args.scenario]()
    autopilot_cls = _load_autopilot_class(args.autopilot)
    autopilot = autopilot_cls()
    ScenarioRunner(scenario, autopilot, enable_visualization=not args.no_viz).run()


if __name__ == "__main__":
    main()
