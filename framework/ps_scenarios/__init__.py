"""
framework/ps_scenarios/ -- the 5 scenarios the PS explicitly names for
validation (unmarked village road, busy urban intersection without
signals, highway merge with slow-moving vehicles, dense market with
mixed traffic, sudden cattle-crossing), kept separate from
framework/scenarios/'s ad-hoc dev/test scenarios (pedestrian_jumpout,
traffic_stress, chaotic_traffic) on purpose -- these are the "official"
PS-required scenarios, not general-purpose test scenes.

No separate sys.path bootstrap needed here -- framework/__init__.py
already makes the repo root importable for anything under framework/.

Run any of these exactly like the existing scenarios, same CLI, no
separate runner:
    python framework/run_scenario.py unmarked_village_road
    python framework/run_scenario.py urban_intersection_no_signals
    python framework/run_scenario.py highway_merge_slow_traffic
    python framework/run_scenario.py dense_market_mixed_traffic
    python framework/run_scenario.py cattle_crossing

See each file's own module docstring for the town chosen and why, and
docs/pipeline-decision-log.md for the two shared prerequisite changes
these needed (carla_runtime.py's role_name-based classification,
pipeline/traffic_chaos.py's "slow_orderly" profile).

IMPORTANT, read before running any of these: EGO_SPAWN/FINAL_GOAL (and
any other scenario-specific coordinates) for unmarked_village_road.py,
highway_merge_slow_traffic.py, dense_market_mixed_traffic.py, and
cattle_crossing.py are PLACEHOLDERS -- Town07/Town06/Town10HD have never
been loaded in this repo before and their real coordinates cannot be
fabricated without a live CARLA server. Each file marks its placeholders
explicitly (`# TODO: capture via carla_print_coordinates.py`). Only
urban_intersection_no_signals.py has real, previously-verified
coordinates (reused directly from framework/scenarios/traffic_stress.py's
own live-tested Town03 spawn point).
"""
