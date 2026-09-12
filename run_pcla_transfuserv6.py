"""
Comparison run (ii): fully end-to-end PCLA-bundled TransFuser v6, driving
directly off its own sensors -- no involvement of this project's own
perception/prediction/planning stack at all. This is PCLA's standard,
documented, tested use case (see external/PCLA/sample.py) -- the only
new things here are the background chaotic traffic (pipeline/
traffic_chaos.py, replacing the dropped SUMMIT plan -- see
docs/pipeline-decision-log.md) and wiring to this project's own
MetricsRecorder so results land in the same metrics_export.py
aggregation as run_own_perception_plant2.py and run_live.py.

Counterpart run: run_own_perception_plant2.py (approach i). See that
file's docstring and docs/TRAFFIC_AND_INTEGRATION.md for why these two
are not a strictly fair head-to-head (TransFuser v6 is sensor-based,
PlanT2 is object-level/planning-only) -- treat this run as a reference
point, not a like-for-like rival.

BEFORE RUNNING: fill in every CONFIG value below. Requires PCLA's own
environment set up first (external/PCLA/environment.yml, plus its
download_weights.py / download_assets.py for the tfv6_regnet checkpoint
-- see external/PCLA/README.md). This has NEVER been run against a live
CARLA server in this environment -- syntax/import-checked only.

Run: .venv\\Scripts\\python.exe run_pcla_transfuserv6.py
"""
from __future__ import annotations

import os
import sys

import carla
import numpy as np

# Make the cloned PCLA repo importable. PCLA.py itself inserts its own
# directory at the front of sys.path on import (see external/PCLA/PCLA.py),
# which is also required for its many internal "from pcla_functions import
# ..." / "from leaderboard_codes import ..." absolute imports to resolve --
# so this path must be added BEFORE `from PCLA import PCLA`.
_PCLA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "external", "PCLA")
if _PCLA_DIR not in sys.path:
    sys.path.insert(0, _PCLA_DIR)

from PCLA import PCLA  # noqa: E402  (must follow sys.path setup above)

from pipeline.metrics import MetricsRecorder  # noqa: E402
from pipeline.pipeline import TickTimings  # noqa: E402
from pipeline.traffic_chaos import destroy_chaotic_traffic, spawn_chaotic_traffic  # noqa: E402

# ============================================================
# CONFIG -- fill these in for your actual setup before running
# ============================================================
CARLA_HOST = "localhost"
CARLA_PORT = 2000
FIXED_DELTA_SECONDS = 0.05

MAP_NAME = "REPLACE_WITH_MAP_NAME"          # e.g. "Town02" -- must match TOWN_NAME in run_own_perception_plant2.py
                                              # for the two comparison runs to use the same scene
EGO_SPAWN_POINT_INDEX = 0                    # index into world.get_map().get_spawn_points()
EGO_VEHICLE_BLUEPRINT = "vehicle.tesla.model3"

# PCLA agent key -- "tfv6_regnet" resolves via external/PCLA/agents.json's
# nested {"tfv6": {"regnet": {...}}} entry (pcla_functions/give_path.py
# splits on "_": agent_name="tfv6", variant="regnet"). Swap the variant
# for a different TransFuser v6 checkpoint (resnet34/4cameras/etc, see
# agents.json).
PCLA_AGENT_KEY = "tfv6_regnet"

# PCLA needs a route XML file (leaderboard-format). external/PCLA/
# sample_route.xml is a placeholder for a different town -- REPLACE with
# a real route file for MAP_NAME (see external/PCLA/README.md's route
# format, or generate one from a sequence of waypoints on your map).
ROUTE_XML_PATH = os.path.join(_PCLA_DIR, "sample_route.xml")

NUM_BACKGROUND_VEHICLES = 40
NUM_BACKGROUND_WALKERS = 30
TRAFFIC_SEED = 42          # keep fixed across the two comparison scripts' runs for a fair comparison;
                            # vary across repeated runs of the SAME script per metrics_export.py's
                            # >=3-runs-per-scenario seed-sensitivity guidance

SCENARIO_NAME = "pcla_tfv6_regnet"   # REPLACE per run if testing multiple TFv6 variants
GOAL_REACHED_RADIUS_M = 3.0
MAX_TICKS = 3000            # hard stop -- avoid an unbounded run if the agent never reaches the route end
# ============================================================


def main():
    if MAP_NAME.startswith("REPLACE_"):
        print("Fill in MAP_NAME, EGO_SPAWN_POINT_INDEX, and ROUTE_XML_PATH in the CONFIG section first.")
        sys.exit(1)

    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(20.0)
    print(f"Client version: {client.get_client_version()}, Server version: {client.get_server_version()}")

    world = client.load_world(MAP_NAME)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)

    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.find(EGO_VEHICLE_BLUEPRINT)
    spawn_points = world.get_map().get_spawn_points()
    if EGO_SPAWN_POINT_INDEX >= len(spawn_points):
        print(f"EGO_SPAWN_POINT_INDEX={EGO_SPAWN_POINT_INDEX} out of range "
              f"(map has {len(spawn_points)} spawn points).")
        sys.exit(1)
    ego = world.spawn_actor(ego_bp, spawn_points[EGO_SPAWN_POINT_INDEX])
    world.tick()
    print(f"Spawned ego: {ego.type_id} (id={ego.id}) at spawn point {EGO_SPAWN_POINT_INDEX}")

    print(f"Spawning background traffic ({NUM_BACKGROUND_VEHICLES} vehicles, "
          f"{NUM_BACKGROUND_WALKERS} walkers, seed={TRAFFIC_SEED})...")
    traffic_actors = spawn_chaotic_traffic(
        client, world,
        num_vehicles=NUM_BACKGROUND_VEHICLES,
        num_walkers=NUM_BACKGROUND_WALKERS,
        seed=TRAFFIC_SEED,
    )

    metrics = MetricsRecorder(scenario_name=SCENARIO_NAME, dt=FIXED_DELTA_SECONDS)
    collision_bp = bp_lib.find("sensor.other.collision")
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego)
    collision_sensor.listen(lambda event: metrics.record_collision(event.other_actor.type_id))

    print(f"Loading PCLA agent '{PCLA_AGENT_KEY}'...")
    pcla = PCLA(PCLA_AGENT_KEY, ego, ROUTE_XML_PATH, client)

    # PCLA's own route (parsed from ROUTE_XML_PATH) defines the goal --
    # use its last waypoint as the completion target, same "distance to
    # goal" semantics as run_live.py and run_own_perception_plant2.py.
    goal_transform = spawn_points[EGO_SPAWN_POINT_INDEX]  # fallback if route parsing below fails
    try:
        route_wps = pcla.agent_instance._global_plan_world_coord
        if route_wps:
            goal_transform = route_wps[-1][0]
    except AttributeError:
        print("Could not read agent's parsed route for a goal point -- "
              "completion tracking will use the spawn point instead (won't trigger). "
              "Fix by inspecting pcla.agent_instance's route attributes for this agent type.")

    print(f"Starting closed loop, scenario='{SCENARIO_NAME}'. Ctrl+C to stop.")
    completed, reason = False, "stopped before reaching goal"
    tick_count = 0
    try:
        while tick_count < MAX_TICKS:
            control = pcla.get_action()
            ego.apply_control(control)
            world.tick()
            tick_count += 1

            transform = ego.get_transform()
            velocity = ego.get_velocity()
            accel = ego.get_acceleration()
            distance_to_goal = float(np.hypot(
                goal_transform.location.x - transform.location.x,
                goal_transform.location.y - transform.location.y,
            ))

            # PCLA/TransFuser v6 has no replanning-latency / planning-
            # validity concept exposed through get_action() -- unlike
            # run_live.py's own TickTimings (7 named stages), this is one
            # opaque forward pass. Record it as planning_ms (the closest
            # semantic fit -- "how long did producing this tick's control
            # take") with every other stage at 0, so metrics_export.py's
            # aggregation still runs without special-casing this scenario,
            # but total_latency_ms here means something different than in
            # the own-perception scenarios -- don't directly compare the
            # two total_latency_ms numbers without accounting for this.
            timings = TickTimings(
                fusion_ms=0.0, tracking_ms=0.0, prediction_ms=0.0,
                drivable_area_ms=0.0, decision_ms=0.0,
                planning_ms=0.0,  # PCLA's get_action() is called before this block; wall-clock
                                   # timing would need to wrap the get_action() call itself --
                                   # left at 0 here deliberately rather than a misleading estimate.
                control_ms=0.0,
            )

            metrics.record_tick(
                tick=tick_count, timings=timings,
                replanned=False, path_valid=True,   # not meaningful for this agent -- see note above
                decision_mode="pcla_tfv6_endtoend",
                speed_mps=float(np.hypot(velocity.x, velocity.y)),
                accel_x=accel.x, accel_y=accel.y,
                distance_to_goal_m=distance_to_goal,
            )

            if distance_to_goal <= GOAL_REACHED_RADIUS_M:
                completed, reason = True, "reached goal"
                print(f"\nGoal reached (within {GOAL_REACHED_RADIUS_M}m) after {tick_count} ticks.")
                break

        if tick_count >= MAX_TICKS:
            reason = f"hit MAX_TICKS={MAX_TICKS} without reaching goal"

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")
        reason = "manually stopped"
    finally:
        metrics.finalize(completed=completed, reason=reason)
        collision_sensor.stop(); collision_sensor.destroy()
        destroy_chaotic_traffic(client, world, traffic_actors)
        pcla.cleanup()  # also destroys ego (see PCLA.py's cleanup())
        settings.synchronous_mode = False
        world.apply_settings(settings)
        tm.set_synchronous_mode(False)
        print("Cleaned up: PCLA agent, ego vehicle, background traffic, synchronous mode disabled.")


if __name__ == "__main__":
    main()
