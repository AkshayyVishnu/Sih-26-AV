"""
Dense, heterogeneous, aggressively-behaved background traffic -- a
substitute for SUMMIT (github.com/AdaCompNUS/summit), built on CARLA's
own Traffic Manager instead.

WHY THIS EXISTS INSTEAD OF ACTUAL SUMMIT (decision log has the full
writeup, this is the short version): SUMMIT is a full Unreal-Engine
source FORK of CARLA, pinned to CARLA 0.9.8 (confirmed by reading its
own PythonAPI/carla/setup.py: version='0.9.8', and its CHANGELOG.md,
which opens at 0.9.8). It is not a plugin or a script that attaches to a
running server -- it ships its own simulator binary, 8 major CARLA
releases behind the friend's live 0.9.16 server and the carla==0.9.16
pip client this whole project is built against. Building SUMMIT from
source would mean building a second, separate Unreal Engine 4.24-based
simulator (Epic Games account linked to CARLA's GitHub org, tens of GB,
hours of build time) that ends up NOT being the same world the friend's
CARLA server runs, and PCLA's agents (tested at 0.9.15/0.9.16) are not
confirmed to even run correctly against a 0.9.8 server (sensor
definitions, blueprint names, and semantic-segmentation tag IDs are
already confirmed to differ across CARLA versions in this project's own
history -- see docs/pipeline-decision-log.md's tag-ID bug). None of this
is buildable/testable in the time available, so SUMMIT itself is
dropped.

What this module does instead: uses CARLA's built-in Traffic Manager
(present in every CARLA server, no separate build) with parameters tuned
toward the same qualitative goal SUMMIT's GAMMA model targets --
dense, low-following-distance, frequent lane changes, mixed vehicle
types, jaywalking pedestrians, light-signal non-compliance -- as an
approximation of unregulated/heterogeneous Indian-road traffic. This is
a coarser approximation than SUMMIT's actual crowd model (no true
lane-less/gap-filling behavior, no two-wheeler-specific weaving), but it
runs against the exact server everything else in this project targets,
today, with zero extra build steps.

NEVER RUN against a live CARLA server in this environment -- syntax/
import-checked only, same as run_live.py at the time it was written.
"""
from __future__ import annotations

import logging
import random

import carla

logger = logging.getLogger("pipeline.traffic_chaos")

# Blueprint filters used to approximate heterogeneous Indian traffic --
# CARLA's blueprint library has no auto-rickshaw/cow/loaded-cart, this is
# the closest available mix (cars + two-wheeler-shaped vehicles +
# pedestrians). Extend if the friend's CARLA build has extra custom
# blueprints imported for this project.
_VEHICLE_FILTERS = ["vehicle.*"]
_TWO_WHEELER_HINTS = ("harley", "kawasaki", "yamaha", "vespa", "bh.crossbike", "gazelle", "diamondback")


def _split_vehicle_blueprints(bp_library) -> tuple[list, list]:
    """Returns (four_plus_wheelers, two_wheelers) blueprint lists, split
    by a name-substring heuristic (CARLA has no explicit wheel-count
    attribute on most versions) -- separated so two-wheelers can be
    spawned at a higher proportion than CARLA's default mix, closer to
    real Indian traffic composition.
    """
    all_vehicles = list(bp_library.filter("vehicle.*"))
    two_wheelers = [
        bp for bp in all_vehicles
        if any(hint in bp.id.lower() for hint in _TWO_WHEELER_HINTS)
        or bp.get_attribute("number_of_wheels") is not None
        and bp.get_attribute("number_of_wheels").as_int() == 2
    ]
    four_plus = [bp for bp in all_vehicles if bp not in two_wheelers]
    return four_plus, two_wheelers


def _apply_traffic_manager_tuning(tm, actor, profile: str) -> None:
    """Per-vehicle Traffic Manager parameter tuning for one of the two
    supported profiles -- see spawn_chaotic_traffic()'s profile docstring
    for the intent behind each. Factored out so both profiles share one
    call site rather than duplicating the per-vehicle loop.
    """
    if profile == "aggressive":
        # Approximates SUMMIT's unregulated-traffic qualitative behavior
        # (frequent lane changes, tailgating, light-signal non-compliance)
        # without SUMMIT's actual GAMMA model -- see
        # docs/pipeline-decision-log.md for why SUMMIT itself was dropped.
        tm.vehicle_percentage_speed_difference(actor, random.uniform(-40, -5))  # negative = faster than speed limit
        tm.distance_to_leading_vehicle(actor, random.uniform(0.5, 2.0))
        tm.ignore_lights_percentage(actor, random.uniform(0, 30))
        tm.ignore_signs_percentage(actor, random.uniform(0, 20))
        tm.random_left_lanechange_percentage(actor, random.uniform(20, 60))
        tm.random_right_lanechange_percentage(actor, random.uniform(20, 60))
        tm.auto_lane_change(actor, True)
    else:  # "slow_orderly" -- the opposite quality, for a slow-moving-vehicles highway merge
        tm.vehicle_percentage_speed_difference(actor, random.uniform(15, 40))  # positive = slower than speed limit
        tm.distance_to_leading_vehicle(actor, random.uniform(3.0, 6.0))  # generous following distance
        tm.ignore_lights_percentage(actor, 0)
        tm.ignore_signs_percentage(actor, 0)
        tm.random_left_lanechange_percentage(actor, 0)
        tm.random_right_lanechange_percentage(actor, 0)
        tm.auto_lane_change(actor, False)


def spawn_chaotic_traffic(
    client: "carla.Client",
    world: "carla.World",
    num_vehicles: int = 40,
    num_walkers: int = 30,
    two_wheeler_fraction: float = 0.4,
    traffic_manager_port: int = 8000,
    seed: int | None = None,
    avoid_locations: list | None = None,
    min_clearance_m: float = 8.0,
    profile: str = "aggressive",
) -> dict:
    """Spawns background traffic and configures the Traffic Manager.
    Returns a dict of the spawned actor ID lists ({'vehicles': [...],
    'walkers': [...], 'controllers': [...]}) so the caller can clean them
    up later (world.get_actors().filter(...) + destroy(), or
    client.apply_batch(carla.command.DestroyActor(id)) for each id in the
    returned lists).

    profile: "aggressive" (default -- tight following, frequent lane
    changes, some light/sign non-compliance, faster than the speed limit;
    the original behavior, unchanged for every existing caller) or
    "slow_orderly" (slower than the speed limit, generous following
    distance, no random lane changes, full light/sign compliance) --
    added for framework/ps_scenarios/highway_merge_slow_traffic.py, where
    the PS specifically calls for "slow-moving vehicles," the opposite
    quality "aggressive" was built for. See _apply_traffic_manager_tuning
    below for the actual per-vehicle parameter values of each.

    two_wheeler_fraction: target proportion of spawned vehicles that are
    two-wheeler-shaped (see _split_vehicle_blueprints) -- real Indian
    traffic mixes run far higher two-wheeler share than CARLA's default
    town traffic, this biases the spawn mix toward that.

    seed: pass the same value across repeated runs for the seed-
    sensitivity handling metrics_export.py already expects (>=3 runs per
    scenario, per carla_garage's common-mistakes guide already cited in
    docs/architecture.md).

    avoid_locations: list of carla.Location to keep clear of spawned
    vehicles (typically the ego's spawn point). REQUIRED consideration
    when calling this from framework.base.Scenario.spawn_actors() --
    that runs BEFORE the ego is spawned (see framework/base.py's own
    Scenario docstring), so there is nothing to collide-check against
    except a known, static location passed in explicitly. Same pattern
    framework/scenarios/traffic_stress.py already uses for its own
    background vehicles (a 5m clearance check against EGO_SPAWN) --
    this generalizes it to an arbitrary list of locations and a
    caller-chosen radius, since chaotic-traffic scenarios may also want
    to protect a goal point or a scripted actor's spawn.
    """
    if profile not in ("aggressive", "slow_orderly"):
        raise ValueError(f"Unknown traffic profile '{profile}' -- expected 'aggressive' or 'slow_orderly'.")

    if seed is not None:
        random.seed(seed)

    tm = client.get_trafficmanager(traffic_manager_port)
    tm.set_synchronous_mode(world.get_settings().synchronous_mode)
    tm.set_global_distance_to_leading_vehicle(1.0)  # tight following -- part of the "chaotic" approximation

    bp_library = world.get_blueprint_library()
    four_plus_bps, two_wheeler_bps = _split_vehicle_blueprints(bp_library)
    if not two_wheeler_bps:
        logger.warning("No two-wheeler-shaped blueprints found in this CARLA build's "
                        "blueprint library -- falling back to four-plus-wheelers only. "
                        "Traffic composition will look less like real Indian traffic.")

    spawn_points = world.get_map().get_spawn_points()
    if avoid_locations:
        before = len(spawn_points)
        spawn_points = [
            sp for sp in spawn_points
            if all(sp.location.distance(loc) >= min_clearance_m for loc in avoid_locations)
        ]
        logger.info("Filtered %d/%d spawn points within %.1fm of %d protected location(s).",
                     before - len(spawn_points), before, min_clearance_m, len(avoid_locations))
    random.shuffle(spawn_points)

    vehicle_ids: list[int] = []
    batch = []
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    for i in range(min(num_vehicles, len(spawn_points))):
        use_two_wheeler = two_wheeler_bps and random.random() < two_wheeler_fraction
        bp = random.choice(two_wheeler_bps if use_two_wheeler else four_plus_bps)
        if bp.has_attribute("color"):
            bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
        if bp.has_attribute("driver_id"):
            bp.set_attribute("driver_id", random.choice(bp.get_attribute("driver_id").recommended_values))
        bp.set_attribute("role_name", "chaotic_traffic")

        batch.append(
            SpawnActor(bp, spawn_points[i]).then(SetAutopilot(FutureActor, True, tm.get_port()))
        )

    responses = client.apply_batch_sync(batch, world.get_settings().synchronous_mode)
    for response in responses:
        if response.error:
            logger.warning("Vehicle spawn failed: %s", response.error)
        else:
            vehicle_ids.append(response.actor_id)

    for vid in vehicle_ids:
        actor = world.get_actor(vid)
        if actor is None:
            continue
        _apply_traffic_manager_tuning(tm, actor, profile)

    logger.info("Spawned %d/%d background vehicles (%d requested two-wheeler-biased), TM tuned '%s'.",
                len(vehicle_ids), num_vehicles, int(num_vehicles * two_wheeler_fraction), profile)

    # Walkers -- CARLA's own bulk-spawn pattern (world.spawn_actor for
    # walkers + a matching WalkerAIController per walker, batched).
    walker_bps = list(bp_library.filter("walker.pedestrian.*"))
    walker_spawn_points = []
    for _ in range(num_walkers):
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            walker_spawn_points.append(carla.Transform(loc))

    walker_batch = [SpawnActor(random.choice(walker_bps), tp) for tp in walker_spawn_points]
    walker_responses = client.apply_batch_sync(walker_batch, world.get_settings().synchronous_mode)
    walker_ids = [r.actor_id for r in walker_responses if not r.error]

    controller_bp = bp_library.find("controller.ai.walker")
    controller_batch = [SpawnActor(controller_bp, carla.Transform(), wid) for wid in walker_ids]
    controller_responses = client.apply_batch_sync(controller_batch, world.get_settings().synchronous_mode)
    controller_ids = [r.actor_id for r in controller_responses if not r.error]

    world.tick() if world.get_settings().synchronous_mode else world.wait_for_tick()

    for cid in controller_ids:
        controller = world.get_actor(cid)
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        # Jaywalking approximation: CARLA walker AI controllers otherwise
        # stick to crosswalks/sidewalks -- max_speed variance + the
        # unregulated crossing behavior itself is set globally below.
        controller.set_max_speed(random.uniform(1.0, 2.2))

    world.set_pedestrians_cross_factor(0.6)  # fraction of walkers willing to cross anywhere, not just crosswalks

    logger.info("Spawned %d/%d walkers with AI controllers.", len(walker_ids), num_walkers)

    return {"vehicles": vehicle_ids, "walkers": walker_ids, "controllers": controller_ids}


def destroy_chaotic_traffic(client: "carla.Client", world: "carla.World", actor_ids: dict) -> None:
    """Cleans up everything spawn_chaotic_traffic returned. Controllers
    must be stopped before destruction (CARLA requirement) and walkers
    destroyed separately from their controllers.
    """
    for cid in actor_ids.get("controllers", []):
        controller = world.get_actor(cid)
        if controller is not None:
            controller.stop()

    destroy_ids = (
        actor_ids.get("vehicles", [])
        + actor_ids.get("walkers", [])
        + actor_ids.get("controllers", [])
    )
    client.apply_batch([carla.command.DestroyActor(x) for x in destroy_ids])
    logger.info("Destroyed %d background traffic actors.", len(destroy_ids))
