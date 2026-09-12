"""
Waypoint-walking helpers -- derive real, always-on-road positions from
CARLA's own map/waypoint graph at runtime, instead of hardcoded
(x, y, z, yaw) literals. Used by highway_merge_slow_traffic.py (deriving
FINAL_GOAL and the scripted lead vehicle's position from wherever
EGO_SPAWN resolves to) and wrong_way_mixin.py (deriving the wrong-way
vehicle's position and its per-tick steering target).

WHY THIS EXISTS: every other scenario in this package ships hardcoded
placeholder coordinates marked "# TODO: capture via
carla_print_coordinates.py" because there's no live CARLA server in this
dev environment to fly a spectator through Town06/Town07/Town10HD and
read real numbers off. Waypoint-walking sidesteps that entirely for
whatever it's used for -- `map.get_waypoint(spawn.location)` +
`waypoint.next(distance)`/`.previous(distance)` are real, always-valid
CARLA API calls (confirmed present in the installed carla==0.9.16
client) that return an actual point ON the actual road network of
whatever map is loaded, without needing anyone to have looked at that
map first. This doesn't eliminate every coordinate in this package (an
EGO_SPAWN still has to come from SOMEWHERE -- see EGO_SPAWN_POINT_INDEX
below), but it eliminates every coordinate that can be expressed as
"some distance along the road from there," which covers most of what a
scenario actually needs.

NEVER tested against a live CARLA server in this environment -- verified
only that the underlying carla.Waypoint.next()/.previous() methods exist
with the expected signature; the walking logic itself (loop, junction
handling, end-of-road handling) has not been exercised against a real
map's waypoint graph.
"""
from __future__ import annotations

import carla


def walk_forward(start_waypoint: "carla.Waypoint", distance_m: float, step_m: float = 10.0) -> "carla.Waypoint":
    """Returns the waypoint approximately distance_m ahead of
    start_waypoint, walked in steps of step_m via repeated
    Waypoint.next() calls (rather than one single next(distance_m) call)
    -- next() follows the CURRENT lane only for as long as the road
    doesn't fork; walking in smaller steps and always taking next()[0]
    (the first/primary continuation) makes it far more likely to still
    be on a sane, connected path if a junction happens to fall within
    the requested distance, at the cost of more calls for a longer walk.

    If the road ends (next() returns an empty list) before distance_m is
    covered, returns the last waypoint actually reached rather than
    raising -- a genuine dead end within a scenario's chosen distance is
    a configuration problem worth surfacing by an obviously-too-short
    resulting distance, not a crash.
    """
    current = start_waypoint
    remaining = distance_m
    while remaining > 0:
        step = min(step_m, remaining)
        next_waypoints = current.next(step)
        if not next_waypoints:
            break  # end of the road (or an unmapped area) -- stop here
        current = next_waypoints[0]
        remaining -= step
    return current


def walk_backward(start_waypoint: "carla.Waypoint", distance_m: float, step_m: float = 10.0) -> "carla.Waypoint":
    """Same as walk_forward, but via Waypoint.previous() -- i.e. walking
    OPPOSITE the lane's own defined direction of travel. Used by
    wrong_way_mixin.py: a vehicle deliberately driving against traffic
    flow has its own "ahead" pointing toward what the lane calls
    "behind" -- previous() is what tracks that vehicle's actual forward
    path, not next().
    """
    current = start_waypoint
    remaining = distance_m
    while remaining > 0:
        step = min(step_m, remaining)
        prev_waypoints = current.previous(step)
        if not prev_waypoints:
            break
        current = prev_waypoints[0]
        remaining -= step
    return current


def waypoint_to_transform(waypoint: "carla.Waypoint", z_offset: float = 0.3) -> "carla.Transform":
    """Converts a Waypoint's own transform into a carla.Transform usable
    for spawn_actor() -- z_offset lifts the spawn point slightly above
    the road surface (CARLA convention: spawning exactly AT road height
    can clip into the mesh and fail; every existing hardcoded spawn
    Transform in this project already includes a similar small lift,
    e.g. z=1.5 for vehicles measured from a slightly-below-wheel-center
    origin -- this is a smaller nudge since waypoint locations are
    already road-surface-accurate, not a rough guess).
    """
    loc = waypoint.transform.location
    return carla.Transform(
        carla.Location(x=loc.x, y=loc.y, z=loc.z + z_offset),
        waypoint.transform.rotation,
    )
