"""
WrongWayVehicleMixin -- a vehicle scripted to drive against traffic
flow, directly in the ego's own lane, facing oncoming -- "opposite lane
driving" / a prohibited-lane incursion. A genuine, common Indian-road
hazard (wrong-side overtaking around a blind corner, a ghost driver
entering via an exit ramp), built entirely from CARLA's own waypoint
graph so it needs ZERO hardcoded coordinates beyond whatever EGO_SPAWN
the mixed-in Scenario already has (placeholder or real, doesn't matter
-- this mixin only ever positions things RELATIVE to it).

WHY A SCRIPTED VEHICLE, NOT TRAFFIC MANAGER: investigated earlier
(see docs/pipeline-decision-log.md) -- TrafficManager.force_lane_change()
only moves an actor into whatever Waypoint.get_left_lane()/
get_right_lane() returns as the ADJACENT lane, which on some maps
(confirmed: the real Warangal OSM-derived network from an earlier
investigation) is a same-direction lane, not the true opposing
carriageway -- and TM's own collision avoidance actively fights against
sustaining a wrong-way maneuver anyway. A directly-controlled scripted
vehicle sidesteps both problems: it's placed and driven exactly where
this mixin says, on any map, with no TM behavior fighting it.

HOW IT STAYS ON THE ROAD: a vehicle driving with steer=0 and constant
throttle will drift off any curved road within seconds -- not
acceptable for something meant to be seen and reacted to, not just
spawned and forgotten. Each tick, `_drive_wrong_way_vehicle()` re-locates
the vehicle's current waypoint and steers toward a point
WRONG_WAY_LOOKAHEAD_M further along its OWN direction of travel -- which,
since this vehicle deliberately drives against the lane's defined
forward direction, is the lane's `previous()` direction, not `next()`
(see waypoint_utils.walk_backward's docstring). This is a small,
self-contained proportional heading controller (`_steer_toward` below),
not a reuse of pipeline/controller.py's PurePursuitController -- that
class is built around the EGO's own pipeline call signature and lookahead
model; a scripted hazard actor doesn't need that machinery, just enough
steering to look and behave like a real wrong-way vehicle for the
duration of an encounter.

USAGE: `class MyScenario(WrongWayVehicleMixin, Scenario)` -- mixin FIRST
in the class bases, same convention framework/scenarios/chaotic_mixin.py
already established. If the concrete Scenario has its own spawn_actors()
override (most will), that override MUST end with
`super().spawn_actors(world, bp_lib, client)` to actually chain into
this mixin -- see framework/ps_scenarios/unmarked_village_road.py for a
worked example. If the concrete Scenario has no on_tick()/cleanup_extra()
of its own, nothing extra is needed there -- Python's MRO lookup finds
this mixin's versions automatically.

NEVER tested against a live CARLA server in this environment -- the
waypoint-walking and steering math is grounded in confirmed-real CARLA
API signatures (see waypoint_utils.py), but the actual driving behavior
(does the steering gain feel right, does it convincingly track a real
curved road) has not been observed live.
"""
from __future__ import annotations

import math

import carla

from framework.base import Scenario, TickContext
from framework.ps_scenarios.waypoint_utils import walk_forward, waypoint_to_transform


def _steer_toward(transform: "carla.Transform", target_location: "carla.Location", gain: float = 1.0) -> float:
    """Minimal proportional heading controller: how hard to steer
    (CARLA's [-1, 1] range) to point the vehicle's current heading
    toward target_location. Not a real path tracker (no lookahead
    curvature reasoning like pipeline/controller.py's PurePursuitController)
    -- deliberately simple, matching this project's established pattern
    of scripted hazard actors getting the minimum machinery needed to be
    convincing (see framework/scenarios/pedestrian_jumpout.py's own
    constant-direction WalkerControl for the same philosophy applied to
    a pedestrian instead of a vehicle).
    """
    dx = target_location.x - transform.location.x
    dy = target_location.y - transform.location.y
    desired_yaw_deg = math.degrees(math.atan2(dy, dx))
    heading_error = (desired_yaw_deg - transform.rotation.yaw + 180.0) % 360.0 - 180.0  # wrap to [-180, 180]
    steer = (heading_error / 90.0) * gain  # normalize a +-90 deg error to the full +-1 steer range
    return max(-1.0, min(1.0, steer))


class WrongWayVehicleMixin:
    WRONG_WAY_SPAWN_OFFSET_M = 60.0  # how far ahead of EGO_SPAWN, along the ego's own lane, the vehicle first appears
    WRONG_WAY_SPEED_THROTTLE = 0.5   # constant throttle -- simple, not speed-controlled to an exact m/s target
    WRONG_WAY_LOOKAHEAD_M = 8.0      # how far along its own (backward-relative-to-the-lane) direction it steers toward
    WRONG_WAY_BLUEPRINT = "vehicle.audi.tt"  # any reasonably compact, commonly-available CARLA vehicle blueprint

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        carla_map = world.get_map()
        ego_waypoint = carla_map.get_waypoint(self.EGO_SPAWN.location)
        spawn_waypoint = walk_forward(ego_waypoint, self.WRONG_WAY_SPAWN_OFFSET_M)

        spawn_transform = waypoint_to_transform(spawn_waypoint)
        # Face the OPPOSITE way to the lane's own forward direction --
        # this is what makes it oncoming from the ego's perspective,
        # not just another vehicle going the same way.
        spawn_transform.rotation.yaw += 180.0

        bp = bp_lib.find(self.WRONG_WAY_BLUEPRINT)
        self.wrong_way_vehicle = self.track(world.spawn_actor(bp, spawn_transform))
        print(f"Spawned wrong-way vehicle {self.WRONG_WAY_SPAWN_OFFSET_M}m ahead of ego, facing oncoming.")

        super().spawn_actors(world, bp_lib, client)

    def on_tick(self, ctx: TickContext) -> None:
        if getattr(self, "wrong_way_vehicle", None) is not None and self.wrong_way_vehicle.is_alive:
            self._drive_wrong_way_vehicle(ctx.world)
        super().on_tick(ctx)

    def _drive_wrong_way_vehicle(self, world: carla.World) -> None:
        carla_map = world.get_map()
        transform = self.wrong_way_vehicle.get_transform()
        current_waypoint = carla_map.get_waypoint(transform.location)

        # This vehicle drives OPPOSITE the lane's defined forward
        # direction -- its own "ahead" is the lane's previous()
        # direction, not next(). See waypoint_utils.walk_backward's
        # docstring.
        target_waypoints = current_waypoint.previous(self.WRONG_WAY_LOOKAHEAD_M)
        if not target_waypoints:
            return  # ran off the mapped end of the road -- stop steering, let it coast to a halt naturally

        steer = _steer_toward(transform, target_waypoints[0].transform.location)
        self.wrong_way_vehicle.apply_control(carla.VehicleControl(
            throttle=self.WRONG_WAY_SPEED_THROTTLE, steer=steer, brake=0.0,
        ))
