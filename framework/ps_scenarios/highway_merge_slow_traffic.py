"""
HighwayMergeSlowTraffic -- PS scenario #3: "a highway merge involving
slow-moving vehicles."

Town06, CARLA's canonical highway-merge map (long highways, multiple
entry/exit ramps, "Michigan left" turns). FALLBACK, if Town06 turns out
unavailable on the actual CARLA install (confirm via
`client.get_available_maps()` before relying on this): Town04, which
has its own highway loop.

COORDINATE-FREE BY CONSTRUCTION -- no hardcoded (x, y, z, yaw) literals
anywhere in this file. `EGO_SPAWN_POINT_INDEX` picks a real entry out of
`world.get_map().get_spawn_points()` (always valid on any loaded map,
no live coordinate capture needed); `FINAL_GOAL` and `LEAD_VEHICLE_SPAWN`
are then DERIVED from that spawn point by walking forward along the
actual road network via `framework/ps_scenarios/waypoint_utils.py`'s
`walk_forward()` (built on CARLA's own `Waypoint.next()`, confirmed
present in the installed carla==0.9.16 client). This sidesteps the
placeholder-coordinate problem this scenario had before entirely --
runnable, correctly, on whatever Town06-like map is actually loaded,
without anyone needing to fly a spectator through it first. The only
thing still worth tuning once this DOES run live is
`EGO_SPAWN_POINT_INDEX` itself, to land on a spawn point that's actually
near a real merge/on-ramp geometry rather than a generic stretch --
that's a single integer to try different values of, not a coordinate to
hunt for.

NOT purely ambient. Two layers, deliberately combined (mirrors
framework/scenarios/pedestrian_jumpout.py's own pattern of "ambient
scene + one scripted, findable hazard" rather than leaving everything to
Traffic-Manager randomness):

1. Ambient background traffic via pipeline/traffic_chaos.py's
   "slow_orderly" profile -- slower than the speed limit, generous
   following distance, no random lane changes, full light/sign
   compliance. The opposite quality of every other scenario's "chaotic"
   traffic, intentionally.
2. One specific slow lead vehicle, spawned directly (not via Traffic
   Manager) with a hand-set low constant target speed, placed directly
   in the ego's own lane (via walk_forward from the ego's own waypoint,
   so it's guaranteed to be in the same lane, not just nearby) close
   enough ahead to force an actual overtake/merge decision within the
   observable test window.
"""
from __future__ import annotations

import carla

from framework.base import Scenario, TickContext
from framework.ps_scenarios.waypoint_utils import walk_forward, waypoint_to_transform
from pipeline.traffic_chaos import destroy_chaotic_traffic, spawn_chaotic_traffic


class HighwayMergeSlowTraffic(Scenario):
    MAP_NAME = "Town06"  # fallback: "Town04" if unavailable -- confirm live first

    EGO_SPAWN_POINT_INDEX = 0  # index into world.get_map().get_spawn_points() -- the one thing worth
                                 # trying different values of once this runs live, to land near a real
                                 # merge/on-ramp; always a VALID on-road spawn regardless of the value.
    ROUTE_LENGTH_M = 400.0       # how far down the road FINAL_GOAL sits, walked from EGO_SPAWN
    LEAD_VEHICLE_OFFSET_M = 50.0  # how far ahead of the ego (same lane) the scripted slow lead vehicle spawns
    WAYPOINT_STEP_M = 10.0        # step size for the waypoint-walking loop -- see waypoint_utils.walk_forward

    LEAD_VEHICLE_SPEED_THROTTLE = 0.25  # constant throttle -- a genuinely slow truck/cart-like pace
                                          # against highway-speed surroundings, simplest reliable way to
                                          # hold a low, predictable speed (see on_tick() below)
    LEAD_VEHICLE_BLUEPRINT = "vehicle.carlamotors.carlacola"  # bulky, visibly slow-vehicle-shaped

    NUM_AMBIENT_VEHICLES = 15
    AMBIENT_TRAFFIC_SEED = 7

    SPECTATOR_TRANSFORM = None  # falls back to EGO_SPAWN if left unset

    def __init__(self):
        super().__init__()
        self._ambient_traffic_actors: dict = {}
        self.lead_vehicle: carla.Vehicle | None = None
        # EGO_SPAWN/FINAL_GOAL are NOT class attributes here -- they're
        # computed fresh in spawn_actors() below, once a real `world` (and
        # therefore a real map/waypoint graph) exists. ScenarioRunner.run()
        # doesn't read either until AFTER spawn_actors() has already run
        # (confirmed: framework/base.py spawns scenario actors before the
        # ego), so setting them as instance attributes here is safe --
        # they'll be in place by the time anything needs them.

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        carla_map = world.get_map()
        spawn_points = carla_map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError(f"{self.MAP_NAME} has no spawn points -- is this actually a drivable map?")
        self.EGO_SPAWN = spawn_points[self.EGO_SPAWN_POINT_INDEX % len(spawn_points)]

        ego_waypoint = carla_map.get_waypoint(self.EGO_SPAWN.location)

        goal_waypoint = walk_forward(ego_waypoint, self.ROUTE_LENGTH_M, self.WAYPOINT_STEP_M)
        goal_loc = goal_waypoint.transform.location
        self.FINAL_GOAL = (goal_loc.x, goal_loc.y)

        lead_waypoint = walk_forward(ego_waypoint, self.LEAD_VEHICLE_OFFSET_M, self.WAYPOINT_STEP_M)
        lead_spawn_transform = waypoint_to_transform(lead_waypoint)

        self._ambient_traffic_actors = spawn_chaotic_traffic(
            client, world,
            num_vehicles=self.NUM_AMBIENT_VEHICLES,
            num_walkers=0,  # no pedestrians on a highway
            seed=self.AMBIENT_TRAFFIC_SEED,
            avoid_locations=[self.EGO_SPAWN.location, lead_spawn_transform.location],
            profile="slow_orderly",
        )
        for actor_id in self._ambient_traffic_actors["vehicles"] + self._ambient_traffic_actors["controllers"]:
            self.track(actor_id)

        # The one deliberately scripted hazard -- spawned directly, NOT
        # via Traffic Manager/autopilot, so its speed is exactly and
        # reliably held by on_tick() below, not a TM-randomized
        # approximation. Spawned via the SAME waypoint the goal-distance
        # walk passed through, guaranteeing it's actually in the ego's
        # own lane, not just spatially nearby.
        lead_bp = bp_lib.find(self.LEAD_VEHICLE_BLUEPRINT)
        self.lead_vehicle = self.track(world.spawn_actor(lead_bp, lead_spawn_transform))
        print(f"EGO_SPAWN resolved to spawn point #{self.EGO_SPAWN_POINT_INDEX} "
              f"({self.EGO_SPAWN.location.x:.1f}, {self.EGO_SPAWN.location.y:.1f}). "
              f"FINAL_GOAL derived {self.ROUTE_LENGTH_M}m ahead: {self.FINAL_GOAL}. "
              f"Spawned scripted slow lead vehicle {self.LEAD_VEHICLE_OFFSET_M}m ahead in the same lane, "
              f"plus {len(self._ambient_traffic_actors['vehicles'])} ambient slow-orderly vehicles.")

    def on_tick(self, ctx: TickContext) -> None:
        if self.lead_vehicle is not None and self.lead_vehicle.is_alive:
            self.lead_vehicle.apply_control(carla.VehicleControl(
                throttle=self.LEAD_VEHICLE_SPEED_THROTTLE, steer=0.0, brake=0.0,
            ))

    def cleanup_extra(self, client: carla.Client) -> None:
        if self._ambient_traffic_actors:
            destroy_chaotic_traffic(client, client.get_world(), self._ambient_traffic_actors)
