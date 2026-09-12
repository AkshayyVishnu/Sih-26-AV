"""
HighwayMergeSlowTraffic -- PS scenario #3: "a highway merge involving
slow-moving vehicles."

Town06, CARLA's canonical highway-merge map (long highways, multiple
entry/exit ramps, "Michigan left" turns). FALLBACK, if Town06 turns out
unavailable on the actual CARLA install (confirm via
`client.get_available_maps()` before relying on this): Town04, which
has its own highway loop.

NOT purely ambient. Two layers, deliberately combined (mirrors
framework/scenarios/pedestrian_jumpout.py's own pattern of "ambient
scene + one scripted, findable hazard" rather than leaving everything to
Traffic-Manager randomness):

1. Ambient background traffic via pipeline/traffic_chaos.py's NEW
   "slow_orderly" profile (added specifically for this scenario -- see
   docs/pipeline-decision-log.md) -- slower than the speed limit,
   generous following distance, no random lane changes, full light/sign
   compliance. The opposite quality of every other scenario's "chaotic"
   traffic, intentionally.
2. One specific slow lead vehicle, spawned directly (not via Traffic
   Manager) with a hand-set low constant target speed, placed directly
   in the ego's lane close enough ahead to force an actual overtake/
   merge decision within the observable test window -- rather than
   relying on ambient randomness to maybe produce a meaningful
   interaction.

COORDINATES ARE PLACEHOLDERS. Town06 has never been loaded in this repo
before -- capture real ones via Simulation Files/carla_print_coordinates.py
(edit MAP_NAME to "Town06"), specifically near a real merge/on-ramp
geometry, which can only be found by flying the spectator on the actual
map.
"""
from __future__ import annotations

import carla

from framework.base import Scenario, TickContext
from pipeline.traffic_chaos import destroy_chaotic_traffic, spawn_chaotic_traffic


class HighwayMergeSlowTraffic(Scenario):
    MAP_NAME = "Town06"  # fallback: "Town04" if unavailable -- confirm live first

    # TODO: capture via carla_print_coordinates.py on Town06 -- placeholders,
    # need a real spot on/approaching an actual merge lane.
    EGO_SPAWN = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    FINAL_GOAL = (400.0, 0.0)  # TODO: replace -- a point past the merge, far enough for a highway-speed drive
    SPECTATOR_TRANSFORM = None

    # The one scripted slow lead vehicle -- TODO: replace once
    # EGO_SPAWN/FINAL_GOAL are real; should sit in the ego's own lane,
    # close enough ahead (order of 40-60m on a highway) that the ego
    # can't avoid reacting to it well before the goal.
    LEAD_VEHICLE_SPAWN = carla.Transform(
        carla.Location(x=60.0, y=0.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    LEAD_VEHICLE_SPEED_MPS = 4.0  # a genuinely slow truck/cart-like pace against highway-speed surroundings

    NUM_AMBIENT_VEHICLES = 15
    AMBIENT_TRAFFIC_SEED = 7

    def __init__(self):
        super().__init__()
        self._ambient_traffic_actors: dict = {}
        self.lead_vehicle: carla.Vehicle | None = None

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        self._ambient_traffic_actors = spawn_chaotic_traffic(
            client, world,
            num_vehicles=self.NUM_AMBIENT_VEHICLES,
            num_walkers=0,  # no pedestrians on a highway
            seed=self.AMBIENT_TRAFFIC_SEED,
            avoid_locations=[self.EGO_SPAWN.location, self.LEAD_VEHICLE_SPAWN.location],
            profile="slow_orderly",
        )
        for actor_id in self._ambient_traffic_actors["vehicles"] + self._ambient_traffic_actors["controllers"]:
            self.track(actor_id)

        # The one deliberately scripted hazard -- spawned directly, NOT
        # via Traffic Manager/autopilot, so its speed is exactly and
        # reliably LEAD_VEHICLE_SPEED_MPS every tick, not a TM-randomized
        # approximation.
        lead_bp = bp_lib.find("vehicle.carlamotors.carlacola")  # bulky, visibly slow-vehicle-shaped
        self.lead_vehicle = self.track(world.spawn_actor(lead_bp, self.LEAD_VEHICLE_SPAWN))
        print(f"Spawned scripted slow lead vehicle (target {self.LEAD_VEHICLE_SPEED_MPS} m/s) "
              f"plus {len(self._ambient_traffic_actors['vehicles'])} ambient slow-orderly vehicles.")

    def on_tick(self, ctx: TickContext) -> None:
        # Constant low-speed throttle -- simplest reliable way to hold a
        # fixed slow speed without needing a full PID/cruise-control
        # loop; a slow-moving obstacle doesn't need to be a good driver,
        # just a predictable one.
        if self.lead_vehicle is not None and self.lead_vehicle.is_alive:
            self.lead_vehicle.apply_control(carla.VehicleControl(throttle=0.25, steer=0.0, brake=0.0))

    def cleanup_extra(self, client: carla.Client) -> None:
        if self._ambient_traffic_actors:
            destroy_chaotic_traffic(client, client.get_world(), self._ambient_traffic_actors)
