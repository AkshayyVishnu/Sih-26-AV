"""
UnmarkedVillageRoad -- PS scenario #1: "an unmarked village road."

Town07, CARLA's canonical rural map (narrow roads, sparse/no lane
markings) -- directly matches "unmarked" without any asset changes, per
the explicit instruction to use stock CARLA towns/assets only (no custom
OpenDRIVE/RoadRunner import for this batch of scenarios).

Two hazards, both matching the PS's own named failure modes for this
kind of road ("informal merging" explicitly, plus a genuine, common
Indian-road hazard: wrong-side overtaking around a blind corner) --
still deliberately LIGHT on ambient traffic/chaos otherwise, since the
point of this scenario is the unmarked/narrow road GEOMETRY itself (how
pipeline/drivable_area.py's ground-truth-segmentation-based drivable-area
estimate and pipeline/planner.py's A* costmap behave with no lane-line
signal to lean on) -- that's dense_market_mixed_traffic.py's job, adding
heavy background traffic here would confound what's being tested:

1. A static half-on-road obstruction (unchanged from before) -- directly
   matches "informal merging": navigating around an encroachment on a
   narrow road with no marked shoulder to signal where the "correct"
   space to pass is.
2. `framework/ps_scenarios/wrong_way_mixin.py`'s WrongWayVehicleMixin --
   a vehicle scripted to drive against traffic flow, directly in the
   ego's own lane. COORDINATE-FREE: it derives its own position and
   per-tick steering target entirely from CARLA's waypoint graph relative
   to EGO_SPAWN (see that mixin's own docstring) -- it needed no new
   coordinates of its own, unlike the obstruction below, which still
   does (see the TODO markers).

COORDINATES FOR THE OBSTRUCTION/EGO_SPAWN/FINAL_GOAL ARE STILL
PLACEHOLDERS. Town07 has never been loaded in this repo before -- real
values cannot be fabricated without a live CARLA server. Capture real
ones using the already-existing Simulation Files/carla_print_coordinates.py
(edit its MAP_NAME to "Town07" first), fly the spectator to a genuinely
narrow/unmarked stretch of road, and replace every value marked TODO
below. The wrong-way vehicle needs none of this -- it positions itself
relative to whatever EGO_SPAWN ends up being, real or placeholder.
"""
from __future__ import annotations

import carla

from framework.base import Scenario
from framework.ps_scenarios.wrong_way_mixin import WrongWayVehicleMixin


class UnmarkedVillageRoad(WrongWayVehicleMixin, Scenario):
    MAP_NAME = "Town07"

    # TODO: capture via carla_print_coordinates.py on Town07 -- these are
    # placeholders only, not measured against the real map.
    EGO_SPAWN = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    FINAL_GOAL = (150.0, 0.0)  # TODO: replace with a real point further down the same road
    SPECTATOR_TRANSFORM = None  # falls back to EGO_SPAWN if left unset -- fine for this scenario

    # Where the static obstruction sits partway along the route --
    # TODO: replace once EGO_SPAWN/FINAL_GOAL are real; should be roughly
    # midway, encroaching on part of the road width (not blocking it
    # entirely -- the point is testing a partial-obstruction squeeze-by,
    # not a dead end). Keep this closer to EGO_SPAWN than
    # WrongWayVehicleMixin's own WRONG_WAY_SPAWN_OFFSET_M (60m default),
    # so the two hazards don't overlap into one confusing pileup.
    OBSTRUCTION_SPAWN = carla.Transform(
        carla.Location(x=35.0, y=1.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )

    def __init__(self):
        super().__init__()
        self.obstruction: carla.Vehicle | None = None

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        # Same static-obstruction pattern already proven live in
        # framework/scenarios/pedestrian_jumpout.py (vehicle.carlamotors.
        # carlacola, hand-braked) -- reused here rather than reinvented.
        obs_bp = bp_lib.find("vehicle.carlamotors.carlacola")
        self.obstruction = self.track(world.spawn_actor(obs_bp, self.OBSTRUCTION_SPAWN))
        self.obstruction.apply_control(carla.VehicleControl(hand_brake=True))
        print("Spawned static roadside obstruction (informal encroachment).")

        # Chains into WrongWayVehicleMixin.spawn_actors(), which spawns
        # the wrong-way vehicle and then chains into Scenario's own
        # (abstract, no-op-for-this-class) spawn_actors().
        super().spawn_actors(world, bp_lib, client)

    # No on_tick() override of our own -- WrongWayVehicleMixin's on_tick()
    # (driving the wrong-way vehicle every tick) is found automatically
    # via Python's normal method resolution, since this class doesn't
    # define one of its own to shadow it.
