"""
UnmarkedVillageRoad -- PS scenario #1: "an unmarked village road."

Town07, CARLA's canonical rural map (narrow roads, sparse/no lane
markings) -- directly matches "unmarked" without any asset changes, per
the explicit instruction to use stock CARLA towns/assets only (no custom
OpenDRIVE/RoadRunner import for this batch of scenarios).

Deliberately LIGHT on traffic/hazards -- the point of this scenario is
the unmarked/narrow road GEOMETRY itself (how pipeline/drivable_area.py's
ground-truth-segmentation-based drivable-area estimate and
pipeline/planner.py's A* costmap behave with no lane-line signal to lean
on), not chaos -- that's dense_market_mixed_traffic.py's job. Adding
heavy background traffic here would confound what's actually being
tested. The one obstacle included (a static half-on-road obstruction)
directly matches the PS's own phrase "informal merging" -- navigating
around an encroachment on a narrow road with no marked shoulder to
signal where the "correct" space to pass is.

COORDINATES ARE PLACEHOLDERS. Town07 has never been loaded in this repo
before -- real values cannot be fabricated without a live CARLA server.
Capture real ones using the already-existing
Simulation Files/carla_print_coordinates.py (edit its MAP_NAME to
"Town07" first), fly the spectator to a genuinely narrow/unmarked
stretch of road, and replace every value marked TODO below.
"""
from __future__ import annotations

import carla

from framework.base import Scenario


class UnmarkedVillageRoad(Scenario):
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
    # not a dead end).
    OBSTRUCTION_SPAWN = carla.Transform(
        carla.Location(x=75.0, y=1.0, z=1.5),
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

    # No on_tick() override -- nothing scripted mid-run. This is a
    # straight, uninterrupted drive testing perception/drivable-area/
    # planning against real unmarked-road geometry, not a triggered event.
