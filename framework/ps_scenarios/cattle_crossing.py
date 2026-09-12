"""
CattleCrossing -- PS scenario #5: "a sudden cattle-crossing event."

Town07, same map as unmarked_village_road.py -- deliberate, not just
convenient: cattle crossings are overwhelmingly a rural/village-road
phenomenon in India, not urban, so reusing that setting is the
thematically correct choice. (One-line MAP_NAME change if a distinct
5th town is preferred instead.)

Structurally near-identical to framework/scenarios/pedestrian_jumpout.py's
hidden-actor + trigger-distance + scripted-motion pattern, with two
deliberate, DISCLOSED differences -- this is a walker-blueprint stand-in,
not a real livestock model (none exists in stock CARLA -- already noted
in HANDOFF.md #7), same honesty standard already applied throughout this
project to its other simulation-only shortcuts
(carla_runtime.GroundTruthDetector, pipeline/drivable_area.py's
ground-truth segmentation):

1. 2-3 walker actors (a small "herd" -- more realistic than a lone
   pedestrian-style crossing), each spawned with role_name="livestock".
   This is what makes them classify as "animal" instead of "pedestrian"
   via carla_runtime.py's role_name-based override (added specifically
   for this scenario -- see docs/pipeline-decision-log.md) -- WITHOUT
   this, pipeline/decision_logic.py's ANIMAL_ON_ROAD branch would never
   fire for this scenario, it would just look like a generic
   OBSTACLE_DETECTED/pedestrian-jumpout duplicate.
2. Slower crossing speed (CROSSING_SPEED_MPS = 1.2, a real walking-cow
   pace) vs. pedestrian_jumpout.py's RUN_SPEED_MPS = 3.5 human-run pace.

COORDINATES: reuses Town07 (same map as unmarked_village_road.py, a
different spot along it) -- still placeholders, same capture requirement.
"""
from __future__ import annotations

import math
import random

import carla

from framework.base import Scenario, TickContext


class CattleCrossing(Scenario):
    MAP_NAME = "Town07"

    # TODO: capture via carla_print_coordinates.py on Town07 -- placeholders,
    # a different spot on the same map than unmarked_village_road.py's.
    EGO_SPAWN = carla.Transform(
        carla.Location(x=0.0, y=50.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    FINAL_GOAL = (150.0, 50.0)  # TODO: replace with a real point further down the road
    SPECTATOR_TRANSFORM = None

    HIDING_SPOT_SPAWN = carla.Transform(  # TODO: replace -- roadside cover the herd waits behind (a tree/building/etc)
        carla.Location(x=70.0, y=53.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0),
    )
    TRIGGER_DISTANCE = 20.0  # slightly more than pedestrian_jumpout.py's 15.0 -- a slower-moving
                             # hazard needs an earlier trigger to still be a meaningful test, not
                             # an instant, unavoidable brake-check
    CROSSING_SPEED_MPS = 1.2  # a real walking-cow pace -- deliberately much slower than a human "run"
    HERD_SIZE = 3

    def __init__(self):
        super().__init__()
        self.herd: list[carla.Walker] = []
        self._triggered = False

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        walker_bps = list(bp_lib.filter("walker.pedestrian.*"))
        for i in range(self.HERD_SIZE):
            bp = random.choice(walker_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            bp.set_attribute("role_name", "livestock")  # THE classification fix -- see module docstring
            # Small lateral offset per herd member so they don't spawn stacked on each other.
            offset_spawn = carla.Transform(
                carla.Location(
                    x=self.HIDING_SPOT_SPAWN.location.x,
                    y=self.HIDING_SPOT_SPAWN.location.y + i * 1.0,
                    z=self.HIDING_SPOT_SPAWN.location.z,
                ),
                self.HIDING_SPOT_SPAWN.rotation,
            )
            walker = self.track(world.spawn_actor(bp, offset_spawn))
            self.herd.append(walker)
        print(f"Spawned a hidden herd of {len(self.herd)} livestock-labeled walkers.")

    def on_tick(self, ctx: TickContext) -> None:
        if not self.herd:
            return

        if not self._triggered:
            lead_loc = self.herd[0].get_transform().location
            distance = math.hypot(ctx.ego_x - lead_loc.x, ctx.ego_y - lead_loc.y)
            if distance <= self.TRIGGER_DISTANCE:
                print(f"!!! CATTLE CROSSING TRIGGERED !!! Distance: {distance:.2f}m")
                self._triggered = True

        if self._triggered:
            for walker in self.herd:
                if walker.is_alive:
                    walker.apply_control(carla.WalkerControl(
                        direction=carla.Vector3D(x=0.0, y=-1.0, z=0.0),
                        speed=self.CROSSING_SPEED_MPS,
                    ))
