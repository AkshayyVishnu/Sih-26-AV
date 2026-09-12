"""
PedestrianJumpOut -- a static obstruction vehicle with a pedestrian
hidden just past it, who runs out into the road once the ego gets within
TRIGGER_DISTANCE. Migrated from (not replacing) Simulation Files/
scenario_2.py -- that script is untouched; this is the same scenario
expressed as a framework.base.Scenario subclass.
"""
from __future__ import annotations

import math
import random

import carla

from framework.base import Scenario, TickContext


class PedestrianJumpOut(Scenario):
    EGO_SPAWN = carla.Transform(
        carla.Location(x=20.0, y=134.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    FINAL_GOAL = (110.0, 137.0)  # straight down the same lane, past the obstruction/pedestrian
    SPECTATOR_TRANSFORM = carla.Transform(
        carla.Location(x=80.0, y=125.0, z=6.0),
        carla.Rotation(pitch=-30.0, yaw=150.0, roll=0.0),
    )

    OBSTRUCTION_SPAWN = carla.Transform(
        carla.Location(x=67.0, y=137.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    PEDESTRIAN_SPAWN = carla.Transform(
        carla.Location(x=72.0, y=137.0, z=1.5),  # 5m in front of the obstruction, hiding the pedestrian
        carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0),
    )
    TRIGGER_DISTANCE = 15.0
    RUN_SPEED_MPS = 3.5

    def __init__(self):
        super().__init__()
        self.obstruction: carla.Vehicle | None = None
        self.pedestrian: carla.Walker | None = None
        self._triggered = False

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        obs_bp = bp_lib.find("vehicle.carlamotors.carlacola")
        self.obstruction = self.track(world.spawn_actor(obs_bp, self.OBSTRUCTION_SPAWN))
        self.obstruction.apply_control(carla.VehicleControl(hand_brake=True))
        print("Spawned static obstruction.")

        walker_bp = random.choice(bp_lib.filter("walker.pedestrian.*"))
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")
        self.pedestrian = self.track(world.spawn_actor(walker_bp, self.PEDESTRIAN_SPAWN))
        print("Spawned hidden pedestrian.")

    def on_tick(self, ctx: TickContext) -> None:
        if not self._triggered:
            ped_loc = self.pedestrian.get_transform().location
            distance = math.hypot(ctx.ego_x - ped_loc.x, ctx.ego_y - ped_loc.y)
            if distance <= self.TRIGGER_DISTANCE:
                print(f"!!! SUDDEN OBSTACLE TRIGGERED !!! Distance: {distance:.2f}m")
                self._triggered = True

        if self._triggered:
            self.pedestrian.apply_control(carla.WalkerControl(
                direction=carla.Vector3D(x=0.0, y=-1.0, z=0.0),
                speed=self.RUN_SPEED_MPS,
            ))
