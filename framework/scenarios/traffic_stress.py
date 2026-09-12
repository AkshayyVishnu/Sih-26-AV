"""
TrafficStress -- ~300 background vehicles + walkers on autopilot via
CARLA's Traffic Manager, all traffic lights disabled, ego driving toward
a fixed target intersection amid the traffic. Migrated from (not
replacing) Simulation Files/scenario_1.py -- that script is untouched;
this is the same scenario expressed as a framework.base.Scenario
subclass.

Simplified vs. the original script, disclosed here rather than silently:
the original's full argparse surface (--safe/--hybrid/--respawn/
--car-lights-on/--hero/--no-rendering/--asynch/etc.) is dropped in favor
of a few constructor kwargs (number_of_vehicles, number_of_walkers,
seed). Easy to reintroduce a specific one as a constructor kwarg later if
it turns out to matter -- none of those paths are exercised by anything
built on top of this yet.
"""
from __future__ import annotations

import carla
from carla.command import FutureActor, SetAutopilot, SpawnActor
from numpy import random

from framework.base import Scenario, TickContext


def _get_vehicle_blueprints(world: carla.World) -> list:
    return sorted(world.get_blueprint_library().filter("vehicle.*"), key=lambda bp: bp.id)


class TrafficStress(Scenario):
    EGO_SPAWN = carla.Transform(
        carla.Location(x=9.0, y=-77.7, z=1.5),
        carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0),
    )
    FINAL_GOAL = (-13.5, -156.84)  # the "target intersection" -- also aims the spectator camera
    SPECTATOR_TRANSFORM = carla.Transform(
        carla.Location(x=-13.5, y=-156.84, z=20.0),
        carla.Rotation(pitch=-30.0, yaw=54.0, roll=0.0),
    )

    def __init__(self, number_of_vehicles: int = 300, number_of_walkers: int = 10, seed: int = 42, tm_port: int = 8000):
        super().__init__()
        self.number_of_vehicles = number_of_vehicles
        self.number_of_walkers = number_of_walkers
        self.seed = seed
        self.tm_port = tm_port

        # Populated during spawn_actors(); initialized empty up front so
        # cleanup_extra() is safe even if spawn_actors() fails partway
        # through (the original script's finally block could NameError
        # in that case -- fixed here).
        self.all_lights: list = []
        self._walker_controller_ids: list[int] = []
        self._world: carla.World | None = None

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        self._world = world
        random.seed(self.seed)

        traffic_manager = client.get_trafficmanager(self.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(self.seed)
        self.traffic_manager = traffic_manager

        # CARLA's internal Traffic Light Manager needs one tick to finish
        # initializing before we freeze lights, or our commands get
        # silently overwritten.
        world.tick()

        self.all_lights = list(world.get_actors().filter("traffic.traffic_light"))
        print(f"Disabling ALL {len(self.all_lights)} traffic light actors in the map...")
        for tl in self.all_lights:
            tl.set_state(carla.TrafficLightState.Off)
            tl.freeze(True)
        world.tick()  # apply the light-state change

        # --- Background vehicles ---
        blueprints = _get_vehicle_blueprints(world)
        spawn_points = world.get_map().get_spawn_points()
        if self.number_of_vehicles < len(spawn_points):
            random.shuffle(spawn_points)
        n_vehicles = min(self.number_of_vehicles, len(spawn_points))

        batch = []
        for transform in spawn_points[:n_vehicles]:
            if transform.location.distance(self.EGO_SPAWN.location) < 5.0:
                continue  # avoid spawning on top of the ego (ego doesn't exist yet -- use the known spawn point)
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                blueprint.set_attribute("color", random.choice(blueprint.get_attribute("color").recommended_values))
            blueprint.set_attribute("role_name", "autopilot")
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        vehicle_ids = []
        for response in client.apply_batch_sync(batch, True):
            if not response.error:
                vehicle_ids.append(self.track(response.actor_id))

        for actor in world.get_actors(vehicle_ids):
            traffic_manager.ignore_lights_percentage(actor, 100)

        # --- Walkers ---
        walker_bps = list(world.get_blueprint_library().filter("walker.pedestrian.*"))
        walker_spawn_points = []
        for _ in range(self.number_of_walkers):
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                walker_spawn_points.append(carla.Transform(loc))

        batch = [SpawnActor(random.choice(walker_bps), sp) for sp in walker_spawn_points]
        walker_ids = []
        for response in client.apply_batch_sync(batch, True):
            if not response.error:
                walker_ids.append(self.track(response.actor_id))

        controller_bp = world.get_blueprint_library().find("controller.ai.walker")
        batch = [SpawnActor(controller_bp, carla.Transform(), walker_id) for walker_id in walker_ids]
        for response in client.apply_batch_sync(batch, True):
            if not response.error:
                self._walker_controller_ids.append(self.track(response.actor_id))

        world.tick()  # let walkers + controllers physically appear before starting them

        for controller in world.get_actors(self._walker_controller_ids):
            controller.start()
            controller.go_to_location(world.get_random_location_from_navigation())
            controller.set_max_speed(1.4)

        traffic_manager.global_percentage_speed_difference(30.0)
        print(f"Spawned {len(vehicle_ids)} background vehicles and {len(walker_ids)} walkers.")

    def cleanup_extra(self, client: carla.Client) -> None:
        if self._world is not None and self._walker_controller_ids:
            for controller in self._world.get_actors(self._walker_controller_ids):
                controller.stop()

        if self.all_lights:
            print("Restoring all frozen traffic lights...")
            for tl in self.all_lights:
                tl.freeze(False)
