#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import os
import sys

import carla
from carla import VehicleLightState as vls
from carla.command import SpawnActor, SetAutopilot, FutureActor, DestroyActor

import argparse
import logging
import numpy as np
from numpy import random
import time

# Make the repo root importable (this script lives in "Simulation Files/",
# one level below it) so `carla_runtime` and `pipeline` can be imported
# regardless of which directory this script is run from.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_runtime import (
    GroundTruthDetector,
    build_camera_intrinsic,
    build_camera_to_lidar_extrinsic,
    build_ego_state,
    carla_image_to_rgb_array,
    carla_lidar_to_xyz,
    carla_segmentation_to_tags,
    compute_local_goal,
    spawn_ego_sensors,
)
from pipeline.controller import PurePursuitController
from pipeline.pipeline import Pipeline

# ==============================================================================
# -- GLOBAL CONFIGURATION ------------------------------------------------------
# ==============================================================================
# Set this to a specific integer (e.g., 42) for repeatable, deterministic runs. 
GLOBAL_RANDOM_SEED = 42

# Global variable to decide how many background traffic vehicles are spawned
GLOBAL_NUMBER_OF_VEHICLES = 300

# Global Ego Vehicle Spawn Transform (x, y, z, pitch, yaw, roll)
GLOBAL_EGO_SPAWN = {
    'x': 9.0,
    'y': -77.7,
    'z': 1.5,       # (0.5 + 1.0)
    'pitch': 0.0,
    'yaw': 270.0,
    'roll': 0.0
}

# The spectator camera will teleport directly above these coordinates
GLOBAL_TARGET_INTERSECTION = {
    'x': -13.5,
    'y': -156.84
}

# Ego sensor rig -- SAME mount position/resolution/FOV as run_live.py's
# CAMERA_MOUNT/LIDAR_MOUNT/SEG_CAMERA_MOUNT and CAMERA_WIDTH/HEIGHT/FOV_DEG,
# so pipeline/'s CAMERA_INTRINSIC + CAMERA_TO_LIDAR_EXTRINSIC (built for that
# exact config) stay valid if this scenario gets wired into the pipeline later.
SENSOR_CAMERA_WIDTH = 800
SENSOR_CAMERA_HEIGHT = 600
SENSOR_CAMERA_FOV_DEG = 90.0
SENSOR_MOUNT = carla.Transform(carla.Location(x=1.5, z=2.4))

# Final destination for the pipeline-driven ego: the scenario's own
# already-named point of interest (see GLOBAL_TARGET_INTERSECTION above)
# -- previously only used to aim the spectator camera, now doubles as the
# driving goal.
FINAL_GOAL_X = GLOBAL_TARGET_INTERSECTION['x']
FINAL_GOAL_Y = GLOBAL_TARGET_INTERSECTION['y']

VEHICLE_WHEELBASE_M = 2.8  # vehicle.tesla.model3's approx real wheelbase

# Live debug dashboard (viz.py): RGB+detections / segmentation / LiDAR BEV
# / control-over-time graph, one tiled pygame window. Needs `pygame` and a
# display (DISPLAY env var + a reachable X server) -- set False to run
# headless. Conditional import below means pygame isn't required at all
# when this is off.
ENABLE_VISUALIZATION = True
VIZ_UPDATE_EVERY_N_TICKS = 1  # raise this if the dashboard becomes a bottleneck
# ==============================================================================

if ENABLE_VISUALIZATION:
    from viz import Dashboard


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)
    if generation.lower() == "all":
        return bps
    if len(bps) == 1:
        return bps
    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=GLOBAL_NUMBER_OF_VEHICLES, type=int, help=f'Number of vehicles (default: {GLOBAL_NUMBER_OF_VEHICLES})')
    argparser.add_argument('-w', '--number-of-walkers', metavar='W', default=10, type=int, help='Number of walkers (default: 10)')
    argparser.add_argument('--safe', action='store_true', help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument('--filterv', metavar='PATTERN', default='vehicle.*', help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument('--generationv', metavar='G', default='All', help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument('--filterw', metavar='PATTERN', default='walker.pedestrian.*', help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument('--generationw', metavar='G', default='2', help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument('--tm-port', metavar='P', default=8000, type=int, help='Port to communicate with TM (default: 8000)')
    argparser.add_argument('--asynch', action='store_true', help='Activate asynchronous mode execution')
    argparser.add_argument('--hybrid', action='store_true', help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument('-s', '--seed', metavar='S', type=int, default=GLOBAL_RANDOM_SEED, help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument('--seedw', metavar='S', default=GLOBAL_RANDOM_SEED, type=int, help='Set the seed for pedestrians module')
    argparser.add_argument('--car-lights-on', action='store_true', default=False, help='Enable automatic car light management')
    argparser.add_argument('--hero', action='store_true', default=False, help='Set one of the vehicles as hero')
    argparser.add_argument('--respawn', action='store_true', default=False, help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument('--no-rendering', action='store_true', default=False, help='Activate no rendering mode')

    args = argparser.parse_args()
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    all_lights = [] # Initialize here so the finally block can access it safely
    ego_sensors = [] # Initialize here so the finally block can access it safely
    tick_latencies_ms = [] # Initialize here so the finally block can access it safely
    dashboard = None # Initialize here so the finally block can access it safely

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        print("Loading Town03...")
        world = client.load_world('Town03') 

        # ==============================================================================
        # -- 1. CONFIGURE SYNC MODE & FORCE TICK FIRST ---------------------------------
        # ==============================================================================
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05  # Locks the simulation to 20 FPS
            else:
                synchronous_master = False
        if args.no_rendering:
            settings.no_rendering_mode = True
        
        world.apply_settings(settings)

        # CRITICAL FIX: We must tick the world *once* so CARLA's internal C++ 
        # Traffic Light Manager finishes its initialization. If we freeze them 
        # before this tick, CARLA just overwrites our commands immediately.
        if not args.asynch and synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()

        # ==============================================================================
        # -- 2. DISABLE ALL TRAFFIC LIGHTS & MOVE SPECTATOR -----------------------------
        # ==============================================================================
        all_lights = world.get_actors().filter("traffic.traffic_light")
        print(f"\nDisabling ALL {len(all_lights)} traffic light actors in the map...")

        for tl in all_lights:
            tl.set_state(carla.TrafficLightState.Off)
            tl.freeze(True) # Now this will permanently stick

            # Draw a red debug point above each dead signal for visual confirmation
            pos = tl.get_transform().location
            world.debug.draw_point(
                pos + carla.Location(z=3.5),
                size=0.3,
                color=carla.Color(255, 0, 0),
                life_time=3600.0
            )

        # Tick the world one more time to apply the visual changes instantly
        if not args.asynch and synchronous_master:
            world.tick()

        # Spawn spectator camera directly above the target coordinates
        target_x = GLOBAL_TARGET_INTERSECTION['x']
        target_y = GLOBAL_TARGET_INTERSECTION['y']
        
        spectator = world.get_spectator()
        spectator.set_transform(
            carla.Transform(
                carla.Location(x=target_x, y=target_y, z=20.0),
                carla.Rotation(pitch=-30, yaw=54, roll=0)
            )
        )
        print(f"All lights disabled! Spectator camera moved above (X: {target_x}, Y: {target_y}).")

        # ==============================================================================
        # -- 3. SPAWN EGO VEHICLE ------------------------------------------------------
        # ==============================================================================
        bp_lib = world.get_blueprint_library()
        ego_bp = bp_lib.find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')

        ego_spawn_point = carla.Transform(
            carla.Location(x=GLOBAL_EGO_SPAWN['x'], y=GLOBAL_EGO_SPAWN['y'], z=GLOBAL_EGO_SPAWN['z']), 
            carla.Rotation(pitch=GLOBAL_EGO_SPAWN['pitch'], yaw=GLOBAL_EGO_SPAWN['yaw'], roll=GLOBAL_EGO_SPAWN['roll'])
        )

        ego_vehicle = world.spawn_actor(ego_bp, ego_spawn_point)
        print(f"\nEgo vehicle spawned successfully at (x={GLOBAL_EGO_SPAWN['x']}, y={GLOBAL_EGO_SPAWN['y']})!")
        vehicles_list.append(ego_vehicle.id)
        
        # Ensure traffic manager completely ignores lights and ego behavior for this vehicle
        traffic_manager.ignore_lights_percentage(ego_vehicle, 100)

        # ==============================================================================
        # -- 3b. ATTACH EGO SENSORS -----------------------------------------------------
        # ==============================================================================
        ego_sensors, ego_latest = spawn_ego_sensors(
            world, ego_vehicle, SENSOR_MOUNT, SENSOR_CAMERA_WIDTH, SENSOR_CAMERA_HEIGHT, SENSOR_CAMERA_FOV_DEG,
        )
        ego_camera = ego_sensors[0]  # RGB camera actor, needed by GroundTruthDetector's own projection

        # ==============================================================================
        # -- 3c. BUILD THE pipeline/ STACK FOR THIS SCENARIO ----------------------------
        # ==============================================================================
        camera_intrinsic = build_camera_intrinsic(SENSOR_CAMERA_WIDTH, SENSOR_CAMERA_HEIGHT, SENSOR_CAMERA_FOV_DEG)
        camera_to_lidar_extrinsic = build_camera_to_lidar_extrinsic()

        av_pipeline = Pipeline(
            camera_intrinsic=camera_intrinsic,
            camera_to_lidar_extrinsic=camera_to_lidar_extrinsic,
            dt=0.05,
        )
        av_pipeline.controller = PurePursuitController(wheelbase_m=VEHICLE_WHEELBASE_M)

        detector = GroundTruthDetector(
            camera_actor=ego_camera,
            image_width=SENSOR_CAMERA_WIDTH,
            image_height=SENSOR_CAMERA_HEIGHT,
            camera_intrinsic=camera_intrinsic,
        )

        dashboard = Dashboard() if ENABLE_VISUALIZATION else None
        viz_active = ENABLE_VISUALIZATION
        tick_count = 0

        # ==============================================================================
        # -- 4. SPAWN BACKGROUND TRAFFIC -----------------------------------------------
        # ==============================================================================
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            args.number_of_vehicles = number_of_spawn_points

        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
                
            # Prevent spawning on top of ego
            if transform.location.distance(ego_spawn_point.location) < 5.0:
                continue

            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # Ensure background traffic ignores the dark traffic lights so they don't stop forever
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            traffic_manager.ignore_lights_percentage(actor, 100)
            if args.car_lights_on and actor.id != ego_vehicle.id:
                traffic_manager.update_vehicle_lights(actor, True)

        # ==============================================================================
        # -- 5. SPAWN WALKERS ----------------------------------------------------------
        # ==============================================================================
        percentagePedestriansRunning = 0.0
        percentagePedestriansCrossing = 0.0
        
        if args.seedw is not None:
            world.set_pedestrians_seed(args.seedw)
            random.seed(args.seedw)
            
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            probability = random.randint(0,100 + 1)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            if walker_bp.has_attribute('can_use_wheelchair') and probability < 11:
                walker_bp.set_attribute('use_wheelchair', 'true')
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
            
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2

        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id

        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            all_actors[i].start()
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print(f'Spawned {len(vehicles_list)-1} background vehicles and {len(walkers_list)} walkers. Press Ctrl+C to exit.')

        traffic_manager.global_percentage_speed_difference(30.0)

        # ==============================================================================
        # -- 6. MAIN SIMULATION LOOP ---------------------------------------------------
        # ==============================================================================
        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

            # ---------------------------------------------------
            # ALGORITHM: pipeline/ closed loop (drives the ego through
            # the ~300-vehicle background traffic toward
            # GLOBAL_TARGET_INTERSECTION)
            # ---------------------------------------------------
            if ego_latest["rgb"] is None or ego_latest["lidar"] is None or ego_latest["seg"] is None:
                # First few ticks: sensors haven't produced a frame yet --
                # brake-and-hold rather than drive blind (same pattern
                # run_live.py uses).
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            else:
                tick_count += 1
                lidar_xyz = carla_lidar_to_xyz(ego_latest["lidar"])
                seg_tags = carla_segmentation_to_tags(ego_latest["seg"])
                detections = detector.detect(world, ego_vehicle)

                ego_loc = ego_vehicle.get_transform().location
                local_goal_x, local_goal_y = compute_local_goal(ego_loc.x, ego_loc.y, FINAL_GOAL_X, FINAL_GOAL_Y)
                ego_state = build_ego_state(ego_vehicle, local_goal_x, local_goal_y)

                control, planned, timings = av_pipeline.tick(detections, lidar_xyz, ego_state, segmentation_tags=seg_tags)
                tick_latencies_ms.append(timings.total_ms)

                ego_vehicle.apply_control(carla.VehicleControl(
                    throttle=control.throttle, steer=control.steer, brake=control.brake,
                ))

                if dashboard is not None and viz_active and tick_count % VIZ_UPDATE_EVERY_N_TICKS == 0:
                    rgb_array = carla_image_to_rgb_array(ego_latest["rgb"])
                    sim_time_s = world.get_snapshot().timestamp.elapsed_seconds
                    viz_active = dashboard.update(
                        rgb_array, seg_tags, lidar_xyz, detections, control,
                        planned.waypoints, ego_state, sim_time_s,
                    )

    finally:
        if dashboard is not None:
            dashboard.close()

        # Restore Traffic Lights
        if all_lights:
            print('\nRestoring all frozen traffic lights...')
            for tl in all_lights:
                tl.freeze(False)

        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        if ego_sensors:
            print(f'\nStopping and destroying {len(ego_sensors)} ego sensor(s)...')
            for sensor in ego_sensors:
                sensor.stop()
            client.apply_batch([DestroyActor(x) for x in ego_sensors])

        print(f'\nDestroying {len(vehicles_list)} vehicles (including ego)...')
        client.apply_batch([DestroyActor(x) for x in vehicles_list])

        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print(f'Destroying {len(walkers_list)} walkers...')
        client.apply_batch([DestroyActor(x) for x in all_id])

        time.sleep(0.5)

        if tick_latencies_ms:
            warm = tick_latencies_ms[5:] or tick_latencies_ms  # skip cold-start ticks if we have enough
            print(f"\n--- pipeline.tick() latency summary ({len(tick_latencies_ms)} ticks, "
                  f"warmed-up mean of last {len(warm)}) ---")
            print(f"Mean: {np.mean(warm):.2f}ms  Max: {np.max(warm):.2f}ms  Min: {np.min(warm):.2f}ms")
            print("Full per-tick decision trail: logs/run_<timestamp>.log")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone.')
