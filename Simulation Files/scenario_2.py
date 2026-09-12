#!/usr/bin/env python3

import os
import sys

import carla
import time
import math
import random
import numpy as np

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
# Ego vehicle starting position
EGO_SPAWN = {
    'x': 20.0,
    'y': 134.0,
    'z': 1.5,
    'pitch': 0.0,
    'yaw': 0.0,
    'roll': 0.0
}

# Static obstruction position
OBSTRUCTION_SPAWN = {
    'x': 67.0,
    'y': 137.0,
    'z': 1.5,
    'pitch': 0.0,
    'yaw': 0.0,
    'roll': 0.0
}

# Pedestrian spawn (placed immediately in front of the obstruction to hide them)
PEDESTRIAN_SPAWN = {
    'x': 72.0,      # 5 meters in front of the obstruction
    'y': 137.0,     # Same lane as the obstruction
    'z': 1.5,
    'pitch': 0.0,
    'yaw': 270.0,   # Facing the ego vehicle's lane (Negative Y)
    'roll': 0.0
}

# Distance (in meters) at which the pedestrian jumps out
TRIGGER_DISTANCE = 15.0

# Spectator camera position and rotation
CAMERA_SPAWN = {
    'x': 80.0,
    'y': 125.0,
    'z': 6.0,
    'pitch': -30.0,
    'yaw': 150.0,
    'roll': 0.0
}

# Ego sensor rig -- SAME mount position/resolution/FOV as run_live.py's
# CAMERA_MOUNT/LIDAR_MOUNT/SEG_CAMERA_MOUNT and CAMERA_WIDTH/HEIGHT/FOV_DEG,
# so pipeline/'s CAMERA_INTRINSIC + CAMERA_TO_LIDAR_EXTRINSIC (built for that
# exact config) stay valid if this scenario gets wired into the pipeline later.
SENSOR_CAMERA_WIDTH = 800
SENSOR_CAMERA_HEIGHT = 600
SENSOR_CAMERA_FOV_DEG = 90.0
SENSOR_MOUNT = carla.Transform(carla.Location(x=1.5, z=2.4))

# Final destination for the pipeline-driven ego: straight down the same
# lane, well past OBSTRUCTION_SPAWN/PEDESTRIAN_SPAWN (x=67-72, y=137),
# consistent with EGO_SPAWN's yaw=0 (+X-facing) heading. Adjust if your
# actual road geometry curves before this point.
FINAL_GOAL_X = 110.0
FINAL_GOAL_Y = 137.0

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


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    actor_list = []
    ego_sensors = []  # Initialize here so the finally block can access it safely
    tick_latencies_ms = []  # Initialize here so the finally block can access it safely
    dashboard = None  # Initialize here so the finally block can access it safely

    try:
        print("Loading Town03...")
        world = client.load_world('Town03')
        
        # 1. Set Synchronous Mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05 # 20 FPS
        world.apply_settings(settings)
        
        bp_lib = world.get_blueprint_library()

        # ---------------------------------------------------
        # 2. Spawn the Static Obstruction Vehicle
        # ---------------------------------------------------
        obs_bp = bp_lib.find('vehicle.carlamotors.carlacola') 
        obs_transform = carla.Transform(
            carla.Location(x=OBSTRUCTION_SPAWN['x'], y=OBSTRUCTION_SPAWN['y'], z=OBSTRUCTION_SPAWN['z']),
            carla.Rotation(pitch=OBSTRUCTION_SPAWN['pitch'], yaw=OBSTRUCTION_SPAWN['yaw'], roll=OBSTRUCTION_SPAWN['roll'])
        )
        obs_vehicle = world.spawn_actor(obs_bp, obs_transform)
        actor_list.append(obs_vehicle)
        obs_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
        print("Spawned static obstruction.")

        # ---------------------------------------------------
        # 3. Spawn the Pedestrian (Hidden)
        # ---------------------------------------------------
        walker_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')

        ped_transform = carla.Transform(
            carla.Location(x=PEDESTRIAN_SPAWN['x'], y=PEDESTRIAN_SPAWN['y'], z=PEDESTRIAN_SPAWN['z']),
            carla.Rotation(pitch=PEDESTRIAN_SPAWN['pitch'], yaw=PEDESTRIAN_SPAWN['yaw'], roll=PEDESTRIAN_SPAWN['roll'])
        )
        pedestrian = world.spawn_actor(walker_bp, ped_transform)
        actor_list.append(pedestrian)
        print("Spawned hidden pedestrian.")

        # ---------------------------------------------------
        # 4. Spawn the Ego Vehicle
        # ---------------------------------------------------
        ego_bp = bp_lib.find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name', 'ego')
        
        ego_transform = carla.Transform(
            carla.Location(x=EGO_SPAWN['x'], y=EGO_SPAWN['y'], z=EGO_SPAWN['z']),
            carla.Rotation(pitch=EGO_SPAWN['pitch'], yaw=EGO_SPAWN['yaw'], roll=EGO_SPAWN['roll'])
        )
        ego_vehicle = world.spawn_actor(ego_bp, ego_transform)
        actor_list.append(ego_vehicle)
        print("Spawned Ego vehicle.")

        # ---------------------------------------------------
        # 4b. Attach Ego Sensors
        # ---------------------------------------------------
        ego_sensors, ego_latest = spawn_ego_sensors(
            world, ego_vehicle, SENSOR_MOUNT, SENSOR_CAMERA_WIDTH, SENSOR_CAMERA_HEIGHT, SENSOR_CAMERA_FOV_DEG,
        )
        ego_camera = ego_sensors[0]  # RGB camera actor, needed by GroundTruthDetector's own projection

        # ---------------------------------------------------
        # 4c. Build the pipeline/ stack for this scenario
        # ---------------------------------------------------
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

        # ---------------------------------------------------
        # 5. Set Spectator Camera
        # ---------------------------------------------------
        spectator = world.get_spectator()
        camera_transform = carla.Transform(
            carla.Location(x=CAMERA_SPAWN['x'], y=CAMERA_SPAWN['y'], z=CAMERA_SPAWN['z']), 
            carla.Rotation(pitch=CAMERA_SPAWN['pitch'], yaw=CAMERA_SPAWN['yaw'], roll=CAMERA_SPAWN['roll'])
        )
        spectator.set_transform(camera_transform)
        print("Spectator camera positioned.")

        # Force a tick so the actors physically appear in the world
        world.tick()
        print("Simulation is ready. Starting control loop...")

        # ---------------------------------------------------
        # 6. Main Control Loop
        # ---------------------------------------------------
        pedestrian_triggered = False

        while True:
            # Step the simulation forward by 0.05 seconds
            world.tick()
            time.sleep(0.050)

            # --- SCRIPTED PEDESTRIAN LOGIC ---
            if not pedestrian_triggered:
                # Calculate distance between Ego and Pedestrian
                ego_loc = ego_vehicle.get_transform().location
                ped_loc = pedestrian.get_transform().location
                distance = math.sqrt((ego_loc.x - ped_loc.x)**2 + (ego_loc.y - ped_loc.y)**2)

                # Trigger condition
                if distance <= TRIGGER_DISTANCE:
                    print(f"!!! SUDDEN OBSTACLE TRIGGERED !!! Distance: {distance:.2f}m")
                    pedestrian_triggered = True

            if pedestrian_triggered:
                # Force the pedestrian to run straight across the road (towards negative Y)
                ped_control = carla.WalkerControl()
                ped_control.direction = carla.Vector3D(x=0.0, y=-1.0, z=0.0)
                ped_control.speed = 3.5 # Running speed in m/s
                pedestrian.apply_control(ped_control)
            
            # ====================================================
            # >>> ALGORITHM: pipeline/ closed loop <<<
            # ====================================================
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
            # ====================================================

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
        
    finally:
        if dashboard is not None:
            dashboard.close()

        # Restore world settings to normal asynchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if ego_sensors:
            print(f'\nStopping and destroying {len(ego_sensors)} ego sensor(s)...')
            for sensor in ego_sensors:
                sensor.stop()
            client.apply_batch([carla.command.DestroyActor(x) for x in ego_sensors])

        print(f'\nDestroying {len(actor_list)} actors...')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        time.sleep(0.5)

        if tick_latencies_ms:
            warm = tick_latencies_ms[5:] or tick_latencies_ms  # skip cold-start ticks if we have enough
            print(f"\n--- pipeline.tick() latency summary ({len(tick_latencies_ms)} ticks, "
                  f"warmed-up mean of last {len(warm)}) ---")
            print(f"Mean: {np.mean(warm):.2f}ms  Max: {np.max(warm):.2f}ms  Min: {np.min(warm):.2f}ms")
            print("Full per-tick decision trail: logs/run_<timestamp>.log")

        print('Done.')

if __name__ == '__main__':
    main()