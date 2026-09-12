"""
Real CARLA entry point -- connects to a live CARLA server, uses your
existing fine-tuned YOLO model for detection, and pulls LiDAR + semantic
segmentation directly from CARLA's ground truth (no separate model
needed for those two -- see pipeline/drivable_area.py for why ground
truth is used deliberately, not just for convenience).

BEFORE RUNNING: fill in every value in the CONFIG section below for your
actual setup. Nothing here will produce meaningful output with the
defaults -- they're placeholders, not working values.

Run: .venv\\Scripts\\python.exe run_live.py
"""
from __future__ import annotations

import sys

import carla
import numpy as np

from carla_runtime import (
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
from pipeline.metrics import MetricsRecorder
from pipeline.pipeline import Pipeline
from pipeline.types import Detection

# ============================================================
# CONFIG -- fill these in for your actual setup before running
# ============================================================
CARLA_HOST = "localhost"   # "localhost" if this script runs on the same machine as the CARLA server
CARLA_PORT = 2000
FIXED_DELTA_SECONDS = 0.05  # sim step size -- CARLA docs' own recommendation is synchronous mode + fixed step
                             # for a "slow external client" like this pipeline (see docs/architecture.md)

YOLO_MODEL_PATH = "REPLACE_WITH_PATH_TO_YOUR_FINE_TUNED_YOLO.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.4

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
CAMERA_FOV_DEG = 90.0
# Sensor mount position relative to the vehicle (CARLA convention:
# x=forward, y=right, z=up from the vehicle's origin). REPLACE with your
# actual sensor rig's real mount position -- these values change the
# math in pipeline/perception_fusion.py and pipeline/drivable_area.py,
# they are not cosmetic.
CAMERA_MOUNT = carla.Transform(carla.Location(x=1.5, z=2.4))
LIDAR_MOUNT = carla.Transform(carla.Location(x=1.5, z=2.4))       # co-located with camera by default
SEG_CAMERA_MOUNT = CAMERA_MOUNT                                    # co-located with RGB camera by default
# If your rig mounts these separately, give each its own Transform AND
# compute a separate extrinsic per sensor pair instead of reusing
# CAMERA_TO_LIDAR_EXTRINSIC below for both.

VEHICLE_WHEELBASE_M = 2.8   # REPLACE with your ego vehicle blueprint's real wheelbase

# CARLA WORLD-FRAME destination coordinates (NOT "meters ahead of spawn" --
# see the frame-consistency note below). Get real values from whoever
# built your scene (e.g. a spawn point or route waypoint on the map).
GOAL_X, GOAL_Y = 0.0, 0.0   # REPLACE -- (0,0) will almost certainly be wrong for a real map

SCENARIO_NAME = "unnamed_scenario"  # REPLACE per run, e.g. "village_road", "cattle_crossing" --
                                     # metrics_export.py groups/aggregates runs by this name
GOAL_REACHED_RADIUS_M = 3.0          # how close counts as "reached the goal" for completion tracking
# ============================================================


# Camera intrinsic + camera<->LiDAR extrinsic now live in carla_runtime.py
# (shared with the scenario scripts) -- see that module's
# build_camera_to_lidar_extrinsic() for the CARLA-axis-convention fix
# this matrix received (an earlier version here had a sign bug: it used
# X_optical = -Y_local, but CARLA's own examples confirm X_optical =
# +Y_local). This assumes camera and LiDAR are co-located (same mount
# transform) -- see the CONFIG note above if that's not true for your rig.
CAMERA_TO_LIDAR_EXTRINSIC = build_camera_to_lidar_extrinsic()
CAMERA_INTRINSIC = build_camera_intrinsic(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV_DEG)


class YoloAdapter:
    """Wraps your existing fine-tuned YOLO model, converting its output
    into pipeline.types.Detection objects (class, confidence, x1,y1,x2,y2
    -- the format this whole pipeline was built against). Written
    assuming a standard Ultralytics .pt checkpoint; adjust infer() if
    your actual model's calling convention differs.
    """

    def __init__(self, model_path: str, confidence_threshold: float = 0.4):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold

    def infer(self, rgb_array: np.ndarray) -> list[Detection]:
        results = self.model.predict(rgb_array, verbose=False, conf=self.confidence_threshold)
        detections = []
        for r in results:
            for box in r.boxes:
                cls_name = self.model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append(Detection(cls_name, conf, x1, y1, x2, y2))
        return detections


def main():
    if YOLO_MODEL_PATH.startswith("REPLACE_"):
        print("Fill in YOLO_MODEL_PATH and the other CONFIG values at the top of this file first.")
        sys.exit(1)

    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    print(f"Client version: {client.get_client_version()}, Server version: {client.get_server_version()}")
    world = client.get_world()

    # Synchronous mode + fixed timestep -- see FIXED_DELTA_SECONDS comment above.
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)

    vehicles = world.get_actors().filter("vehicle.*")
    if len(vehicles) == 0:
        print("No vehicle found in the world -- spawn/select your ego vehicle first "
              "(coordinate with whoever built the scene).")
        sys.exit(1)
    ego = vehicles[0]
    print(f"Using ego vehicle: {ego.type_id} (id={ego.id})")

    bp_lib = world.get_blueprint_library()

    # NOTE: spawn_ego_sensors() assumes all three sensors share one mount
    # (CAMERA_MOUNT below). If your rig needs CAMERA_MOUNT/LIDAR_MOUNT/
    # SEG_CAMERA_MOUNT to differ (see the CONFIG note above), spawn them
    # individually instead of via this helper.
    sensors, latest = spawn_ego_sensors(world, ego, CAMERA_MOUNT, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV_DEG)
    camera, lidar, seg_camera = sensors

    collision_bp = bp_lib.find("sensor.other.collision")
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego)

    metrics = MetricsRecorder(scenario_name=SCENARIO_NAME, dt=FIXED_DELTA_SECONDS)
    collision_sensor.listen(lambda event: metrics.record_collision(event.other_actor.type_id))

    print(f"Loading YOLO model from {YOLO_MODEL_PATH}...")
    yolo = YoloAdapter(YOLO_MODEL_PATH, YOLO_CONFIDENCE_THRESHOLD)

    pipeline = Pipeline(
        camera_intrinsic=CAMERA_INTRINSIC,
        camera_to_lidar_extrinsic=CAMERA_TO_LIDAR_EXTRINSIC,
        dt=FIXED_DELTA_SECONDS,
    )
    pipeline.controller = PurePursuitController(wheelbase_m=VEHICLE_WHEELBASE_M)

    print(f"Starting closed loop, scenario='{SCENARIO_NAME}'. Ctrl+C to stop.")
    completed, reason = False, "stopped before reaching goal"
    try:
        tick_count = 0
        while True:
            world.tick()
            tick_count += 1

            if latest["rgb"] is None or latest["lidar"] is None or latest["seg"] is None:
                continue  # wait for the first frame from all three sensors

            rgb_array = carla_image_to_rgb_array(latest["rgb"])
            lidar_xyz = carla_lidar_to_xyz(latest["lidar"])
            seg_tags = carla_segmentation_to_tags(latest["seg"])

            detections = yolo.infer(rgb_array)

            accel = ego.get_acceleration()
            # NOTE: GOAL_X/GOAL_Y above must be CARLA WORLD-frame
            # coordinates (see the CONFIG note), and pipeline.py
            # transforms LiDAR-derived positions into this same world
            # frame internally before tracking/planning.
            #
            # compute_local_goal() clamps the real goal to within the
            # planner's 60x60m costmap window (re-centered on the ego
            # every tick) -- passing GOAL_X/GOAL_Y directly would make
            # the planner silently fail to find a path for any goal more
            # than ~30m away. See carla_runtime.compute_local_goal()'s
            # docstring.
            local_goal_x, local_goal_y = compute_local_goal(
                ego.get_transform().location.x, ego.get_transform().location.y, GOAL_X, GOAL_Y,
            )
            ego_state = build_ego_state(ego, local_goal_x, local_goal_y)

            control, planned, timings = pipeline.tick(
                detections, lidar_xyz, ego_state, segmentation_tags=seg_tags,
            )

            ego.apply_control(carla.VehicleControl(
                throttle=control.throttle, steer=control.steer, brake=control.brake,
            ))

            distance_to_goal = float(np.hypot(GOAL_X - ego_state.x, GOAL_Y - ego_state.y))
            metrics.record_tick(
                tick=tick_count, timings=timings,
                replanned=planned.replanned, path_valid=planned.is_valid,
                decision_mode=pipeline.decision_logic.mode.name,
                speed_mps=ego_state.speed,
                accel_x=accel.x, accel_y=accel.y,
                distance_to_goal_m=distance_to_goal,
            )

            if distance_to_goal <= GOAL_REACHED_RADIUS_M:
                completed, reason = True, "reached goal"
                print(f"\nGoal reached (within {GOAL_REACHED_RADIUS_M}m) after {tick_count} ticks.")
                break

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")
        reason = "manually stopped"
    finally:
        metrics.finalize(completed=completed, reason=reason)
        camera.stop(); camera.destroy()
        lidar.stop(); lidar.destroy()
        seg_camera.stop(); seg_camera.destroy()
        collision_sensor.stop(); collision_sensor.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        tm.set_synchronous_mode(False)
        print("Sensors cleaned up, synchronous mode disabled.")


if __name__ == "__main__":
    main()
