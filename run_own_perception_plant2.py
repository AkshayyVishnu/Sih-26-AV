"""
Comparison run (i): this project's OWN perception (perception_fusion.py +
tracker.py, real YOLO detections + LiDAR fusion + Kalman tracking) feeding
PCLA's bundled PlanT2 agent's planner, instead of PlanT2's own ground-
truth actor query. PlanT2's route-following, traffic-light, and stop-sign
handling are left completely untouched -- only the dynamic-actor
perception step is replaced. See pipeline/plant2_adapter.py's module
docstring for exactly what was investigated, the injection point found,
and the disclosed limitations (class-vocabulary mismatch, no real 3D
extents, no heading estimation, never tested against a live run).

HOW THE INJECTION WORKS: PCLA's own PCLA() wrapper is used unmodified for
agent construction, route setup, and its own pseudo-sensors (imu,
speedometer, gnss) -- all tested, working code (external/PCLA/PCLA.py).
Immediately after construction, pcla.agent_instance.get_bounding_boxes is
monkey-patched to return label_raw built from OUR tracker's latest output
(via plant2_adapter.convert_tracked_to_label_raw) instead of querying
CARLA's ground truth. Everything else in PlanTAgent.run_step() -- called
via pcla.get_action() exactly as in run_pcla_transfuserv6.py -- runs
unchanged.

Counterpart run: run_pcla_transfuserv6.py (approach ii, fully end-to-end,
no injection). NOT a strictly fair head-to-head against that script --
see plant2_adapter.py point 4 and docs/TRAFFIC_AND_INTEGRATION.md.

BEFORE RUNNING: fill in every CONFIG value below (this file's + reuses
the same camera/LiDAR/YOLO config shape as run_live.py -- copy real
values from there once confirmed). Requires PCLA's environment set up
(external/PCLA/environment.yml + its plant2 checkpoint download -- see
external/PCLA/README.md). NEVER run against a live CARLA server in this
environment -- syntax/import-checked only.

Run: .venv\\Scripts\\python.exe run_own_perception_plant2.py
"""
from __future__ import annotations

import os
import sys
import time

import carla
import numpy as np

_PCLA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "external", "PCLA")
if _PCLA_DIR not in sys.path:
    sys.path.insert(0, _PCLA_DIR)

from PCLA import PCLA  # noqa: E402

from pipeline.metrics import MetricsRecorder  # noqa: E402
from pipeline.perception_fusion import LidarCameraFuser  # noqa: E402
from pipeline.pipeline import TickTimings, _local_to_world_xy  # noqa: E402
from pipeline.plant2_adapter import convert_tracked_to_label_raw  # noqa: E402
from pipeline.traffic_chaos import destroy_chaotic_traffic, spawn_chaotic_traffic  # noqa: E402
from pipeline.tracker import MultiObjectTracker  # noqa: E402
from pipeline.types import EgoState  # noqa: E402
from run_live import (  # noqa: E402  -- reuse run_live.py's already-verified helpers, don't duplicate them
    CAMERA_TO_LIDAR_EXTRINSIC,
    YoloAdapter,
    build_camera_intrinsic,
    carla_image_to_rgb_array,
    carla_lidar_to_xyz,
)

# ============================================================
# CONFIG -- fill these in for your actual setup before running.
# Camera/LiDAR/YOLO values should match whatever's confirmed correct in
# run_live.py's CONFIG section -- copy them over once that file's values
# are filled in for real, don't maintain two independently-guessed sets.
# ============================================================
CARLA_HOST = "localhost"
CARLA_PORT = 2000
FIXED_DELTA_SECONDS = 0.05

MAP_NAME = "REPLACE_WITH_MAP_NAME"           # must match run_pcla_transfuserv6.py's MAP_NAME for a fair comparison
EGO_SPAWN_POINT_INDEX = 0
EGO_VEHICLE_BLUEPRINT = "vehicle.tesla.model3"

YOLO_MODEL_PATH = "REPLACE_WITH_PATH_TO_YOUR_FINE_TUNED_YOLO.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.4
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV_DEG = 800, 600, 90.0
CAMERA_MOUNT = carla.Transform(carla.Location(x=1.5, z=2.4))
LIDAR_MOUNT = carla.Transform(carla.Location(x=1.5, z=2.4))

ROUTE_XML_PATH = os.path.join(_PCLA_DIR, "sample_route.xml")  # REPLACE -- see run_pcla_transfuserv6.py's note
PCLA_PLANT2_AGENT_KEY = "plant2_plant2"  # external/PCLA/agents.json's plant2 entry (only one variant currently)

NUM_BACKGROUND_VEHICLES = 40
NUM_BACKGROUND_WALKERS = 30
TRAFFIC_SEED = 42   # SAME value as run_pcla_transfuserv6.py's TRAFFIC_SEED for a fair scene comparison

SCENARIO_NAME = "own_perception_plant2"
GOAL_REACHED_RADIUS_M = 3.0
MAX_TICKS = 3000
# ============================================================

CAMERA_INTRINSIC = build_camera_intrinsic(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV_DEG)


def main():
    if MAP_NAME.startswith("REPLACE_") or YOLO_MODEL_PATH.startswith("REPLACE_"):
        print("Fill in MAP_NAME, YOLO_MODEL_PATH, EGO_SPAWN_POINT_INDEX, and ROUTE_XML_PATH first.")
        sys.exit(1)

    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(20.0)
    print(f"Client version: {client.get_client_version()}, Server version: {client.get_server_version()}")

    world = client.load_world(MAP_NAME)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)

    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.find(EGO_VEHICLE_BLUEPRINT)
    spawn_points = world.get_map().get_spawn_points()
    ego = world.spawn_actor(ego_bp, spawn_points[EGO_SPAWN_POINT_INDEX])
    world.tick()
    print(f"Spawned ego: {ego.type_id} (id={ego.id})")

    print(f"Spawning background traffic (seed={TRAFFIC_SEED})...")
    traffic_actors = spawn_chaotic_traffic(
        client, world, num_vehicles=NUM_BACKGROUND_VEHICLES,
        num_walkers=NUM_BACKGROUND_WALKERS, seed=TRAFFIC_SEED,
    )

    # Our OWN sensors, separate from whatever PlanT2's PCLA setup attaches
    # (imu/speedometer/gnss -- see plant2_adapter.py's docstring, no
    # conflict spawning both sets on the same vehicle).
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(CAMERA_WIDTH))
    cam_bp.set_attribute("image_size_y", str(CAMERA_HEIGHT))
    cam_bp.set_attribute("fov", str(CAMERA_FOV_DEG))
    camera = world.spawn_actor(cam_bp, CAMERA_MOUNT, attach_to=ego)

    lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
    lidar = world.spawn_actor(lidar_bp, LIDAR_MOUNT, attach_to=ego)
    # No semantic-segmentation camera here (unlike run_live.py) -- this
    # comparison run has no drivable_area.py / own-planner stage for it to
    # feed (see plant2_adapter.py docstring point 4: PlanT2 has no
    # drivable-area input, and this script's dynamic-actor detections go
    # straight into PlanT2 via the monkey-patch below, not our own planner).

    latest = {"rgb": None, "lidar": None}
    camera.listen(lambda img: latest.__setitem__("rgb", img))
    lidar.listen(lambda data: latest.__setitem__("lidar", data))

    metrics = MetricsRecorder(scenario_name=SCENARIO_NAME, dt=FIXED_DELTA_SECONDS)
    collision_bp = bp_lib.find("sensor.other.collision")
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego)
    collision_sensor.listen(lambda event: metrics.record_collision(event.other_actor.type_id))

    print(f"Loading YOLO model from {YOLO_MODEL_PATH}...")
    yolo = YoloAdapter(YOLO_MODEL_PATH, YOLO_CONFIDENCE_THRESHOLD)
    fuser = LidarCameraFuser(CAMERA_INTRINSIC, CAMERA_TO_LIDAR_EXTRINSIC)
    tracker = MultiObjectTracker(dt=FIXED_DELTA_SECONDS)
    # No DrivableAreaEstimator here -- PlanT2 has no drivable-area input of
    # its own (see plant2_adapter.py docstring point 4), so there's nothing
    # for it to feed in this comparison run.

    print(f"Loading PCLA PlanT2 agent '{PCLA_PLANT2_AGENT_KEY}'...")
    pcla = PCLA(PCLA_PLANT2_AGENT_KEY, ego, ROUTE_XML_PATH, client)

    # THE INJECTION POINT. See pipeline/plant2_adapter.py's module
    # docstring for the full investigation. `latest_tracked` is a mutable
    # cell updated each tick before calling pcla.get_action() -- the
    # patched method reads whatever's in it at call time.
    latest_tracked: dict = {"objects": [], "ego": None}

    def _patched_get_bounding_boxes(lidar=None):
        if latest_tracked["ego"] is None:
            return []
        return convert_tracked_to_label_raw(latest_tracked["objects"], latest_tracked["ego"])

    pcla.agent_instance.get_bounding_boxes = _patched_get_bounding_boxes

    goal_transform = spawn_points[EGO_SPAWN_POINT_INDEX]
    try:
        route_wps = pcla.agent_instance._global_plan_world_coord
        if route_wps:
            goal_transform = route_wps[-1][0]
    except AttributeError:
        print("Could not read agent's parsed route for a goal point -- "
              "completion tracking will use the spawn point instead (won't trigger).")

    print(f"Starting closed loop, scenario='{SCENARIO_NAME}'. Ctrl+C to stop.")
    completed, reason = False, "stopped before reaching goal"
    tick_count = 0
    try:
        while tick_count < MAX_TICKS:
            world.tick()
            tick_count += 1

            if latest["rgb"] is None or latest["lidar"] is None:
                continue  # wait for our own sensors' first frame

            rgb_array = carla_image_to_rgb_array(latest["rgb"])
            lidar_xyz = carla_lidar_to_xyz(latest["lidar"])

            transform = ego.get_transform()
            velocity = ego.get_velocity()
            accel = ego.get_acceleration()
            ego_state = EgoState(
                x=transform.location.x, y=transform.location.y,
                yaw=np.radians(transform.rotation.yaw),
                speed=float(np.hypot(velocity.x, velocity.y)),
                goal_x=goal_transform.location.x, goal_y=goal_transform.location.y,
            )

            t0 = time.perf_counter()
            detections = yolo.infer(rgb_array)
            t1 = time.perf_counter()

            fused = fuser.fuse(detections, lidar_xyz)
            for fd in fused:
                if fd.position_3d is not None:
                    wx, wy = _local_to_world_xy(fd.position_3d[0], fd.position_3d[1], ego_state)
                    fd.position_3d = (wx, wy, fd.position_3d[2])
            t2 = time.perf_counter()

            tracked = tracker.step(fused)
            t3 = time.perf_counter()

            latest_tracked["objects"] = tracked
            latest_tracked["ego"] = ego_state

            control = pcla.get_action()
            t4 = time.perf_counter()

            ego.apply_control(control)

            distance_to_goal = float(np.hypot(
                goal_transform.location.x - ego_state.x, goal_transform.location.y - ego_state.y,
            ))

            timings = TickTimings(
                fusion_ms=(t2 - t1) * 1000, tracking_ms=(t3 - t2) * 1000,
                prediction_ms=0.0,       # PlanT2 does its own internal motion reasoning -- no separate
                drivable_area_ms=0.0,    # predictor.py stage in this comparison run
                decision_ms=0.0,         # no decision_logic.py stage -- PlanT2's control synthesis replaces it
                planning_ms=(t4 - t3) * 1000,  # PlanT2's run_step (called inside pcla.get_action())
                control_ms=0.0,
            )
            # fusion_ms above measures fuse()+frame-transform only; YOLO inference
            # (t1-t0) is a real, separate cost not captured by TickTimings' fixed
            # field set -- log it directly rather than mislabeling it as another stage.
            yolo_ms = (t1 - t0) * 1000

            metrics.record_tick(
                tick=tick_count, timings=timings,
                replanned=False, path_valid=True,  # not meaningful for PlanT2 -- see run_pcla_transfuserv6.py's note
                decision_mode="own_perception_plant2",
                speed_mps=ego_state.speed,
                accel_x=accel.x, accel_y=accel.y,
                distance_to_goal_m=distance_to_goal,
            )
            if tick_count % 100 == 0:
                print(f"tick {tick_count}: yolo={yolo_ms:.1f}ms fusion+track={timings.fusion_ms+timings.tracking_ms:.1f}ms "
                      f"plant2={timings.planning_ms:.1f}ms dist_to_goal={distance_to_goal:.1f}m "
                      f"tracked_objs={len(tracked)}")

            if distance_to_goal <= GOAL_REACHED_RADIUS_M:
                completed, reason = True, "reached goal"
                print(f"\nGoal reached (within {GOAL_REACHED_RADIUS_M}m) after {tick_count} ticks.")
                break

        if tick_count >= MAX_TICKS:
            reason = f"hit MAX_TICKS={MAX_TICKS} without reaching goal"

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")
        reason = "manually stopped"
    finally:
        metrics.finalize(completed=completed, reason=reason)
        camera.stop(); camera.destroy()
        lidar.stop(); lidar.destroy()
        collision_sensor.stop(); collision_sensor.destroy()
        destroy_chaotic_traffic(client, world, traffic_actors)
        pcla.cleanup()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        tm.set_synchronous_mode(False)
        print("Cleaned up: own sensors, PCLA agent, ego vehicle, background traffic, synchronous mode disabled.")


if __name__ == "__main__":
    main()
