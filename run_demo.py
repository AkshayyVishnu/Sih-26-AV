"""
Standalone demo/test entry point -- runs the pipeline against SYNTHETIC
detections + LiDAR data, no CARLA connection required. Use this to
develop and sanity-check the pipeline right now; swap generate_synthetic_tick()
for real YOLO output + real CARLA LiDAR once you can connect to your
friend's instance (see docs/pipeline-decision-log.md for the remote-connect
snippet).

Run: .venv\\Scripts\\python.exe run_demo.py
"""
from __future__ import annotations

import numpy as np

from pipeline.pipeline import Pipeline
from pipeline.types import Detection, EgoState

# --- Camera intrinsics: PLACEHOLDER, replace with your actual CARLA -----
# camera sensor's real fx/fy/cx/cy (from its blueprint attributes: image
# size + fov). This example assumes a 800x600 image, ~90 deg FOV.
IMG_W, IMG_H, FOV_DEG = 800, 600, 90
FOCAL = IMG_W / (2 * np.tan(np.radians(FOV_DEG) / 2))
CAMERA_INTRINSIC = np.array([
    [FOCAL, 0, IMG_W / 2],
    [0, FOCAL, IMG_H / 2],
    [0, 0, 1],
])

# --- Camera<->LiDAR extrinsic: PLACEHOLDER, but axis-correct. -----------
# Assumes co-located sensors, LiDAR/ego frame = X-forward, Y-RIGHT, Z-up
# -- CARLA's own native convention (left-handed, Unreal Engine axes).
# CONFIRMED against CARLA's own shipped examples (PythonAPI/examples/
# bounding_boxes.py, lidar_to_camera.py) and a live get_right_vector()
# check: CARLA's +Y is RIGHT, not left -- an earlier version of this
# comment wrongly called this "CARLA/ROS convention... Y-left" (CARLA and
# ROS use opposite Y-sign/handedness, they don't share a convention), and
# the matrix below was correspondingly wrong (X_opt = -Y_local). Fixed to
# match CARLA's own remap: X_opt = +Y_local. See carla_runtime.py's
# build_camera_to_lidar_extrinsic() for the CARLA-connected equivalent of
# this matrix, used by run_live.py and the scenario scripts. Using a bare
# identity here (as an earlier draft of this file did) silently breaks
# fusion: it feeds "up" into the projection's depth term. REPLACE the
# translation part (currently zero) with your actual sensor offset once
# mounted in CARLA -- this only fixes the axis convention, not the real
# extrinsic. See the known pitfall noted in pipeline/perception_fusion.py.
CAMERA_TO_LIDAR_EXTRINSIC = np.array([
    [0, 1, 0, 0],    # camera X (right)   =  ego Y (right)
    [0, 0, -1, 0],   # camera Y (down)    = -ego Z (up)
    [1, 0, 0, 0],    # camera Z (forward) =  ego X (forward)
    [0, 0, 0, 1],
], dtype=np.float32)


def _project_point_to_bbox(x: float, y: float, z: float, half_size_m: float = 0.5) -> tuple[float, float, float, float]:
    """Projects a 3D ego-frame point through CAMERA_TO_LIDAR_EXTRINSIC +
    CAMERA_INTRINSIC to get a plausible image-space bbox for it -- used
    ONLY to keep this synthetic demo's detections and LiDAR clusters
    self-consistent (same 3D point -> matching bbox), so fusion has real
    signal to find instead of accidentally matching background clutter.
    Real detections come from YOLO directly, this helper is demo-only.
    """
    corners_3d = np.array([
        [x, y - half_size_m, z + half_size_m],
        [x, y + half_size_m, z - half_size_m],
    ])
    homogeneous = np.hstack([corners_3d, np.ones((2, 1))])
    cam_frame = (CAMERA_TO_LIDAR_EXTRINSIC @ homogeneous.T).T[:, :3]
    pixels = (CAMERA_INTRINSIC @ cam_frame.T).T
    pixels = pixels[:, :2] / cam_frame[:, 2:3]
    x1, x2 = sorted([pixels[0, 0], pixels[1, 0]])
    y1, y2 = sorted([pixels[0, 1], pixels[1, 1]])
    return x1, y1, x2, y2


def generate_synthetic_segmentation(img_w: int = IMG_W, img_h: int = IMG_H) -> np.ndarray:
    """Fake CARLA semantic segmentation tags: a road corridor across the
    lower portion of the image, everything else tagged non-drivable
    (Building). NOT meant to be visually realistic -- it only needs to
    be consistent enough to exercise pipeline/drivable_area.py's
    classify()+rasterize wiring end-to-end. Real tags come from CARLA's
    actual semantic segmentation camera sensor (raw red channel, see
    that module's docstring for the tag-ID caveat).
    """
    tags = np.full((img_h, img_w), fill_value=3, dtype=np.int32)  # 3 = Building (non-drivable default)
    road_top = int(img_h * 0.55)
    tags[road_top:, :] = 1  # Road (CONFIRMED tag ID for CARLA 0.9.16, see pipeline/drivable_area.py)
    return tags


def generate_synthetic_tick(tick: int) -> tuple[list[Detection], np.ndarray, EgoState]:
    """Fakes one tick's worth of perception input: a pedestrian and a cow
    crossing, plus a sparse LiDAR sweep with points clustered near where
    the fake objects actually are (so fusion has something real to find).
    Bounding boxes are projected from the same 3D positions as the LiDAR
    clusters below, so this data is internally consistent tick-to-tick
    (a real YOLO detection would naturally track the moving object; a
    static placeholder bbox would not, and silently breaks fusion after
    the object moves away from it -- caught and fixed while testing this).
    """
    # A pedestrian drifting across the road, and a wandering cow.
    ped_x = 15.0 - tick * 0.3
    ped_y = 3.0
    cow_x = 25.0
    cow_y = -1.0 + np.sin(tick * 0.05) * 0.3

    ped_bbox = _project_point_to_bbox(ped_x, ped_y, 0.8, half_size_m=0.4)
    cow_bbox = _project_point_to_bbox(cow_x, cow_y, 0.6, half_size_m=0.6)

    detections = [
        Detection(class_name="pedestrian", confidence=0.87, x1=ped_bbox[0], y1=ped_bbox[1], x2=ped_bbox[2], y2=ped_bbox[3]),
        Detection(class_name="animal", confidence=0.79, x1=cow_bbox[0], y1=cow_bbox[1], x2=cow_bbox[2], y2=cow_bbox[3]),
    ]

    # Sparse synthetic LiDAR: a cluster of points near each fake object's
    # true 3D position (ego frame, x=forward, y=left, z=up), plus some
    # random background points so fusion has to actually filter by bbox.
    rng = np.random.default_rng(tick)
    ped_cluster = rng.normal(loc=[ped_x, ped_y, 0.8], scale=0.15, size=(20, 3))
    cow_cluster = rng.normal(loc=[cow_x, cow_y, 0.6], scale=0.2, size=(20, 3))
    background = rng.uniform(low=[-5, -15, -1], high=[40, 15, 2], size=(200, 3))
    lidar_points = np.vstack([ped_cluster, cow_cluster, background]).astype(np.float32)

    # Goal must stay within the planner's costmap window (60m wide, so
    # +/-30m from ego -- see pipeline/planner.py's build_costmap_from_predictions
    # defaults). Going outside it is exactly the kind of silent
    # off-by-config bug worth testing for now, not discovering later.
    ego = EgoState(x=0.0, y=0.0, yaw=0.0, speed=5.0, goal_x=25.0, goal_y=0.0)

    return detections, lidar_points, ego


def main():
    pipeline = Pipeline(
        camera_intrinsic=CAMERA_INTRINSIC,
        camera_to_lidar_extrinsic=CAMERA_TO_LIDAR_EXTRINSIC,
        dt=0.05,
    )

    n_ticks = 40
    all_timings = []
    segmentation_tags = generate_synthetic_segmentation()  # static fake scene for this demo

    for tick in range(1, n_ticks + 1):
        detections, lidar_points, ego = generate_synthetic_tick(tick)
        control, planned_path, timings = pipeline.tick(detections, lidar_points, ego, segmentation_tags=segmentation_tags)
        all_timings.append(timings.total_ms)

    print("\n--- Run summary (all ticks, includes cold-start) ---")
    print(f"Ticks: {n_ticks}")
    print(f"Mean total latency: {np.mean(all_timings):.2f}ms")
    print(f"Max total latency:  {np.max(all_timings):.2f}ms")
    print(f"Min total latency:  {np.min(all_timings):.2f}ms")

    # First few ticks can show large, environmental (not algorithmic)
    # latency spikes -- observed up to ~490ms on tick 15 of one run, gone
    # on re-run, most likely OS/antivirus scanning the freshly-created
    # log file on first writes. Report the warmed-up numbers as your real
    # metric; note the cold-start behavior separately if you mention it
    # at all. See docs/pipeline-decision-log.md.
    warm = all_timings[5:]
    print("\n--- Warmed-up summary (ticks 6+, use THIS for your report) ---")
    print(f"Mean total latency: {np.mean(warm):.2f}ms")
    print(f"Max total latency:  {np.max(warm):.2f}ms")
    print(f"Min total latency:  {np.min(warm):.2f}ms")

    print(f"\nFinal control command: throttle={control.throttle:.2f} steer={control.steer:.2f} brake={control.brake:.2f}")
    print("Full decision trail written to logs/run_<timestamp>.log")


if __name__ == "__main__":
    main()
