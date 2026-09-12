"""
Shared CARLA-glue helpers used by run_live.py AND by the scenario
scripts under `Simulation Files/` -- pulled into one place so both stop
duplicating the same sensor-conversion/projection math, and so a fix
made once (e.g. the extrinsic sign fix below) doesn't have to be
re-applied in three files.

DELIBERATELY separate from pipeline/: everything in pipeline/ stays
100% CARLA-free (see pipeline/pipeline.py's docstring and
docs/pipeline-decision-log.md #1) so run_demo.py keeps working with zero
CARLA dependency. This module is the boundary -- it imports `carla` and
hands pipeline/ plain numpy arrays + pipeline.types objects.

Requires the `carla` package to be importable (see SETUP.md / COMMANDS.md
-- on this machine that's the `carla_env` conda environment).
"""
from __future__ import annotations

import numpy as np

import carla

from pipeline.types import Detection, EgoState


# ============================================================
# Camera intrinsic / camera<->LiDAR extrinsic
# ============================================================

def build_camera_intrinsic(width: int, height: int, fov_deg: float) -> np.ndarray:
    """3x3 K matrix from a CARLA sensor.camera.rgb blueprint's own
    image_size_x/image_size_y/fov attributes. Formula CONFIRMED to match
    CARLA's own shipped examples exactly (PythonAPI/examples/
    bounding_boxes.py's build_projection_matrix(), lidar_to_camera.py) --
    f = width / (2 * tan(fov/2)).
    """
    focal = width / (2 * np.tan(np.radians(fov_deg) / 2))
    return np.array([
        [focal, 0, width / 2],
        [0, focal, height / 2],
        [0, 0, 1],
    ])


def build_camera_to_lidar_extrinsic() -> np.ndarray:
    """4x4 homogeneous transform, ego/LiDAR-local frame -> camera OPTICAL
    frame, for a camera+LiDAR co-located at the same mount (zero relative
    rotation between them).

    CORRECTED matrix -- see docs/pipeline-decision-log.md and the plan
    that added this module for the full story. An earlier version of
    this matrix (still present, unfixed, in run_demo.py's inline copy
    -- see that file's own note) used X_optical = -Y_local, along with a
    comment wrongly calling CARLA's LiDAR/ego frame convention
    "CARLA/ROS... Y-left". Both were wrong: CARLA's native frame is
    LEFT-handed with +Y = RIGHT (Unreal Engine convention), not ROS's
    right-handed +Y = left. Live-verified via
    carla.Transform(yaw=0).get_right_vector() == (0, 1, 0), and confirmed
    against CARLA's own PythonAPI/examples/bounding_boxes.py and
    lidar_to_camera.py, both of which apply the exact remap used below:
    (x, y, z)_local -> (y, -z, x)_optical, i.e.
        X_optical = +Y_local   (right)
        Y_optical = -Z_local   (down)
        Z_optical = +X_local   (forward/depth)
    If you ever see fused/projected positions that look horizontally
    mirrored relative to what's actually in view, this is the first
    place to check.
    """
    return np.array([
        [0, 1, 0, 0],
        [0, 0, -1, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)


# ============================================================
# Raw CARLA sensor buffer -> numpy array conversions
# ============================================================

def carla_image_to_rgb_array(image) -> np.ndarray:
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return arr[:, :, :3][:, :, ::-1]  # BGRA -> RGB


def carla_lidar_to_xyz(lidar_data) -> np.ndarray:
    points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape((-1, 4))
    return points[:, :3]  # drop intensity


def carla_segmentation_to_tags(image) -> np.ndarray:
    # Raw tag lives in the red channel; CARLA's raw buffer is BGRA order,
    # so red is index 2. Do NOT call image.convert(CityScapesPalette)
    # upstream of this -- that destroys the raw tag values.
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return arr[:, :, 2].astype(np.int32)


# ============================================================
# Ego sensor rig
# ============================================================

def spawn_ego_sensors(
    world,
    ego_vehicle,
    mount: carla.Transform,
    width: int = 800,
    height: int = 600,
    fov_deg: float = 90.0,
):
    """Attaches an RGB camera, LiDAR, and semantic-segmentation camera to
    ego_vehicle, all co-located at `mount` -- the exact three sensors
    pipeline/pipeline.py's tick() consumes (fusion needs camera+LiDAR,
    drivable_area.py needs the segmentation camera, see SETUP.md).

    Returns (sensors, latest):
      sensors -- list of the 3 spawned sensor actors. Call .stop() then
        destroy them on shutdown -- they don't get cleaned up
        automatically just because ego_vehicle gets destroyed.
      latest -- dict updated in-place by each sensor's listen() callback:
        latest['rgb'] / ['lidar'] / ['seg'] hold the most recent raw
        CARLA sensor data (None until that sensor's first frame
        arrives). Feed these through carla_image_to_rgb_array() /
        carla_lidar_to_xyz() / carla_segmentation_to_tags() above.
    """
    bp_lib = world.get_blueprint_library()

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(width))
    cam_bp.set_attribute("image_size_y", str(height))
    cam_bp.set_attribute("fov", str(fov_deg))
    camera = world.spawn_actor(cam_bp, mount, attach_to=ego_vehicle)

    lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
    lidar = world.spawn_actor(lidar_bp, mount, attach_to=ego_vehicle)

    seg_bp = bp_lib.find("sensor.camera.semantic_segmentation")
    seg_bp.set_attribute("image_size_x", str(width))
    seg_bp.set_attribute("image_size_y", str(height))
    seg_bp.set_attribute("fov", str(fov_deg))
    seg_camera = world.spawn_actor(seg_bp, mount, attach_to=ego_vehicle)

    latest = {"rgb": None, "lidar": None, "seg": None}
    camera.listen(lambda img: latest.__setitem__("rgb", img))
    lidar.listen(lambda data: latest.__setitem__("lidar", data))
    seg_camera.listen(lambda img: latest.__setitem__("seg", img))

    print(f"Ego sensors attached: RGB camera, LiDAR, semantic segmentation camera "
          f"({width}x{height}, {fov_deg} deg FOV).")
    return [camera, lidar, seg_camera], latest


# ============================================================
# Ground-truth detector (stands in for YOLO -- see the plan's
# "Perception" decision: use CARLA ground truth, same disclosed-
# shortcut pattern pipeline/drivable_area.py already uses for
# segmentation, until a real fine-tuned checkpoint exists)
# ============================================================

_CLASS_MAP_PREFIXES = (
    ("vehicle.", "car"),
    ("walker.pedestrian.", "pedestrian"),
)

# role_name -> class_name overrides, checked BEFORE the type_id-prefix
# table above. Exists specifically for actors that need a class_name the
# underlying CARLA blueprint can't express on its own -- e.g. no
# livestock blueprint exists in stock CARLA, so a scenario standing in a
# walker for a cow (see framework/ps_scenarios/cattle_crossing.py) spawns
# it with role_name="livestock" to get it classified as "animal" instead
# of "pedestrian". This is what makes pipeline/decision_logic.py's
# ANIMAL_ON_ROAD branch (checks class_name in ("animal", "cow")) reachable
# at all -- before this, nothing ever produced that class_name. Confirmed
# non-colliding against every role_name already in use in this repo:
# "ego" (the ego vehicle itself, set in framework/base.py's
# ScenarioRunner), "autopilot" (framework/scenarios/traffic_stress.py's
# background vehicles), "chaotic_traffic" (pipeline/traffic_chaos.py's
# spawns). Purely additive -- any actor that doesn't set role_name (or
# sets some other value) falls through to the existing type_id table
# unchanged.
_ROLE_NAME_OVERRIDES = {
    "livestock": "animal",
}


def _classify_actor(actor) -> str | None:
    # actor.attributes is a CARLA-native mapping, not a plain dict --
    # confirmed (via external/PCLA's own data_agent.py) that bracket
    # indexing works; .get() is not confirmed to exist on this type, so
    # use "in" + indexing rather than assume dict-like .get().
    if "role_name" in actor.attributes and actor.attributes["role_name"] in _ROLE_NAME_OVERRIDES:
        return _ROLE_NAME_OVERRIDES[actor.attributes["role_name"]]
    for prefix, class_name in _CLASS_MAP_PREFIXES:
        if actor.type_id.startswith(prefix):
            return class_name
    return None


class GroundTruthDetector:
    """Fabricates YOLO-shaped Detection objects (2D pixel bboxes) from
    CARLA's own ground-truth actor state, instead of running a real
    detector. Reuses CARLA's own canonical world-point -> pixel
    projection pattern (matches PythonAPI/examples/bounding_boxes.py's
    build_projection_matrix()/get_image_point() exactly) -- this uses
    the camera actor's LIVE pose (privileged ground truth), which is a
    different projection path than pipeline/perception_fusion.py's
    fixed, calibrated camera_to_lidar_extrinsic (which projects real
    LiDAR points, not ground truth). That's intentional: one stands in
    for "the real world", the other is "the calibrated sensor rig" --
    same as a real system would have both.
    """

    def __init__(
        self,
        camera_actor,
        image_width: int,
        image_height: int,
        camera_intrinsic: np.ndarray,
        max_range_m: float = 50.0,
    ):
        self.camera = camera_actor
        self.width = image_width
        self.height = image_height
        self.K = camera_intrinsic
        self.max_range_m = max_range_m

    def detect(self, world, ego_vehicle) -> list[Detection]:
        world_to_camera = np.array(self.camera.get_transform().get_inverse_matrix())
        camera_loc = self.camera.get_transform().location

        detections: list[Detection] = []
        actors = list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("walker.pedestrian.*"))

        for actor in actors:
            if actor.id == ego_vehicle.id:
                continue

            class_name = _classify_actor(actor)
            if class_name is None:
                continue

            actor_loc = actor.get_transform().location
            if actor_loc.distance(camera_loc) > self.max_range_m:
                continue

            corners_world = actor.bounding_box.get_world_vertices(actor.get_transform())

            pixels = []
            any_in_front = False
            for corner in corners_world:
                point = np.array([corner.x, corner.y, corner.z, 1.0])
                point_camera_local = world_to_camera @ point  # world -> camera-local (UE4 axes)
                # UE4 local (x=forward, y=right, z=up) -> camera optical
                # (x=right, y=down, z=forward/depth) -- CARLA's own remap,
                # same one build_camera_to_lidar_extrinsic() encodes above.
                x_opt, y_opt, z_opt = point_camera_local[1], -point_camera_local[2], point_camera_local[0]
                if z_opt <= 0.1:
                    continue  # behind (or right at) the camera plane
                any_in_front = True
                pixel = self.K @ np.array([x_opt, y_opt, z_opt])
                pixels.append((pixel[0] / pixel[2], pixel[1] / pixel[2]))

            if not any_in_front or not pixels:
                continue

            xs = [p[0] for p in pixels]
            ys = [p[1] for p in pixels]
            x1, x2 = max(min(xs), 0.0), min(max(xs), float(self.width))
            y1, y2 = max(min(ys), 0.0), min(max(ys), float(self.height))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue  # degenerate (fully clipped / edge-on) box

            detections.append(Detection(class_name=class_name, confidence=1.0, x1=x1, y1=y1, x2=x2, y2=y2))

        return detections


# ============================================================
# Ego state / goal helpers
# ============================================================

def build_ego_state(ego_vehicle, goal_x: float, goal_y: float) -> EgoState:
    transform = ego_vehicle.get_transform()
    velocity = ego_vehicle.get_velocity()
    return EgoState(
        x=transform.location.x,
        y=transform.location.y,
        yaw=np.radians(transform.rotation.yaw),
        speed=float(np.hypot(velocity.x, velocity.y)),
        goal_x=goal_x,
        goal_y=goal_y,
    )


def compute_local_goal(
    ego_x: float,
    ego_y: float,
    final_goal_x: float,
    final_goal_y: float,
    max_radius_m: float = 25.0,
) -> tuple[float, float]:
    """REQUIRED, not cosmetic -- see docs/pipeline-decision-log.md #4 and
    pipeline/planner.py's build_costmap_from_predictions(): the planner's
    costmap is a 60x60m window RE-CENTERED ON THE EGO EVERY TICK, so a
    goal more than ~30m away is always outside it -- A* would silently
    return is_valid=False (no feasible path) every tick until the ego
    happened to already be within range of the real destination.

    This clamps the vector from the ego's current position toward the
    real final goal to max_radius_m, handing the planner a receding-
    horizon local goal that's always in range. The caller is responsible
    for tracking the real final goal separately (e.g. for "reached
    destination" logic) -- this function only ever returns a point
    between the ego and that final goal, never past it.

    KNOWN LIMITATION: straight-line clamping, not route-aware. Fine for
    a fairly straight road segment (both current scenarios), but will
    cut corners through buildings/off-road on a curved route -- if that
    matters later, replace with successive carla.Map.get_waypoint()-based
    route waypoints instead of a straight line to the final goal.
    """
    dx = final_goal_x - ego_x
    dy = final_goal_y - ego_y
    dist = float(np.hypot(dx, dy))
    if dist <= max_radius_m or dist < 1e-6:
        return final_goal_x, final_goal_y
    scale = max_radius_m / dist
    return ego_x + dx * scale, ego_y + dy * scale
