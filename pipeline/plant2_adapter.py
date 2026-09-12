"""
Converts our own tracked-object output (pipeline/tracker.py's TrackedObject
list) into the exact "label_raw" list-of-dicts format PCLA's bundled PlanT2
agent expects as input to its planner.

WHY THIS EXISTS / WHAT WAS INVESTIGATED:
PCLA's documented public API (PCLA(agent, vehicle, route, client) ->
get_action()) does not expose a "feed your own perception" path. Reading
external/PCLA/pcla_agents/plant2/PlanT_agent.py directly (PlanTAgent.run_step)
found the actual injection point:

    label_raw = self.get_bounding_boxes()   # <- ground-truth actor query
    ...
    self.control = self._get_control(label_raw, tick_data)

get_bounding_boxes() (defined in pcla_agents/plant2/carla_garage/data_agent.py)
does nothing but query CARLA's ground truth (self._world.get_actors()) and
convert each actor into an ego-relative dict. It is a clean, self-contained
method with no side effects other than reading the world -- so it can be
replaced on the agent INSTANCE (agent_instance.get_bounding_boxes = ...)
without touching route planning, traffic-light/stop-sign handling, or
control synthesis, all of which stay exactly as PCLA ships them.

The label_raw dict schema (confirmed by reading get_bounding_boxes() +
plant_variables.py + get_input_batch()):
    {
        "class": str,        # must be one of: car, walker, static, static_car,
                              # stop_sign, traffic_light, emergency
                              # (pcla_agents/plant2/plant_variables.py: class_nums)
        "extent": [x, y, z], # HALF-extents in meters (length/2, width/2, height/2)
        "position": [x, y, z],  # ego-relative, meters. x=forward, y=right
                                 # (CARLA's own convention, from
                                 # transfuser_utils.get_relative_transform
                                 # using CARLA's world-matrix transforms --
                                 # NOT this project's "y=left" FusedDetection
                                 # convention, see convert_tracked_to_label_raw)
        "yaw": float,        # ego-relative heading, RADIANS
        "speed": float,      # forward speed, m/s
        "id": int,
        # "type_id" is REQUIRED when class == "car": get_input_batch()'s
        # emergency-vehicle special case does `x["type_id"] in [...]`
        # unconditionally for every "car"-classed dict -- omitting it raises
        # KeyError. Any type_id string not in that list is safe.
        "type_id": str,
    }

WHAT THIS ADAPTER DOES NOT COVER (disclosed limitations, same honesty
standard as MoFlow's stub in predictor.py):

1. Class vocabulary mismatch: PlanT2's ontology (trained on CARLA's own
   actor types) has no "animal"/"auto-rickshaw"/"cow" category -- only
   car/walker/static/static_car/stop_sign/traffic_light/emergency. Our
   YOLO classes (fine-tuned on IDD, which has exactly these unstructured-
   India categories) get best-effort mapped onto this fixed vocabulary
   below (_CLASS_MAP). This is a real information loss: PlanT2 will treat
   a cow and a pedestrian identically once mapped to "walker", and a
   bicycle/auto-rickshaw identically to a car once mapped to "car". This
   was PlanT2's ontology choice, not something this adapter can fix
   without retraining PlanT2 itself.

2. No 3D bounding-box extents: our perception (perception_fusion.py) only
   gives a LiDAR-point-cluster centroid position, not a full 3D box size.
   _DEFAULT_EXTENTS below are hand-picked per-class placeholders (same
   category of simplification as predictor.py's _LATERAL_UNCERTAINTY_M),
   not measured from real data.

3. No heading estimation: our tracker (tracker.py) has no orientation
   filter, only position + velocity. Object yaw is approximated from the
   velocity vector direction when the object is moving above
   _MIN_SPEED_FOR_YAW_MPS; stationary/slow objects default to yaw=0
   (facing the same direction as ego). This is a genuine approximation --
   a parked car facing sideways will be misrepresented until it starts
   moving.

4. Static hazards (traffic lights, stop signs, static props) are
   deliberately NOT emitted by this adapter -- our YOLO model was not
   asked to detect them (out of scope per the fine-tuning plan), and
   PlanTAgent.run_step() already injects real traffic-light/stop-sign
   dicts into label_raw itself (from CARLA ground truth, independent of
   get_bounding_boxes()) AFTER calling the patched method. So this
   comparison run gets: dynamic actors from OUR perception, traffic
   control devices from ground truth, static drivable-area info from our
   own drivable_area.py feeding the OWN A*-based fallback path (not
   PlanT2, which has no drivable-area concept of its own). PlanT2 here is
   only ever used for the dynamic-actor planning decision.

5. NEVER TESTED against a live CARLA/PlanT2 run -- this is a design
   derived from reading the source, not verified against real inference
   output. Sanity-check the very first few ticks' label_raw against a
   printed get_bounding_boxes() ground-truth call (e.g. keep the original
   method around and log both side by side for one warm-up run) before
   trusting the comparison numbers.
"""
from __future__ import annotations

import math

from pipeline.types import EgoState, TrackedObject

# Our class_name (from the fine-tuned YOLO model / IDD+CARLA training,
# lowercased) -> PlanT2's fixed vocabulary. Extend this as the real class
# list from the YOLO model is confirmed -- these are best guesses at
# likely IDD-style class names, not a confirmed exhaustive list.
_CLASS_MAP: dict[str, str] = {
    "car": "car",
    "truck": "car",
    "bus": "car",
    "motorcycle": "car",
    "motorbike": "car",
    "bicycle": "car",
    "auto-rickshaw": "car",
    "autorickshaw": "car",
    "rickshaw": "car",
    "vehicle": "car",
    "ambulance": "emergency",
    "police": "emergency",
    "fire_truck": "emergency",
    "pedestrian": "walker",
    "person": "walker",
    "animal": "walker",
    "cow": "walker",
    "dog": "walker",
    "goat": "walker",
}
_DEFAULT_CLASS = "car"  # unknown class -> treat as a bulky moving hazard
                        # rather than silently dropping it from PlanT2's input

# Half-extents [x=half-length, y=half-width, z=half-height], meters.
# Hand-picked placeholders -- see module docstring point 2.
_DEFAULT_EXTENTS: dict[str, tuple[float, float, float]] = {
    "car": (2.2, 1.0, 0.75),
    "truck": (4.0, 1.3, 1.6),
    "bus": (5.5, 1.4, 1.7),
    "motorcycle": (0.9, 0.4, 0.6),
    "motorbike": (0.9, 0.4, 0.6),
    "bicycle": (0.8, 0.3, 0.55),
    "auto-rickshaw": (1.7, 0.7, 0.9),
    "autorickshaw": (1.7, 0.7, 0.9),
    "rickshaw": (1.7, 0.7, 0.9),
    "ambulance": (2.6, 1.1, 1.1),
    "police": (2.3, 1.0, 0.8),
    "fire_truck": (4.5, 1.3, 1.6),
    "pedestrian": (0.3, 0.3, 0.9),
    "person": (0.3, 0.3, 0.9),
    "animal": (0.9, 0.4, 0.7),
    "cow": (1.1, 0.45, 0.75),
    "dog": (0.45, 0.2, 0.3),
    "goat": (0.6, 0.25, 0.45),
}
_FALLBACK_EXTENT = (1.0, 0.5, 0.75)

_MIN_SPEED_FOR_YAW_MPS = 0.5  # below this, velocity direction is too noisy
                              # (Kalman-filter jitter) to trust as heading

# type_id strings PlanT2's get_input_batch() checks for the emergency
# special-case (pcla_agents/plant2/PlanT_agent.py get_input_batch). Any
# string NOT in this list is safe for a "car"-classed dict.
_EMERGENCY_TYPE_IDS = frozenset({
    "vehicle.dodge.charger_police",
    "vehicle.dodge.charger_police_2020",
    "vehicle.carlamotors.firetruck",
    "vehicle.ford.ambulance",
})


def _world_to_ego_relative_xy(world_x: float, world_y: float, ego: EgoState) -> tuple[float, float]:
    """Inverse of pipeline.py's _local_to_world_xy: world frame -> ego-
    relative frame, x=forward, y=RIGHT (CARLA's own convention, matching
    what transfuser_utils.get_relative_transform produces from CARLA's
    world matrices). NOTE the sign flip on y vs. this project's internal
    FusedDetection convention (x=forward, y=LEFT, see types.py) -- PlanT2
    was trained on CARLA-native data, so its label_raw input must use
    CARLA's y=right convention, not ours. This flip is a design decision
    made from reading source, not verified against a live run -- see
    module docstring point 5.
    """
    dx = world_x - ego.x
    dy = world_y - ego.y
    cos_y, sin_y = math.cos(ego.yaw), math.sin(ego.yaw)
    x_forward = dx * cos_y + dy * sin_y
    y_left = -dx * sin_y + dy * cos_y
    return x_forward, -y_left  # flip left -> right for CARLA/PlanT2 convention


def _map_class(class_name: str) -> str:
    return _CLASS_MAP.get(class_name.lower().strip(), _DEFAULT_CLASS)


def convert_tracked_to_label_raw(tracked: list[TrackedObject], ego: EgoState) -> list[dict]:
    """Builds the label_raw list PlanT2's get_input_batch() expects, from
    our own tracker output. Intended to be returned by a monkey-patched
    get_bounding_boxes() on a live PlanTAgent instance -- see
    run_own_perception_plant2.py for the patch site and why patching this
    one method (rather than bypassing run_step() entirely) keeps route
    planning / traffic-light / stop-sign handling untouched.
    """
    label_raw: list[dict] = []
    for obj in tracked:
        if not obj.position_history:
            continue  # nothing to report yet (shouldn't happen post-tracker, defensive)
        world_x, world_y = obj.position_history[-1]
        x_fwd, y_right = _world_to_ego_relative_xy(world_x, world_y, ego)

        vx, vy = obj.velocity
        speed = math.hypot(vx, vy)
        if speed >= _MIN_SPEED_FOR_YAW_MPS:
            # velocity is in world frame -- rotate into ego-relative heading,
            # same convention flip as the position transform above.
            world_heading = math.atan2(vy, vx)
            yaw = world_heading - ego.yaw
        else:
            yaw = 0.0

        plant_class = _map_class(obj.class_name)
        extent = _DEFAULT_EXTENTS.get(obj.class_name.lower().strip(), _FALLBACK_EXTENT)

        entry = {
            "class": plant_class,
            "extent": list(extent),
            "position": [x_fwd, y_right, 0.0],  # z unknown from our 2D fusion; ground-plane assumption
            "yaw": yaw,
            "speed": speed,
            "id": obj.track_id,
        }
        if plant_class == "car":
            # Required key for this class -- see module docstring. Use a
            # generic, deliberately non-matching type_id unless our own
            # class_name indicates an emergency vehicle (mapped to
            # "emergency" above already, so this branch is effectively
            # always the safe generic case -- kept explicit for clarity).
            entry["type_id"] = "vehicle.generic.unknown"
        label_raw.append(entry)

    return label_raw
