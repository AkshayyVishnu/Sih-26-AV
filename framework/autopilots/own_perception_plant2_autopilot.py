"""
OwnPerceptionPlanT2Autopilot -- this project's OWN perception (the exact
same carla_runtime.GroundTruthDetector + pipeline/perception_fusion.py +
pipeline/tracker.py stack PipelineAutopilot uses, for a fair
apples-to-apples comparison) feeding PCLA's bundled PlanT2 agent's
planner, instead of PlanT2's own ground-truth actor query. PlanT2's
route-following, traffic-light, and stop-sign handling are left
completely untouched -- only the dynamic-actor perception step is
replaced. See pipeline/plant2_adapter.py's module docstring for the full
investigation of PlanT2's actual model interface, the injection point
found, and its disclosed limitations (class-vocabulary mismatch, no real
3D extents, no heading estimation, never tested against a live run).

Registered as "own_perception_plant2" in framework/run_scenario.py.
Counterpart: pcla_transfuser_autopilot.py's Transfuserv6Autopilot (fully
end-to-end, no injection) -- NOT a strictly fair head-to-head against
that one (PlanT2 is object-level/planning-only, TransFuser v6 is
sensor-based end-to-end); the meaningful comparison this autopilot
actually enables is against "pipeline" (PipelineAutopilot) -- same
perception, different planner (this project's own A* vs. PlanT2).

HOW THE INJECTION WORKS: PCLA's own PCLA() wrapper is used unmodified for
agent construction, route setup, and PlanT2's own pseudo-sensors
(imu/speedometer/gnss) -- all tested, working code
(external/PCLA/PCLA.py). Immediately after construction,
pcla.agent_instance.get_bounding_boxes is monkey-patched to return
label_raw built from OUR tracker's latest output (via
plant2_adapter.convert_tracked_to_label_raw) instead of querying CARLA's
ground truth. Everything else in PlanTAgent.run_step() -- called via
pcla.get_action() each tick -- runs unchanged.

REQUIRES: same PCLA setup as pcla_transfuser_autopilot.py (see that
file's docstring), plus the plant2 checkpoint specifically
(download_weights.py pulls this too).

NEVER run against a live CARLA server in this environment -- syntax/
import-checked only.
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np

_PCLA_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "external", "PCLA",
)
if _PCLA_DIR not in sys.path:
    sys.path.insert(0, _PCLA_DIR)

from PCLA import PCLA  # noqa: E402

from carla_runtime import GroundTruthDetector  # noqa: E402
from framework.base import Autopilot, TickContext  # noqa: E402
from pipeline.perception_fusion import LidarCameraFuser  # noqa: E402
from pipeline.pipeline import _local_to_world_xy  # noqa: E402  -- reuse the already-verified frame transform,
                                                                  # don't reimplement it (see pipeline/pipeline.py's
                                                                  # own docstring for why this transform is required)
from pipeline.plant2_adapter import convert_tracked_to_label_raw  # noqa: E402
from pipeline.tracker import MultiObjectTracker  # noqa: E402
from pipeline.types import ControlCommand, EgoState  # noqa: E402

_PLANT2_AGENT_KEY = "plant2_plant2"  # external/PCLA/agents.json's plant2 entry (currently one variant)


class OwnPerceptionPlanT2Autopilot(Autopilot):
    def __init__(self):
        self.pcla = None
        self.detector: GroundTruthDetector | None = None
        self.fuser: LidarCameraFuser | None = None
        self.tracker: MultiObjectTracker | None = None
        self._latest_tracked: list = []
        self._latest_ego: EgoState | None = None
        self._last_detections: list = []
        self._last_timings_ms = {"yolo_ms": 0.0, "fusion_track_ms": 0.0, "plant2_ms": 0.0}

    def setup(
        self,
        *,
        ego_camera,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        image_width: int,
        image_height: int,
        wheelbase_m: float,
        dt: float,
        client=None,
        ego_vehicle=None,
        route_xml_path: str | None = None,
    ) -> None:
        if client is None or ego_vehicle is None or route_xml_path is None:
            raise RuntimeError(
                "OwnPerceptionPlanT2Autopilot requires client/ego_vehicle/route_xml_path -- "
                "these are supplied automatically by ScenarioRunner.run(). Seeing this means "
                "something called setup() directly instead of going through ScenarioRunner."
            )

        # Same perception PipelineAutopilot uses -- GroundTruthDetector
        # stands in for a real YOLO checkpoint (none exists yet on this
        # machine, see HANDOFF.md), using the SAME ego_camera actor
        # ScenarioRunner already spawned. Using the identical perception
        # source as PipelineAutopilot is deliberate: it isolates the
        # planner-choice variable (own A* vs. PlanT2) for a fair
        # comparison, rather than also changing the perception input.
        self.detector = GroundTruthDetector(
            camera_actor=ego_camera,
            image_width=image_width,
            image_height=image_height,
            camera_intrinsic=camera_intrinsic,
        )
        self.fuser = LidarCameraFuser(camera_intrinsic, camera_to_lidar_extrinsic)
        self.tracker = MultiObjectTracker(dt=dt)

        print(f"Loading PCLA agent '{_PLANT2_AGENT_KEY}' (this can take a while for a cold model load)...")
        self.pcla = PCLA(_PLANT2_AGENT_KEY, ego_vehicle, route_xml_path, client)

        # THE INJECTION POINT -- see module docstring / pipeline/plant2_adapter.py.
        def _patched_get_bounding_boxes(lidar=None):
            if self._latest_ego is None:
                return []
            return convert_tracked_to_label_raw(self._latest_tracked, self._latest_ego)

        self.pcla.agent_instance.get_bounding_boxes = _patched_get_bounding_boxes
        print(f"PCLA agent '{_PLANT2_AGENT_KEY}' ready, get_bounding_boxes patched to use our own perception.")

    def compute(self, ctx: TickContext, goal_xy: tuple[float, float]) -> ControlCommand:
        ego_state = EgoState(ctx.ego_x, ctx.ego_y, ctx.ego_yaw, ctx.ego_speed, *goal_xy)

        t0 = time.perf_counter()
        detections = self.detector.detect(ctx.world, ctx.ego_vehicle)
        t1 = time.perf_counter()

        fused = self.fuser.fuse(detections, ctx.lidar_xyz)
        for fd in fused:
            if fd.position_3d is not None:
                wx, wy = _local_to_world_xy(fd.position_3d[0], fd.position_3d[1], ego_state)
                fd.position_3d = (wx, wy, fd.position_3d[2])
        tracked = self.tracker.step(fused)
        t2 = time.perf_counter()

        # Update the state the monkey-patched get_bounding_boxes reads --
        # must happen BEFORE calling get_action() below, which triggers
        # PlanT2's run_step() -> our patched method synchronously.
        self._latest_tracked = tracked
        self._latest_ego = ego_state

        vehicle_control = self.pcla.get_action()
        t3 = time.perf_counter()

        self._last_detections = detections
        self._last_timings_ms = {
            "yolo_ms": (t1 - t0) * 1000,           # "yolo" naming kept for parity with a real-YOLO swap-in later;
            "fusion_track_ms": (t2 - t1) * 1000,   # today this is GroundTruthDetector.detect()'s cost
            "plant2_ms": (t3 - t2) * 1000,
        }

        return ControlCommand(
            throttle=vehicle_control.throttle,
            steer=vehicle_control.steer,
            brake=vehicle_control.brake,
        )

    def debug_info(self) -> dict:
        total_ms = sum(self._last_timings_ms.values())
        return {
            "detections": self._last_detections,
            # No 'planned_waypoints' -- PlanT2's predicted path (pred_path/
            # pred_wps in PlanT_agent.py's _get_control()) is a local
            # variable, not exposed through PCLA's public API; reaching
            # into it was judged not worth the fragility (same reasoning
            # as pcla_transfuser_autopilot.py's debug_info()).
            "timings": total_ms,
        }

    def cleanup(self) -> None:
        """See pcla_transfuser_autopilot.py's cleanup() docstring --
        identical reasoning: PCLA.cleanup() destroys the ego vehicle and
        its own sensors; ScenarioRunner's own teardown afterward is safe,
        deliberate redundancy, not a conflict.
        """
        if self.pcla is not None:
            self.pcla.cleanup()
            self.pcla = None
