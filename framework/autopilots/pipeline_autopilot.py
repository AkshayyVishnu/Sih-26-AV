"""
PipelineAutopilot -- wraps pipeline/Pipeline (perception fusion -> tracker
-> predictor -> drivable-area -> decision logic -> planner -> controller)
+ carla_runtime.GroundTruthDetector behind the framework.base.Autopilot
interface. This is today's exact driving behavior (previously inlined
directly in Simulation Files/scenario_1.py/scenario_2.py's tick loops),
reused via import -- nothing about pipeline/'s or GroundTruthDetector's
own logic is reimplemented here.
"""
from __future__ import annotations

import numpy as np

from carla_runtime import GroundTruthDetector
from framework.base import Autopilot, TickContext
from pipeline.controller import PurePursuitController
from pipeline.pipeline import Pipeline
from pipeline.types import ControlCommand, EgoState


class PipelineAutopilot(Autopilot):
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
        # client/ego_vehicle/route_xml_path are unused here -- this
        # autopilot drives entirely off pipeline/ + GroundTruthDetector,
        # no PCLA/route involved. Accepted (not just tolerated via
        # **kwargs) so the parameter list stays self-documenting about
        # what ScenarioRunner.run() always passes -- see
        # framework/base.py's Autopilot.setup() docstring.
        self.pipeline = Pipeline(
            camera_intrinsic=camera_intrinsic,
            camera_to_lidar_extrinsic=camera_to_lidar_extrinsic,
            dt=dt,
        )
        self.pipeline.controller = PurePursuitController(wheelbase_m=wheelbase_m)
        self.detector = GroundTruthDetector(
            camera_actor=ego_camera,
            image_width=image_width,
            image_height=image_height,
            camera_intrinsic=camera_intrinsic,
        )
        self._last_detections = []
        self._last_planned_waypoints: list[tuple[float, float]] = []
        self._last_timings = None

    def compute(self, ctx: TickContext, goal_xy: tuple[float, float]) -> ControlCommand:
        detections = self.detector.detect(ctx.world, ctx.ego_vehicle)
        ego_state = EgoState(ctx.ego_x, ctx.ego_y, ctx.ego_yaw, ctx.ego_speed, *goal_xy)

        control, planned, timings = self.pipeline.tick(
            detections, ctx.lidar_xyz, ego_state, segmentation_tags=ctx.seg_tags,
        )

        self._last_detections = detections
        self._last_planned_waypoints = planned.waypoints
        self._last_timings = timings
        return control

    def debug_info(self) -> dict:
        return {
            "detections": self._last_detections,
            "planned_waypoints": self._last_planned_waypoints,
            "timings": self._last_timings,
        }
