"""
Orchestrator: ties perception_fusion -> tracker -> predictor ->
drivable_area -> decision_logic -> planner -> controller into one
tick() call, logging every stage's latency and decisions -- this is
what produces the "replanning latency" metric data directly.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import numpy as np

from pipeline.controller import PurePursuitController
from pipeline.decision_logic import DecisionLogic
from pipeline.drivable_area import DrivableAreaEstimator
from pipeline.logging_utils import get_logger
from pipeline.perception_fusion import LidarCameraFuser
from pipeline.planner import Planner
from pipeline.predictor import ConstantVelocityPredictor, Predictor
from pipeline.tracker import MultiObjectTracker
from pipeline.types import ControlCommand, Detection, EgoState, PlannedPath

logger = logging.getLogger("pipeline.orchestrator")


@dataclass
class TickTimings:
    fusion_ms: float
    tracking_ms: float
    prediction_ms: float
    drivable_area_ms: float
    decision_ms: float
    planning_ms: float
    control_ms: float

    @property
    def total_ms(self) -> float:
        return (
            self.fusion_ms + self.tracking_ms + self.prediction_ms
            + self.drivable_area_ms + self.decision_ms + self.planning_ms + self.control_ms
        )


class Pipeline:
    def __init__(
        self,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        dt: float = 0.05,
        predictor: Predictor | None = None,
    ):
        get_logger("pipeline")  # sets up file+console logging for the whole run, once

        self.fuser = LidarCameraFuser(camera_intrinsic, camera_to_lidar_extrinsic)
        self.tracker = MultiObjectTracker(dt=dt)
        self.predictor = predictor or ConstantVelocityPredictor()
        self.drivable_area = DrivableAreaEstimator(camera_intrinsic, camera_to_lidar_extrinsic)
        self.decision_logic = DecisionLogic()
        self.planner = Planner()
        self.controller = PurePursuitController()
        self.dt = dt
        self._tick_count = 0

        logger.info("Pipeline initialized. dt=%.3fs, predictor=%s", dt, type(self.predictor).__name__)

    def tick(
        self,
        detections: list[Detection],
        lidar_points_xyz: np.ndarray,
        ego: EgoState,
        segmentation_tags: np.ndarray | None = None,
    ) -> tuple[ControlCommand, PlannedPath, TickTimings]:
        """segmentation_tags: optional (H, W) raw CARLA semantic tag
        image, same frame as the RGB detection camera. If omitted, the
        planner runs on obstacle-avoidance costs only, with NO drivable-
        area signal -- see pipeline/drivable_area.py for why that matters
        specifically for unmarked roads. Pass it whenever you have it.
        """
        self._tick_count += 1
        logger.debug("--- Tick %d start --- ego=(%.2f, %.2f, yaw=%.2f) goal=(%.2f, %.2f)",
                      self._tick_count, ego.x, ego.y, ego.yaw, ego.goal_x, ego.goal_y)

        t0 = time.perf_counter()
        fused = self.fuser.fuse(detections, lidar_points_xyz)
        t1 = time.perf_counter()

        tracked = self.tracker.step(fused)
        t2 = time.perf_counter()

        predictions = self.predictor.predict(tracked, self.dt)
        t3 = time.perf_counter()

        non_drivable_xy: list[tuple[float, float]] = []
        if segmentation_tags is not None:
            _, non_drivable_xy = self.drivable_area.classify(lidar_points_xyz, segmentation_tags)
        else:
            logger.warning("Tick %d: no segmentation_tags provided -- planning without a drivable-area signal.",
                            self._tick_count)
        t3b = time.perf_counter()

        decision = self.decision_logic.step(ego.speed, tracked, predictions)
        t3c = time.perf_counter()

        planned = self.planner.plan(
            ego, predictions,
            non_drivable_points=non_drivable_xy,
            force_replan=decision.replan_requested,
        )
        t4 = time.perf_counter()

        control = self.controller.compute(ego, planned)
        if decision.emergency_brake_active:
            # Decision layer overrides throttle/brake, but keeps the
            # controller's steering so the vehicle still tracks the path
            # (or swerves per the planner's obstacle avoidance) while
            # braking, rather than just locking straight-line.
            logger.warning("Tick %d: EMERGENCY_BRAKE active -- overriding throttle/brake (steer preserved).",
                            self._tick_count)
            control = ControlCommand(throttle=0.0, steer=control.steer, brake=1.0)
        t5 = time.perf_counter()

        timings = TickTimings(
            fusion_ms=(t1 - t0) * 1000,
            tracking_ms=(t2 - t1) * 1000,
            prediction_ms=(t3 - t2) * 1000,
            drivable_area_ms=(t3b - t3) * 1000,
            decision_ms=(t3c - t3b) * 1000,
            planning_ms=(t4 - t3c) * 1000,
            control_ms=(t5 - t4) * 1000,
        )

        logger.info(
            "Tick %d done: fusion=%.2fms track=%.2fms predict=%.2fms drivable_area=%.2fms decision=%.2fms "
            "plan=%.2fms control=%.2fms TOTAL=%.2fms | %d det -> %d tracked -> %d pred modes, %d non-drivable pts | "
            "mode=%s path_valid=%s replanned=%s | throttle=%.2f steer=%.2f brake=%.2f",
            self._tick_count, timings.fusion_ms, timings.tracking_ms, timings.prediction_ms,
            timings.drivable_area_ms, timings.decision_ms, timings.planning_ms, timings.control_ms, timings.total_ms,
            len(detections), len(tracked), len(predictions), len(non_drivable_xy),
            decision.mode.name, planned.is_valid, planned.replanned,
            control.throttle, control.steer, control.brake,
        )

        if timings.total_ms > 150:
            logger.warning(
                "Tick %d total latency %.2fms EXCEEDS the ~150-200ms closed-loop success-collapse "
                "threshold documented in docs/architecture.md -- investigate before relying on this tick's timing.",
                self._tick_count, timings.total_ms,
            )

        return control, planned, timings
