"""
Orchestrator: ties perception_fusion -> tracker -> predictor -> planner
into one tick() call, logging every stage's latency and decisions --
this is what produces the "replanning latency" metric data directly.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import numpy as np

from pipeline.logging_utils import get_logger
from pipeline.perception_fusion import LidarCameraFuser
from pipeline.planner import Planner
from pipeline.predictor import ConstantVelocityPredictor, Predictor
from pipeline.tracker import MultiObjectTracker
from pipeline.types import Detection, EgoState, PlannedPath

logger = logging.getLogger("pipeline.orchestrator")


@dataclass
class TickTimings:
    fusion_ms: float
    tracking_ms: float
    prediction_ms: float
    planning_ms: float

    @property
    def total_ms(self) -> float:
        return self.fusion_ms + self.tracking_ms + self.prediction_ms + self.planning_ms


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
        self.planner = Planner()
        self.dt = dt
        self._tick_count = 0

        logger.info("Pipeline initialized. dt=%.3fs, predictor=%s", dt, type(self.predictor).__name__)

    def tick(
        self,
        detections: list[Detection],
        lidar_points_xyz: np.ndarray,
        ego: EgoState,
    ) -> tuple[PlannedPath, TickTimings]:
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

        planned = self.planner.plan(ego, predictions)
        t4 = time.perf_counter()

        timings = TickTimings(
            fusion_ms=(t1 - t0) * 1000,
            tracking_ms=(t2 - t1) * 1000,
            prediction_ms=(t3 - t2) * 1000,
            planning_ms=(t4 - t3) * 1000,
        )

        logger.info(
            "Tick %d done: fusion=%.2fms track=%.2fms predict=%.2fms plan=%.2fms TOTAL=%.2fms | "
            "%d detections -> %d tracked -> %d predicted modes | path_valid=%s replanned=%s",
            self._tick_count, timings.fusion_ms, timings.tracking_ms, timings.prediction_ms,
            timings.planning_ms, timings.total_ms,
            len(detections), len(tracked), len(predictions), planned.is_valid, planned.replanned,
        )

        if timings.total_ms > 150:
            logger.warning(
                "Tick %d total latency %.2fms EXCEEDS the ~150-200ms closed-loop success-collapse "
                "threshold documented in docs/architecture.md -- investigate before relying on this tick's timing.",
                self._tick_count, timings.total_ms,
            )

        return planned, timings
