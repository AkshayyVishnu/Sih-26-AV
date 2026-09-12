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

from pipeline.drivable_area import DrivableAreaEstimator
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
    drivable_area_ms: float
    planning_ms: float

    @property
    def total_ms(self) -> float:
        return self.fusion_ms + self.tracking_ms + self.prediction_ms + self.drivable_area_ms + self.planning_ms


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
        self.planner = Planner()
        self.dt = dt
        self._tick_count = 0

        logger.info("Pipeline initialized. dt=%.3fs, predictor=%s", dt, type(self.predictor).__name__)

    def tick(
        self,
        detections: list[Detection],
        lidar_points_xyz: np.ndarray,
        ego: EgoState,
        segmentation_tags: np.ndarray | None = None,
    ) -> tuple[PlannedPath, TickTimings]:
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

        planned = self.planner.plan(ego, predictions, non_drivable_points=non_drivable_xy)
        t4 = time.perf_counter()

        # --- Populate nearest-obstacle fields for Stateflow decision logic (Plan A/B) ---
        nearest_class = ""
        nearest_conf = 0.0
        nearest_dist = float("inf")
        nearest_ttc = float("inf")

        if fused:  # find the nearest fused detection by distance
            nearest_fd = min(fused, key=lambda fd: fd.distance_m if fd.distance_m is not None else float("inf"))
            if nearest_fd.position_3d is not None and nearest_fd.distance_m is not None:
                nearest_class = nearest_fd.detection.class_name
                nearest_conf = nearest_fd.detection.confidence
                nearest_dist = nearest_fd.distance_m
                # TTC = distance / closing_speed
                # closing_speed = (ego_speed_vec - obstacle_velocity) dot (obstacle_direction)
                # Simplification: use relative speed along ego heading
                ego_speed_ms = ego.speed
                if tracked:  # use velocity from tracked object if available
                    nearest_track = next((t for t in tracked if t.class_name == nearest_class), None)
                    if nearest_track:
                        # relative speed: take the component along ego's direction (yaw)
                        ego_vx = ego_speed_ms * np.cos(ego.yaw)
                        ego_vy = ego_speed_ms * np.sin(ego.yaw)
                        rel_vx = ego_vx - nearest_track.velocity[0]
                        rel_vy = ego_vy - nearest_track.velocity[1]
                        closing_speed = np.sqrt(rel_vx**2 + rel_vy**2)  # relative speed magnitude
                        if closing_speed > 0.1:  # avoid division by near-zero
                            nearest_ttc = nearest_dist / closing_speed

        planned.nearest_obstacle_class = nearest_class
        planned.nearest_obstacle_confidence = nearest_conf
        planned.nearest_obstacle_distance_m = nearest_dist
        planned.nearest_obstacle_ttc_s = nearest_ttc

        timings = TickTimings(
            fusion_ms=(t1 - t0) * 1000,
            tracking_ms=(t2 - t1) * 1000,
            prediction_ms=(t3 - t2) * 1000,
            drivable_area_ms=(t3b - t3) * 1000,
            planning_ms=(t4 - t3b) * 1000,
        )

        logger.info(
            "Tick %d done: fusion=%.2fms track=%.2fms predict=%.2fms drivable_area=%.2fms plan=%.2fms TOTAL=%.2fms | "
            "%d detections -> %d tracked -> %d predicted modes, %d non-drivable pts | path_valid=%s replanned=%s",
            self._tick_count, timings.fusion_ms, timings.tracking_ms, timings.prediction_ms,
            timings.drivable_area_ms, timings.planning_ms, timings.total_ms,
            len(detections), len(tracked), len(predictions), len(non_drivable_xy), planned.is_valid, planned.replanned,
        )

        if timings.total_ms > 150:
            logger.warning(
                "Tick %d total latency %.2fms EXCEEDS the ~150-200ms closed-loop success-collapse "
                "threshold documented in docs/architecture.md -- investigate before relying on this tick's timing.",
                self._tick_count, timings.total_ms,
            )

        return planned, timings
