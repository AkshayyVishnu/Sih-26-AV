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


def _local_to_world_xy(x_local: float, y_local: float, ego: EgoState) -> tuple[float, float]:
    """Rotates+translates a sensor-local-frame (x, y) point into world
    frame using the ego's current pose. REQUIRED fix: LiDAR/fusion output
    is in the sensor's own local frame, but EgoState.x/y (and everything
    the planner's costmap does) is in CARLA world frame. This was a
    silent bug until now -- the synthetic demo never caught it because
    its fake ego sat at (0,0) with yaw=0 the whole run, making local and
    world frames accidentally identical. With a real, moving vehicle,
    they diverge, and tracking/planning would silently operate on
    inconsistent coordinates without this transform.
    """
    cos_y, sin_y = np.cos(ego.yaw), np.sin(ego.yaw)
    x_world = x_local * cos_y - y_local * sin_y + ego.x
    y_world = x_local * sin_y + y_local * cos_y + ego.y
    return x_world, y_world


class Pipeline:
    def __init__(
        self,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        dt: float = 0.05,
        predictor: Predictor | None = None,
        prediction_horizon_steps: int | None = None,
        cost_head=None,
    ):
        get_logger("pipeline")  # sets up file+console logging for the whole run, once

        self.fuser = LidarCameraFuser(camera_intrinsic, camera_to_lidar_extrinsic)
        self.tracker = MultiObjectTracker(dt=dt)
        if predictor is None:
            # Horizon plumbing: explicit override wins; otherwise the
            # predictor's own default. Keeps every existing call site
            # (e.g. PipelineAutopilot) behavior-identical while letting
            # the server side tune horizon against its tick dt later.
            from pipeline.predictor import PREDICTION_HORIZON_STEPS
            predictor = ConstantVelocityPredictor(
                horizon_steps=prediction_horizon_steps or PREDICTION_HORIZON_STEPS
            )
        self.predictor = predictor
        self.drivable_area = DrivableAreaEstimator(camera_intrinsic, camera_to_lidar_extrinsic)
        self.decision_logic = DecisionLogic()
        self.planner = Planner(cost_head=cost_head)
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
        # Transform sensor-local fused positions into world frame -- see
        # _local_to_world_xy's docstring. No-op in effect when ego is at
        # the origin with yaw=0 (e.g. the synthetic demo), REQUIRED once
        # ego.x/y/yaw reflect a real, moving CARLA vehicle.
        for fd in fused:
            if fd.position_3d is not None:
                wx, wy = _local_to_world_xy(fd.position_3d[0], fd.position_3d[1], ego)
                fd.position_3d = (wx, wy, fd.position_3d[2])
        t1 = time.perf_counter()

        tracked = self.tracker.step(fused)
        t2 = time.perf_counter()

        predictions = self.predictor.predict(tracked, self.dt)
        t3 = time.perf_counter()

        non_drivable_xy: list[tuple[float, float]] = []
        if segmentation_tags is not None:
            _, non_drivable_local = self.drivable_area.classify(lidar_points_xyz, segmentation_tags)
            non_drivable_xy = [_local_to_world_xy(x, y, ego) for x, y in non_drivable_local]
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
            tracked_by_id={t.track_id: t for t in tracked},
            dt=self.dt,
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
