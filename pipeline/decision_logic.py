"""
Decision logic: the Python state machine standing in for Stateflow on
Path B (docs/two-path-strategy.md). Implements the exact design already
worked out for Path A's Stateflow chart, per docs/architecture.md Stage 6:
hierarchical obstacle handling with a distinct ANIMAL_ON_ROAD sibling,
parallel EMERGENCY_BRAKE+REPLAN (not exclusive alternatives), a parallel
SAFETY_SUPERVISOR watchdog, debounce on obstacle entry, and a
sustained-clearance gate before resuming.

DECISION (see docs/pipeline-decision-log.md): hand-rolled explicit
states rather than a state-machine library (e.g. `transitions`),
specifically so this reads as a direct 1:1 mirror of whatever your
teammate builds as an actual Stateflow chart for Path A -- the design
gets done once (here), Path A just re-draws the same states/transitions
visually, per docs/two-path-strategy.md's plan for minimizing duplicated
design work between the two paths.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto

from pipeline.types import PredictedTrajectory, TrackedObject

logger = logging.getLogger("pipeline.decision_logic")


class DriveMode(Enum):
    NORMAL_DRIVE = auto()
    OBSTACLE_DETECTED = auto()   # generic obstacle handling
    ANIMAL_ON_ROAD = auto()      # distinct sibling -- PS names this scenario explicitly
    EMERGENCY_BRAKE = auto()     # can be active TOGETHER with REPLAN, not exclusive
    REPLAN = auto()
    RESUME = auto()


@dataclass
class SafetyStatus:
    """The parallel SAFETY_SUPERVISOR watchdog -- checked every tick
    regardless of current mode, can force EMERGENCY_BRAKE from ANY state
    without a guard duplicated on each one individually.
    """
    sensor_ok: bool = True
    last_tick_time: float = field(default_factory=time.time)
    max_tick_gap_s: float = 1.0  # if no tick for this long, assume comms/sensor dropout

    def check(self) -> bool:
        gap = time.time() - self.last_tick_time
        if gap > self.max_tick_gap_s:
            logger.warning("SAFETY_SUPERVISOR: tick gap %.2fs exceeds %.2fs -- forcing EMERGENCY_BRAKE.",
                            gap, self.max_tick_gap_s)
            return False
        if not self.sensor_ok:
            logger.warning("SAFETY_SUPERVISOR: sensor_ok=False -- forcing EMERGENCY_BRAKE.")
            return False
        return True

    def tick(self):
        self.last_tick_time = time.time()


@dataclass
class DecisionOutput:
    mode: DriveMode
    emergency_brake_active: bool
    replan_requested: bool
    notes: str = ""


class DecisionLogic:
    def __init__(
        self,
        obstacle_ttc_threshold_s: float = 3.0,
        emergency_ttc_threshold_s: float = 1.5,
        debounce_ticks: int = 3,
        clearance_ticks_required: int = 20,  # ~1s at 20Hz -- tune against your actual dt
    ):
        self.obstacle_ttc_threshold_s = obstacle_ttc_threshold_s
        self.emergency_ttc_threshold_s = emergency_ttc_threshold_s
        self.debounce_ticks = debounce_ticks
        self.clearance_ticks_required = clearance_ticks_required

        self.mode = DriveMode.NORMAL_DRIVE
        self.safety = SafetyStatus()
        self._pending_obstacle_ticks = 0
        self._pending_clear_ticks = 0
        self._prior_mode_before_interrupt: DriveMode = DriveMode.NORMAL_DRIVE

    @staticmethod
    def _min_ttc(ego_speed: float, tracked: list[TrackedObject]) -> tuple[float, str | None]:
        """Cheap, direct time-to-collision estimate for the decision
        layer's mode-switching guard ONLY -- for each tracked object,
        approximate TTC as distance / closing speed. NOT a substitute for
        real collision-checking: the planner's own costmap already does
        full obstacle avoidance geometry; this just decides which MODE
        the decision layer should be in.
        """
        if not tracked:
            return float("inf"), None

        min_ttc = float("inf")
        min_ttc_class = None
        for obj in tracked:
            if not obj.position_history:
                continue
            ox, oy = obj.position_history[-1]
            dist = (ox ** 2 + oy ** 2) ** 0.5
            vx, vy = obj.velocity
            obj_speed = (vx ** 2 + vy ** 2) ** 0.5
            closing_speed = max(ego_speed - obj_speed, 0.5)  # floor avoids div-by-~0 when speeds are similar
            ttc = dist / closing_speed
            if ttc < min_ttc:
                min_ttc = ttc
                min_ttc_class = obj.class_name
        return min_ttc, min_ttc_class

    def step(
        self,
        ego_speed: float,
        tracked: list[TrackedObject],
        predictions: list[PredictedTrajectory],
    ) -> DecisionOutput:
        self.safety.tick()

        # SAFETY_SUPERVISOR runs first, every tick, regardless of mode.
        if not self.safety.check():
            self.mode = DriveMode.EMERGENCY_BRAKE
            return DecisionOutput(self.mode, emergency_brake_active=True, replan_requested=False,
                                   notes="Forced by SAFETY_SUPERVISOR (sensor/tick-gap fault).")

        min_ttc, ttc_class = self._min_ttc(ego_speed, tracked)
        is_animal = ttc_class is not None and ttc_class.lower() in ("animal", "cow")

        if self.mode == DriveMode.NORMAL_DRIVE:
            if min_ttc < self.obstacle_ttc_threshold_s:
                self._pending_obstacle_ticks += 1
                if self._pending_obstacle_ticks >= self.debounce_ticks:
                    # Debounced: a single noisy-frame detection doesn't trigger this.
                    self._prior_mode_before_interrupt = DriveMode.NORMAL_DRIVE
                    self.mode = DriveMode.ANIMAL_ON_ROAD if is_animal else DriveMode.OBSTACLE_DETECTED
                    logger.info("NORMAL_DRIVE -> %s (min_ttc=%.2fs, class=%s, debounced over %d ticks)",
                                self.mode.name, min_ttc, ttc_class, self._pending_obstacle_ticks)
            else:
                self._pending_obstacle_ticks = 0
            return DecisionOutput(self.mode, emergency_brake_active=False, replan_requested=False,
                                   notes=f"min_ttc={min_ttc:.2f}s")

        elif self.mode in (DriveMode.OBSTACLE_DETECTED, DriveMode.ANIMAL_ON_ROAD):
            if min_ttc < self.emergency_ttc_threshold_s:
                logger.warning("%s -> EMERGENCY_BRAKE + REPLAN (parallel), min_ttc=%.2fs",
                                self.mode.name, min_ttc)
                self.mode = DriveMode.EMERGENCY_BRAKE
                return DecisionOutput(self.mode, emergency_brake_active=True, replan_requested=True,
                                       notes=f"TTC {min_ttc:.2f}s below emergency threshold ({ttc_class}).")
            elif min_ttc >= self.obstacle_ttc_threshold_s:
                self._pending_clear_ticks += 1
                if self._pending_clear_ticks >= self.clearance_ticks_required:
                    logger.info("%s -> RESUME (sustained clearance over %d ticks)",
                                self.mode.name, self._pending_clear_ticks)
                    self.mode = DriveMode.RESUME
                    self._pending_clear_ticks = 0
            else:
                self._pending_clear_ticks = 0
            return DecisionOutput(self.mode, emergency_brake_active=False, replan_requested=True,
                                   notes=f"Tracking obstacle, min_ttc={min_ttc:.2f}s.")

        elif self.mode == DriveMode.EMERGENCY_BRAKE:
            still_dangerous = min_ttc < self.obstacle_ttc_threshold_s
            if not still_dangerous:
                logger.info("EMERGENCY_BRAKE -> REPLAN (clear of immediate danger)")
                self.mode = DriveMode.REPLAN
            return DecisionOutput(self.mode, emergency_brake_active=still_dangerous, replan_requested=True,
                                   notes=f"min_ttc={min_ttc:.2f}s")

        elif self.mode == DriveMode.REPLAN:
            if min_ttc < self.emergency_ttc_threshold_s:
                self._pending_clear_ticks = 0
                self.mode = DriveMode.EMERGENCY_BRAKE
                logger.warning("REPLAN -> EMERGENCY_BRAKE (danger recurred, min_ttc=%.2fs)", min_ttc)
                return DecisionOutput(self.mode, emergency_brake_active=True, replan_requested=True,
                                       notes="Danger recurred during REPLAN.")
            self._pending_clear_ticks += 1
            if self._pending_clear_ticks >= self.clearance_ticks_required:
                logger.info("REPLAN -> RESUME (sustained clearance)")
                self.mode = DriveMode.RESUME
                self._pending_clear_ticks = 0
            return DecisionOutput(self.mode, emergency_brake_active=False, replan_requested=True, notes="Replanning.")

        elif self.mode == DriveMode.RESUME:
            # History-junction equivalent: return to whichever mode was
            # active before the interrupt, rather than hard-resetting.
            # Only NORMAL_DRIVE is tracked as a "prior mode" today since
            # this pipeline doesn't yet model finer sub-modes (lane-keep
            # vs. overtake-in-progress) -- extend _prior_mode_before_interrupt
            # if you add those later.
            logger.info("RESUME -> %s", self._prior_mode_before_interrupt.name)
            self.mode = self._prior_mode_before_interrupt
            self._pending_obstacle_ticks = 0
            return DecisionOutput(self.mode, emergency_brake_active=False, replan_requested=False,
                                   notes="Resumed after sustained clearance.")

        # Unreachable given the Enum, but keeps this defensive rather than silent.
        logger.error("DecisionLogic in unhandled mode %s -- forcing EMERGENCY_BRAKE.", self.mode)
        return DecisionOutput(DriveMode.EMERGENCY_BRAKE, emergency_brake_active=True, replan_requested=False,
                               notes="Unhandled mode, failed safe.")
