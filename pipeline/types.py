"""
Shared data structures for the pipeline. Kept as plain dataclasses so
every stage has one unambiguous contract to code against and log.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Detection:
    """One raw detection from the perception (YOLO) stage, per teammate's
    confirmed output format: class, confidence, top-left x,y, bottom-right x,y.
    Pixel coordinates, image space.
    """
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center_px(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


@dataclass
class FusedDetection:
    """A Detection fused with LiDAR range data: adds a 3D position in the
    ego/vehicle frame (x forward, y left, z up -- adjust to match your
    actual LiDAR mount convention, see decision log).
    """
    detection: Detection
    position_3d: Optional[tuple[float, float, float]]  # (x, y, z) in ego frame, None if fusion failed
    distance_m: Optional[float]  # straight-line distance from ego, None if fusion failed
    num_lidar_points: int  # how many LiDAR points supported this estimate (0 = no fusion)


@dataclass
class TrackedObject:
    """One tracked object with a short position history, output of the
    tracking stage, input to the prediction stage.
    """
    track_id: int
    class_name: str
    position_history: list[tuple[float, float]] = field(default_factory=list)  # (x, y) ego-frame, oldest first
    velocity: tuple[float, float] = (0.0, 0.0)  # (vx, vy), ego frame, m/s
    last_seen_tick: int = 0
    confidence: float = 0.0


@dataclass
class PredictedTrajectory:
    """One predicted future trajectory for one tracked object. `points` is
    a list of (x, y) ego-frame positions at future timesteps. Multiple
    PredictedTrajectory entries per track_id = multimodal prediction.
    """
    track_id: int
    class_name: str
    points: list[tuple[float, float]]
    probability: float = 1.0  # relative likelihood of this mode, sums to 1 across a track's modes


@dataclass
class EgoState:
    """Current ego-vehicle state, needed by the planner."""
    x: float
    y: float
    yaw: float  # radians
    speed: float  # m/s
    goal_x: float
    goal_y: float


@dataclass
class PlannedPath:
    """Final output of the pipeline: what gets handed to Stateflow/control."""
    waypoints: list[tuple[float, float]]  # (x, y) ego-frame, ordered from current position to horizon
    is_valid: bool  # False if planning failed (e.g. no feasible path found)
    replanned: bool  # True if this tick triggered a fresh plan vs. reusing the previous one
    notes: str = ""  # short human-readable reason, goes straight into the log
