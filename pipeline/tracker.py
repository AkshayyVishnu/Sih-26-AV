"""
Multi-object tracker: assigns consistent IDs to fused detections across
ticks and maintains a short position history + Kalman velocity estimate
per track -- the input format both the predictor (MoFlow or fallback)
and the planner need.

DECISION (see docs/pipeline-decision-log.md): implemented natively in
Python with filterpy rather than routing through MATLAB's multiObjectTracker,
so this whole pipeline is testable standalone without MATLAB running.
Swap-in point for MATLAB's tracker is marked below if you integrate later.
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

from pipeline.types import FusedDetection, TrackedObject

logger = logging.getLogger("pipeline.tracker")

MAX_HISTORY_LEN = 12  # matches MoFlow's expected 8-12 frame input window
MAX_MISSES_BEFORE_DROP = 5  # ticks a track can go unmatched before being deleted


def _make_kalman_filter(x0: float, y0: float, dt: float) -> KalmanFilter:
    """Constant-velocity Kalman filter, state = [x, y, vx, vy]."""
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    kf.H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ])
    kf.x = np.array([x0, y0, 0.0, 0.0])
    kf.P *= 10.0
    kf.R *= 0.5  # measurement noise -- tune against your actual LiDAR fusion noise once you have real data
    kf.Q *= 0.1  # process noise -- tune against how erratic the traffic actually is (higher for pedestrians/animals)
    return kf


class _Track:
    def __init__(self, track_id: int, class_name: str, x: float, y: float, dt: float, tick: int):
        self.track_id = track_id
        self.class_name = class_name
        self.kf = _make_kalman_filter(x, y, dt)
        self.history: list[tuple[float, float]] = [(x, y)]
        self.misses = 0
        self.last_seen_tick = tick
        self.confidence = 0.0

    def predict_step(self):
        self.kf.predict()

    def update(self, x: float, y: float, confidence: float, tick: int):
        self.kf.update(np.array([x, y]))
        self.history.append((float(self.kf.x[0]), float(self.kf.x[1])))
        if len(self.history) > MAX_HISTORY_LEN:
            self.history.pop(0)
        self.misses = 0
        self.last_seen_tick = tick
        self.confidence = confidence

    def to_tracked_object(self) -> TrackedObject:
        return TrackedObject(
            track_id=self.track_id,
            class_name=self.class_name,
            position_history=list(self.history),
            velocity=(float(self.kf.x[2]), float(self.kf.x[3])),
            last_seen_tick=self.last_seen_tick,
            confidence=self.confidence,
        )


class MultiObjectTracker:
    """Greedy nearest-neighbor association via the Hungarian algorithm on
    a Euclidean gating distance -- simple, fast, adequate for a 2-day
    build. NOT a full JPDA/IMM implementation; if track-swap errors show
    up under real dense-traffic testing, that's the known limitation to
    upgrade first (see docs/component-deep-dive.md's tracker discussion).
    """

    def __init__(self, dt: float, gating_distance_m: float = 4.0):
        self.dt = dt
        self.gating_distance = gating_distance_m
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self._tick = 0

    def step(self, fused_detections: list[FusedDetection]) -> list[TrackedObject]:
        self._tick += 1

        for track in self._tracks.values():
            track.predict_step()

        usable = [fd for fd in fused_detections if fd.position_3d is not None]
        dropped = len(fused_detections) - len(usable)
        if dropped:
            logger.debug("%d detection(s) had no valid 3D position this tick -- excluded from tracking.", dropped)

        track_ids = list(self._tracks.keys())
        assigned_tracks: set[int] = set()
        assigned_dets: set[int] = set()

        if track_ids and usable:
            cost = np.zeros((len(track_ids), len(usable)))
            for i, tid in enumerate(track_ids):
                tx, ty = self._tracks[tid].kf.x[0], self._tracks[tid].kf.x[1]
                for j, fd in enumerate(usable):
                    dx = fd.position_3d[0] - tx
                    dy = fd.position_3d[1] - ty
                    cost[i, j] = np.hypot(dx, dy)

            row_ind, col_ind = linear_sum_assignment(cost)
            for r, c in zip(row_ind, col_ind):
                if cost[r, c] <= self.gating_distance:
                    tid = track_ids[r]
                    fd = usable[c]
                    self._tracks[tid].update(fd.position_3d[0], fd.position_3d[1], fd.detection.confidence, self._tick)
                    assigned_tracks.add(tid)
                    assigned_dets.add(c)
                    logger.debug("Track %d <- detection %s (dist=%.2fm)", tid, fd.detection.class_name, cost[r, c])

        # Unmatched tracks: count a miss, drop if stale too long.
        for tid in track_ids:
            if tid not in assigned_tracks:
                self._tracks[tid].misses += 1
                if self._tracks[tid].misses > MAX_MISSES_BEFORE_DROP:
                    logger.info("Dropping track %d (%s) -- unmatched for %d ticks.",
                                tid, self._tracks[tid].class_name, self._tracks[tid].misses)
                    del self._tracks[tid]

        # Unmatched detections: spawn new tracks.
        for j, fd in enumerate(usable):
            if j not in assigned_dets:
                tid = self._next_id
                self._next_id += 1
                self._tracks[tid] = _Track(tid, fd.detection.class_name, fd.position_3d[0], fd.position_3d[1], self.dt, self._tick)
                logger.info("New track %d spawned (%s) at (%.2f, %.2f)",
                            tid, fd.detection.class_name, fd.position_3d[0], fd.position_3d[1])

        return [t.to_tracked_object() for t in self._tracks.values()]

    # --- MATLAB integration swap-in point -------------------------------
    # If you later route this through MATLAB's multiObjectTracker instead
    # (native, already licensed, per docs/architecture.md), replace the
    # body of step() with a call through your py.* bridge and keep this
    # class's input/output contract (list[FusedDetection] -> list[TrackedObject])
    # identical so nothing downstream needs to change.
