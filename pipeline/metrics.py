"""
Structured per-tick metrics recording for closed-loop runs. Produces
exactly the data docs/architecture.md's metrics-to-architecture mapping
and the PS's three named metrics need: replanning latency, path
smoothness (jerk), and scenario completion rate.

Writes one CSV (per-tick rows) + one summary.json per run to logs/.
See metrics_export.py for aggregating across multiple runs/scenarios --
closed-loop results are seed-sensitive (per carla_garage's own
common-mistakes guide, already cited in docs/architecture.md), so a
single run's numbers aren't the ones to report.
"""
from __future__ import annotations

import csv
import json
import logging
import os
import time
from dataclasses import asdict, dataclass

import numpy as np

logger = logging.getLogger("pipeline.metrics")

METRICS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


@dataclass
class TickMetric:
    tick: int
    sim_time_s: float
    total_latency_ms: float
    fusion_ms: float
    tracking_ms: float
    prediction_ms: float
    drivable_area_ms: float
    decision_ms: float
    planning_ms: float
    control_ms: float
    replanned: bool
    path_valid: bool
    decision_mode: str
    speed_mps: float
    accel_x: float
    accel_y: float
    jerk_mps3: float           # magnitude of d(accel)/dt -- the actual path-smoothness number
    distance_to_goal_m: float
    collision: bool = False
    collision_actor: str = ""


class MetricsRecorder:
    def __init__(self, scenario_name: str = "unnamed", dt: float = 0.05):
        """dt: the FIXED SIMULATION timestep (e.g. FIXED_DELTA_SECONDS in
        run_live.py) -- NOT derived from wall-clock time between
        record_tick() calls. This matters: in CARLA's synchronous mode,
        physics always advances by exactly `dt` per world.tick()
        regardless of how long client-side processing took that tick
        (which is exactly what TickTimings measures) -- using wall-clock
        deltas for a physics derivative like jerk is wrong, and produces
        wildly incorrect (way too large) values whenever processing is
        fast. Caught this via a self-test before it reached real data.
        """
        self.scenario_name = scenario_name
        self.dt = dt
        # Millisecond precision, not just seconds -- multiple runs
        # executed back-to-back (e.g. a scripted multi-seed test loop)
        # can easily land in the same second, which silently overwrote
        # earlier runs' files under the old %Y%m%d_%H%M%S-only id.
        # Caught via a 3-run self-test before it could lose real data.
        self.run_id = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
        self.ticks: list[TickMetric] = []
        self._prev_accel: tuple[float, float] | None = None
        self._collision_flag = False
        self._collision_actor = ""
        self._start_time = time.time()

        os.makedirs(METRICS_DIR, exist_ok=True)
        self.csv_path = os.path.join(METRICS_DIR, f"metrics_{scenario_name}_{self.run_id}.csv")
        logger.info("MetricsRecorder started for scenario '%s' (dt=%.3fs) -> %s", scenario_name, dt, self.csv_path)

    def _compute_jerk(self, accel_x: float, accel_y: float) -> float:
        """Jerk = magnitude of the change in acceleration per simulation
        timestep -- the standard definition used as this project's 'path
        smoothness' number. Uses self.dt (fixed sim timestep), not
        wall-clock time -- see __init__'s docstring. Returns 0.0 for the
        first sample (nothing to diff against).
        """
        if self._prev_accel is None:
            jerk = 0.0
        else:
            dax = (accel_x - self._prev_accel[0]) / self.dt
            day = (accel_y - self._prev_accel[1]) / self.dt
            jerk = float(np.hypot(dax, day))
        self._prev_accel = (accel_x, accel_y)
        return jerk

    def record_collision(self, other_actor_type: str):
        """Call from your collision sensor's callback. Sets a flag
        consumed by the NEXT record_tick() call -- collisions are async
        events, this attaches them to whichever tick they land in.
        """
        self._collision_flag = True
        self._collision_actor = other_actor_type
        logger.warning("Collision recorded: %s", other_actor_type)

    def record_tick(
        self,
        tick: int,
        timings,  # pipeline.pipeline.TickTimings
        replanned: bool,
        path_valid: bool,
        decision_mode: str,
        speed_mps: float,
        accel_x: float,
        accel_y: float,
        distance_to_goal_m: float,
    ):
        sim_time_s = self.dt * tick  # deterministic sim-time-elapsed from tick count and fixed dt,
                                      # NOT wall-clock -- consistent with the jerk fix above
        jerk = self._compute_jerk(accel_x, accel_y)

        metric = TickMetric(
            tick=tick, sim_time_s=sim_time_s,
            total_latency_ms=timings.total_ms, fusion_ms=timings.fusion_ms,
            tracking_ms=timings.tracking_ms, prediction_ms=timings.prediction_ms,
            drivable_area_ms=timings.drivable_area_ms, decision_ms=timings.decision_ms,
            planning_ms=timings.planning_ms, control_ms=timings.control_ms,
            replanned=replanned, path_valid=path_valid, decision_mode=decision_mode,
            speed_mps=speed_mps, accel_x=accel_x, accel_y=accel_y, jerk_mps3=jerk,
            distance_to_goal_m=distance_to_goal_m,
            collision=self._collision_flag, collision_actor=self._collision_actor,
        )
        self.ticks.append(metric)
        self._collision_flag = False
        self._collision_actor = ""

    def finalize(self, completed: bool, reason: str = "") -> dict:
        """Writes the per-tick CSV and a summary.json for this run.
        completed: True if the scenario reached its goal without a
        disqualifying failure -- caller decides what "completed" means
        for their scenario (reached goal region, no collision, etc.).
        """
        if not self.ticks:
            logger.warning("finalize() called with zero recorded ticks -- nothing to write.")
            return {}

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(asdict(self.ticks[0]).keys()))
            writer.writeheader()
            for t in self.ticks:
                writer.writerow(asdict(t))

        latencies = [t.total_latency_ms for t in self.ticks]
        # Warmed-up: exclude first 5 ticks, same cold-start reasoning as run_demo.py.
        warm_latencies = latencies[5:] if len(latencies) > 5 else latencies
        jerks = [t.jerk_mps3 for t in self.ticks if t.sim_time_s > 0]
        collisions = [t for t in self.ticks if t.collision]

        summary = {
            "scenario_name": self.scenario_name,
            "run_id": self.run_id,
            "completed": completed,
            "reason": reason,
            "num_ticks": len(self.ticks),
            "duration_s": self.ticks[-1].sim_time_s,
            "collision_count": len(collisions),
            "collision_actors": [c.collision_actor for c in collisions],
            "replanning_latency_ms": {
                "mean": float(np.mean(warm_latencies)),
                "max": float(np.max(warm_latencies)),
                "min": float(np.min(warm_latencies)),
                "mean_all_ticks_incl_cold_start": float(np.mean(latencies)),
            },
            "path_smoothness_jerk_mps3": {
                "mean": float(np.mean(jerks)) if jerks else None,
                "max": float(np.max(jerks)) if jerks else None,
            },
            "final_distance_to_goal_m": self.ticks[-1].distance_to_goal_m,
        }

        summary_path = self.csv_path.replace(".csv", "_summary.json")
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        logger.info("Metrics written: %s, %s", self.csv_path, summary_path)
        print(f"\n--- Metrics summary ({self.scenario_name}) ---")
        print(json.dumps(summary, indent=2))

        return summary
