"""
Planning stage: EgoState + PredictedTrajectory list -> PlannedPath.

DECISION (see docs/pipeline-decision-log.md): implemented as a pure-Python
grid-based A* with soft costmap inflation, rather than calling MATLAB's
plannerHybridAStar directly -- same reason as the tracker: keeps this
whole pipeline testable standalone without MATLAB running. The
interface (costmap in, waypoint list out) matches what you'd feed
plannerHybridAStar's occupancy input, so swapping the search algorithm
for MATLAB's later is a implementation swap, not a redesign.
"""
from __future__ import annotations

import heapq
import logging
import time

import numpy as np

from pipeline.types import EgoState, PlannedPath, PredictedTrajectory

logger = logging.getLogger("pipeline.planner")


class GridCostmap:
    """A local occupancy/cost grid centered conceptually on the planning
    region (not necessarily on the ego vehicle -- pass an explicit
    origin). Cost is soft: obstacles inflate with a falloff rather than
    being hard 0/1 blocks, so the planner can still find a path through
    a cluttered market scene instead of failing outright.
    """

    def __init__(self, width_m: float, height_m: float, resolution_m: float, origin_x: float, origin_y: float):
        self.resolution = resolution_m
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.cols = max(int(width_m / resolution_m), 1)
        self.rows = max(int(height_m / resolution_m), 1)
        self.cost = np.zeros((self.rows, self.cols), dtype=np.float32)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def add_obstacle_inflation(self, x: float, y: float, base_cost: float, inflation_radius_m: float):
        """Adds a soft cost bump around (x, y), Gaussian-ish falloff."""
        center_row, center_col = self.world_to_grid(x, y)
        radius_cells = max(int(inflation_radius_m / self.resolution), 1)

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                r, c = center_row + dr, center_col + dc
                if not self.in_bounds(r, c):
                    continue
                dist = np.hypot(dr, dc) * self.resolution
                if dist > inflation_radius_m:
                    continue
                falloff = np.exp(-(dist ** 2) / (2 * (inflation_radius_m / 2) ** 2))
                self.cost[r, c] = max(self.cost[r, c], base_cost * falloff)


def build_costmap_from_predictions(
    ego: EgoState,
    predictions: list[PredictedTrajectory],
    width_m: float = 60.0,
    height_m: float = 60.0,
    resolution_m: float = 0.5,
) -> GridCostmap:
    origin_x = ego.x - width_m / 2
    origin_y = ego.y - height_m / 2
    costmap = GridCostmap(width_m, height_m, resolution_m, origin_x, origin_y)

    for traj in predictions:
        for k, (px, py) in enumerate(traj.points):
            # Cost decays for further-future points (more uncertain, and
            # by the time the ego vehicle gets there conditions may have
            # changed) and scales with the mode's probability.
            time_decay = 1.0 - (k / max(len(traj.points), 1)) * 0.5
            base_cost = traj.probability * time_decay
            costmap.add_obstacle_inflation(px, py, base_cost=base_cost, inflation_radius_m=1.5)

    return costmap


def _astar(costmap: GridCostmap, start_rc: tuple[int, int], goal_rc: tuple[int, int], cost_weight: float = 50.0):
    """8-connected grid A*. Returns list of (row, col) or None if no path found."""
    if not costmap.in_bounds(*start_rc) or not costmap.in_bounds(*goal_rc):
        return None

    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def heuristic(rc):
        return np.hypot(rc[0] - goal_rc[0], rc[1] - goal_rc[1])

    open_set = [(heuristic(start_rc), 0.0, start_rc, None)]
    came_from: dict = {}
    g_score = {start_rc: 0.0}
    visited = set()

    while open_set:
        _, g, current, parent = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        came_from[current] = parent

        if current == goal_rc:
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from[node]
            return list(reversed(path))

        for dr, dc in neighbors:
            nr, nc = current[0] + dr, current[1] + dc
            if not costmap.in_bounds(nr, nc) or (nr, nc) in visited:
                continue
            step_cost = np.hypot(dr, dc) + costmap.cost[nr, nc] * cost_weight
            tentative_g = g + step_cost
            if tentative_g < g_score.get((nr, nc), float("inf")):
                g_score[(nr, nc)] = tentative_g
                f = tentative_g + heuristic((nr, nc))
                heapq.heappush(open_set, (f, tentative_g, (nr, nc), current))

    return None


class Planner:
    def __init__(self, replan_cost_change_threshold: float = 0.15):
        self._last_path: list[tuple[float, float]] | None = None
        self._last_costmap_signature: float | None = None
        self.replan_cost_change_threshold = replan_cost_change_threshold

    def plan(self, ego: EgoState, predictions: list[PredictedTrajectory]) -> PlannedPath:
        t0 = time.perf_counter()
        costmap = build_costmap_from_predictions(ego, predictions)

        # Cheap "did the scene actually change enough to justify a fresh
        # search" signal -- sum of costmap as a rough signature. Real
        # replanning-trigger tuning belongs in docs/architecture.md Stage 5;
        # this is a minimal version so replanning isn't literally every tick.
        signature = float(costmap.cost.sum())
        needs_replan = (
            self._last_path is None
            or self._last_costmap_signature is None
            or abs(signature - self._last_costmap_signature) > self.replan_cost_change_threshold * max(self._last_costmap_signature, 1e-6)
        )

        if not needs_replan:
            elapsed_ms = (time.perf_counter() - t0) * 1000
            logger.debug("No significant scene change (signature %.3f vs %.3f) -- reusing previous path. (%.2fms)",
                         signature, self._last_costmap_signature, elapsed_ms)
            return PlannedPath(self._last_path, is_valid=True, replanned=False,
                                notes="Reused previous path, costmap change below threshold.")

        start_rc = costmap.world_to_grid(ego.x, ego.y)
        goal_rc = costmap.world_to_grid(ego.goal_x, ego.goal_y)

        grid_path = _astar(costmap, start_rc, goal_rc)
        elapsed_ms = (time.perf_counter() - t0) * 1000

        if grid_path is None:
            logger.warning("A* found NO FEASIBLE PATH (start=%s goal=%s). (%.2fms)", start_rc, goal_rc, elapsed_ms)
            return PlannedPath([], is_valid=False, replanned=True, notes="No feasible path found by A*.")

        waypoints = [costmap.grid_to_world(r, c) for r, c in grid_path]
        self._last_path = waypoints
        self._last_costmap_signature = signature

        logger.info("Replanned: %d waypoints, %.2fms, costmap signature %.3f.",
                    len(waypoints), elapsed_ms, signature)
        return PlannedPath(waypoints, is_valid=True, replanned=True,
                            notes=f"Fresh A* plan, {elapsed_ms:.2f}ms.")
