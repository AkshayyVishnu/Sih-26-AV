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

    def rasterize_non_drivable(self, points_xy: list[tuple[float, float]], cost: float = 1.0):
        """Marks each point's exact grid cell as high-cost. Used for
        drivable-area classification (pipeline/drivable_area.py), where
        the signal is many individual LiDAR points rather than a few
        discrete predicted-obstacle centers -- a direct per-cell mark is
        more appropriate here than add_obstacle_inflation's Gaussian
        falloff, which would be redundant/expensive at this point density.

        KNOWN LIMITATION: coverage depends on LiDAR point density. Gaps
        (occluded regions, far range) are left at their existing cost --
        i.e. treated as passable by default, NOT as confirmed-safe. This
        is a real safety caveat, not just a demo simplification -- don't
        assume "no non-drivable points here" means "definitely drivable."
        """
        for x, y in points_xy:
            r, c = self.world_to_grid(x, y)
            if self.in_bounds(r, c):
                self.cost[r, c] = max(self.cost[r, c], cost)

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


# Per-class obstacle inflation radii (meters). Erratic/vulnerable classes get
# a wider berth so A* routes around their *possible* motion, not just their
# current cell. Confidence-agnostic by design -- radii depend only on class,
# not on YOLO confidence (which varies per class and viewing angle).
_INFLATION_RADIUS_M = {
    "pedestrian": 2.0,
    "person": 2.0,
    "animal": 2.0,
    "cow": 2.0,
    "bicycle": 1.75,
    "motorcycle": 1.75,
    "pushcart": 1.75,
    "auto-rickshaw": 1.5,
    "autorickshaw": 1.5,
    "car": 1.5,
    "bus": 1.5,
    "truck": 1.5,
}
_DEFAULT_INFLATION_RADIUS_M = 1.5

# Hard cap on auto-expanded costmap span (meters). Bounds A* worst-case cost
# when the goal is far away; beyond this the goal is clipped to the edge
# (loudly, via notes) instead of growing the grid unboundedly.
_MAX_COSTMAP_SPAN_M = 120.0


def _inflation_for_class(class_name: str, overrides: dict | None = None) -> float:
    if overrides and class_name.lower() in overrides:
        return float(overrides[class_name.lower()])
    return _INFLATION_RADIUS_M.get(class_name.lower(), _DEFAULT_INFLATION_RADIUS_M)


def build_costmap_from_predictions(
    ego: EgoState,
    predictions: list[PredictedTrajectory],
    width_m: float = 60.0,
    height_m: float = 60.0,
    resolution_m: float = 0.5,
    inflation_overrides: dict | None = None,
    goal_padding_m: float = 5.0,
) -> GridCostmap:
    # Goal-window fix: expand bounds to include the goal (+padding) instead
    # of silently failing A* when the goal sits outside the default 60x60m
    # ego-centered window (the old off-by-config bug in run_demo.py). Span
    # is capped at _MAX_COSTMAP_SPAN_M; beyond that the Planner clips the
    # goal to the edge and says so in PlannedPath.notes.
    min_x = min(ego.x - width_m / 2, ego.goal_x - goal_padding_m)
    max_x = max(ego.x + width_m / 2, ego.goal_x + goal_padding_m)
    min_y = min(ego.y - height_m / 2, ego.goal_y - goal_padding_m)
    max_y = max(ego.y + height_m / 2, ego.goal_y + goal_padding_m)
    width_eff = min(max_x - min_x, _MAX_COSTMAP_SPAN_M)
    height_eff = min(max_y - min_y, _MAX_COSTMAP_SPAN_M)
    # Keep ego-anchored origin when no expansion needed (identical to old
    # behavior); re-anchor to min corner only when expansion happened.
    if width_eff == width_m and height_eff == height_m:
        origin_x, origin_y = ego.x - width_m / 2, ego.y - height_m / 2
        expanded = False
    else:
        origin_x, origin_y = min_x, min_y
        expanded = True
        logger.warning("Costmap expanded to include goal: %.0fx%.0fm (default %.0fx%.0fm).",
                       width_eff, height_eff, width_m, height_m)
    costmap = GridCostmap(width_eff, height_eff, resolution_m, origin_x, origin_y)
    costmap.expanded_for_goal = expanded  # type: ignore[attr-defined]

    for traj in predictions:
        for k, (px, py) in enumerate(traj.points):
            # Cost decays for further-future points (more uncertain, and
            # by the time the ego vehicle gets there conditions may have
            # changed) and scales with the mode's probability.
            time_decay = 1.0 - (k / max(len(traj.points), 1)) * 0.5
            base_cost = traj.probability * time_decay
            costmap.add_obstacle_inflation(
                px, py, base_cost=base_cost,
                inflation_radius_m=_inflation_for_class(traj.class_name, inflation_overrides),
            )

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


def _segment_max_cost(costmap: GridCostmap, a: tuple[float, float], b: tuple[float, float]) -> float:
    """Max costmap cost sampled along segment a->b (world coords)."""
    dist = np.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(int(dist / (costmap.resolution / 2)), 1)
    worst = 0.0
    for i in range(steps + 1):
        t = i / steps
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        r, c = costmap.world_to_grid(x, y)
        if not costmap.in_bounds(r, c):
            return float("inf")  # leaving the map is never a valid shortcut
        worst = max(worst, float(costmap.cost[r, c]))
    return worst


def shortcut_waypoints(waypoints: list[tuple[float, float]], costmap: GridCostmap,
                       max_shortcut_cost: float = 0.35) -> list[tuple[float, float]]:
    """Greedy line-of-sight shortcutting: from each kept point, jump to the
    furthest later point whose straight segment stays in cheap cells. Turns
    A*'s 8-connected staircase into longer straight legs."""
    if len(waypoints) <= 2:
        return list(waypoints)
    out = [waypoints[0]]
    i = 0
    while i < len(waypoints) - 1:
        j = len(waypoints) - 1
        while j > i + 1:
            if _segment_max_cost(costmap, waypoints[i], waypoints[j]) <= max_shortcut_cost:
                break
            j -= 1
        out.append(waypoints[j])
        i = j
    return out


def smooth_waypoints(waypoints: list[tuple[float, float]], passes: int = 2) -> list[tuple[float, float]]:
    """Endpoint-pinned moving-average smoothing (0.25/0.5/0.25). Cheap,
    no optimizer dependency, directly serves the path-smoothness metric."""
    pts = list(waypoints)
    for _ in range(max(passes, 0)):
        if len(pts) <= 2:
            break
        smoothed = [pts[0]]
        for i in range(1, len(pts) - 1):
            sx = 0.25 * pts[i - 1][0] + 0.5 * pts[i][0] + 0.25 * pts[i + 1][0]
            sy = 0.25 * pts[i - 1][1] + 0.5 * pts[i][1] + 0.25 * pts[i + 1][1]
            smoothed.append((sx, sy))
        smoothed.append(pts[-1])
        pts = smoothed
    return pts


def decimate_waypoints(waypoints: list[tuple[float, float]], min_spacing_m: float = 1.5) -> list[tuple[float, float]]:
    """Resample to ~min_spacing so downstream control gets ~1-2m spaced
    targets. Interpolates along long straight legs (shortcutting can leave
    50m+ gaps) instead of just filtering -- always keeps final goal."""
    if len(waypoints) <= 1:
        return list(waypoints)
    out = [waypoints[0]]
    for pt in waypoints[1:]:
        while True:
            dist = np.hypot(pt[0] - out[-1][0], pt[1] - out[-1][1])
            if dist < min_spacing_m:
                break
            # Step min_spacing from the last emitted point toward pt, so
            # long legs get interpolated rather than left as huge jumps.
            t = min_spacing_m / dist
            nx = out[-1][0] + (pt[0] - out[-1][0]) * t
            ny = out[-1][1] + (pt[1] - out[-1][1]) * t
            out.append((nx, ny))
            if len(out) > 10000:  # safety cap, should never hit
                break
    if out[-1] != waypoints[-1]:
        out.append(waypoints[-1])
    return out


class Planner:
    def __init__(
        self,
        replan_cost_change_threshold: float = 0.15,
        min_replan_interval_ticks: int = 3,
        width_m: float = 60.0,
        height_m: float = 60.0,
        resolution_m: float = 0.5,
        cost_weight: float = 50.0,
        waypoint_spacing_m: float = 1.5,
        inflation_overrides: dict | None = None,
    ):
        self._last_path: list[tuple[float, float]] | None = None
        self._last_costmap_signature: float | None = None
        self.replan_cost_change_threshold = replan_cost_change_threshold
        # Hysteresis: even when the signature moves, don't re-search more
        # often than this -- bounds planning latency in dense scenes and
        # stops replan-every-tick oscillation from LiDAR sampling noise.
        self.min_replan_interval_ticks = min_replan_interval_ticks
        self._ticks_since_replan = min_replan_interval_ticks  # allow immediate first plan
        self.width_m = width_m
        self.height_m = height_m
        self.resolution_m = resolution_m
        self.cost_weight = cost_weight
        self.waypoint_spacing_m = waypoint_spacing_m
        self.inflation_overrides = inflation_overrides

    def plan(
        self,
        ego: EgoState,
        predictions: list[PredictedTrajectory],
        non_drivable_points: list[tuple[float, float]] | None = None,
        non_drivable_cost: float = 1.0,
    ) -> PlannedPath:
        t0 = time.perf_counter()
        costmap = build_costmap_from_predictions(
            ego, predictions,
            width_m=self.width_m, height_m=self.height_m,
            resolution_m=self.resolution_m,
            inflation_overrides=self.inflation_overrides,
        )

        # Replan-trigger signature is computed from PREDICTED-OBSTACLE
        # cost only, deliberately BEFORE merging in non-drivable-area
        # cost below. The environment (buildings/sidewalks) is static
        # tick-to-tick, but raw LiDAR sampling noise means the exact
        # point set differs every tick anyway -- if that noise were
        # included in the signature, it would either mask real obstacle
        # changes (swamped by a large near-constant baseline) or trigger
        # spurious replans every tick (chasing sampling noise). Real
        # replanning-trigger tuning belongs in docs/architecture.md Stage 5;
        # this is a minimal version so replanning isn't literally every tick.
        signature = float(costmap.cost.sum())

        if non_drivable_points:
            costmap.rasterize_non_drivable(non_drivable_points, cost=non_drivable_cost)
            logger.debug("Applied %d non-drivable points to costmap (after signature computed).", len(non_drivable_points))
        scene_changed = (
            self._last_path is None
            or self._last_costmap_signature is None
            or abs(signature - self._last_costmap_signature) > self.replan_cost_change_threshold * max(self._last_costmap_signature, 1e-6)
        )
        self._ticks_since_replan += 1
        needs_replan = scene_changed and self._ticks_since_replan >= self.min_replan_interval_ticks

        if not needs_replan:
            self._ticks_since_replan = min(self._ticks_since_replan, self.min_replan_interval_ticks)
            elapsed_ms = (time.perf_counter() - t0) * 1000
            reason = "below threshold" if not scene_changed else f"min-interval ({self.min_replan_interval_ticks} ticks)"
            logger.debug("No replan (signature %.3f vs %.3f, %s) -- reusing previous path. (%.2fms)",
                         signature, self._last_costmap_signature, reason, elapsed_ms)
            return PlannedPath(self._last_path, is_valid=True, replanned=False,
                                notes=f"Reused previous path, {reason}.")

        start_rc = costmap.world_to_grid(ego.x, ego.y)
        goal_rc = costmap.world_to_grid(ego.goal_x, ego.goal_y)

        # Span is capped in build_costmap_from_predictions; if the true goal
        # still falls outside, clip it to the map edge LOUDLY (notes) instead
        # of failing A* silently. Server-side note: verify against the real
        # scenario scale -- a clipped goal means "drive toward goal", not
        # "goal reached".
        goal_clipped = False
        if not costmap.in_bounds(*goal_rc):
            goal_rc = (min(max(goal_rc[0], 0), costmap.rows - 1),
                       min(max(goal_rc[1], 0), costmap.cols - 1))
            goal_clipped = True

        grid_path = _astar(costmap, start_rc, goal_rc, cost_weight=self.cost_weight)
        elapsed_ms = (time.perf_counter() - t0) * 1000

        if grid_path is None:
            # Failure fallback: hold the last good path (don't leave control
            # with nothing) unless there never was one. One noisy tick with
            # no feasible A* solution must not halt the vehicle.
            if self._last_path is not None:
                logger.warning("A* found NO FEASIBLE PATH (start=%s goal=%s) -- holding last path. (%.2fms)",
                               start_rc, goal_rc, elapsed_ms)
                return PlannedPath(list(self._last_path), is_valid=True, replanned=False,
                                   notes="A* failed this tick; holding last good path.")
            logger.warning("A* found NO FEASIBLE PATH (start=%s goal=%s) and no prior path. (%.2fms)",
                           start_rc, goal_rc, elapsed_ms)
            return PlannedPath([], is_valid=False, replanned=True, notes="No feasible path found by A*.")

        raw_waypoints = [costmap.grid_to_world(r, c) for r, c in grid_path]
        # Raw A* cell centers every resolution_m are unfollowable: shortcut
        # the staircase, smooth the corners, decimate to ~1.5m spacing so
        # downstream control (server-side VehicleControl) gets sane targets.
        waypoints = decimate_waypoints(
            smooth_waypoints(shortcut_waypoints(raw_waypoints, costmap)),
            min_spacing_m=self.waypoint_spacing_m,
        )
        self._last_path = waypoints
        self._last_costmap_signature = signature
        self._ticks_since_replan = 0

        # Speed hint for the future server-side controller, kept inside notes
        # so PlannedPath stays flat/struct-mappable for the later py.*/.slx
        # bridge: slow down where the final path threads through high cost.
        worst = max((_segment_max_cost(costmap, waypoints[i], waypoints[i + 1])
                     for i in range(len(waypoints) - 1)), default=0.0)
        speed_scale = max(1.0 - min(worst, 1.0) * 0.7, 0.3)
        clip_note = " goal-clipped-to-edge." if goal_clipped else ""
        expanded_note = " costmap-expanded-for-goal." if getattr(costmap, "expanded_for_goal", False) else ""

        logger.info("Replanned: %d waypoints (raw %d), %.2fms, signature %.3f, speed_scale %.2f.%s%s",
                    len(waypoints), len(raw_waypoints), elapsed_ms, signature, speed_scale,
                    expanded_note, clip_note)
        return PlannedPath(waypoints, is_valid=True, replanned=True,
                            notes=f"Fresh A* plan, {elapsed_ms:.2f}ms, speed_scale={speed_scale:.2f}.{expanded_note}{clip_note}")
