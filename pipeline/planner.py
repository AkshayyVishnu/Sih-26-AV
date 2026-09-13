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


def build_costmap_from_predictions(
    ego: EgoState,
    predictions: list[PredictedTrajectory],
    width_m: float = 60.0,
    height_m: float = 60.0,
    resolution_m: float = 0.5,
    cost_head=None,
    tracked_by_id: dict | None = None,
    dt: float = 0.05,
) -> GridCostmap:
    """cost_head: optional pipeline.cost_head.CostHead. When loaded, each
    predicted point's inflation radius and danger come from the MLP;
    otherwise (None, unloaded, or track lookup miss) the exact pre-MLP
    behavior applies (flat 1.5m, danger=1.0). `tracked_by_id` maps
    track_id -> TrackedObject for speed/history features; kept as `None`
    (untyped to avoid a hard dependency) by callers that don't have tracks.
    """
    origin_x = ego.x - width_m / 2
    origin_y = ego.y - height_m / 2
    costmap = GridCostmap(width_m, height_m, resolution_m, origin_x, origin_y)

    use_learned = cost_head is not None and getattr(cost_head, "loaded", False)
    if cost_head is not None and not use_learned:
        logger.debug("cost_head provided but unloaded -- table costs for all points.")

    # Phase 1: per-trajectory features (or table fallback where no track).
    # Phase 2: ONE batched MLP forward for the whole tick, not N tiny ones
    # (torch per-call overhead would otherwise dominate small scenes).
    from pipeline.cost_head import DEFAULT_BASE_RADIUS_M, build_feature_vector, ttc_estimate

    feats, feat_idx, costs = [], [], []
    for i, traj in enumerate(predictions):
        costs.append(None)  # placeholder; filled below
        track = tracked_by_id.get(traj.track_id) if tracked_by_id else None
        if not use_learned or track is None:
            costs[i] = (DEFAULT_BASE_RADIUS_M, 1.0)
            continue
        vx, vy = track.velocity
        hist = track.position_history
        ox, oy = hist[-1] if hist else traj.points[0]
        dist = float(np.hypot(ox - ego.x, oy - ego.y))
        ego_vx, ego_vy = ego.speed * np.cos(ego.yaw), ego.speed * np.sin(ego.yaw)
        dx, dy = (ox - ego.x) / max(dist, 1e-3), (oy - ego.y) / max(dist, 1e-3)
        closing = -((vx - ego_vx) * dx + (vy - ego_vy) * dy)
        feats.append(build_feature_vector(
            traj.class_name, float(np.hypot(vx, vy)),
            ttc_estimate(dist, closing), len(hist), traj.probability, ego.speed))
        feat_idx.append(i)
    if feats:
        learned = cost_head.predict_batch(feats, [predictions[i].class_name for i in feat_idx])
        for i, c in zip(feat_idx, learned):
            costs[i] = c

    for traj, (radius_m, danger) in zip(predictions, costs):
        for k, (px, py) in enumerate(traj.points):
            # Cost decays for further-future points (more uncertain, and
            # by the time the ego vehicle gets there conditions may have
            # changed) and scales with the mode's probability and the
            # learned danger (danger=1.0 reproduces old behavior exactly).
            time_decay = 1.0 - (k / max(len(traj.points), 1)) * 0.5
            base_cost = danger * traj.probability * time_decay
            costmap.add_obstacle_inflation(px, py, base_cost=base_cost, inflation_radius_m=radius_m)

    return costmap


def _point_costs(ego: EgoState, traj: PredictedTrajectory, cost_head, tracked_by_id: dict | None, dt: float):
    """(radius_m, danger) for one predicted trajectory. Table values unless
    a loaded cost_head AND a matching track are both available.

    Kept as the single-point entry (used by tests/debugging); the hot path
    in build_costmap_from_predictions batches via predict_batch instead."""

    from pipeline.cost_head import DEFAULT_BASE_RADIUS_M, build_feature_vector, ttc_estimate

    if cost_head is None or not getattr(cost_head, "loaded", False):
        return DEFAULT_BASE_RADIUS_M, 1.0
    track = tracked_by_id.get(traj.track_id) if tracked_by_id else None
    if track is None:
        return DEFAULT_BASE_RADIUS_M, 1.0
    vx, vy = track.velocity
    speed = float(np.hypot(vx, vy))
    hist = track.position_history
    ox, oy = hist[-1] if hist else traj.points[0]
    dist = float(np.hypot(ox - ego.x, oy - ego.y))
    # Closing speed along line of sight, ego motion included (guard-only).
    ego_vx, ego_vy = ego.speed * np.cos(ego.yaw), ego.speed * np.sin(ego.yaw)
    dx, dy = (ox - ego.x) / max(dist, 1e-3), (oy - ego.y) / max(dist, 1e-3)
    closing = -((vx - ego_vx) * dx + (vy - ego_vy) * dy)
    feats = build_feature_vector(
        traj.class_name, speed, ttc_estimate(dist, closing),
        len(hist), traj.probability, ego.speed,
    )
    return cost_head.predict_costs(feats, traj.class_name, traj.probability)


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


def decimate_waypoints(waypoints: list[tuple[float, float]], min_spacing_m: float = 1.5) -> list[tuple[float, float]]:
    """Resample to ~min_spacing so downstream control gets ~1-2m spaced
    targets instead of every 0.5m cell center. Interpolates along long
    straight legs instead of just filtering -- a filter-only version
    collapses a 100m straight leg to 2 waypoints, leaving PurePursuit's
    lookahead search (controller.py) with nothing between here and the
    horizon. Always keeps the final goal."""
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
        waypoint_spacing_m: float | None = 1.5,
        cost_head=None,
    ):
        self._last_path: list[tuple[float, float]] | None = None
        self._last_costmap_signature: float | None = None
        self.replan_cost_change_threshold = replan_cost_change_threshold
        # Learned cost head (pipeline.cost_head.CostHead) or None. None /
        # unloaded == pre-MLP behavior exactly; see build_costmap_from_predictions.
        self.cost_head = cost_head
        # Resample spacing for output waypoints (see decimate_waypoints).
        # None disables resampling (raw 0.5m cell centers, pre-port behavior).
        self.waypoint_spacing_m = waypoint_spacing_m

    def plan(
        self,
        ego: EgoState,
        predictions: list[PredictedTrajectory],
        non_drivable_points: list[tuple[float, float]] | None = None,
        non_drivable_cost: float = 1.0,
        force_replan: bool = False,
        tracked_by_id: dict | None = None,
        dt: float = 0.05,
    ) -> PlannedPath:
        """force_replan: set True to bypass the costmap-signature check
        and always search fresh -- wired from DecisionLogic's
        replan_requested output (pipeline/decision_logic.py) so the
        decision layer can force a fresh plan (e.g. on entering
        OBSTACLE_DETECTED/EMERGENCY_BRAKE/REPLAN) even if the costmap's
        own change-detection wouldn't have triggered one yet.
        """
        t0 = time.perf_counter()
        costmap = build_costmap_from_predictions(
            ego, predictions,
            cost_head=self.cost_head, tracked_by_id=tracked_by_id, dt=dt,
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
        if force_replan:
            logger.debug("force_replan=True (from DecisionLogic) -- bypassing signature check.")

        if non_drivable_points:
            costmap.rasterize_non_drivable(non_drivable_points, cost=non_drivable_cost)
            logger.debug("Applied %d non-drivable points to costmap (after signature computed).", len(non_drivable_points))
        needs_replan = (
            force_replan
            or self._last_path is None
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
            # Failure fallback: hold the last good path (don't leave control
            # with nothing) unless there never was one. One noisy tick with
            # no feasible A* solution must not halt the vehicle -- and with
            # soft costs a true None is rare, so this triggers only on real
            # blockage, not sampling noise.
            if self._last_path is not None:
                logger.warning("A* found NO FEASIBLE PATH (start=%s goal=%s) -- holding last path. (%.2fms)",
                               start_rc, goal_rc, elapsed_ms)
                return PlannedPath(list(self._last_path), is_valid=True, replanned=False,
                                   notes="A* failed this tick; holding last good path.")
            logger.warning("A* found NO FEASIBLE PATH (start=%s goal=%s) and no prior path. (%.2fms)",
                           start_rc, goal_rc, elapsed_ms)
            return PlannedPath([], is_valid=False, replanned=True, notes="No feasible path found by A*.")

        raw_waypoints = [costmap.grid_to_world(r, c) for r, c in grid_path]
        waypoints = (decimate_waypoints(raw_waypoints, min_spacing_m=self.waypoint_spacing_m)
                     if self.waypoint_spacing_m else raw_waypoints)
        self._last_path = waypoints
        self._last_costmap_signature = signature

        logger.info("Replanned: %d waypoints (raw %d), %.2fms, costmap signature %.3f.",
                    len(waypoints), len(raw_waypoints), elapsed_ms, signature)
        return PlannedPath(waypoints, is_valid=True, replanned=True,
                            notes=f"Fresh A* plan, {elapsed_ms:.2f}ms.")
