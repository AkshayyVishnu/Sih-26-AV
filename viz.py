"""
Live debug dashboard: one tiled pygame window showing what the pipeline
sees (RGB + ground-truth detection boxes, colorized segmentation, LiDAR
top-down view + planned path) and what it outputs (throttle/steer/brake
over time) -- built specifically so issues like a detector flickering
in/out of frame are visible live instead of only discoverable by
grepping logs after a run.

DELIBERATELY separate from carla_runtime.py and pipeline/: this module
never imports `carla` at all -- everything it needs (numpy sensor
arrays, pipeline.types dataclasses) is already available in the caller's
main loop each tick. Same "keep CARLA out of reusable logic" boundary
pipeline/ already follows (see pipeline/pipeline.py's docstring).

Pattern follows CARLA's own PythonAPI/examples/visualize_multiple_sensors.py:
one pygame window subdivided into a grid, each panel blitted into its own
cell -- chosen over separate OS windows (would need opencv-python, not
currently installed) or matplotlib (a second GUI backend/thread in the
same process) specifically to avoid a new dependency, per the decision
recorded in the plan that added this file.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import pygame

from pipeline.types import ControlCommand, Detection, EgoState

LIDAR_RANGE_M = 40.0  # BEV panel covers +/- this many meters around the ego

# CARLA's standard semantic segmentation tag -> RGB color, for display
# only (not safety-relevant, so a standard reference palette is fine
# without further verification -- unlike pipeline/drivable_area.py's
# TAG_ROAD/TAG_ROADLINE, which ARE safety-relevant and were verified
# against CARLA 0.9.16's own docs directly).
SEG_COLOR_MAP: dict[int, tuple[int, int, int]] = {
    0: (0, 0, 0),          # Unlabeled
    1: (128, 64, 128),     # Road
    2: (244, 35, 232),     # SideWalk
    3: (70, 70, 70),       # Building
    4: (102, 102, 156),    # Wall
    5: (190, 153, 153),    # Fence
    6: (153, 153, 153),    # Pole
    7: (250, 170, 30),     # TrafficLight
    8: (220, 220, 0),      # TrafficSign
    9: (107, 142, 35),     # Vegetation
    10: (145, 170, 100),   # Terrain
    11: (70, 130, 180),    # Sky
    12: (220, 20, 60),     # Pedestrian
    13: (255, 0, 0),       # Rider
    14: (0, 0, 142),       # Car
    15: (0, 0, 70),        # Truck
    16: (0, 60, 100),      # Bus
    17: (0, 80, 100),      # Train
    18: (0, 0, 230),       # Motorcycle
    19: (119, 11, 32),     # Bicycle
    20: (110, 190, 160),   # Static
    21: (170, 120, 50),    # Dynamic
    22: (55, 90, 80),      # Other
    23: (45, 60, 150),     # Water
    24: (157, 234, 50),    # RoadLine -- matches pipeline/drivable_area.py's TAG_ROADLINE
    25: (81, 0, 81),       # Ground
    26: (150, 100, 100),   # Bridge
    27: (230, 150, 140),   # RailTrack
    28: (180, 165, 180),   # GuardRail
}
_DEFAULT_SEG_COLOR = (60, 60, 60)


def _build_seg_lut() -> np.ndarray:
    lut = np.full((256, 3), _DEFAULT_SEG_COLOR, dtype=np.uint8)
    for tag, color in SEG_COLOR_MAP.items():
        lut[tag] = color
    return lut


_SEG_LUT = _build_seg_lut()


def colorize_segmentation(seg_tags: np.ndarray) -> np.ndarray:
    """Raw CARLA tag array (H, W) int -> RGB image (H, W, 3) uint8, via a
    vectorized LUT lookup. Operates on carla_runtime.carla_segmentation_to_tags()'s
    return value, which is already a real copy decoupled from the source
    carla.Image's buffer -- deliberately does NOT call CARLA's own
    image.convert(CityScapesPalette), which would mutate that buffer in
    place and risk corrupting the raw tags if this ever got called before
    the pipeline reads them. This sidesteps that ordering hazard entirely.
    """
    clipped = np.clip(seg_tags, 0, 255).astype(np.int32)
    return _SEG_LUT[clipped]


def render_lidar_bev(lidar_xyz: np.ndarray, size_px: int, range_m: float = LIDAR_RANGE_M) -> np.ndarray:
    """Top-down occupancy raster of a raw LiDAR point cloud (ego-local
    frame, x-forward/y-right/z-up -- see carla_runtime.py's extrinsic
    docstring for the axis-convention story), ego-centered, forward = up.
    Same scatter-into-pixel-space technique CARLA's own
    visualize_multiple_sensors.py uses for its LiDAR panel.
    """
    img = np.zeros((size_px, size_px, 3), dtype=np.uint8)
    if lidar_xyz.shape[0] == 0:
        return img
    x = lidar_xyz[:, 0]
    y = lidar_xyz[:, 1]
    scale = size_px / (2 * range_m)
    col = (size_px // 2 + y * scale).astype(np.int32)
    row = (size_px // 2 - x * scale).astype(np.int32)
    mask = (col >= 0) & (col < size_px) & (row >= 0) & (row < size_px)
    img[row[mask], col[mask]] = (0, 255, 120)
    return img


def world_to_local_xy(wx: float, wy: float, ego_state: EgoState) -> tuple[float, float]:
    """Inverse of pipeline/pipeline.py's private _local_to_world_xy
    (rotate by -yaw, translate by -ego) -- reimplemented here since it
    isn't exported. Used only to bring planner waypoints (world frame)
    into the same ego-local frame as raw LiDAR points, for the BEV panel.
    """
    dx = wx - ego_state.x
    dy = wy - ego_state.y
    cos_y, sin_y = math.cos(ego_state.yaw), math.sin(ego_state.yaw)
    local_x = dx * cos_y + dy * sin_y
    local_y = -dx * sin_y + dy * cos_y
    return local_x, local_y


def _array_to_surface(arr: np.ndarray) -> pygame.Surface:
    """(H, W, 3) uint8 RGB array -> pygame Surface. Same
    array.swapaxes(0, 1) CARLA's own manual_control.py/
    visualize_multiple_sensors.py use -- pygame surfaces are column-major
    (width, height), numpy images are (height, width, channels).
    """
    return pygame.surfarray.make_surface(arr.swapaxes(0, 1))


def _label(surface: pygame.Surface, font: pygame.font.Font, text: str, rect: pygame.Rect):
    txt = font.render(text, True, (200, 200, 200))
    surface.blit(txt, (rect.x + 4, rect.y + 2))


def draw_detections(
    surface: pygame.Surface,
    detections: list[Detection],
    rect: pygame.Rect,
    scale_x: float,
    scale_y: float,
    font: pygame.font.Font,
):
    """Draws each Detection's box + class/confidence label, scaled from
    the source camera image's pixel space into the RGB panel's on-screen
    cell. This is the direct debugging payoff: watching a box flicker
    in/out right at the panel edge makes a detector's FOV/range behavior
    visible instead of only inferable from a 0-det/2-det count in a log.
    """
    for det in detections:
        x1 = rect.x + det.x1 * scale_x
        y1 = rect.y + det.y1 * scale_y
        x2 = rect.x + det.x2 * scale_x
        y2 = rect.y + det.y2 * scale_y
        color = (255, 60, 60) if det.class_name == "pedestrian" else (60, 160, 255)
        pygame.draw.rect(surface, color, pygame.Rect(x1, y1, max(x2 - x1, 1), max(y2 - y1, 1)), 2)
        label = f"{det.class_name} {det.confidence:.2f}"
        text = font.render(label, True, color)
        surface.blit(text, (x1, max(y1 - 14, rect.y)))


def _draw_bev_overlay(
    surface: pygame.Surface,
    planned_waypoints_world: list[tuple[float, float]],
    ego_state: EgoState,
    offset_x: int,
    offset_y: int,
    bev_dim: int,
    range_m: float,
):
    scale = bev_dim / (2 * range_m)
    center_x = offset_x + bev_dim // 2
    center_y = offset_y + bev_dim // 2

    if planned_waypoints_world:
        pts = []
        for wx, wy in planned_waypoints_world:
            lx, ly = world_to_local_xy(wx, wy, ego_state)
            pts.append((center_x + ly * scale, center_y - lx * scale))
        if len(pts) >= 2:
            pygame.draw.lines(surface, (255, 80, 80), False, pts, 2)

    # Ego marker + forward-heading tick (forward is always "up" in this
    # ego-local BEV, by construction).
    pygame.draw.circle(surface, (255, 255, 0), (center_x, center_y), 5)
    pygame.draw.line(surface, (255, 255, 0), (center_x, center_y), (center_x, center_y - 15), 2)


class ControlGraphPanel:
    """Rolling line chart of throttle/steer/brake vs. sim time -- hand-drawn
    with pygame primitives rather than matplotlib, to avoid a second GUI
    backend/thread in the same process (see this file's module docstring).
    """

    def __init__(self, window_s: float = 15.0):
        self.window_s = window_s
        self.samples: deque[tuple[float, float, float, float]] = deque()  # (t, throttle, steer, brake)

    def update(self, t: float, control: ControlCommand):
        self.samples.append((t, control.throttle, control.steer, control.brake))
        cutoff = t - self.window_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def draw(self, surface: pygame.Surface, rect: pygame.Rect, font: pygame.font.Font):
        pygame.draw.rect(surface, (20, 20, 20), rect)
        if len(self.samples) < 2:
            return

        t_latest = self.samples[-1][0]
        t_earliest = t_latest - self.window_s
        span = max(t_latest - t_earliest, 1e-6)

        def to_px(t: float, v: float) -> tuple[float, float]:
            px = rect.x + (t - t_earliest) / span * rect.w
            py = rect.y + rect.h / 2 - v * (rect.h / 2 - 10)
            return (px, py)

        for v in (-1.0, -0.5, 0.0, 0.5, 1.0):
            _, py = to_px(t_earliest, v)
            color = (100, 100, 100) if v == 0.0 else (55, 55, 55)
            pygame.draw.line(surface, color, (rect.x, py), (rect.x + rect.w, py), 1)

        throttle_pts = [to_px(t, v) for t, v, _, _ in self.samples]
        steer_pts = [to_px(t, s) for t, _, s, _ in self.samples]
        brake_pts = [to_px(t, b) for t, _, _, b in self.samples]
        pygame.draw.lines(surface, (80, 220, 80), False, throttle_pts, 2)
        pygame.draw.lines(surface, (80, 140, 255), False, steer_pts, 2)
        pygame.draw.lines(surface, (255, 80, 80), False, brake_pts, 2)

        for i, (label, color) in enumerate([("throttle", (80, 220, 80)), ("steer", (80, 140, 255)), ("brake", (255, 80, 80))]):
            ly = rect.y + 4 + i * 15
            pygame.draw.rect(surface, color, pygame.Rect(rect.x + rect.w - 90, ly, 10, 10))
            surface.blit(font.render(label, True, (220, 220, 220)), (rect.x + rect.w - 75, ly - 2))


class Dashboard:
    """Owns the single tiled pygame window (2x2 grid: RGB+detections /
    segmentation / LiDAR BEV+planned-path / control graph). Pygame init
    is wrapped in try/except -- a machine with no display just gets a
    warning and a disabled (no-op) dashboard, not a crashed scenario run.
    """

    WINDOW_W = 1000
    WINDOW_H = 800

    def __init__(self):
        self.enabled = True
        try:
            pygame.init()
            pygame.font.init()
            self.screen = pygame.display.set_mode((self.WINDOW_W, self.WINDOW_H))
            pygame.display.set_caption("AV Pipeline Debug Dashboard")
            self.font = pygame.font.SysFont("monospace", 14)
            self.small_font = pygame.font.SysFont("monospace", 12)
        except Exception as e:  # pragma: no cover -- environment-dependent
            print(f"[viz] Failed to initialize pygame display ({type(e).__name__}: {e}) -- visualization disabled.")
            self.enabled = False
            return

        cw, ch = self.WINDOW_W // 2, self.WINDOW_H // 2
        self.rgb_rect = pygame.Rect(0, 0, cw, ch)
        self.seg_rect = pygame.Rect(cw, 0, cw, ch)
        self.lidar_rect = pygame.Rect(0, ch, cw, ch)
        self.graph_rect = pygame.Rect(cw, ch, cw, ch)
        self.control_graph = ControlGraphPanel()

    def update(
        self,
        rgb_array: np.ndarray,
        seg_tags: np.ndarray,
        lidar_xyz: np.ndarray,
        detections: list[Detection],
        control: ControlCommand,
        planned_waypoints_world: list[tuple[float, float]],
        ego_state: EgoState,
        sim_time_s: float,
    ) -> bool:
        """Returns False if the user closed the window (X button / Esc/Q)
        -- the caller should stop calling update() from then on, but this
        does NOT stop the CARLA run itself, only the debug view.
        """
        if not self.enabled:
            return False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.close()
                return False
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                self.close()
                return False

        self.screen.fill((10, 10, 10))

        # --- RGB + detections ---
        rgb_surface = pygame.transform.scale(_array_to_surface(rgb_array), (self.rgb_rect.w, self.rgb_rect.h))
        self.screen.blit(rgb_surface, self.rgb_rect.topleft)
        scale_x = self.rgb_rect.w / rgb_array.shape[1]
        scale_y = self.rgb_rect.h / rgb_array.shape[0]
        draw_detections(self.screen, detections, self.rgb_rect, scale_x, scale_y, self.small_font)
        _label(self.screen, self.font, "RGB + detections", self.rgb_rect)

        # --- Segmentation ---
        seg_surface = pygame.transform.scale(_array_to_surface(colorize_segmentation(seg_tags)), (self.seg_rect.w, self.seg_rect.h))
        self.screen.blit(seg_surface, self.seg_rect.topleft)
        _label(self.screen, self.font, "Segmentation", self.seg_rect)

        # --- LiDAR BEV + planned path ---
        bev_dim = min(self.lidar_rect.w, self.lidar_rect.h)
        bev_surface = _array_to_surface(render_lidar_bev(lidar_xyz, bev_dim))
        offset_x = self.lidar_rect.x + (self.lidar_rect.w - bev_dim) // 2
        offset_y = self.lidar_rect.y + (self.lidar_rect.h - bev_dim) // 2
        self.screen.blit(bev_surface, (offset_x, offset_y))
        _draw_bev_overlay(self.screen, planned_waypoints_world, ego_state, offset_x, offset_y, bev_dim, LIDAR_RANGE_M)
        _label(self.screen, self.font, "LiDAR BEV + planned path", self.lidar_rect)

        # --- Control graph ---
        self.control_graph.update(sim_time_s, control)
        self.control_graph.draw(self.screen, self.graph_rect, self.small_font)
        _label(self.screen, self.font, "throttle(green) / steer(blue) / brake(red) vs time", self.graph_rect)

        pygame.display.flip()
        return True

    def close(self):
        if self.enabled:
            pygame.quit()
            self.enabled = False
