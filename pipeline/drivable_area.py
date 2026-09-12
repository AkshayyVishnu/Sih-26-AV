"""
Drivable-area estimation: classifies raw LiDAR points as
on-road/drivable vs. not, using CARLA's ground-truth semantic
segmentation camera as a free, zero-training stand-in for a learned
segmentation model.

DECISION (see docs/pipeline-decision-log.md): this is deliberately
simulation-only. In the real world you wouldn't have ground-truth
per-pixel labels -- the production equivalent would be a learned
segmentation model (e.g. DeepLab/SegFormer fine-tuned on IDD's
segmentation labels, per docs/component-deep-dive.md). Using CARLA's
own ground truth here is an honest, disclosed shortcut for the demo
timeline, not a hidden one -- state this plainly in the technical report.

WHY THIS EXISTS AT ALL: object detection + LiDAR (pipeline/perception_fusion.py)
tells you where obstacles are, but nothing about where the drivable road
surface itself is. On unmarked roads (this PS's whole premise) there's no
lane line to substitute for that boundary -- without this, the planner
has no signal stopping it from planning a path onto a sidewalk or off
the road entirely.
"""
from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger("pipeline.drivable_area")

# CARLA's semantic segmentation tag IDs (raw values from the segmentation
# camera's red channel, BEFORE CityScapes-palette color conversion --
# request the raw image, don't call image.convert(CityScapesPalette)).
#
# CONFIRMED against CARLA's official docs for version 0.9.16 specifically
# (carla.readthedocs.io/en/0.9.16/ref_sensors/), 2026-09 -- Road=1,
# RoadLine=24. An earlier draft of this file guessed Road=7/RoadLine=6,
# which are actually TrafficLight/Pole -- a real bug, caught before ever
# running against live data. CARLA's own docs explicitly note "tags
# changed from version 0.9.13 to 0.9.14" -- if you end up on a different
# CARLA server version than 0.9.16, RE-VERIFY these numbers against that
# version's own docs page (carla.readthedocs.io/en/<your-version>/ref_sensors/)
# rather than trusting this default.
TAG_ROAD = 1
TAG_ROADLINE = 24
DEFAULT_DRIVABLE_TAG_IDS = frozenset({TAG_ROAD, TAG_ROADLINE})


class DrivableAreaEstimator:
    def __init__(
        self,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        drivable_tag_ids: frozenset[int] | None = None,
    ):
        """
        camera_intrinsic / camera_to_lidar_extrinsic: SAME matrices you
        pass to LidarCameraFuser -- this assumes the semantic segmentation
        camera and the RGB detection camera are co-located (same
        transform). If they're mounted separately in CARLA, pass this
        class its own extrinsic instead.
        """
        self.K = camera_intrinsic
        self.extrinsic = camera_to_lidar_extrinsic
        self.drivable_tag_ids = drivable_tag_ids or DEFAULT_DRIVABLE_TAG_IDS

    def _project_lidar_to_image(self, lidar_points_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Identical math to LidarCameraFuser._project_lidar_to_image --
        deliberately duplicated rather than imported/shared, to avoid
        touching that already-verified module under time pressure. If
        you refactor later, this is the obvious place to unify them.
        """
        n = lidar_points_xyz.shape[0]
        homogeneous = np.hstack([lidar_points_xyz, np.ones((n, 1))])
        cam_frame = (self.extrinsic @ homogeneous.T).T
        cam_xyz = cam_frame[:, :3]

        depth = cam_xyz[:, 2]
        in_front = depth > 0.1

        safe_depth = np.where(in_front, depth, 1.0)
        pixel_h = (self.K @ cam_xyz.T).T
        pixel_uv = pixel_h[:, :2] / safe_depth[:, None]

        return pixel_uv, depth, in_front

    def classify(
        self,
        lidar_points_xyz: np.ndarray,
        segmentation_tags: np.ndarray,
    ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
        """
        segmentation_tags: (H, W) integer array -- raw CARLA semantic tag
        per pixel, same image dimensions the camera_intrinsic assumes.

        Returns (drivable_xy, non_drivable_xy): lists of (x, y) ego-frame
        points, ready to hand to GridCostmap.rasterize_non_drivable().
        Points that don't land in the image frame at all are excluded
        from both lists (no information either way) and counted in the
        log line -- a large out-of-frame count usually means your
        intrinsic/extrinsic don't actually match the segmentation
        camera's real configuration.
        """
        if lidar_points_xyz.shape[0] == 0:
            logger.warning("Empty LiDAR point cloud -- no drivable-area classification possible this tick.")
            return [], []

        h, w = segmentation_tags.shape
        pixel_uv, _, in_front = self._project_lidar_to_image(lidar_points_xyz)

        u = pixel_uv[:, 0].astype(int)
        v = pixel_uv[:, 1].astype(int)
        in_frame = in_front & (u >= 0) & (u < w) & (v >= 0) & (v < h)

        drivable_xy: list[tuple[float, float]] = []
        non_drivable_xy: list[tuple[float, float]] = []

        idx_in_frame = np.nonzero(in_frame)[0]
        for i in idx_in_frame:
            tag = int(segmentation_tags[v[i], u[i]])
            xy = (float(lidar_points_xyz[i, 0]), float(lidar_points_xyz[i, 1]))
            if tag in self.drivable_tag_ids:
                drivable_xy.append(xy)
            else:
                non_drivable_xy.append(xy)

        out_of_frame = lidar_points_xyz.shape[0] - len(idx_in_frame)
        logger.debug(
            "Drivable-area classification: %d drivable, %d non-drivable, %d out-of-frame (of %d total LiDAR points).",
            len(drivable_xy), len(non_drivable_xy), out_of_frame, lidar_points_xyz.shape[0],
        )
        if out_of_frame > lidar_points_xyz.shape[0] * 0.5:
            logger.warning(
                "More than half of LiDAR points (%d/%d) fell outside the segmentation image frame -- "
                "check that camera_intrinsic/extrinsic actually match the segmentation camera's real config.",
                out_of_frame, lidar_points_xyz.shape[0],
            )

        return drivable_xy, non_drivable_xy
