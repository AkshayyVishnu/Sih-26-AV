"""
Fuses YOLO's 2D image detections with the raw LiDAR point cloud to get a
3D position/distance per detection -- classical projection, no learned
model, per docs/architecture.md Stage 3's sensor-fusion decision.

KNOWN PITFALL (from docs/architecture.md, CARLA issue #3795, carried
over here deliberately): the camera<->LiDAR extrinsic calibration is not
guaranteed constant across frames for a fixed sensor rig in CARLA's own
reference example. If fused distances look wrong or jump around, check
this FIRST before assuming the fusion math below is broken.
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from pipeline.types import Detection, FusedDetection

logger = logging.getLogger("pipeline.perception_fusion")


class LidarCameraFuser:
    def __init__(
        self,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        min_points_for_valid_fusion: int = 3,
    ):
        """
        camera_intrinsic: 3x3 K matrix (fx, 0, cx / 0, fy, cy / 0, 0, 1).
        camera_to_lidar_extrinsic: 4x4 homogeneous transform, LiDAR frame
            -> camera frame (i.e. p_cam = extrinsic @ p_lidar_homogeneous).
            MUST be calibrated for your actual CARLA sensor mount -- do
            not assume the default/example values transfer to your rig.
        min_points_for_valid_fusion: if fewer LiDAR points land inside a
            detection's bbox than this, treat the fusion as failed rather
            than trust a 1-2-point noisy estimate.
        """
        self.K = camera_intrinsic
        self.extrinsic = camera_to_lidar_extrinsic
        self.min_points = min_points_for_valid_fusion

    def _project_lidar_to_image(self, lidar_points_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """lidar_points_xyz: (N, 3) array in LiDAR frame.
        Returns (pixel_uv (N,2), depth (N,), in_front_mask (N,) bool).
        """
        n = lidar_points_xyz.shape[0]
        homogeneous = np.hstack([lidar_points_xyz, np.ones((n, 1))])  # (N, 4)
        cam_frame = (self.extrinsic @ homogeneous.T).T  # (N, 4) -> use first 3 cols
        cam_xyz = cam_frame[:, :3]

        depth = cam_xyz[:, 2]
        in_front = depth > 0.1  # ignore points behind or right at the camera plane

        # Perspective projection: avoid div-by-zero for points we're about to mask out anyway.
        safe_depth = np.where(in_front, depth, 1.0)
        pixel_h = (self.K @ cam_xyz.T).T  # (N, 3)
        pixel_uv = pixel_h[:, :2] / safe_depth[:, None]

        return pixel_uv, depth, in_front

    def fuse(self, detections: list[Detection], lidar_points_xyz: np.ndarray) -> list[FusedDetection]:
        """lidar_points_xyz: (N, 3) raw LiDAR point cloud for this tick,
        LiDAR frame, ego/vehicle-relative.
        """
        if lidar_points_xyz.shape[0] == 0:
            logger.warning("Empty LiDAR point cloud this tick -- all detections will fall back to no-fusion.")
            return [FusedDetection(d, None, None, 0) for d in detections]

        pixel_uv, depth, in_front = self._project_lidar_to_image(lidar_points_xyz)

        fused: list[FusedDetection] = []
        for det in detections:
            mask = (
                in_front
                & (pixel_uv[:, 0] >= det.x1) & (pixel_uv[:, 0] <= det.x2)
                & (pixel_uv[:, 1] >= det.y1) & (pixel_uv[:, 1] <= det.y2)
            )
            n_points = int(mask.sum())

            if n_points < self.min_points:
                logger.debug(
                    "Detection %s (conf=%.2f) got only %d LiDAR points (need >=%d) -- fusion FAILED, no distance estimate.",
                    det.class_name, det.confidence, n_points, self.min_points,
                )
                fused.append(FusedDetection(det, None, None, n_points))
                continue

            points_in_box = lidar_points_xyz[mask]
            median_position = tuple(np.median(points_in_box, axis=0).tolist())
            distance = float(np.linalg.norm(median_position))

            logger.debug(
                "Detection %s fused: %d LiDAR points, position=%s, distance=%.2fm",
                det.class_name, n_points, median_position, distance,
            )
            fused.append(FusedDetection(det, median_position, distance, n_points))

        return fused


def make_fallback_distance_estimator(assumed_object_height_m: float = 1.6):
    """If LiDAR fusion fails for a detection (no points, or fewer than
    min_points), this gives a rough monocular distance estimate from bbox
    height as a last resort -- much less accurate than LiDAR, but better
    than dropping the detection entirely. Use ONLY when FusedDetection.distance_m is None.
    """
    def estimate(det: Detection, focal_length_px: float) -> float:
        bbox_height_px = max(det.y2 - det.y1, 1.0)
        return (assumed_object_height_m * focal_length_px) / bbox_height_px
    return estimate
