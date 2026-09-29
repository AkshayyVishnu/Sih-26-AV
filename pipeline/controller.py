"""
Low-level control: PlannedPath waypoints -> steer/throttle/brake.

DECISION (see docs/pipeline-decision-log.md): Pure Pursuit for steering
(geometric, no per-vehicle model tuning needed, standard choice for
waypoint following) + a simple proportional controller on speed for
throttle/brake, with target speed reduced automatically on sharp
curvature. Chosen over PID-only because docs/architecture.md's own
controller-comparison research found PID has the weakest high-curvature
tracking of the standard options; chosen over MPC because implementing a
real MPC solver from scratch under this timeline isn't worth the risk
against Pure Pursuit's simplicity and predictability -- MATLAB's
Adaptive MPC block remains the documented Path A equivalent.

Output range matches carla.VehicleControl directly:
    vehicle.apply_control(carla.VehicleControl(
        throttle=cmd.throttle, steer=cmd.steer, brake=cmd.brake))
"""
from __future__ import annotations

import logging

import numpy as np

from pipeline.types import ControlCommand, EgoState, PlannedPath

logger = logging.getLogger("pipeline.controller")


class PurePursuitController:
    def __init__(
        self,
        lookahead_base_m: float = 4.0,
        lookahead_speed_gain: float = 0.5,
        wheelbase_m: float = 2.8,
        max_steer_rad: float = 0.6,
        target_speed_mps: float = 8.0,
        speed_kp: float = 0.6,
        curvature_slowdown_gain: float = 2.0,
    ):
        """
        lookahead_base_m / lookahead_speed_gain: lookahead distance grows
            with speed (lookahead = base + gain*speed) -- standard Pure
            Pursuit practice; avoids oscillation at higher speed and
            "twitchiness" at low speed.
        wheelbase_m: PLACEHOLDER -- replace with your actual ego
            vehicle's real wheelbase (check its CARLA blueprint/vehicle
            physics attributes). Wrong wheelbase makes the steering
            geometry systematically wrong, not just "a bit off" -- this
            is a real value to get from your friend along with the
            camera/LiDAR calibration.
        target_speed_mps: default cruise speed, automatically reduced on
            sharp turns via curvature_slowdown_gain.
        """
        self.lookahead_base_m = lookahead_base_m
        self.lookahead_speed_gain = lookahead_speed_gain
        self.wheelbase_m = wheelbase_m
        self.max_steer_rad = max_steer_rad
        self.target_speed_mps = target_speed_mps
        self.speed_kp = speed_kp
        self.curvature_slowdown_gain = curvature_slowdown_gain

    def _find_lookahead_point(self, ego: EgoState, waypoints: list[tuple[float, float]], lookahead_m: float):
        for wx, wy in waypoints:
            if np.hypot(wx - ego.x, wy - ego.y) >= lookahead_m:
                return wx, wy
        return waypoints[-1] if waypoints else None

    def compute(self, ego: EgoState, path: PlannedPath) -> ControlCommand:
        if not path.is_valid or not path.waypoints:
            logger.warning("No valid path -- issuing full brake as a safe default.")
            return ControlCommand(throttle=0.0, steer=0.0, brake=1.0)

        lookahead = self.lookahead_base_m + self.lookahead_speed_gain * ego.speed
        target = self._find_lookahead_point(ego, path.waypoints, lookahead)
        if target is None:
            logger.warning("Could not find a lookahead point -- issuing full brake.")
            return ControlCommand(throttle=0.0, steer=0.0, brake=1.0)

        tx, ty = target

        # Transform target into the ego's local (heading-aligned) frame.
        dx = tx - ego.x
        dy = ty - ego.y
        local_x = dx * np.cos(-ego.yaw) - dy * np.sin(-ego.yaw)
        local_y = dx * np.sin(-ego.yaw) + dy * np.cos(-ego.yaw)

        # Pure Pursuit curvature law: kappa = 2*y / L^2 (L = distance to target point)
        L = max(float(np.hypot(local_x, local_y)), 0.1)
        curvature = 2.0 * local_y / (L ** 2)
        steer_rad = np.arctan(curvature * self.wheelbase_m)
        steer = float(np.clip(steer_rad / self.max_steer_rad, -1.0, 1.0))

        # Speed control: slow down proportional to how sharp the turn is,
        # simple proportional throttle/brake toward that target speed.
        target_speed = self.target_speed_mps / (1.0 + self.curvature_slowdown_gain * abs(curvature))
        speed_error = target_speed - ego.speed

        if speed_error >= 0:
            throttle = float(np.clip(self.speed_kp * speed_error, 0.0, 1.0))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(np.clip(-self.speed_kp * speed_error, 0.0, 1.0))

        logger.debug(
            "Control: target=(%.2f,%.2f) local=(%.2f,%.2f) curvature=%.3f steer=%.3f "
            "target_speed=%.2f throttle=%.2f brake=%.2f",
            tx, ty, local_x, local_y, curvature, steer, target_speed, throttle, brake,
        )

        return ControlCommand(throttle=throttle, steer=steer, brake=brake)
