"""
Motion prediction stage: TrackedObject (with position history + velocity)
-> PredictedTrajectory (future positions, possibly multimodal).

DECISION (see docs/pipeline-decision-log.md for the full reasoning):
given the 2-day timeline, the PRIMARY predictor here is a constant-
velocity Kalman extrapolation -- fully working, zero checkpoint/API risk,
and this is exactly the native fallback docs/architecture.md already
specifies for this stage (trackingIMM-equivalent). MoFlow is wired in as
a clearly-marked extension point, not fully reverse-engineered, because
its real inference API (models/flow_matching.py, models/imle.py,
backbone_eth_ucy.py) needs a downloaded checkpoint + exact tensor-shape
matching against cfg/eth_ucy/imle.yml that wasn't safe to guess against
the clock. Swap it in once you have time to verify it end-to-end against
a real checkpoint -- the interface below is designed so that's a
drop-in replacement, not a rewrite.
"""
from __future__ import annotations

import abc
import logging

import numpy as np

from pipeline.types import PredictedTrajectory, TrackedObject

logger = logging.getLogger("pipeline.predictor")

PREDICTION_HORIZON_STEPS = 12  # how many future steps to predict, tune against your dt and planner horizon

# Extra lateral uncertainty (std dev, meters, applied at the far end of the horizon)
# per class -- rough, hand-picked given no time to fit this from data. Erratic
# classes get wider spread so the planner's costmap inflates more around them.
_LATERAL_UNCERTAINTY_M = {
    "pedestrian": 1.2,
    "person": 1.2,
    "animal": 0.8,
    "cow": 0.8,
    "bicycle": 0.6,
    "motorcycle": 0.6,
    "auto-rickshaw": 0.4,
    "car": 0.3,
}
_DEFAULT_LATERAL_UNCERTAINTY_M = 0.5


class Predictor(abc.ABC):
    @abc.abstractmethod
    def predict(self, tracked_objects: list[TrackedObject], dt: float) -> list[PredictedTrajectory]:
        ...


class ConstantVelocityPredictor(Predictor):
    """Primary predictor for this build. Produces 3 modes per track
    (straight extrapolation, and two laterally-offset variants scaled by
    per-class uncertainty) so the planner still gets a multimodal input
    shape even without a learned model -- cheap way to avoid a silent
    architecture mismatch if/when MoFlow gets swapped in later.
    """

    def __init__(self, horizon_steps: int = PREDICTION_HORIZON_STEPS):
        self.horizon_steps = horizon_steps

    def predict(self, tracked_objects: list[TrackedObject], dt: float) -> list[PredictedTrajectory]:
        results: list[PredictedTrajectory] = []

        for obj in tracked_objects:
            if not obj.position_history:
                continue

            x0, y0 = obj.position_history[-1]
            vx, vy = obj.velocity
            speed = np.hypot(vx, vy)
            uncertainty = _LATERAL_UNCERTAINTY_M.get(obj.class_name.lower(), _DEFAULT_LATERAL_UNCERTAINTY_M)

            # Mode 0: straight-line constant-velocity extrapolation.
            straight = [(x0 + vx * dt * k, y0 + vy * dt * k) for k in range(1, self.horizon_steps + 1)]
            results.append(PredictedTrajectory(obj.track_id, obj.class_name, straight, probability=0.6))

            # Modes 1/2: laterally offset, growing with horizon step, to
            # approximate "might drift left/right" -- perpendicular to
            # current heading. Skip if the object isn't moving (no defined heading).
            if speed > 0.05:
                perp = (-vy / speed, vx / speed)
                for sign, prob in [(1, 0.2), (-1, 0.2)]:
                    offset_points = []
                    for k in range(1, self.horizon_steps + 1):
                        frac = k / self.horizon_steps
                        offset = sign * uncertainty * frac
                        px = x0 + vx * dt * k + perp[0] * offset
                        py = y0 + vy * dt * k + perp[1] * offset
                        offset_points.append((px, py))
                    results.append(PredictedTrajectory(obj.track_id, obj.class_name, offset_points, probability=prob))
            else:
                logger.debug("Track %d (%s) near-stationary -- only 1 mode predicted (no defined heading for offsets).",
                             obj.track_id, obj.class_name)

        logger.debug("Predicted %d trajectory mode(s) across %d tracked object(s).", len(results), len(tracked_objects))
        return results


class MoFlowPredictor(Predictor):
    """NOT YET WIRED TO A REAL CHECKPOINT -- extension point only.

    To finish this: download a MoFlow checkpoint (see external/MoFlow's
    README / HuggingFace `fyxfelixfu/moflow`), instantiate the model
    classes from external/MoFlow/models/ (FlowMatcher + IMLE +
    ETHIMLETransformer per imle_eth.py), load the checkpoint, and in
    predict() below: build the input tensor from each TrackedObject's
    position_history (pad/truncate to the model's expected window length
    per cfg/eth_ucy/imle.yml), run inference, and unpack the K sampled
    trajectories per track into PredictedTrajectory objects with
    probability = 1/K each (or the model's own mode weights if it
    outputs them). Raises NotImplementedError so a half-wired predictor
    can never silently produce wrong output.
    """

    def __init__(self, checkpoint_path: str):
        self.checkpoint_path = checkpoint_path
        logger.warning("MoFlowPredictor constructed but not implemented -- calling predict() will raise. "
                        "Use ConstantVelocityPredictor until this is wired to a verified checkpoint.")

    def predict(self, tracked_objects: list[TrackedObject], dt: float) -> list[PredictedTrajectory]:
        raise NotImplementedError(
            "MoFlow inference not wired up yet -- see the class docstring for exactly what's left to do. "
            "Use ConstantVelocityPredictor for now."
        )
