"""
Learned cost shaping for the planner: a tiny MLP mapping per-predicted-point
features -> (inflation_radius_m, danger), replacing the hand-picked
inflation table with data-driven costs while keeping A* itself as the
search. This is the "novel DL" component that consumes the perception/
tracking stack's outputs (class, speed, TTC, history) instead of raw
sensors -- no end-to-end model anywhere near the loop.

Contract (owned here; the notebook training script and any future
MATLAB bridge code against it):
  FEATURE_ORDER = ["class_id", "speed", "ttc", "lateral_spread",
                   "history_len", "mode_prob", "ego_speed"]
  outputs = (inflation_radius_m in [0.5, 3.0], danger in [0, 1])
  final base_cost used by the planner = danger * mode_prob * time_decay,
  mirroring the previous hand-tuned `base_cost = probability * time_decay`
  with danger=1.0 always. So danger=1.0 reproduces old behavior exactly.

Fallback philosophy (same as prediction's Kalman fallback): if no weights
are loaded, or a feature vector is degenerate, every call degrades to the
hand-tuned table -- a missing .pt file can never break planning.
"""
from __future__ import annotations

import json
import logging
import os

import numpy as np

logger = logging.getLogger("pipeline.cost_head")

FEATURE_ORDER = [
    "class_id",       # int-encoded agent class (see CLASS_IDS)
    "speed",          # m/s, track Kalman speed (or traj-derived)
    "ttc",            # s, dist / closing speed, capped (guard-only estimate)
    "lateral_spread", # m, class prior uncertainty (same table as predictor)
    "history_len",    # int, track history points (reliability proxy)
    "mode_prob",      # float, predicted mode probability
    "ego_speed",      # m/s, ego speed this tick
]

CLASS_IDS = {
    "pedestrian": 0,
    "person": 0,
    "animal": 1,
    "cow": 1,
    "bicycle": 2,
    "motorcycle": 2,
    "auto-rickshaw": 3,
    "autorickshaw": 3,
    "pushcart": 4,
    "car": 5,
    "bus": 6,
    "truck": 6,
}
DEFAULT_CLASS_ID = 7

# Static priors shared with the predictor's lateral-uncertainty table.
# The MLP learns the *mapping* from these (plus dynamics) to costs;
# the priors themselves stay hand-set and interpretable.
CLASS_BASE_RADIUS_M = {
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
DEFAULT_BASE_RADIUS_M = 1.5  # == the planner's pre-MLP flat radius

RADIUS_MIN_M, RADIUS_MAX_M = 0.5, 3.0
TTC_CAP_S = 10.0

DEFAULT_MODEL_PATH = os.path.join("models", "cost_head_mlp.pt")
DEFAULT_META_PATH = os.path.join("models", "cost_head_meta.json")


def class_id_of(class_name: str) -> int:
    return CLASS_IDS.get(class_name.lower(), DEFAULT_CLASS_ID)


def base_radius_of(class_name: str) -> float:
    return CLASS_BASE_RADIUS_M.get(class_name.lower(), DEFAULT_BASE_RADIUS_M)


def ttc_estimate(dist_m: float, closing_speed_mps: float) -> float:
    """Cheap guard-only TTC: dist / closing speed, capped. Same caveat as
    decision_logic._min_ttc -- mode-switching/cost feature only, not a
    substitute for the planner's own geometry."""
    if closing_speed_mps <= 0.05:
        return TTC_CAP_S
    return float(min(dist_m / closing_speed_mps, TTC_CAP_S))


def build_feature_vector(
    class_name: str,
    speed_mps: float,
    ttc_s: float,
    history_len: int,
    mode_prob: float,
    ego_speed_mps: float,
) -> np.ndarray:
    """Single source of truth for feature construction -- the training
    script and the planner's runtime path both call this, so train/serve
    skew is impossible by construction."""
    lateral = CLASS_BASE_RADIUS_M.get(class_name.lower(), DEFAULT_BASE_RADIUS_M)
    return np.array(
        [
            float(class_id_of(class_name)),
            float(speed_mps),
            float(min(ttc_s, TTC_CAP_S)),
            float(lateral),
            float(history_len),
            float(mode_prob),
            float(ego_speed_mps),
        ],
        dtype=np.float32,
    )


def _torch():
    import torch  # local import: numpy-only callers never pay for torch

    return torch


class CostHeadMLP:
    """Architecture owned here: 7 -> 32 -> 16 -> 2. ~1.5k params;
    microseconds per call next to A*'s milliseconds."""

    def __init__(self):
        torch = _torch()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(7, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 2),
        )

    def parameters(self):
        return self.net.parameters()

    def state_dict(self):
        return self.net.state_dict()

    def load_state_dict(self, sd):
        return self.net.load_state_dict(sd)


class CostHead:
    """Runtime wrapper: normalized MLP inference with table fallback.
    `loaded` is False when no weights file exists -- every method then
    returns the hand-tuned values, so planning is behavior-identical to
    the pre-MLP build."""

    def __init__(self, model_path: str = DEFAULT_MODEL_PATH, meta_path: str = DEFAULT_META_PATH):
        self.loaded = False
        self.mean: np.ndarray | None = None
        self.std: np.ndarray | None = None
        self._mlp: CostHeadMLP | None = None
        try:
            torch = _torch()
            with open(meta_path) as f:
                meta = json.load(f)
            assert meta.get("feature_order") == FEATURE_ORDER, "meta feature_order mismatch"
            self.mean = np.array(meta["feature_mean"], dtype=np.float32)
            self.std = np.array(meta["feature_std"], dtype=np.float32)
            mlp = CostHeadMLP()
            sd = torch.load(model_path, map_location="cpu", weights_only=True)
            mlp.load_state_dict(sd)
            mlp.net.eval()
            self._mlp = mlp
            self.loaded = True
            # Warm-up forward: pays torch's first-call overhead (lazy init,
            # kernel autotune) here at load, not inside a latency-critical
            # planning tick where it would spike one tick's total_ms.
            with torch.no_grad():
                mlp.net(torch.zeros(1, len(FEATURE_ORDER)))
            logger.info("CostHead loaded: %s (+meta %s)", model_path, meta_path)
        except FileNotFoundError:
            logger.info("CostHead weights not found (%s) -- table fallback active.", model_path)
        except Exception as e:  # corrupt weights/meta must never break planning
            logger.warning("CostHead failed to load (%s) -- table fallback active.", e)

    @staticmethod
    def table_costs(class_name: str, mode_prob: float) -> tuple[float, float]:
        """Pre-MLP behavior exactly: flat 1.5m radius, danger=1.0."""
        return DEFAULT_BASE_RADIUS_M, 1.0

    def predict_costs(self, features: np.ndarray, class_name: str, mode_prob: float) -> tuple[float, float]:
        """Returns (inflation_radius_m, danger). Falls back to the table on
        any problem (unloaded, NaN features, inference error)."""
        return self.predict_batch([features], [class_name])[0]

    def predict_batch(self, features_list: list[np.ndarray], class_names: list[str]) -> list[tuple[float, float]]:
        """One batched forward for a whole tick's trajectories -- a single
        torch call instead of N tiny ones. Any failure degrades the whole
        batch to table values, never raises."""
        import torch.nn.functional as F

        torch = _torch()
        try:
            if not self.loaded or self._mlp is None:
                raise ValueError("unloaded")
            X = np.stack(features_list).astype(np.float32)
            if not np.all(np.isfinite(X)):
                raise ValueError("non-finite features")
            Xn = (X - self.mean) / self.std
            with torch.no_grad():
                out = self._mlp.net(torch.from_numpy(Xn))
                radii = RADIUS_MIN_M + (RADIUS_MAX_M - RADIUS_MIN_M) * torch.sigmoid(out[:, 0])
                dangers = torch.sigmoid(out[:, 1])
            return [(float(r), float(d)) for r, d in zip(radii.tolist(), dangers.tolist())]
        except Exception as e:
            logger.debug("CostHead batch inference failed (%s) -- table fallback.", e)
            return [self.table_costs(cn, 1.0) for cn in class_names]
