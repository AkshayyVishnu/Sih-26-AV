"""
MoFlow IMLE student inference bridge: TrackedObject list -> K multimodal
PredictedTrajectory list, reusing the exact tensor contract verified in
the /tmp spike (random-init ETHIMLETransformer forward OK: [B,1,20,1,24]
-> [B,K,A,F,2], finite, sane scale).

Pinned contract (from external/MoFlow source, not guessed):
  - Input past window P=8, future F=12, K=denoising_head_preds=20
    (cfg/eth_ucy/imle.yml).
  - Per-agent 6ch frame = [abs_xy, rel_to_last_xy, vel_xy], encoder consumes
    ORIGINAL scale (backbone_eth_ucy.py forward), min-max constants come
    from the canonical train split (deterministic files).
  - Released ETH checkpoints are SINGLE-agent scenes (cfg.agents=1,
    agent_query_embedding has 1 row) -> batch tracks as B independent
    A=1 scenes. No zero-padding phantoms. Tradeoff stated plainly: no
    interaction modeling in this configuration, multimodality per agent.
  - Outputs are RELATIVE to the last history point -> add it back.
  - Eval call pattern mirrors trainer test: model.eval(),
    imle(data, num_to_gen=1), reshape, unnormalize.

KNOWN CADENCE GAP (disclosed, not hidden): ETH-UCY trains at ~2.5fps
(0.4s steps; 8 frames = 3.2s history, 12 steps = 4.8s horizon) while the
live tracker runs at 20Hz (dt=0.05). v1 bridges it by strided subsampling
of history and interpolation of predictions onto the planner grid; the
real fix is fine-tuning on native-cadence tracks (server job, free GT
labels). Short-history tracks (<8 pts, stride-padded) and the
ConstantVelocityPredictor fallback cover the rest.

Checkpoints (gitignored *.pt; HF fyxfelixfu/moflow, Apache-2.0 per card):
  models/moflow/eth_teacher.pt   - released ETH teacher (flow-matching).
      Present for future distillation; NOT used by this inference path.
  models/moflow/nba_imle_student.pt - released NBA student. Proves the
      IMLE-wrapper call shape only (basketball coords); NOT usable as a
      driving predictor. The deployable ETH student must be trained on
      the server (teacher-sample + 150-epoch IMLE per README).
"""
from __future__ import annotations

import logging
import os
import sys

import numpy as np

logger = logging.getLogger("pipeline.moflow_io")

MOFLOW_PAST_FRAMES = 8
MOFLOW_FUTURE_FRAMES = 12
MOFLOW_K_MODES = 20  # == denoising_head_preds; num_to_gen=1 at inference

# Placeholder until a real ETH student checkpoint exists; loader asserts
# the file before touching torch.
DEFAULT_STUDENT_PATH = os.path.join("models", "moflow", "eth_imle_student.pt")


def _mflow():
    """Deferred import: `import pipeline.moflow_io` must stay light --
    external/MoFlow pulls torch/einops/yaml/easydict/GitPython."""
    repo = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "external", "MoFlow")
    if repo not in sys.path:
        sys.path.insert(0, repo)
    return repo


def tracks_to_tensor(tracks, past_frames: int = MOFLOW_PAST_FRAMES):
    """TrackedObject list -> (past6 [B,1,P,6] original-scale, initial_pos
    [B,1,2], track_ids). Strided subsample spreads short histories over the
    window; pads by repeating the oldest point."""
    ids, batch, inits = [], [], []
    for t in tracks:
        hist = list(t.position_history)
        if not hist:
            continue
        # Stride so available history spans the window (v1 cadence bridge).
        stride = max(1, len(hist) // past_frames)
        picked = hist[::-stride][:past_frames][::-1]
        while len(picked) < past_frames:
            picked = [picked[0]] + picked
        abs_hist = np.array(picked[-past_frames:], dtype=np.float32)  # [P,2]
        initial = abs_hist[-1:]
        rel = abs_hist - initial
        vel = np.concatenate([rel[1:] - rel[:-1], np.zeros((1, 2), np.float32)], axis=0)
        batch.append(np.concatenate([abs_hist, rel, vel], axis=-1))
        inits.append(initial)
        ids.append(t.track_id)
    if not batch:
        return None, None, []
    return (np.stack(batch)[:, None, :, :],
            np.stack(inits), ids)


def samples_to_trajectories(out, initial_pos, track_ids, class_by_id,
                            horizon_steps: int, fut_min: float, fut_max: float,
                            top_k: int = MOFLOW_K_MODES):
    """Raw student output [B,1,20,1,F*D] (normalized, relative) ->
    PredictedTrajectory list. Unnormalizes, adds back the last history
    point, interpolates the 4.8s model horizon onto the planner's dt grid
    (v1 cadence bridge), keeps top_k modes at prob 1/k."""
    from pipeline.types import PredictedTrajectory

    B = out.shape[0]
    F = out.shape[-1] // 2
    K = min(out.shape[2], top_k)
    rel = (out.reshape(B, -1, F, 2)[:, :K] - (-1)) / 2 * (fut_max - fut_min) + fut_min
    trajs = []
    for b in range(B):
        cls = class_by_id.get(track_ids[b], "car")
        for k in range(K):
            pts = rel[b, k] + initial_pos[b, 0]  # absolute, model cadence
            # Interpolate model horizon onto planner grid (first window).
            src_t = np.linspace(0, 1, pts.shape[0])
            dst_t = np.linspace(0, 1, horizon_steps)
            fine = np.stack([np.interp(dst_t, src_t, pts[:, 0]),
                             np.interp(dst_t, src_t, pts[:, 1])], axis=-1)
            trajs.append(PredictedTrajectory(
                track_ids[b], cls,
                [(float(x), float(y)) for x, y in fine],
                probability=1.0 / K))
    return trajs


class MoFlowStudent:
    """Load-once wrapper: config + norm constants + weights. `loaded` False
    on any problem (missing files/weights) -- callers fall back to the
    kinematic predictor, never crash."""

    def __init__(self, checkpoint_path: str | None = DEFAULT_STUDENT_PATH,
                 subset: str = "eth", device: str = "cpu"):
        self.loaded = False
        self._model = None
        self.cfg = None
        self.device = device
        self.subset = subset
        checkpoint_path = checkpoint_path or DEFAULT_STUDENT_PATH
        if not os.path.exists(checkpoint_path):
            logger.warning("MoFlow student checkpoint not found (%s) -- "
                           "kinematic fallback active.", checkpoint_path)
            return
        try:
            _mflow()
            import torch
            from utils.config import Config
            from utils.normalization import normalize_min_max  # noqa: F401 (contract ref)
            from models.backbone_eth_ucy import ETHIMLETransformer
            from models.imle import IMLE
            from data.dataloader_eth_ucy import ETHDataset

            cfg = Config("cfg/eth_ucy/imle.yml", "infer", train_mode=False)
            cfg.subset, cfg.rotate, cfg.rotate_aug, cfg.data_norm = subset, False, False, "min_max"
            # Deterministic constants from the canonical train split --
            # must travel with (subset, checkpoint) as one unit.
            probe = ETHDataset(cfg=cfg, training=True, data_dir="./data/eth_ucy",
                               subset=subset, rotate_time_frame=0, imle=False, type="original")
            del probe
            model = ETHIMLETransformer(model_config=cfg.MODEL, logger=logger, config=cfg)
            sd = torch.load(checkpoint_path, map_location="cpu")["model"]
            # Released checkpoints prefix keys with 'model.' (FlowMatcher
            # wrapper); student arch has bare keys -- strip on load.
            sd = {k.replace("model.", "", 1): v for k, v in sd.items()}
            missing, unexpected = model.load_state_dict(sd, strict=False)
            if unexpected:
                logger.warning("MoFlow student: %d unexpected keys (e.g. %s).",
                               len(unexpected), unexpected[:3])
            model.eval()
            self._model, self.cfg = IMLE(cfg=cfg, model=model, logger=logger), cfg
            self._model.eval()
            self.loaded = not bool(missing)
            if missing:
                logger.warning("MoFlow student: %d missing keys -- NOT loaded.", len(missing))
            else:
                logger.info("MoFlow student loaded (%s, subset=%s).", checkpoint_path, subset)
        except Exception as e:
            logger.warning("MoFlow student failed to load (%s) -- fallback active.", e)

    def predict(self, tracks, dt: float, horizon_steps: int, top_k: int = MOFLOW_K_MODES):
        """TrackedObject list -> PredictedTrajectory list. Raises
        RuntimeError when unusable so callers fail over loudly."""
        if not self.loaded or self._model is None:
            raise RuntimeError("MoFlow student not loaded")
        import torch
        from utils.normalization import normalize_min_max

        past6, inits, ids = tracks_to_tensor(tracks)
        if past6 is None:
            return []
        class_by_id = {t.track_id: t.class_name for t in tracks}
        normed = normalize_min_max(torch.from_numpy(past6).float(),
                                   self.cfg.past_traj_min, self.cfg.past_traj_max, -1, 1)
        x_data = {"past_traj": normed,
                  "past_traj_original_scale": torch.from_numpy(past6).float()}
        for v in x_data.values():
            v.requires_grad_(False)
        with torch.no_grad():
            out = self._model(x_data, num_to_gen=1).cpu().numpy()
        return samples_to_trajectories(
            out, inits, ids, class_by_id, horizon_steps,
            float(self.cfg.fut_traj_min), float(self.cfg.fut_traj_max), top_k)
