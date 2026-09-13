"""
Trains pipeline/cost_head.py's inflation MLP on synthetic logs, fully
offline (CPU fine) -- no CARLA, no GPU needed.

Each synthetic episode scripts GT actor motion (head-on pass, cut-in,
crossing pedestrian, wandering livestock, parked roadside negative) with
the ego driving +x at constant speed. Features are built with the SAME
pipeline.cost_head.build_feature_vector() the planner's runtime path
calls, so train/serve skew is impossible by construction. Labels come
from GT futures (free supervision):
  danger = exp(-min_ego_dist_over_horizon / 4.0)
  radius = class_base * (1 + 1.5 * danger)

Exit criteria (printed + non-zero exit on failure so a server/CI gate can
use this directly): held-out-episode MSE strictly below the hand-tuned
table baseline (flat 1.5m radius, danger=1.0 == current shipped behavior)
on BOTH outputs.

Notebook use (Kaggle/Colab): copy this file + pipeline/cost_head.py, run
`python train_cost_head.py --epochs 300 --out models/`. Swap
gen_synthetic_dataset() for recorded server logs in the same (X, y, w)
format to train on real data -- the interface is one function.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pipeline.cost_head import (  # noqa: E402
    DEFAULT_BASE_RADIUS_M,
    FEATURE_ORDER,
    CostHead,
    CostHeadMLP,
    base_radius_of,
    build_feature_vector,
    ttc_estimate,
)

DT = 0.05
HORIZON_STEPS = 12
EGO_SPEED = 5.0
DANGER_SCALE_M = 4.0
NEAR_MISS_M = 3.0


def _label(min_dist_m: float, class_name: str) -> tuple[float, float]:
    danger = float(np.exp(-min_dist_m / DANGER_SCALE_M))
    # Radius gain capped so labels stay inside the head's [0.5, 3.0] output
    # range (pedestrian base 2.0 * 1.5 = 3.0 exactly at danger=1).
    radius = base_radius_of(class_name) * (1.0 + 0.5 * danger)
    return radius, danger


def _rollout(x0, y0, vx_fn, vy_fn, ticks: int):
    """GT positions over `ticks` steps; velocity fns of (x, y, t)."""
    x, y, out = x0, y0, []
    for t in range(ticks):
        vx, vy = vx_fn(x, y, t), vy_fn(x, y, t)
        out.append((x, y, vx, vy))
        x, y = x + vx * DT, y + vy * DT
    return out


def gen_episode(kind: str, rng: np.random.Generator) -> list[tuple[np.ndarray, float, float, float]]:
    """One episode -> list of (features, radius, danger, weight). Ego at
    (ex, 0) moving +x at EGO_SPEED; rows sampled per tick with a noisy
    Kalman-like velocity and growing history (mimics real track aging)."""
    ex0 = rng.uniform(-5, 5)
    lat_off = rng.uniform(-6, 6)
    rows = []

    if kind == "head_on":
        y = rng.uniform(1.0, 2.5) * rng.choice([-1.0, 1.0])
        traj = _rollout(40.0, y, lambda *a: -rng.uniform(4, 8), lambda *a: 0.0, 120)
        cls = "car"
    elif kind == "cut_in":
        traj = _rollout(18.0, 4.0, lambda *a: -2.0, lambda x, y, t: -1.5 if t < 40 else 0.0, 120)
        cls = rng.choice(["auto-rickshaw", "car", "motorcycle"])
    elif kind == "crossing_ped":
        traj = _rollout(rng.uniform(12, 20), 6.0, lambda *a: 0.0, lambda *a: -rng.uniform(1.0, 1.8), 120)
        cls = "pedestrian"
    elif kind == "wandering_cow":
        amp, ph = rng.uniform(0.5, 1.5), rng.uniform(0, 6.28)
        traj = _rollout(rng.uniform(15, 30), rng.uniform(-2, 2),
                        lambda *a: -0.5,
                        lambda x, y, t: amp * np.sin(0.3 * t + ph), 160)
        cls = "animal"
    else:  # parked roadside negative
        traj = _rollout(rng.uniform(10, 30), rng.choice([-1.0, 1.0]) * rng.uniform(4, 7),
                        lambda *a: 0.0, lambda *a: 0.0, 80)
        cls = rng.choice(["car", "pushcart", "truck", "bus"])

    for t in range(5, len(traj) - HORIZON_STEPS):
        x, y, vx, vy = traj[t]
        ex = ex0 + EGO_SPEED * DT * t
        # Noisy velocity + growing history mimic a real aging track.
        nvx, nvy = vx + rng.normal(0, 0.3), vy + rng.normal(0, 0.3)
        hist_len = min(t + 1, 12)
        dist = float(np.hypot(x - ex, y))
        dx, dy = (x - ex) / max(dist, 1e-3), y / max(dist, 1e-3)
        closing = -((nvx - EGO_SPEED) * dx + nvy * dy)
        feats = build_feature_vector(cls, float(np.hypot(nvx, nvy)),
                                     ttc_estimate(dist, closing), hist_len, 0.6, EGO_SPEED)
        # Label from GT futures (ego keeps constant speed/heading).
        fut = [np.hypot(traj[t + k][0] - (ex + EGO_SPEED * DT * k), traj[t + k][1])
               for k in range(1, HORIZON_STEPS + 1)]
        radius, danger = _label(float(min(fut)), cls)
        rows.append((feats, radius, danger, 4.0 if min(fut) < NEAR_MISS_M else 1.0))
    return rows


KINDS = ["head_on", "cut_in", "crossing_ped", "wandering_cow", "parked"]


def gen_synthetic_dataset(n_episodes: int, seed: int):
    rng = np.random.default_rng(seed)
    train_rows, val_rows = [], []
    for i in range(n_episodes):
        rows = gen_episode(KINDS[i % len(KINDS)], rng)
        (val_rows if i % 5 == 4 else train_rows).extend(rows)
    def stack(rows):
        X = np.stack([r[0] for r in rows])
        return X, np.array([r[1] for r in rows], np.float32), np.array([r[2] for r in rows], np.float32), np.array([r[3] for r in rows], np.float32)
    return stack(train_rows), stack(val_rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--lr", type=float, default=1e-2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="models")
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    import torch

    device = torch.device(args.device if torch.cuda.is_available() or args.device == "cpu" else "cpu")
    (Xtr, rtr, dtr, wtr), (Xva, rva, dva, wva) = gen_synthetic_dataset(args.episodes, args.seed)
    print(f"train rows={len(Xtr)} val rows={len(Xva)} device={device}")

    mean, std = Xtr.mean(0), Xtr.std(0)
    std = np.clip(std, 1e-3, None)  # constant features must not NaN normalization
    def norm(X):
        return (X - mean) / std

    mlp = CostHeadMLP()
    mlp.net.to(device)
    opt = torch.optim.Adam(mlp.parameters(), lr=args.lr)
    Xtr_t = torch.from_numpy(norm(Xtr)).to(device)
    ytr_t = torch.stack([torch.from_numpy(rtr), torch.from_numpy(dtr)], 1).to(device)
    wtr_t = torch.from_numpy(wtr).to(device)
    Xva_t = torch.from_numpy(norm(Xva)).to(device)

    mlp.net.train()
    for ep in range(args.epochs):
        opt.zero_grad()
        pred = mlp.net(Xtr_t)
        # Sigmoid-space training matches inference (see CostHead.predict_costs).
        pr = 0.5 + 2.5 * torch.sigmoid(pred[:, 0])
        pd = torch.sigmoid(pred[:, 1])
        loss = (wtr_t * ((pr - ytr_t[:, 0]) ** 2 + (pd - ytr_t[:, 1]) ** 2)).mean()
        loss.backward()
        opt.step()
    print(f"final train loss={loss.item():.4f}")

    # --- table baseline (== current shipped behavior) vs MLP, held-out ---
    mlp.net.eval()
    with torch.no_grad():
        pv = mlp.net(Xva_t)
        m_r = (0.5 + 2.5 * torch.sigmoid(pv[:, 0])).cpu().numpy()
        m_d = torch.sigmoid(pv[:, 1]).cpu().numpy()
    t_r = np.full_like(rva, DEFAULT_BASE_RADIUS_M)
    t_d = np.ones_like(dva)
    mse = lambda a, b: float(np.mean((a - b) ** 2))
    print(f"radius MSE: table={mse(t_r, rva):.4f} mlp={mse(m_r, rva):.4f}")
    print(f"danger MSE: table={mse(t_d, dva):.4f} mlp={mse(m_d, dva):.4f}")
    beats = mse(m_r, rva) < mse(t_r, rva) and mse(m_d, dva) < mse(t_d, dva)
    print("BEATS TABLE:", beats)

    os.makedirs(args.out, exist_ok=True)
    torch.save(mlp.state_dict(), os.path.join(args.out, "cost_head_mlp.pt"))
    with open(os.path.join(args.out, "cost_head_meta.json"), "w") as f:
        json.dump({
            "feature_order": FEATURE_ORDER,
            "feature_mean": mean.tolist(),
            "feature_std": std.tolist(),
            "dt": DT,
            "horizon_steps": HORIZON_STEPS,
            "danger_scale_m": DANGER_SCALE_M,
            "trained_on": "synthetic",
            "seed": args.seed,
        }, f, indent=2)
    print(f"saved weights + meta to {args.out}/")

    # Loader round-trip: fresh CostHead must reproduce the in-memory numbers.
    head = CostHead(os.path.join(args.out, "cost_head_mlp.pt"),
                    os.path.join(args.out, "cost_head_meta.json"))
    assert head.loaded
    rr, dd = head.predict_costs(Xva[0], "car", 0.6)
    assert abs(rr - m_r[0]) < 1e-4 and abs(dd - m_d[0]) < 1e-4, "round-trip mismatch"
    print(f"round-trip OK (sample: radius={rr:.3f} danger={dd:.3f})")

    if not beats:
        print("FAIL: MLP did not beat the table baseline.")
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
