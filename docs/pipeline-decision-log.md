# Pipeline Build Decision Log

Engineering decisions made while building `pipeline/` under the 2-day
deadline, per the request to keep a record of choices rather than just
ship silent tradeoffs. Ordered roughly as they came up.

## 1. Pure Python throughout, not MATLAB-dependent

**Decision**: implemented perception fusion, tracking, prediction, and
planning entirely in Python (`pipeline/`), rather than calling into
MATLAB's `multiObjectTracker` or `plannerHybridAStar` as `docs/architecture.md`
originally specified for those stages.

**Why**: you can't run CARLA locally and need to develop/test this
standalone. A pure-Python pipeline is testable right now with synthetic
data (`run_demo.py`), with zero dependency on MATLAB being installed or
running. Every module has a clearly marked swap-in point (see each
file's docstring) for routing through MATLAB later if the team decides
to during final integration — the input/output contracts (`pipeline/types.py`)
are designed so that swap doesn't require touching anything downstream.

**Tradeoff accepted**: this diverges from the "pipeline lives natively in
MATLAB/Simulink" architecture decision for these specific sub-stages.
Given the 2-day constraint and no local CARLA/MATLAB access to iterate
against, working-and-testable now was weighted over architecturally-pure.

## 2. Tracker: Kalman + Hungarian assignment, not a learned tracker

Simple constant-velocity Kalman filter per track (`filterpy`), matched to
detections each tick via the Hungarian algorithm on Euclidean gating
distance. Not a full JPDA/IMM implementation. **Known limitation, not a
bug**: will mis-assign IDs under dense, crossing-path traffic (documented
directly in `pipeline/tracker.py`) — this is the same limitation flagged
for MATLAB's own `trackerJPDA` in `docs/component-deep-dive.md`, inherited
deliberately rather than solved, given the time available.

## 3. Predictor: constant-velocity primary, MoFlow NOT wired to a real checkpoint

**Decision**: `ConstantVelocityPredictor` (Kalman extrapolation, 3 modes
per track with class-tuned lateral uncertainty) is the actual working
predictor. `MoFlowPredictor` exists as a class with a clear extension
point but `predict()` deliberately raises `NotImplementedError`.

**Why**: MoFlow ships as training/eval CLI scripts (`imle_eth.py` etc.),
not a clean inference API. Reverse-engineering its exact model
instantiation and input tensor shape from `models/flow_matching.py` +
`models/imle.py` + a config file, without a downloaded checkpoint to
validate against, was assessed as too much unverified risk against a
2-day clock — a half-wired integration that silently produces wrong
numbers is worse than a working fallback. The `NotImplementedError`
is deliberate: it fails loudly rather than quietly, if someone tries to
use it before it's actually finished.

**If time remains**: finishing `MoFlowPredictor` is the highest-value
next step, since it's the only piece of "the models" that isn't a
learned model right now. See the class docstring in `pipeline/predictor.py`
for exactly what's left.

## 4. Planner: pure-Python grid A* with soft costmap inflation

Standard 8-connected A* over a local occupancy grid, obstacles inflated
with Gaussian-ish falloff (not hard blocking) so a path can usually still
be found through cluttered scenes rather than failing outright. Replanning
is gated on a cheap costmap-signature-change check so it doesn't
re-search every single tick unnecessarily.

**Known constraint, already caught once**: the costmap is a fixed window
(default 60m × 60m) centered on the ego vehicle. A goal outside that
window silently fails to plan (`A* found NO FEASIBLE PATH`) — this
actually happened during testing (see §6) and is a real thing to watch
for when wiring in real ego/goal coordinates, not just a demo quirk.

## 5. Central logging, every stage, one file per run

`pipeline/logging_utils.py` sets up one logger writing to both console
and `logs/run_<timestamp>.log`. Every stage logs its decisions (LiDAR
fusion successes/failures with point counts, track spawn/drop events,
prediction mode counts, replan triggers) and the orchestrator logs
per-stage + total latency every tick, with an explicit warning if total
latency exceeds the ~150-200ms closed-loop-success-collapse threshold
documented in `docs/architecture.md`. This directly produces the raw
data for the "replanning latency" metric, not just a debug convenience.

## 6. Two real bugs found and fixed while verifying the demo (worth recording, not hiding)

1. **Camera/LiDAR extrinsic axis convention**: an initial identity
   matrix placeholder silently broke the fusion math — it fed the
   ego-frame "up" axis into the perspective projection's depth term.
   Fixed with a proper (if still uncalibrated) axis-remapping matrix in
   `run_demo.py`. **This is exactly the CARLA-documented pitfall already
   flagged in `pipeline/perception_fusion.py` (issue #3795)** — it bit
   the demo data immediately, which is a good sign the warning was
   correctly placed, not paranoia.
2. **Static synthetic bounding boxes vs. moving synthetic LiDAR clusters**:
   the demo's fake detections didn't move with the fake objects they were
   supposed to represent, so after a few ticks fusion was matching
   background clutter instead of the intended object, while still
   returning plausible-looking (wrong) numbers. Fixed by projecting the
   bbox from the same 3D point as the LiDAR cluster. **Flagging this
   because it's exactly the kind of silent failure mode to watch for
   when your teammate's real YOLO output gets wired in** — if fused
   distances look implausible or don't track a known-moving object,
   check the extrinsic and the detection-to-cloud correspondence first,
   not the tracker/predictor/planner logic downstream.

## 7. Placeholders that MUST be replaced before this means anything on real data

- `CAMERA_INTRINSIC` and `CAMERA_TO_LIDAR_EXTRINSIC` in `run_demo.py` —
  currently a generic 90°-FOV guess and an axis-correct-but-uncalibrated
  identity-offset matrix. Replace with your actual CARLA camera
  blueprint's real intrinsics and the real measured/configured sensor
  transform once your friend's CARLA instance is reachable.
- Costmap window size and Kalman filter noise parameters (`kf.R`, `kf.Q`
  in `pipeline/tracker.py`) are hand-picked, not fit to real data — fine
  for a working demo, worth revisiting if time allows once real sensor
  noise characteristics are known.
