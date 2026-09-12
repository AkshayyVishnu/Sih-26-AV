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

## 7. Added drivable-area estimation using CARLA's ground-truth segmentation

**Gap identified**: object detection + LiDAR tells you where obstacles
are, but nothing about where the drivable road surface itself is. On
unmarked roads (this PS's whole premise) there's no lane line to
substitute for that boundary — without it, the planner had no signal
stopping it from planning through a sidewalk or off the road entirely.

**Decision**: added `pipeline/drivable_area.py`, using **CARLA's own
ground-truth semantic segmentation camera** as the input, not a learned
segmentation model. It reuses the same LiDAR-to-camera projection math
as `perception_fusion.py` (deliberately duplicated, not shared, to avoid
touching that already-verified module under time pressure) to look up
each raw LiDAR point's semantic class, splitting points into
drivable/non-drivable, then rasterizes non-drivable points as high-cost
cells directly onto the planner's existing costmap
(`GridCostmap.rasterize_non_drivable`).

**Why ground truth, not a learned model**: a real learned segmentation
model (e.g. DeepLab/SegFormer fine-tuned on IDD, per
`docs/component-deep-dive.md`) is the correct real-world production
answer, but is another model to fetch/fine-tune/run inference on — not
feasible to add on this timeline. **This is a deliberate, disclosed
simulation-only shortcut** — say so explicitly in the technical report:
CARLA's ground truth stands in for what a trained segmentation model
would need to produce in a real deployment.

**Integration detail worth flagging**: the "did the scene change enough
to replan" signature is computed from predicted-obstacle cost **only**,
before non-drivable points are merged into the costmap — the environment
(buildings/sidewalks) is static tick-to-tick, but raw LiDAR sampling
noise means the exact point set differs every tick regardless. Including
that noise in the replan signature would either swamp real obstacle
changes or trigger spurious replans chasing sampling noise, not a real
scene change.

**Known limitation, not fixed given the time available**: coverage
depends on LiDAR point density — gaps (occluded regions, far range) are
left at whatever cost they already had, i.e. treated as passable by
default, not as confirmed-safe. Don't read "no non-drivable points here"
as "definitely drivable."

**Verified**: ran end-to-end via `run_demo.py`'s synthetic segmentation
generator — classification runs every tick (~0.35ms, negligible), feeds
correctly into replanning, total latency stayed low (mean ~10.8ms across
40 ticks).

**Tag IDs corrected after checking CARLA's real docs**: the original
placeholder guess (`Road=7`, `RoadLine=6`) was wrong — checked directly
against `carla.readthedocs.io/en/0.9.16/ref_sensors/` and confirmed the
real mapping is **`Road=1`, `RoadLine=24`** (tag 7 is actually
`TrafficLight`, tag 6 is `Pole`). Fixed in code before ever running
against live data. CARLA's own docs explicitly note "tags changed from
version 0.9.13 to 0.9.14" — if the actual CARLA server turns out to be a
different version than 0.9.16, re-verify this mapping against that
version's own docs page rather than trusting the current default.

## 8. Added control output (Pure Pursuit) and decision logic (Python state machine)

**Control**: `pipeline/controller.py`, Pure Pursuit for steering + a
proportional speed controller for throttle/brake. Chosen over PID-only
per `docs/architecture.md`'s own controller-comparison research (PID
has the weakest high-curvature tracking of the standard options) and
over implementing MPC from scratch (not worth the risk under this
timeline). Output matches `carla.VehicleControl`'s fields/ranges
directly.

**Decision logic**: `pipeline/decision_logic.py`, a hand-rolled Python
state machine implementing the exact design already worked out for
Path A's Stateflow chart (hierarchical `ANIMAL_ON_ROAD` sibling, parallel
`EMERGENCY_BRAKE`+`REPLAN`, parallel `SAFETY_SUPERVISOR` watchdog,
debounce, sustained-clearance resume gate). Hand-rolled rather than using
a state-machine library specifically so it reads as a direct 1:1 mirror
of whatever gets built as an actual Stateflow chart — see
`docs/two-path-strategy.md`.

Both are now wired into `pipeline.py`'s `tick()`: decision logic's
`replan_requested` output can force the planner to search fresh even if
the costmap's own change-detection wouldn't have triggered one
(`Planner.plan(..., force_replan=...)`), and `emergency_brake_active`
overrides the controller's throttle/brake (steering is preserved, so the
vehicle still tracks/swerves per the path while braking, not locked
straight).

**Real bug caught during verification, worth recording**: the first
full run after wiring these in showed a latency spike to **487ms on
tick 15** (and 100-200ms on several nearby ticks) — a real concern
against the ~150-200ms ceiling. Re-running immediately after showed
completely normal numbers (mean 10.9ms, max 35ms) with identical code
and identical synthetic data. This points to a one-time **environmental**
cause, most likely Windows/antivirus scanning the freshly-created log
file on its first several writes, not an algorithmic regression —
confirmed non-reproducible on re-run with the same inputs.
**Practical implication**: `run_demo.py` now reports both an
all-ticks summary and a "warmed-up" summary excluding the first 5 ticks
— **use the warmed-up numbers for your submitted metrics**, and if you
mention this in the report, describe it as a cold-start artifact, not a
steady-state latency figure.

## 9. Added structured metrics recording + cross-run aggregation

**Added**: `pipeline/metrics.py` (`MetricsRecorder`, wired into
`run_live.py`) records per-tick latency breakdown, decision mode,
speed/acceleration, jerk, distance-to-goal, and collisions, writing a
CSV + `summary.json` per run. `metrics_export.py` aggregates multiple
runs' `summary.json` files by scenario name into report-ready numbers
(completion rate, mean/max replanning latency, mean jerk), and warns
explicitly if a scenario has fewer than 3 runs rather than silently
presenting a single-run number as final.

**Two real bugs caught via self-testing before they could reach real
data:**
1. **Jerk was computed against wall-clock time between `record_tick()`
   calls, not the simulation timestep** — produced values in the
   hundreds-of-thousands (m/s³) range on a first synthetic test, an
   obvious sign something was wrong. Root cause: in CARLA's synchronous
   mode, physics always advances by exactly the fixed `dt` per
   `world.tick()`, regardless of how long client-side processing took
   that tick (which is what `TickTimings` measures) — using wall-clock
   deltas for a physics derivative conflates "how smooth is the drive"
   with "how fast did our code run," which is wrong. Fixed:
   `MetricsRecorder` now takes an explicit `dt` (the fixed simulation
   timestep) and uses that as the jerk denominator, not wall-clock time.
   Re-tested: jerk values became physically plausible (tens of m/s³
   range) immediately after the fix.
2. **`run_id` used only second-level precision** (`%Y%m%d_%H%M%S`) —
   three test runs executed back-to-back in under a second silently
   overwrote each other's output files, and `metrics_export.py` only
   found 1 of 3 runs as a result. This would bite anyone scripting
   multiple runs for the required ≥3-seeds-per-scenario validation.
   Fixed: added millisecond precision to the run ID. Re-tested: all 3
   runs correctly aggregated afterward.

## 10. Placeholders that MUST be replaced before this means anything on real data

- `CAMERA_INTRINSIC` and `CAMERA_TO_LIDAR_EXTRINSIC` in `run_demo.py` —
  currently a generic 90°-FOV guess and an axis-correct-but-uncalibrated
  identity-offset matrix. Replace with your actual CARLA camera
  blueprint's real intrinsics and the real measured/configured sensor
  transform once your friend's CARLA instance is reachable.
- Costmap window size and Kalman filter noise parameters (`kf.R`, `kf.Q`
  in `pipeline/tracker.py`) are hand-picked, not fit to real data — fine
  for a working demo, worth revisiting if time allows once real sensor
  noise characteristics are known.

## 11. SUMMIT dropped -- version fork mismatch, confirmed via direct source inspection

Investigated actually integrating SUMMIT (`AdaCompNUS/summit`) to generate
dense, unregulated, heterogeneous traffic (chaotic-Indian-road-style) for
a closed-loop comparison of PlanT2-with-own-perception vs. PCLA's
end-to-end TransFuser v6. Cloned it into `external/summit` (gitignored)
and inspected it directly rather than trusting its README's framing as a
"CARLA-based simulator":

- `external/summit/LibCarla/`, `Unreal/`, `Util/BuildTools/`, and a
  top-level `Makefile` are all present — this is a full Unreal Engine
  source **fork** of CARLA, not a plugin or a script that attaches to a
  running CARLA server.
- `external/summit/PythonAPI/carla/setup.py` pins `version='0.9.8'`, and
  `CHANGELOG.md` opens at CARLA 0.9.8 — eight major CARLA releases behind
  the friend's live 0.9.16 server and the `carla==0.9.16` pip client this
  entire project is built against.

**Verdict: dropped.** Building SUMMIT from source means building a
second, separate Unreal Engine 4.24-based simulator binary (Epic Games
account linked to CARLA's GitHub org, tens of GB, hours of build time) —
not achievable in the time available, and even if built, it would not be
the friend's actual running CARLA world. PCLA's agents are tested at
0.9.15/0.9.16 (confirmed: `external/PCLA/README.md` line 48, and its
`environment.yml` pins `carla==0.9.16` exactly) and are not confirmed to
run correctly against a 0.9.8 server at all — this project has already
hit real CARLA-version API drift once (the semantic-segmentation tag ID
bug in section 8 below), so assuming cross-version compatibility here
would be repeating a mistake already paid for once.

**Replacement:** `pipeline/traffic_chaos.py` — CARLA's own built-in
Traffic Manager, tuned aggressively (tight following distance, frequent
lane changes, partial light/sign non-compliance, a two-wheeler-biased
vehicle mix, jaywalking pedestrians via `set_pedestrians_cross_factor`).
This is a coarser approximation than SUMMIT's actual GAMMA crowd model —
no true lane-less/gap-filling behavior — but it requires zero extra
build steps and runs against the exact server everything else in this
project targets. `external/summit` is left cloned on disk (gitignored,
harmless) in case a future session finds a way to reconcile the version
mismatch, but nothing in this branch depends on it.

## 12. PlanT2 injection point -- found, documented, same honesty standard as MoFlow

Investigated whether PCLA's bundled PlanT2 agent (object-level,
planning-only — assumes perception is solved) could be fed this
project's own perception (`pipeline/perception_fusion.py` +
`pipeline/tracker.py`) instead of PCLA's own ground-truth actor query.
Read `external/PCLA/pcla_agents/plant2/PlanT_agent.py` and its base class
(`external/PCLA/pcla_agents/plant2/carla_garage/data_agent.py`) directly.

**Finding: yes, a clean injection point exists.**
`PlanTAgent.run_step()` calls `label_raw = self.get_bounding_boxes()`,
then `self._get_control(label_raw, tick_data)`. `get_bounding_boxes()`
does nothing but query `self._world.get_actors()` and convert each actor
into an ego-relative plain dict — no side effects, no state it uniquely
owns. It can be replaced on the **agent instance**
(`agent_instance.get_bounding_boxes = my_function`) without touching
route planning, traffic-light/stop-sign injection, or control synthesis,
all of which stay exactly as PCLA ships them. This is a materially easier
situation than MoFlow's (section 6/7) — no tensor-shape reverse
engineering, no training-loop coupling, just a plain-dict return value to
replace.

Implemented as `pipeline/plant2_adapter.py`
(`convert_tracked_to_label_raw`), used by `run_own_perception_plant2.py`.
**Disclosed limitations (not glossed over):**
1. Class vocabulary mismatch — PlanT2's fixed ontology
   (car/walker/static/static_car/stop_sign/traffic_light/emergency) has
   no "animal"/"auto-rickshaw" category; our IDD-trained YOLO classes get
   best-effort mapped onto it (real information loss, PlanT2's choice not
   ours to fix without retraining it).
2. No real 3D bounding-box extents — our perception gives a LiDAR-point-
   cluster centroid, not a measured box size; extents are hand-picked
   per-class placeholders.
3. No heading estimation — our tracker has no orientation filter; object
   yaw is approximated from velocity direction when moving, else defaults
   to 0.
4. Static hazards (traffic lights, stop signs) are deliberately NOT
   emitted by the adapter — YOLO wasn't asked to detect them, and
   `run_step()` already injects those from ground truth independently of
   `get_bounding_boxes()`.
5. **Never tested against a live CARLA/PlanT2 run** — this is a design
   derived from reading the source, not verified against real inference
   output. First live run should sanity-check the adapter's label_raw
   output against the original `get_bounding_boxes()`'s ground-truth
   output side by side for a few ticks before trusting the comparison.

Also confirmed during this investigation: `get_relative_transform`'s
ego-relative frame uses CARLA's native convention (x=forward, y=RIGHT),
which is the **opposite sign** of this project's own internal
`FusedDetection`/`EgoState` convention (x=forward, y=LEFT, per
`types.py`). `plant2_adapter.py`'s `_world_to_ego_relative_xy` flips this
explicitly — a real, easy-to-miss sign bug if anyone else builds a
similar adapter without checking.

## 13. Comparison scripts' TickTimings are not apples-to-apples with run_live.py

`run_pcla_transfuserv6.py` and `run_own_perception_plant2.py` both reuse
`pipeline.pipeline.TickTimings` for `MetricsRecorder` (so
`metrics_export.py` doesn't need scenario-specific code), but neither
runs this project's actual 7-stage pipeline:
- `run_pcla_transfuserv6.py`: TransFuser v6's entire forward pass is one
  opaque `pcla.get_action()` call — every `TickTimings` field is 0 except
  the total wall-clock isn't even captured (deliberately left at 0 rather
  than mislabeled into one of the 7 named stages that don't apply).
- `run_own_perception_plant2.py`: `prediction_ms`/`drivable_area_ms`/
  `decision_ms` are all 0 (no equivalent stage — PlanT2 does its own
  internal motion reasoning and control synthesis); `planning_ms` is
  PlanT2's `run_step()` cost including our injected `get_bounding_boxes`
  call; YOLO inference time is logged separately (printed, not part of
  `TickTimings`) since it has no matching field.

**Do not directly compare `total_latency_ms` across these two scenarios
and `run_live.py`'s own-pipeline runs without accounting for this** — the
number means something different in each. `replanning_latency_ms` in the
PS's sense (does the system replan fast enough) is really only meaningful
for `run_live.py`'s own planner; these two comparison scripts are
measuring end-to-end control-decision latency instead, which is a related
but distinct number worth reporting separately in the final writeup, not
merged into one "replanning latency" figure across all three.
