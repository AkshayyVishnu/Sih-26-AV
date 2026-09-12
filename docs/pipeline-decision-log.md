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

Investigated integrating SUMMIT (`AdaCompNUS/summit`) to generate dense,
unregulated, heterogeneous traffic (chaotic-Indian-road-style) for a
closed-loop comparison against PCLA's bundled agents. Cloned it into
`external/summit` (gitignored) and inspected it directly rather than
trusting its README's framing as a "CARLA-based simulator":

- `external/summit/LibCarla/`, `Unreal/`, `Util/BuildTools/`, and a
  top-level `Makefile` are all present — this is a full Unreal Engine
  source **fork** of CARLA, not a plugin or a script that attaches to a
  running CARLA server.
- `external/summit/PythonAPI/carla/setup.py` pins `version='0.9.8'`, and
  `CHANGELOG.md` opens at CARLA 0.9.8 — eight major CARLA releases behind
  the `carla==0.9.16` this project targets.

**Verdict: dropped.** Building SUMMIT from source means building a
second, separate Unreal Engine 4.24-based simulator binary (Epic Games
account linked to CARLA's GitHub org, tens of GB, hours of build time),
and even if built it would not be the same running CARLA world this
project's real server is. PCLA's agents are tested at 0.9.15/0.9.16
(confirmed: `external/PCLA/README.md`, and its `environment.yml` pins
`carla==0.9.16` exactly) and are not confirmed to run correctly against
a 0.9.8 server at all — this project has already hit real CARLA-version
API drift once (the semantic-segmentation tag ID bug in §7 above), so
assuming cross-version compatibility here would repeat a mistake already
paid for once.

**Replacement**: `pipeline/traffic_chaos.py` — CARLA's own built-in
Traffic Manager, tuned aggressively (tight following distance, frequent
lane changes, partial light/sign non-compliance, a two-wheeler-biased
vehicle mix, jaywalking pedestrians via `set_pedestrians_cross_factor`).
Coarser than SUMMIT's actual GAMMA crowd model, but needs zero extra
build steps and runs against the exact server everything else in this
project targets. Wired into `framework/` as `ChaoticTrafficMixin` (§13
below) rather than a standalone script, once the `framework-integration`
merge happened.

## 12. PlanT2 injection point -- found, documented, same honesty standard as MoFlow

Investigated whether PCLA's bundled PlanT2 agent (object-level,
planning-only — assumes perception is solved) could be fed this
project's own perception instead of PCLA's own ground-truth actor query.
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
all of which stay exactly as PCLA ships them.

Implemented as `pipeline/plant2_adapter.py`
(`convert_tracked_to_label_raw`), used by
`framework/autopilots/own_perception_plant2_autopilot.py` (originally a
standalone script before the `framework-integration` merge — see §14).
**Disclosed limitations (not glossed over)**:
1. Class vocabulary mismatch — PlanT2's fixed ontology
   (car/walker/static/static_car/stop_sign/traffic_light/emergency) has
   no "animal"/"auto-rickshaw" category; IDD-trained YOLO classes get
   best-effort mapped onto it (real information loss, PlanT2's choice not
   ours to fix without retraining it).
2. No real 3D bounding-box extents — our perception gives a LiDAR-point-
   cluster centroid, not a measured box size; extents are hand-picked
   per-class placeholders.
3. No heading estimation — our tracker has no orientation filter; object
   yaw is approximated from velocity direction when moving, else defaults
   to 0.
4. Static hazards (traffic lights, stop signs) are deliberately NOT
   emitted by the adapter — `run_step()` already injects those from
   ground truth independently of `get_bounding_boxes()`.
5. **Never tested against a live CARLA/PlanT2 run** — this is a design
   derived from reading the source, not verified against real inference
   output. First live run should sanity-check the adapter's label_raw
   output against the original `get_bounding_boxes()`'s ground-truth
   output side by side for a few ticks before trusting the comparison.

Also confirmed: `get_relative_transform`'s ego-relative frame uses
CARLA's native convention (x=forward, y=RIGHT), the **opposite sign** of
this project's own internal `FusedDetection`/`EgoState` convention
(x=forward, y=LEFT, per `types.py`). `plant2_adapter.py`'s
`_world_to_ego_relative_xy` flips this explicitly.

## 13. Merged into `framework-integration` instead of standalone scripts

A teammate independently built `framework/` (a `Scenario`/`Autopilot`/
`ScenarioRunner` split, on a separate `framework-integration` branch)
while §11/§12's work happened on a separate `summit-integration` branch,
each unaware of the other. Rather than keep either as a standalone
script (`run_own_perception_plant2.py`/`run_pcla_transfuserv6.py`, the
`summit-integration` branch's original form), the PCLA work was ported
into `framework/` as two new `Autopilot` subclasses
(`framework/autopilots/pcla_transfuser_autopilot.py`,
`framework/autopilots/own_perception_plant2_autopilot.py`) on a new
branch, `framework-summit-integration`, based on `framework-integration`
— keeping the teammate's base/calling convention as the one everything
else builds on, per explicit instruction, rather than merging in the
other direction.

**A real, load-bearing incompatibility was found and is worth recording
even though it wasn't resolved**: a third branch, `rayyan` (the MATLAB/
Simulink/Stateflow "Plan A" work), independently modified
`pipeline/pipeline.py` to remove `decision_logic.py`/`controller.py`
from `Pipeline.tick()`'s own orchestration — that branch's
`Pipeline.tick()` returns `(planned, timings)` only, not `(control,
planned, timings)`, because decision logic now lives in a Stateflow
chart and control in a MATLAB pure-pursuit block. This is incompatible
with what `framework/autopilots/pipeline_autopilot.py` expects from
`pipeline.tick()` today. **`rayyan` was deliberately left out of this
merge** (explicit instruction) — flagging this here so a future attempt
to unify all three branches doesn't discover the incompatibility from
scratch. See the session's own architecture-comparison writeup (not
committed to any branch — ask whoever has the conversation log if it's
needed) for the fuller "MATLAB wrapper vs. without it" breakdown.

**Two small, additive extensions were needed in `framework/base.py`**
to make the merge possible without touching the teammate's existing
contract for `PipelineAutopilot`/`pedestrian_jumpout`/`traffic_stress`
(verified: all three still import and construct unmodified after this
change):
1. `Autopilot.setup()` gained three new keyword-only, default-`None`
   params (`client`, `ego_vehicle`, `route_xml_path`) — PCLA's
   constructor genuinely needs a live `carla.Client` and the ego actor,
   which the original contract deliberately didn't expose (see
   `DESIGN_GUIDELINES.md` §7's own rule against it). This is the one
   disclosed exception, scoped to exactly what PCLA's constructor needs.
2. `Autopilot.cleanup()` (new, optional hook, default no-op) — needed
   because `PCLA.cleanup()` destroys the ego vehicle and PCLA's own
   attached sensors internally; without a hook, nothing would ever call
   it. `ScenarioRunner`'s own subsequent ego/sensor teardown afterward is
   safe, deliberate redundancy (fire-and-forget `client.apply_batch()`
   silently no-ops on an already-destroyed actor), not a conflict.

See `framework/DESIGN_GUIDELINES.md` §8 for the full PCLA-autopilot
setup/usage writeup, and `framework/pcla_route.py` for why a 2-waypoint
auto-generated route (rather than requiring a hand-authored route XML
per scenario) is sufficient — confirmed by reading PCLA's own
`route_parser.py`, not assumed.

## 14. Comparison-run timing lives in `Autopilot.debug_info()` -- SUPERSEDED, see §15

(Originally: at merge time, `ScenarioRunner`'s loop only collected
`autopilot.debug_info()['timings']` into a plain list and printed a
mean/max/min at shutdown — no CSV/JSON, no completion tracking, no
collision count, nothing for `metrics_export.py` to aggregate. Caught
when asked directly "are the eval scripts ready" — they weren't, for
ALL THREE autopilots, not just the PCLA ones. Fixed properly in §15;
kept this section rather than deleting it so the gap and how it was
found stay on record.)

**This number means something different per autopilot, same caution as
before**: `PipelineAutopilot`'s timing is the real 7-stage pipeline
breakdown; `Transfuserv6Autopilot`'s is one opaque `pcla.get_action()`
call; `OwnPerceptionPlanT2Autopilot`'s is
detection+fusion+tracking+PlanT2's `run_step()` combined, with no
separate decision-logic/planner stages (PlanT2 does its own internal
reasoning). Don't merge these into one "replanning latency" figure
across autopilots without accounting for what's actually being measured
in each case — this is the direct successor of the `summit-integration`
branch's original §13 caveat, restated for where the code actually lives
now.

## 15. `MetricsRecorder` actually wired into `framework/base.py` (fixes §14's gap)

`ScenarioRunner.run()` now constructs one `MetricsRecorder` per run
(`scenario_name=f"{ScenarioClass}_{AutopilotClass}"` — includes the
autopilot deliberately, since comparing autopilots on the same scenario
is the entire point, and `metrics_export.py` groups by `scenario_name`
verbatim), spawns a collision sensor inline (matching `run_live.py`'s
own established pattern — NOT folded into
`carla_runtime.spawn_ego_sensors()`, whose charter is the 3 perception
sensors, not metrics plumbing), calls `record_tick()` every tick, and
`finalize()` in the `finally` block.

Two small, additive `base.py` extensions this needed, verified against
every existing scenario/autopilot (all still construct and import
unmodified):
1. **`Scenario.is_complete(self, ctx) -> bool`** (new, optional, default
   provided: within `GOAL_REACHED_RADIUS_M` of `FINAL_GOAL`) — nothing
   in the original `Scenario` contract had a completion concept at all;
   every scenario just ran until Ctrl+C. `Scenario.MAX_TICKS` (optional,
   default `None`) added alongside it for a deterministic hard stop.
2. **`Autopilot.debug_info()`** gained three more optional keys —
   `'replanned'`, `'path_valid'`, `'decision_mode'` — with defaults
   (`False`/`True`/the autopilot's class name) for autopilots that don't
   have an equivalent concept. `PipelineAutopilot` now reports real
   values for all three (`PlannedPath.replanned`/`.is_valid`,
   `pipeline.decision_logic.mode.name`) — this data already existed
   inside `pipeline.tick()`'s return value, it just wasn't being
   surfaced through `debug_info()` before.

**Normalizing `'timings'` into `pipeline.pipeline.TickTimings`** (needed
because `MetricsRecorder.record_tick()` unconditionally reads
`timings.total_ms`, which a plain float doesn't have): if
`debug_info()['timings']` is already a `TickTimings` (PipelineAutopilot),
use it directly; otherwise wrap the single reported number into
`planning_ms` with the other 6 fields at zero, so `total_ms` still comes
out correct. Same convention the original `summit-integration` standalone
scripts already used for this exact problem — not a new decision, just
carried forward correctly this time.

**Verified** (no CARLA server available in this environment): a pure-
Python dry run feeding `MetricsRecorder.record_tick()`/`finalize()`
directly (bypassing `ScenarioRunner`, since that needs live CARLA)
confirmed the CSV/summary.json write path works end-to-end for both a
full-`TickTimings` autopilot and a single-number one, and that
`is_complete()`'s distance check returns `False`/`True` correctly on
both sides of `GOAL_REACHED_RADIUS_M`. **Not verified**: an actual
live-CARLA run recording real per-tick data — needs the same bounded
smoke test `DESIGN_GUIDELINES.md` §6 already prescribes, on a machine
with a running CARLA server.

## 16. `framework/ps_scenarios/` -- the 5 PS-named validation scenarios

The PS explicitly requires validating against 5 named scenarios: an
unmarked village road, a busy urban intersection without signals, a
highway merge with slow-moving vehicles, a dense market with mixed
traffic, and a sudden cattle-crossing. Built as a **new, separate**
package (`framework/ps_scenarios/`, distinct from `framework/scenarios/`'s
ad-hoc dev/test scenarios), using stock CARLA towns/assets only —
explicitly, no custom OpenDRIVE/RoadRunner import for this batch (that
was raised and set aside as a separate, out-of-scope stretch item in an
earlier discussion).

**Town mapping and reasoning**:
- Village road + cattle-crossing → **Town07** (CARLA's canonical rural
  map — narrow, sparse/no lane markings). Both reuse the same town
  deliberately: cattle crossings are a rural/village phenomenon in India,
  not urban, so sharing the setting is thematically correct, not just
  convenient — also means only 3 towns' worth of live coordinate-capture
  work instead of 4 (see the placeholder-coordinates caveat below).
- Urban intersection, no signals → **Town03**, reusing
  `framework/scenarios/traffic_stress.py`'s own real, already-live-
  verified `EGO_SPAWN`/`FINAL_GOAL` directly — zero new coordinate risk.
- Highway merge → **Town06** (CARLA's canonical highway-merge map;
  fallback Town04 if unavailable on the actual install — never confirmed
  live here).
- Dense market → **Town10HD** (compact downtown, higher density per
  actor than a sprawling town; fallback Town05).

**Two prerequisite infra changes, both small and additive** (verified:
every existing scenario/autopilot still constructs unmodified):

1. **`carla_runtime.py`'s `_classify_actor()`** now checks
   `actor.attributes['role_name']` FIRST (mapping `"livestock"` →
   `"animal"`), falling back to the existing type_id-prefix table.
   Fixes a real, previously-dead branch: `pipeline/decision_logic.py`
   line 134 checks `class_name in ("animal", "cow")` for its
   `ANIMAL_ON_ROAD` state, but nothing had ever produced that class_name
   before this — no livestock blueprint exists in stock CARLA, and
   `_classify_actor` only ever returned `"car"`/`"pedestrian"`/`None`.
   Confirmed non-colliding against every role_name already in use
   (`"ego"`, `"autopilot"`, `"chaotic_traffic"`). This is what makes
   `CattleCrossing`'s substitute walkers (spawned with
   `role_name="livestock"`) actually reach the PS-named
   `ANIMAL_ON_ROAD` branch instead of generic `OBSTACLE_DETECTED`.
2. **`pipeline/traffic_chaos.py`'s `spawn_chaotic_traffic()`** gained a
   `profile: str = "aggressive"` parameter. The existing per-vehicle
   Traffic Manager tuning (fast, tight-following, frequent lane changes)
   was factored into a new `_apply_traffic_manager_tuning()` helper and
   is now the `"aggressive"` branch (default, unchanged for every
   existing caller); a new `"slow_orderly"` branch (slower than the
   speed limit, generous following distance, no random lane changes,
   full light/sign compliance) was added for `HighwayMergeSlowTraffic`,
   which needs the opposite quality every other scenario's traffic was
   built for.

**Design choice made and rejected**: no new `TrafficLightsOffMixin` was
extracted from `TrafficStress`'s tested light-freeze/restore block, even
though `UrbanIntersectionNoSignals` needs the identical behavior.
Reasoning: this is the only other call site for that ~10-line block —
inlining a second proven-live block once is lower-risk than introducing
a second mixin stacked alongside `ChaoticTrafficMixin` (2-mixin MRO
chaining for one reuse site adds indirection without real benefit here).

**`HighwayMergeSlowTraffic` is not purely ambient** — deliberately
combines the new `"slow_orderly"` ambient traffic with ONE specific slow
lead vehicle, spawned directly (not via Traffic Manager) with a hand-set
constant low speed, placed in the ego's own lane close enough ahead to
force an actual overtake/merge decision. Mirrors
`pedestrian_jumpout.py`'s own "ambient scene + one scripted, findable
hazard" pattern rather than leaving the interaction to Traffic-Manager
randomness alone.

**Honest, disclosed limitation, same standard as every other shortcut in
this project**: `CattleCrossing`'s "livestock" is 2-3 ordinary walker
blueprints wearing a `role_name` label — no real animal model exists in
stock CARLA (already noted in `HANDOFF.md` §7). The crossing speed
(1.2 m/s) is set to a plausible walking-cow pace, distinctly slower than
`PedestrianJumpOut`'s 3.5 m/s human "run," but this is still visually a
human-shaped walker, not a cow.

**Hard constraint, stated plainly rather than worked around with
fabricated numbers**: there is no live CARLA server or GPU in this dev
environment. Three of the five new scenarios use towns
(Town06/Town07/Town10HD) never previously loaded in this repo — their
real `EGO_SPAWN`/`FINAL_GOAL`/hazard-placement coordinates are
**placeholders**, each marked `# TODO: capture via
carla_print_coordinates.py` in the file itself (the exact same utility
script already built earlier in this project for exactly this purpose).
Every one of these 5 files passes syntax (`ast.parse`) + headless
import + construction here — none has been run live, and 3 of the 5
are not yet runnable-for-real until those coordinates are captured on
the team's actual CARLA machine. `UrbanIntersectionNoSignals` (reuses
`TrafficStress`'s known-good Town03 coordinates) is the one new scenario
with zero coordinate risk.

## 17. Waypoint-derived positions instead of hardcoded coordinates, for 2 of the 3 remaining gaps

§16 above left 3 scenarios with hardcoded placeholder coordinates
pending live capture via `carla_print_coordinates.py`. Asked directly to
solve this for the highway-merge scenario and to add "opposite lane
driving" (a wrong-way vehicle) without needing new coordinates at all —
answered by deriving positions from CARLA's own map/waypoint graph at
runtime instead of literal `(x, y, z, yaw)` values.

**New shared helper**: `framework/ps_scenarios/waypoint_utils.py` —
`walk_forward(start_waypoint, distance_m, step_m)` /
`walk_backward(...)`, built on `carla.Waypoint.next()`/`.previous()`
(confirmed present with the expected signature in the installed
`carla==0.9.16` client). Walks in small steps (default 10m) rather than
one `next(distance_m)` call, so a junction falling within the requested
distance is less likely to derail the walk onto an unintended branch;
stops early and returns the last valid waypoint if the road ends before
covering the full distance, rather than raising. Verified with a fake
waypoint-chain stand-in (a straight line of mock waypoints, no live
CARLA needed) — confirmed the loop covers the requested distance
exactly, handles a partial final step, and stops correctly at a
simulated dead end rather than overshooting or erroring.

**`highway_merge_slow_traffic.py` rewritten**: `EGO_SPAWN` is now
`world.get_map().get_spawn_points()[EGO_SPAWN_POINT_INDEX]` (always a
valid, on-road, correctly-oriented spawn — no coordinate guessing at
all), computed inside `spawn_actors()` once a real `world` exists rather
than as a class-level literal. `FINAL_GOAL` and the scripted slow lead
vehicle's spawn point are both derived by walking forward from that
spawn point's waypoint — the lead vehicle is now GUARANTEED to be in the
ego's own lane (it's reached via the same waypoint chain), not just
placed at a coordinate that was hopefully close enough. Confirmed safe
that `ScenarioRunner.run()` doesn't read `scenario.EGO_SPAWN`/
`FINAL_GOAL` until after `spawn_actors()` has already run (checked
`framework/base.py`'s actual call order) — so computing them as instance
attributes inside `spawn_actors()`, rather than class-level literals, is
safe or the whole file would have to be restructured around it. The only
thing left worth tuning once this runs live is
`EGO_SPAWN_POINT_INDEX` itself (an integer, to land near a real
merge/on-ramp) — not a coordinate to hunt for with a spectator.

**New `framework/ps_scenarios/wrong_way_mixin.py` — `WrongWayVehicleMixin`**:
a vehicle scripted to drive against traffic flow, directly in the ego's
own lane, facing oncoming — real "opposite lane driving" / a
prohibited-lane incursion, and a genuine common Indian-road hazard
(wrong-side overtaking around a blind corner). Entirely coordinate-free:
positions itself `WRONG_WAY_SPAWN_OFFSET_M` ahead of whatever
`self.EGO_SPAWN` the mixed-in scenario has (placeholder or real, doesn't
matter — purely relative), facing the opposite way to the lane's own
defined direction. Chose a directly-scripted vehicle over
`TrafficManager.force_lane_change()` deliberately: investigated earlier
(§ discussion during the SUMMIT/PCLA work) that `force_lane_change()`
only moves an actor into whatever `Waypoint.get_left_lane()`/
`get_right_lane()` returns as the adjacent lane, which on the real
Warangal OSM-derived network is a same-direction lane, not the true
opposing carriageway — and TM's own collision avoidance actively fights
sustaining a wrong-way maneuver regardless of map. A directly-controlled
vehicle sidesteps both problems.

**Stays on the road via a small per-tick proportional heading
controller** (`_steer_toward()`), NOT a reuse of
`pipeline/controller.py`'s `PurePursuitController` (built for the ego's
own pipeline call signature/lookahead model — unnecessary machinery for
a scripted hazard actor). Each tick, re-locates the vehicle's current
waypoint and steers toward a point along `Waypoint.previous()` (not
`next()` — since this vehicle deliberately drives against the lane's own
defined direction, `previous()` is what tracks ITS forward path).
Verified the heading-error math directly (no live CARLA needed): target
directly ahead → steer ≈ 0; target 45° off → steer ≈ 0.5; target directly
behind → steer clips to ±1; a non-zero starting yaw correctly changes
what counts as "ahead" — all four cases matched expectations exactly.

Wired into `unmarked_village_road.py` (`class
UnmarkedVillageRoad(WrongWayVehicleMixin, Scenario)`) — the strongest
thematic fit among the 5 scenarios (wrong-side overtaking is
overwhelmingly a rural/village-road phenomenon), alongside its existing
static-obstruction hazard (moved closer to `EGO_SPAWN` than the
wrong-way vehicle's default 60m offset, so the two hazards don't overlap
into one confusing pileup). Required updating that file's own
`spawn_actors()` to end with `super().spawn_actors(world, bp_lib,
client)` so it actually chains into the mixin — verified via MRO
inspection (`UnmarkedVillageRoad → WrongWayVehicleMixin → Scenario →
ABC → object`), not just assumed. The mixin is a one-line opt-in for any
other scenario (e.g. `highway_merge_slow_traffic.py`, as a "ghost
driver" entering via an exit ramp) — not forced in anywhere else,
deliberately, to avoid stacking hazards into scenarios that weren't
asked for it.

**Still not resolved, honestly**: `dense_market_mixed_traffic.py`'s own
`EGO_SPAWN`/`FINAL_GOAL` are still hardcoded placeholders — this session
addressed exactly the two things asked for (highway merge, opposite-lane
driving), not every remaining placeholder. The same waypoint-derivation
technique would apply directly if asked for that one too.

**Never tested against a live CARLA server or a real map's waypoint
graph** — the loop/steering logic itself was verified with mock
stand-ins (a fake straight-line waypoint chain, hand-computed heading
cases), which catches logic bugs but not real-map behavior: whether the
steering gain feels right on an actual curved road, whether `next()`/
`previous()` behave as expected across a real junction, and whether
`EGO_SPAWN_POINT_INDEX=0` on Town06 happens to land anywhere near an
actual merge lane are all open questions for the first live run.

## 18. `SafetyEnvelope` -- an independent watchdog wrapping TFv6, plus a real bug found and fixed along the way

Asked what else could be done "on top of" the existing pretrained TFv6
agent (without touching its weights) so it handles chaotic traffic more
safely. Answered with the standard real-world AV pattern: a learned/
opaque policy plus an independent runtime monitor that can veto it —
not a replacement for the policy's own driving decisions, a backstop for
its rare bad ones, specifically under the chaotic scenarios this project
builds (`chaotic_traffic.py`, `urban_intersection_no_signals.py`, the
wrong-way vehicle in `unmarked_village_road.py`).

**New: `framework/safety_envelope.py`'s `SafetyEnvelope`.** Queries
CARLA ground truth directly (same disclosed-shortcut pattern as
`carla_runtime.GroundTruthDetector`/`pipeline/drivable_area.py` — a real
production monitor would use its own independent sensor suite, this is
a simulation-only stand-in, stated plainly) for a cheap min-TTC estimate
across nearby actors, and overrides to a full emergency brake (steering
preserved, same choice `pipeline/pipeline.py`'s own emergency-brake path
already makes) when TTC drops below `critical_ttc_s` (default 1.5s),
releasing only once TTC recovers above a separate, higher
`release_ttc_s` (default 2.5s) — a hysteresis gap so the override
doesn't chatter tick-to-tick right at the threshold.

**Deliberately NOT a reuse of `pipeline/decision_logic.py`'s full state
machine** — that class is a hierarchical mode-switching system built to
drive `PipelineAutopilot`'s own planner/controller loop (debounce
timers, `ANIMAL_ON_ROAD`/`OBSTACLE_DETECTED` sub-states, a
`SAFETY_SUPERVISOR` watchdog for pipeline latency/crashes), none of
which apply to a black-box PCLA agent that exposes no internal state at
all through PCLA's public API. `SafetyEnvelope` is deliberately smaller
and single-purpose.

**A real, previously-undiscovered bug in `pipeline/decision_logic.py`
was found and fixed while building this**, comparing its TTC math
against the (correctly ego-relative from the start) new class:
`_min_ttc` computed `dist = sqrt(ox**2 + oy**2)` — distance from WORLD
ORIGIN, not from the ego. `pipeline.py` transforms tracked positions to
world frame before calling `decision_logic.step()` (see
`_local_to_world_xy`'s own docstring), but `step()` was only ever given
`ego_speed`, never `ego_x`/`ego_y` to subtract. On the synthetic demo
(`run_demo.py`), the fake ego sits at the origin the whole run, making
this invisible — **the exact same class of masking effect that made the
original `_local_to_world_xy` bug invisible before it was fixed** (see
§6). On any real CARLA map (Town03 coordinates run into the hundreds),
this would have silently fed nonsense "distances" into every
`OBSTACLE_DETECTED`/`ANIMAL_ON_ROAD`/`EMERGENCY_BRAKE` mode-switching
decision on the very first live run — a real, latent, undetected safety
bug in code that's been "working" only because it's never yet touched
real coordinates.

**Fixed properly, not worked around**: `DecisionLogic._min_ttc()` and
`.step()` now take `ego_x`/`ego_y` and subtract before computing
distance; `pipeline/pipeline.py`'s one call site updated to pass
`ego.x`/`ego.y`. Confirmed via a direct before/after test (a pedestrian
placed 3m from a non-origin ego at Town03-scale coordinates: the old
formula would have computed ~586m/inf TTC — never detected; the fixed
formula correctly computes 3m/0.3s TTC). Re-ran `run_demo.py`
afterward — still 40 clean ticks, ~9.4ms mean latency, no regression
(the synthetic demo's fake ego sitting at the origin means this
particular bug never affected its output either way, so an unchanged
result is exactly what a correct fix should produce here).

**Also made `carla_runtime.py`'s `_classify_actor` public**
(`classify_actor`, one call site updated) so `SafetyEnvelope` could reuse
its exact type_id/role_name classification table instead of duplicating
it a third time (`GroundTruthDetector` and now `SafetyEnvelope` both use
it) — including the `role_name="livestock"` → `"animal"` override from
§16, so a `SafetyEnvelope`-protected run correctly distinguishes a
livestock hazard from a generic pedestrian/vehicle one too.

**Wired into `Transfuserv6Autopilot`** (`enable_safety_envelope: bool =
True` constructor flag, default on) — `compute()` calls
`safety.check(ctx.world, ctx.ego_vehicle)` then `safety.wrap_control()`
around TFv6's raw output; `debug_info()`'s `decision_mode` reports
`"TFV6_SAFETY_OVERRIDE"`/`"TFV6_NORMAL"`/`"TFV6_UNWRAPPED"` so
`MetricsRecorder` picks up override activity per tick. Setting
`enable_safety_envelope=False` gives a clean A/B: TFv6 alone vs.
TFv6+safety-envelope on the identical scenario — a genuinely useful
comparison for the report. **Not yet wired into
`own_perception_plant2_autopilot.py`** — `SafetyEnvelope` is reusable
there as-is (same `check()`/`wrap_control()` calls), just not done this
round since it wasn't what was asked for.

**Also corrected while touching this file**: `Transfuserv6Autopilot`'s
own docstring previously listed variant names like `"tfv6_4cameras_resnet34"`
— these are the checkpoint FOLDER names from `agents.json`'s `"config"`
values, not the actual `agent_key` strings `give_path.py` expects (which
split on `_` and look up the SHORTER key, e.g. `"tfv6_4cameras"`).
Fixed to the real keys, confirmed directly against `agents.json`.

**Verified without a live server**: `SafetyEnvelope`'s full TTC +
hysteresis logic against 4 hand-built mock-actor cases (no hazard →
passthrough; a 3m pedestrian → override with steering preserved; TTC
recovering to a value between the two thresholds → override correctly
STAYS active, confirming the hysteresis gap actually holds; TTC fully
clearing past `release_ttc_s` → override correctly releases, original
control restored) — all four matched expectations exactly. The
`decision_logic.py` fix verified with a direct numeric before/after
comparison (above) plus a full `run_demo.py` regression run. **Never
tested against real CARLA actor kinematics or an actual TFv6 inference
loop** — the ground-truth query pattern itself (`world.get_actors()`,
`.get_transform()`, `.get_velocity()`) mirrors `GroundTruthDetector`'s
already-used pattern, but this specific class has not been exercised
live.

## 19. 4th autopilot: `PlanT2GroundTruthAutopilot` -- a ceiling for `own_perception_plant2`, plus `SafetyEnvelope` parity

Asked directly why `own_perception_plant2_autopilot.py` bothers
overriding PlanT2's `get_bounding_boxes()` at all, instead of just
letting PlanT2 use its own (ground-truth) version like `pcla_tfv6` does
for TransFuser v6. Answered, then built the natural follow-up: running
PlanT2 with its own ground truth IS a useful data point — not as a
replacement for the own-perception version, but as a **ceiling**. The
gap between "PlanT2 + your perception" and "PlanT2 + perfect perception"
on the identical scenario isolates exactly how much performance is lost
to your perception's real imperfections (tracking jitter, no heading
estimate, the fixed class vocabulary — see `pipeline/plant2_adapter.py`)
versus PlanT2's planning quality itself. Neither number alone answers
that; the two together do. This is a different question than
`pcla_tfv6` already answers — `pcla_tfv6` still has to perceive from
real camera/LiDAR pixels, this ceiling doesn't perceive at all, it reads
perfect simulator state directly, making it the MORE privileged of the
two references, not a duplicate.

**New: `framework/autopilots/plant2_ground_truth_autopilot.py`'s
`PlanT2GroundTruthAutopilot`** — implemented as a thin subclass of
`Transfuserv6Autopilot`, not a new file duplicating its logic. Both
classes are mechanically identical (construct one bundled PCLA agent,
call `get_action()`, optionally wrap with `SafetyEnvelope`) — the only
difference is which checkpoint (`agent_key`). Subclassing means a future
fix to the shared logic (route handling, cleanup, the safety wrapper)
automatically applies to both, rather than needing to be applied twice.
Kept as its own registered class (not just a different `agent_key`
argument at the call site) specifically so
`framework/base.py`'s `ScenarioRunner.run()` — which builds each run's
`MetricsRecorder` `scenario_name` from `type(self.autopilot).__name__`
— produces a distinctly-named metrics file for this configuration
automatically. Its own `debug_info()` override just relabels the
inherited `"TFV6_*"` decision-mode strings to `"PLANT2_*"` so CSV/log
output isn't actively misleading about which model actually ran.
Registered as `"plant2_ground_truth"`.

**`SafetyEnvelope` (§18) also wired into `own_perception_plant2_autopilot.py`**,
closing the "reusable there, not yet wired in" gap explicitly flagged
when it was first built. Same `enable_safety_envelope: bool = True`
constructor flag and `"PLANT2_NORMAL"`/`"PLANT2_SAFETY_OVERRIDE"`/
`"PLANT2_UNWRAPPED"` `decision_mode` labeling as the new autopilot above
— all three PCLA-backed autopilots (`pcla_tfv6`,
`own_perception_plant2`, `plant2_ground_truth`) now have the identical
independent watchdog available, at parity with each other.

**Verified**: all 4 autopilots (`pipeline`, `pcla_tfv6`,
`own_perception_plant2`, `plant2_ground_truth`) and all 8 registered
scenarios still construct cleanly together; `PlanT2GroundTruthAutopilot`'s
MRO confirmed by inspection
(`PlanT2GroundTruthAutopilot → Transfuserv6Autopilot → Autopilot → ABC`);
`agent_key` defaults confirmed correct for both classes
(`plant2_plant2` / `tfv6_regnet`); `enable_safety_envelope=False` path
confirmed to produce the `"*_UNWRAPPED"` label and a `None` safety
object for both new/updated autopilots. **Not verified**: an actual live
run of any of the four — same standing caveat as everything else built
this session.
