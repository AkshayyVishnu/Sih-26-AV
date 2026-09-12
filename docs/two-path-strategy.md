# Two-Path Strategy: MATLAB/Simulink vs. Pure Python

## Context

The PS's expected-solution text explicitly requires the pipeline to be
built **"in MATLAB and Simulink."** What's actually built so far
(`pipeline/`) is 100% Python, verified working end-to-end on synthetic
data. Given genuine uncertainty around the MATLAB↔Python integration
(no prior art found anywhere for calling a custom Python pipeline from
Simulink via `py.*`) and a hard deadline, the team is pursuing **both
paths in parallel** with 2 teammates, one per path — Plan B as insurance,
not as the primary intended deliverable.

**Decision at submission time**: if Plan A works reliably, ship it as
primary and mention Plan B as a validation/backup in the technical
report. If Plan A doesn't come together in time, Plan B is a real,
working fallback — submit it with an honest, explicit paragraph
disclosing the deviation and why, rather than submitting nothing or a
half-broken MATLAB integration.

**The one thing that doesn't transfer between the two plans**: decision
logic. Stateflow (Plan A) can't be called from pure Python (Plan B) — if
each plan needs decision logic, it has to exist in two forms. Minimize
this by finalizing the actual state-machine *design* once (state names,
transitions, guards — the `ANIMAL_ON_ROAD`/`SAFETY_SUPERVISOR` structure
already discussed) in whichever form is faster to prototype, then
re-express that same design as a Stateflow chart for Plan A. The
thinking happens once; only the re-drawing is duplicated.

---

## Plan A — MATLAB/Simulink + `py.*` → CARLA (compliant path)

**Goal**: a `.slx` Simulink model that is the actual simulation pipeline
— Simulink's own clock drives the loop, not a Python script — satisfying
the PS's literal wording.

**Status**: nothing built yet. Everything below starts from zero.

**Design decided**: wrap the existing, already-tested Python pipeline
via **one combined `py.*` call per tick** (not one call per stage) —
```matlab
result = py.pipeline.Pipeline().tick(detections, lidar_points, ego, seg_tags);
```
This minimizes MATLAB↔Python boundary crossings to a single hop per
tick, which matters directly for latency (see below) — five separate
per-stage crossings would multiply the crossing overhead five times for
no benefit.

**What does NOT get rewritten**: `pipeline/perception_fusion.py`,
`tracker.py`, `predictor.py`, `planner.py`, `drivable_area.py`,
`pipeline.py` all stay exactly as they are, in Python, untouched. Only a
thin MATLAB Function block (a few lines) calling into them and unpacking
the returned Python object into MATLAB structs is new code.

**Steps, in order**:
1. **Test the riskiest unknown FIRST, in isolation**: one minimal MATLAB
   Function block calling `py.pipeline.Pipeline().tick(...)` with dummy
   inputs and confirming a sane result comes back — before building
   anything else around it. This is the single most uncertain piece in
   the entire project (no prior art found for this exact pairing).
2. If that works: build the full `.slx` model — the MATLAB Function
   block wrapping the pipeline, the Stateflow chart (ported from
   whatever design Plan B's decision logic settles on) wired to its
   output, and a vehicle-motion block (MPC or simple controller) sending
   commands back to CARLA via `py.*`.
3. Wire the actual CARLA connection through the same `py.*` mechanism —
   either inside the wrapper block or as a separate block feeding
   sensor data in each Simulink tick.
4. Integrate with whatever scenes/environment the environment teammate
   has built.
5. Run full closed-loop validation, capture metrics.

**Expected latency**: ~20-45ms per tick (pipeline's own ~10-15ms +
one `py.*` crossing) — an estimate, not a benchmark. **Must be measured
directly**, not assumed, the moment step 1 above is working.

**Known risks**:
- No prior art anywhere for Simulink calling into a custom Python
  pipeline this way — genuinely unexplored territory.
- Data marshaling between MATLAB structs and the pipeline's Python
  dataclasses (`Detection`, `TrackedObject`, `PlannedPath`, etc.) needs a
  translation layer that doesn't exist yet.
- `pyenv` in MATLAB must point at the same `.venv` this pipeline was
  built and tested in, with a matching Python version.

**If it works**: this is the primary deliverable, produces the literal
artifact the PS asks for, most defensible if evaluators check for actual
MATLAB/Simulink artifacts.

---

## Plan B — Pure Python → CARLA (insurance path)

**Goal**: finish connecting the already-working pipeline to a real
(not synthetic) CARLA instance, so there's a guaranteed, working,
metric-producing demo regardless of how Plan A goes.

**Status**: ~80% done already. `pipeline/` runs correctly end-to-end
(fusion → tracking → drivable-area → prediction → planning), verified
via `run_demo.py` on synthetic data. What's missing is narrower than it
sounds:

**Steps, in order**:
1. Swap `run_demo.py`'s `generate_synthetic_tick()`/`generate_synthetic_segmentation()`
   for real calls against a live CARLA instance:
   ```python
   client = carla.Client('<carla-host-ip>', 2000)
   # pull real camera image -> friend's YOLO -> Detection objects
   # pull real LiDAR point cloud -> lidar_points_xyz array
   # pull real semantic segmentation camera -> segmentation_tags array
   ```
2. Send the resulting `PlannedPath` back to CARLA as actual vehicle
   control (steer/throttle/brake) each tick — this is genuinely new
   code, doesn't exist yet (the pipeline currently only *produces* a
   path, nothing consumes it into `carla.VehicleControl` yet).
3. Build a **Python decision-logic layer** replacing Stateflow — same
   state design already worked out (`NORMAL_DRIVE` /
   `OBSTACLE_DETECTED` → `ANIMAL_ON_ROAD` sibling / `EMERGENCY_BRAKE`
   + `REPLAN` as parallel / `SAFETY_SUPERVISOR` watchdog / debounce /
   history), implemented as a simple hand-rolled state machine or via
   the `transitions` library — sits between the planner's output and
   the control-output step from #2.
4. Run full closed-loop validation against real CARLA scenes, capture
   metrics.

**Expected latency**: ~10-15ms (already measured) + native `carla`
client call overhead (not yet benchmarked, but should be modest — it's
a native Python library call, not a cross-language boundary).

**Known risks**:
- Does **not** satisfy the PS's literal "in MATLAB and Simulink"
  wording — a real, acknowledged compliance risk if submitted as
  primary rather than as a disclosed backup.
- Real sensor data (vs. synthetic) may surface issues the mock data
  didn't — e.g., LiDAR noise characteristics, segmentation tag IDs not
  matching the assumed CARLA version (already flagged as a known
  pitfall in `pipeline/drivable_area.py`).

**If Plan A doesn't finish in time**: this becomes the actual
submission, with an explicit disclosed-deviation paragraph in the
technical report explaining the timeline constraint and why this path
was taken.

---

## Recommendation

Pursue both, as designed above — this isn't "build two full systems,"
it's "finish the system that's already 80% done as guaranteed insurance,
while attempting the compliant one in parallel." Assign the teammate with
less integration-debugging risk tolerance to Plan B (lower risk, clearer
path to a working result); assign the other to Plan A, starting with
step 1's isolated `py.*` test before anything else.
