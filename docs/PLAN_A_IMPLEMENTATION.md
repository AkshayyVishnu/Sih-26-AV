# Plan A: MATLAB/Simulink + `py.*` → CARLA Pipeline

## Context

`docs/two-path-strategy.md` establishes that the PS's expected solution requires the pipeline "in MATLAB and Simulink," but the actual working pipeline (`pipeline/`) is 100% Python, verified end-to-end on synthetic data (`run_demo.py`, ~10.8ms mean latency/tick). The team is running two paths in parallel; **Plan A** — wrapping the existing, untouched Python pipeline in a real `.slx` Simulink model via `py.*` calls, with Simulink's own clock driving the loop — is the compliant, primary-intended deliverable. Nothing for Plan A exists yet; this plan starts from zero.

Two design decisions were confirmed with the user before finalizing this plan:
1. **Extend `PlannedPath`** (in `pipeline/types.py`) with `nearest_obstacle_class`, `nearest_obstacle_confidence`, `nearest_obstacle_distance_m`, `nearest_obstacle_ttc_s` — populated inside `Pipeline.tick()`. Verified these aren't new data: YOLO's class/confidence already flow through `Detection → FusedDetection → tracker.py's _Track.update()/to_tracked_object() → TrackedObject.class_name/confidence` (confirmed in `pipeline/tracker.py:62-79`); distance is already on `FusedDetection.distance_m`. Only TTC/relative-velocity need one line of new arithmetic from data already computed in `tick()`. This is additive/backward-compatible (`run_demo.py` keeps working unmodified) and serves Plan B's parallel Python state machine too, since it needs the identical inputs.
2. **Stateflow chart built via scripted API** (`matlab/build_stateflow_chart.m`, using `Stateflow.Chart`/`addState`/`addTransition`) rather than hand-drawn in the GUI — diffable in git, rebuildable when guard thresholds get tuned.

## What stays untouched
`pipeline/perception_fusion.py`, `tracker.py`, `predictor.py`, `planner.py`, `drivable_area.py`, `pipeline.py`'s orchestration logic, `logging_utils.py` — all stay exactly as-is except the one additive `PlannedPath` change above. All new work lives in new `matlab/` and `models/` directories, plus one new `pipeline/carla_bridge.py` (Phase 3).

---

## Phase 0 — Environment prerequisites (blocking)

1. **Fix `requirements.txt`**: add the missing `filterpy` and `scipy` pins (`pipeline/tracker.py` and `planner.py` import both; neither is currently listed) — pick a `scipy` version compatible with the existing `numpy==1.26.4` pin.
2. **Create `.venv`**: `python3.10 -m venv .venv && pip install -r requirements.txt`. Checkpoint: `python run_demo.py` must reproduce the known-good baseline (~10.8ms mean latency, no import errors) *before touching MATLAB* — isolates "pipeline env is fine" from "MATLAB interop is fine."
3. **MATLAB activation**: MATLAB R2026a is installed (`/usr/local/bin/matlab`) but not signed in — a `-batch` run prompted for MathWorks account login. Launch `matlab` interactively once to complete activation; confirm with `matlab -batch "disp(1+1)"` afterward.
4. **Configure `pyenv`**: create `matlab/init_pyenv.m` as a single shared init helper (called from every other script/block) that sets:
   ```matlab
   setenv('PYTHONPATH', '/home/rayyan/projects/sih_26');
   pyenv(Version="/home/rayyan/projects/sih_26/.venv/bin/python3.10", ExecutionMode="OutOfProcess");
   ```
   `OutOfProcess` chosen deliberately over the default `InProcess` — `torch`/`opencv-python`/`ultralytics` are native-extension-heavy and sharing MATLAB's loaded libraries in-process is a known crash source for this class of package. Checkpoint: `py.sys.version` shows 3.10.x, `py.importlib.import_module('pipeline.pipeline')` doesn't error.

## Phase 1 — Riskiest-unknown isolation test (test this before anything else)

Per `two-path-strategy.md`'s explicit instruction: this exact pairing (Simulink calling a custom Python pipeline via `py.*`) has no prior art anywhere.

1. **`matlab/test_pipeline_call.m`** — plain script, no Simulink: construct one `py.pipeline.pipeline.Pipeline(cam, ext, pyargs('dt', 0.05))`, call `.tick()` with hand-built `py.pipeline.types.Detection(...)`/`EgoState(...)`/`py.numpy.array(...)` LiDAR points, unpack the returned tuple, and confirm real numeric waypoints come back. Run in a 40-iteration loop on the *same* `Pipeline` handle (mirrors `run_demo.py`'s stateful loop — tracker/planner state must persist across calls) and record `total_ms` — **measure the ~20-45ms latency estimate directly, don't assume it.**
2. **`models/plan_a_isolation_test.slx`** — one throwaway model: a MATLAB Function block wrapping the same call (calling out to a companion `.m` function so logic stays diffable, not embedded inline), fed by From-Workspace/Constant blocks. Confirms a distinct risk from step 1: whether Simulink's compiled step-function scheduling tolerates a `py.*` call inside its loop.

**Do not proceed past Phase 1 until both checkpoints are green** — this is the actual unexplored-territory risk; everything after is comparatively conventional MATLAB/Simulink work.

## Phase 2 — Full `.slx` skeleton (still no CARLA)

### 2.1 Extend `pipeline/types.py` and `pipeline/pipeline.py`
Add to `PlannedPath`:
```python
nearest_obstacle_class: str = ""
nearest_obstacle_confidence: float = 0.0
nearest_obstacle_distance_m: float = float("inf")
nearest_obstacle_ttc_s: float = float("inf")
```
Populate inside `Pipeline.tick()` right before `return planned, timings`, using the already-computed `tracked`/`fused` lists (pick nearest by `FusedDetection.distance_m`, copy `class_name`/`confidence` across, compute `ttc_s` from distance and closing velocity vs. `ego.speed`). Checkpoint: `run_demo.py` still produces identical waypoint/latency output — purely additive.

### 2.2 Data-marshaling contract (write once, reuse everywhere)
Concrete per-field rules for MATLAB ↔ Python dataclass conversion:
- Scalars (`float`/`int`) → `double(x)`; `bool` → `logical(x)`; `str` → `string(x)`/`char(x)`.
- Lists of tuples (`waypoints`, `position_history`) → wrap in `py.numpy.array(x)` on the MATLAB call site, then `double()` the result.
- Constructing `Detection`/`EgoState` from MATLAB → positional constructor calls, e.g. `py.pipeline.types.Detection(cname, conf, x1, y1, x2, y2)`.
- `lidar_points_xyz`/`segmentation_tags` → `py.numpy.array(matlab_matrix, pyargs('dtype', 'float32'/'int32'))`.
- **Fixed-size signals**: Simulink ports are fixed-size; pad `waypoints` to `[MAX_WP x 2]` (e.g. `MAX_WP=200`) plus an explicit `n_wp` count, and do the same for the detections input (`[MAX_DET x 5]` + `class_id` vector + `n_det`) — matches MathWorks' own planning-example pattern (Automated Parking Valet).
- **`class_name` string problem**: Simulink signals can't carry Python strings. Create `matlab/class_id_map.m` fixing a numeric ID ↔ string convention (`0=pedestrian, 1=animal, 2=vehicle, 3=unknown`) — **flag this as an open interface item to confirm with whoever owns the perception/YOLO block**, since no canonical enum exists in the repo today.

### 2.3 `matlab/pipeline_wrapper.m` — the MATLAB Function block
Persistent `Pipeline` object (constructed once, held across ticks — it owns tracker/planner state). Body: marshal inputs per 2.2, call `pipelineObj.tick(...)`, unpack `PlannedPath`'s new nearest-obstacle fields alongside waypoints/is_valid/replanned/total_ms, wrapped in `try/catch` that sets an output `pipeline_ok=false` on any Python exception. **This catch is load-bearing, not defensive boilerplate** — MATLAB Function blocks halt the whole simulation on an uncaught error by default; without it, one Python exception mid-run kills the model instead of giving the safety watchdog a fault signal to react to.

Checkpoint: drive this block from a `matlab/generate_synthetic_tick.m` (MATLAB port of `run_demo.py`'s synthetic generator) for 40 steps; confirm output matches Phase 1's numbers call-for-call, and confirm a deliberately-malformed input trips `pipeline_ok=false` without crashing the model.

### 2.4 Stateflow chart — `matlab/build_stateflow_chart.m`, producing `models/plan_a_full.slx`'s `DecisionLogic` chart
Built via the Stateflow API per the confirmed decision. Two parallel top-level regions, matching `docs/architecture.md`'s design exactly:

- **Region 1 (`DRIVE_MODE`)**: `NORMAL_DRIVE` (with a history junction over `LANE_KEEP`/`OVERTAKE_IN_PROGRESS` sub-states) → `OBSTACLE_DETECTED` on a debounced entry (`duration(obstacleConfirmed, 0.3)`); a truth table inside `OBSTACLE_DETECTED` on `(nearest_class_id, nearest_conf, nearest_ttc)` routing to `ANIMAL_ON_ROAD` (livestock → replan/crawl-around, not hard brake) or the parallel AND-states `EMERGENCY_BRAKE`+`REPLAN`; return to `NORMAL_DRIVE` via history only on a sustained-clearance gate (`before(1,obstacleFrame) && duration(clear,2)`) to prevent chattering.
- **Region 2 (`SAFETY_SUPERVISOR`)**: `WATCHDOG_OK` ↔ `WATCHDOG_FAULT`, triggered by `!pipeline_ok` or `total_ms > 150` sustained.
- **Simplification**: combine the two regions' brake signals in plain Simulink logic just after the chart (`final_emergency_brake = chart.emergency_brake_active || chart.safety_override_active`) rather than broadcasting inter-region events — functionally identical, far easier to unit-test each region independently.

Inputs: `is_valid`, `replanned`, `pipeline_ok`, `total_ms`, `nearest_class_id`, `nearest_conf`, `nearest_dist`, `nearest_ttc`, `ego_speed`. Outputs: `emergency_brake_active`, `replan_requested`, `animal_caution_active`, `safety_override_active`, `active_submode`.

Checkpoint: drive the chart directly (Signal Editor with hand-authored traces) covering: latency-spike fault, `pipeline_ok` going false, sub-0.3s obstacle blip (must NOT transition — debounce), clear-then-reappear within 2s (must NOT resume — anti-chatter), and history-junction resume correctness.

### 2.5 Vehicle-motion / control block
Start with a pure-pursuit controller (`matlab/pure_pursuit_controller.m`) rather than Adaptive MPC — treat MPC as a stretch goal, a separate risk from proving `py.*` interop. Emergency-brake/safety-override booleans override the pure-pursuit output unconditionally. `matlab/carla_control_block.m` holds a persistent CARLA vehicle-actor handle (same persistence pattern as 2.3) and applies `carla.VehicleControl(throttle, steer, brake)` via `py.*`, wrapped in the same `try/catch` pattern.

By end of Phase 2: a complete `.slx` exercising wrapper → Stateflow → controller end-to-end on synthetic data, CARLA block stubbed/unconnected.

## Phase 3 — Wire the real CARLA connection
Replace the synthetic generator with real per-tick pulls: `ego_vec` from `carla_vehicle.get_transform()`/`get_velocity()`. **Known integration wrinkle**: CARLA's sensor API is an async `sensor.listen(callback)` push model, which doesn't fit Simulink's synchronous per-step pull model. Resolve by running CARLA in synchronous mode (`world.tick()` once per Simulink step) with a small circular-buffer object living **in Python** (new file `pipeline/carla_bridge.py` — new, not one of the untouched files) that the sensor callback writes into and the MATLAB block reads via `py.*` each tick; keep this buffering logic in Python rather than reimplementing it in MATLAB.

Checkpoint: with a running CARLA server + one spawned vehicle/sensors (bare town, no scenario yet), confirm the control block visibly moves the vehicle and the full loop runs 60s of sim time with no MATLAB/Python exception. Re-measure `total_ms` — real sensor payloads may be materially heavier than Phase 1's synthetic numbers.

## Phase 4 — Integrate with the environment teammate's scenes
Swap `carla_control_block.m`'s placeholder "first vehicle found" actor lookup for the environment teammate's actual spawn/role-name convention (e.g. filter by `role_name == 'ego'`). Checkpoint: run against one real authored scene (market-area or unsignaled-intersection) without manual actor-handle fixes.

## Phase 5 — Closed-loop validation and metrics
Layer, don't jump to full-scenario immediately:
1. Empty straight road — waypoint tracking without oscillation.
2. Single static obstacle — `OBSTACLE_DETECTED → REPLAN` fires, path recomputes.
3. Animal crossing (mirrors `run_demo.py`'s synthetic scenario) — `ANIMAL_ON_ROAD` fires distinctly, resumes correctly via history junction.
4. Injected fault (kill/stall Python process, malformed sensor frame) — `SAFETY_SUPERVISOR` catches it, forces brake, no Simulink crash.
5. Only after 1-4 pass: full scripted scenario run, capturing `total_ms` distribution against the 150-200ms ceiling, Stateflow transition log (for the technical report), collision/off-road events.

## Critical files
- `pipeline/types.py`, `pipeline/pipeline.py` — the one additive `PlannedPath`/`tick()` change (Phase 2.1)
- `pipeline/carla_bridge.py` — new, Phase 3 only
- `matlab/init_pyenv.m`, `matlab/test_pipeline_call.m`, `matlab/pipeline_wrapper.m`, `matlab/class_id_map.m`, `matlab/build_stateflow_chart.m`, `matlab/pure_pursuit_controller.m`, `matlab/carla_control_block.m`
- `models/plan_a_isolation_test.slx` (Phase 1), `models/plan_a_full.slx` (Phase 2+)
- `requirements.txt` — add missing `filterpy`/`scipy` pins

## Verification summary
Each phase has its own fail-fast checkpoint (above) before proceeding to the next — this mirrors `two-path-strategy.md`'s "test the riskiest unknown first" instruction rather than building the full system and debugging it all at once. The single most important gate is Phase 1: if the `py.*` → custom Python pipeline call doesn't work reliably and within latency budget, nothing downstream is worth building yet.
