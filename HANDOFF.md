# Session handoff — 2026-09-12

Written so a different agent (or a human) with **zero prior context**
can pick this project up cold. Read this file first; it points to
everything else. **This repo is not a git repository** (`.gitignore`
exists but `git init` was never run) — there is no commit history to
fall back on, so this document is the only record of what changed and
why.

---

## 1. What this project is

SIH 2026, PS 26037 — an autonomous-driving pipeline for unstructured
Indian roads, simulated in CARLA. Before this session, the repo held
three pieces that **did not talk to each other**:

- `docs/` — research/planning docs for an original MATLAB/Simulink +
  Stateflow + CARLA architecture ("Path A", never built) vs. a
  pure-Python fallback ("Path B", what actually exists). Read
  `docs/pipeline-decision-log.md` and `docs/two-path-strategy.md` for
  that history if you need it — not required for anything below.
- `pipeline/` — the real AV stack (perception fusion → tracking →
  prediction → drivable-area → decision logic → planning → control),
  100% Python, **zero CARLA dependency**. Verified only against
  synthetic data (`run_demo.py`) before this session.
- `Simulation Files/scenario_1.py` / `scenario_2.py` — CARLA scene-setup
  scripts (spawn a world + actors + scripted events), written
  independently, with **no calls into `pipeline/`** before this session.

**This session's work, across several turns, connected all three and
then built a modular framework on top.** Details below, in the order it
happened.

---

## 2. Environment (unchanged by this session, but load-bearing)

- CARLA 0.9.16 is installed **locally** at `~/CARLA_0.9.16/` — start it
  with `~/CARLA_0.9.16/CarlaUE4.sh`. (The docs assumed a remote "friend's
  instance" — that assumption is outdated; ignore it.)
- Python environment: conda env `carla_env`
  (`/home/quad-world/miniconda3/envs/carla_env/bin/python`, Python
  3.12.13). Has `carla==0.9.16`, `numpy`, `scipy`, `filterpy`, `pygame`,
  `matplotlib`, `Pillow`. Does **not** have `ultralytics`/`torch` — no
  real YOLO checkpoint exists anywhere on this machine (see §5).
- System `python3` (3.10.12) does **not** have `carla` importable — always
  use the `carla_env` interpreter for anything touching CARLA:
  `/home/quad-world/miniconda3/envs/carla_env/bin/python`, or
  `conda run -n carla_env python ...` (see §7's caveat about `conda run`
  and signal handling before using that form for a live run).
- `DISPLAY=:1`, X server reachable — the pygame dashboard (§4) needs
  this; pass `DISPLAY=:1` explicitly if running from a context where the
  env var isn't already set (e.g. a background shell).

---

## 3. Part 1 — wiring the scenarios to `pipeline/` (`carla_runtime.py`, sensor attachment)

**Goal**: make `scenario_1.py`/`scenario_2.py` actually drive the ego
through the real `pipeline/` stack, instead of the two being disconnected.

- Added sensor attachment (RGB camera, LiDAR, semantic segmentation
  camera, all 800×600 @ 90° FOV, co-located) to both scenario scripts'
  ego vehicles.
- **New file `carla_runtime.py`** (repo root) — the CARLA-glue boundary
  module. `pipeline/` itself stays CARLA-free by design (so
  `run_demo.py` keeps working with zero CARLA dependency); this module
  is where CARLA-specific code lives, shared by `run_live.py` and (later)
  `framework/`. Contains: `build_camera_intrinsic()`,
  `build_camera_to_lidar_extrinsic()`, `carla_image_to_rgb_array()`,
  `carla_lidar_to_xyz()`, `carla_segmentation_to_tags()`,
  `spawn_ego_sensors()`, `build_ego_state()`, `compute_local_goal()`, and
  **`GroundTruthDetector`**.
- **Perception decision**: no YOLO checkpoint exists, so
  `GroundTruthDetector` stands in for it — it projects CARLA's own
  ground-truth actor bounding boxes into the camera image (via
  `carla.BoundingBox.get_world_vertices()` + the camera's live
  transform), producing the same `Detection` shape a real detector
  would. Same disclosed-shortcut pattern `pipeline/drivable_area.py`
  already uses for segmentation. **This is the biggest thing to swap out
  once a real fine-tuned YOLO model exists** — see §5.
- **Real bug found and fixed**: the camera↔LiDAR extrinsic matrix in
  `run_live.py`/`run_demo.py` had a sign error (`X_optical = -Y_local`
  instead of the correct `X_optical = +Y_local` — confirmed against
  CARLA's own shipped `PythonAPI/examples/bounding_boxes.py` and
  `lidar_to_camera.py`, and a live `get_right_vector()` check). Fixed in
  both files; `carla_runtime.build_camera_to_lidar_extrinsic()` is now
  the single source of truth.
- **`compute_local_goal()`** — required, not cosmetic:
  `pipeline/planner.py`'s costmap is a 60×60m window **re-centered on the
  ego every tick**. Both scenarios' real goals are 90-140m from spawn, so
  passing them directly would make the planner silently fail every tick.
  This clamps the vector from the ego to the real goal to 25m, giving a
  receding-horizon local goal that's always in range.
- Added `FINAL_GOAL_X`/`FINAL_GOAL_Y`/`VEHICLE_WHEELBASE_M` config to
  both scenario scripts, and wired their tick loops to call
  `pipeline.tick()` for real.
- `requirements.txt`: added `scipy`, `filterpy` (were missing despite
  `pipeline/tracker.py` needing both), `pygame` (needed by `viz.py`,
  §4). `filterpy` was installed into `carla_env`.

Files touched: new `carla_runtime.py`; modified `run_live.py` (dedupe
against it), `run_demo.py` (extrinsic fix only), `Simulation
Files/scenario_1.py`/`scenario_2.py`, `requirements.txt`.

---

## 4. Part 2 — live debug dashboard (`viz.py`)

**Goal**: see what the pipeline sees and what it outputs, live, instead
of only from log files. Motivated directly by a real finding: the first
live test of `scenario_2.py` showed the pedestrian-jumpout event never
escalated past `NORMAL_DRIVE`, because `GroundTruthDetector` was
flickering in/out of frame tick-to-tick and there was no way to *see*
that happening.

- **New file `viz.py`** (repo root) — pure `pygame` + `numpy`, **no
  `carla` import** (same boundary as `pipeline/`). One tiled window
  (`Dashboard` class), 2×2 grid: RGB feed + detection boxes drawn live /
  colorized segmentation / LiDAR top-down view + the planner's path
  overlaid / a scrolling throttle-steer-brake-vs-time graph
  (`ControlGraphPanel`, hand-drawn with pygame primitives, no
  matplotlib — avoids a second GUI backend/thread in the same process).
  Chose pygame over OpenCV specifically because pygame was already
  importable in `carla_env` and matches CARLA's own
  `PythonAPI/examples/visualize_multiple_sensors.py` pattern; **this was
  an explicit user choice** (one tiled window, not separate OS windows).
- Wired into both scenario scripts via `ENABLE_VISUALIZATION`/
  `VIZ_UPDATE_EVERY_N_TICKS` config flags. Closing the window (X/Esc/Q)
  disables the dashboard for the rest of the run without stopping the
  simulation.
- **Verified live**: negligible overhead (~1.3-1.6ms mean tick latency
  with the dashboard on, vs. ~1.3ms off — nowhere near the 150-200ms
  budget).

Files touched: new `viz.py`; modified `Simulation
Files/scenario_1.py`/`scenario_2.py` (dashboard wiring), `requirements.txt`.

---

## 5. Part 3 — `framework/`: modular Scenario/Autopilot framework

**Goal, in the user's own words**: "propose a better architecture which
would easily allow me to be modular, simple and allow swapping of
different autopilot modes for the scenarios and also a framework or a
guide by which to design different scenarios... lots of the scenarios
will have the same boilerplate code so how to handle that?"

**Explicit constraint the user gave**: build it as a self-contained
addition in a **new directory**, **do not modify any existing file**.
`Simulation Files/scenario_1.py`/`scenario_2.py` (from §3/§4) are
untouched and still work exactly as they did — `framework/` is a
parallel, independent way to run the same ideas, not a replacement.

**START HERE for the actual design**: **`framework/README.md`** (quick
start) and **`framework/DESIGN_GUIDELINES.md`** (the full extension
guide — read this before adding a scenario or an autopilot; it's written
to be self-sufficient, don't re-derive the design from `base.py`).

Short version of the architecture: `Scenario` (what happens in the
world — actors, scripted events) and `Autopilot` (what drives the ego —
sensor data in, `ControlCommand` out) are independent, swappable axes.
`TickContext` is built once per tick and shared by both, specifically to
fix a measured problem: the pre-`framework/` code was fetching the same
CARLA actor's transform 2-3× per tick from different call sites.
`ScenarioRunner` (in `framework/base.py`) is the boilerplate — CARLA
connect, sync mode, ego+sensor spawn, the tick loop, dashboard, cleanup —
written once, shared by every scenario/autopilot combination.

```
framework/
    __init__.py                    sys.path bootstrap
    base.py                        TickContext, Scenario, Autopilot, ScenarioRunner
    autopilots/pipeline_autopilot.py   PipelineAutopilot -- wraps pipeline/ + GroundTruthDetector (today's exact behavior)
    scenarios/pedestrian_jumpout.py    migrated from scenario_2.py
    scenarios/traffic_stress.py        migrated from scenario_1.py
    run_scenario.py                CLI: python framework/run_scenario.py <scenario> [--autopilot X] [--no-viz]
    README.md
    DESIGN_GUIDELINES.md           <- read before extending
```

**Real bug found and fixed during live verification**: `ScenarioRunner`
originally never destroyed the ego vehicle itself in cleanup (only its
sensors and the scenario's tracked actors) — a leftover actor was caught
after the first live test, fixed in `framework/base.py`'s `finally`
block, re-verified clean on the next run.

Files touched: everything new, entirely under `framework/`. **Nothing
outside `framework/` was touched in this part** — confirmed by re-running
the original `scenario_2.py` afterward and seeing identical behavior.

---

## 6. Current verified state (as of end of this session)

All of the following were run live against a local CARLA 0.9.16 server
and confirmed working, with **zero leftover actors** in the world after
each run (checked via a fresh client query, not assumed):

- `python run_demo.py` (synthetic, zero CARLA) — clean, ~10ms/tick.
- `python "Simulation Files/scenario_2.py"` — clean, pedestrian trigger
  fires, dashboard opens, ~1.1-1.6ms/tick.
- `python "Simulation Files/scenario_1.py"` — not individually
  re-verified in the final pass, but uses identical wiring to
  scenario_2.py (added in the same turn) — should work the same way.
- `python framework/run_scenario.py pedestrian_jumpout` — clean, 1074
  ticks, trigger fires, tracking noticeably more stable than the
  original script's (sustained 1-2 tracked objects most ticks, not
  mostly-zero), ~5.3ms/tick mean.
- `python framework/run_scenario.py traffic_stress` — clean, spawned
  265 vehicles + 3 walkers (requested 300/10 — spawn-point availability
  in Town03, not a bug), 373 ticks, ~5.5ms/tick mean, traffic lights
  correctly restored on shutdown.

**Not yet done**: no live run of `framework/run_scenario.py
traffic_stress` was cross-checked against the *original*
`scenario_1.py` side-by-side (only `scenario_2.py` got that final
side-by-side check) — low risk since both scenario files share the exact
same wiring pattern, but worth a quick confirmation if something seems
off with `scenario_1.py` specifically.

---

## 7. Known issues / open items (not fixed, disclosed on purpose)

- **No real YOLO model.** `GroundTruthDetector` (§3) is the perception
  source everywhere right now. Swapping in a real model later: extend
  `pipeline/predictor.py`-style (there's already a `MoFlowPredictor`
  stub there that was never wired to a checkpoint, same idea) or write a
  new `Autopilot` in `framework/autopilots/` that uses YOLO instead of
  `GroundTruthDetector` — the `Autopilot` interface doesn't care how
  `compute()` gets its detections.
- **Detection flicker at the camera's FOV edge** — `GroundTruthDetector`
  can genuinely see an object (proven: it produced an accurate tracked
  position matching the real pedestrian's spawn location), but its
  projection flickers in/out of frame as the ego's heading shifts
  tick-to-tick, which can prevent `decision_logic`'s 3-tick debounce for
  `OBSTACLE_DETECTED`/`EMERGENCY_BRAKE` from ever firing. **The user
  explicitly said "stop here for now"** on fixing this — it's a known,
  disclosed limitation, not forgotten. (Note: the `framework/` version's
  live test showed *more* stable tracking than the original script's
  baseline test — possibly incidental to this particular run/map state,
  not confirmed as a fix. Don't assume the flicker issue is resolved
  without re-testing specifically for it.)
- **`GroundTruthDetector.detect()` still does 2 separate
  `world.get_actors().filter(...)` RPC round-trips per tick** internally
  — `TickContext` (§5) fixed the *ego*-transform redundancy but not this
  one. Documented as a known, smaller, not-yet-addressed optimization in
  `framework/DESIGN_GUIDELINES.md`.
- **No animal/livestock CARLA blueprint exists** in this install — the
  `ANIMAL_ON_ROAD` branch in `pipeline/decision_logic.py` can't be
  exercised with a real actor class.
- **`framework/scenarios/traffic_stress.py` drops most of the original
  `scenario_1.py`'s CLI surface** (`--safe`/`--hybrid`/`--respawn`/
  `--car-lights-on`/`--hero`/`--no-rendering`/`--asynch`/etc.) —
  intentional simplification, disclosed in that file's own docstring.
  Easy to reintroduce a specific flag as a constructor kwarg if one turns
  out to matter.
- **This repo is not a git repository.** No commit history exists for
  any of this — this file and the code itself are the only record.

---

## 8. How to run everything

```bash
# 1. Start CARLA (separate terminal)
~/CARLA_0.9.16/CarlaUE4.sh

# 2. Everything below uses the carla_env conda interpreter
export DISPLAY=:1   # if not already set, needed for the pygame dashboard

# Synthetic-only, no CARLA needed:
/home/quad-world/miniconda3/envs/carla_env/bin/python run_demo.py

# Original scenario scripts (pipeline-integrated + dashboard):
cd "Simulation Files"
/home/quad-world/miniconda3/envs/carla_env/bin/python scenario_1.py
/home/quad-world/miniconda3/envs/carla_env/bin/python scenario_2.py
cd ..

# New modular framework:
/home/quad-world/miniconda3/envs/carla_env/bin/python framework/run_scenario.py pedestrian_jumpout
/home/quad-world/miniconda3/envs/carla_env/bin/python framework/run_scenario.py traffic_stress
/home/quad-world/miniconda3/envs/carla_env/bin/python framework/run_scenario.py pedestrian_jumpout --no-viz

# Generic fallback (attaches to whatever vehicle exists in the world;
# needs YOLO_MODEL_PATH filled in at the top of the file -- not usable
# without a real checkpoint):
/home/quad-world/miniconda3/envs/carla_env/bin/python run_live.py
```

**Caveat learned the hard way, worth repeating**: don't background a run
via `timeout ... conda run -n carla_env python ...` — this combination
was found to leave an **orphaned process still ticking the CARLA world**
in the background, silently corrupting a later "clean" run (it looked
like flaky detections; it was actually two processes racing to tick the
same world). If you need a bounded/scripted run, capture the PID
directly and send a real `SIGINT`:
```bash
/home/quad-world/miniconda3/envs/carla_env/bin/python -u <script> &
PY_PID=$!
sleep 30
kill -INT "$PY_PID"
wait "$PY_PID"
```
Then always verify zero leftover actors afterward (fresh client, filter
`vehicle.*`/`walker.*`/`sensor.*`) before trusting the next run's data —
`framework/DESIGN_GUIDELINES.md` §6 has this codified as a checklist.

---

## 9. Where to look for more detail

- `framework/DESIGN_GUIDELINES.md` — how to add a new `Scenario` or
  `Autopilot`, the `TickContext` field reference, verification checklist.
- `docs/pipeline-decision-log.md` — the original team's own engineering
  decisions for `pipeline/` (predates this session).
- `SETUP.md` / `COMMANDS.md` — original setup instructions (predate this
  session; mostly still accurate, written for a remote-CARLA assumption
  that's now outdated per §2).
- This file (`HANDOFF.md`) — supersedes assumptions in the above two
  where they conflict (e.g. "no local CARLA access").
