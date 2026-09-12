# `fsi-R1` Change Report — Minimal Predictor/Planner Hardening Port

- **Branch**: `fsi-R1` (from `framework-summit-integration` @ `ad64845`)
- **Commit**: `7d00893` — "Port minimal predictor/planner hardening onto framework pipeline"
- **Scope decision**: minimal port only (per team vote). Per-class inflation radii,
  shortcut+smooth pass, goal-window expansion, and replan hysteresis stay deferred
  — each needs its own live smoke test. The `force_replan` bypass from
  `DecisionLogic` is preserved byte-for-byte.
- **Out of scope (already owned by this branch, verified present)**: decision
  logic (`pipeline/decision_logic.py`), control (`PurePursuitController`),
  CARLA runner (`ScenarioRunner`/`TickContext`/`PipelineAutopilot`), metrics
  (`MetricsRecorder`), the 5 PS scenarios, and the GT perception layer
  (`GroundTruthDetector` → fusion → tracker).

## Changes by file

**`pipeline/predictor.py`**
- Short-history guard (`MIN_HISTORY_FOR_MULTIMODAL = 3`): tracks with <3 history
  points emit straight-only. A fresh GT track has 1 history point and an
  unconverged Kalman velocity — lateral modes off that heading inflated phantom
  cost into the planner. `GroundTruthDetector` spawns such tracks routinely the
  moment actors enter sensor range.
- Class map: added `bus`, `truck`, `pushcart`, `autorickshaw` (test images contain
  a bus; all four previously fell through to the 0.5 default).
- Mode weights stay fixed `0.6 / 0.2 / 0.2` — confidence-agnostic by design,
  since detection confidence varies per class and viewing angle. Robustness to
  low-confidence flicker comes from the tracker's coast logic, not here.

**`pipeline/pipeline.py`**
- `Pipeline(..., prediction_horizon_steps=...)` override; default-identical, so
  `PipelineAutopilot.setup()` and every existing call site behave exactly as
  before. Lets the server side tune horizon against its tick `dt` later.

**`pipeline/planner.py`**
- `decimate_waypoints()` now **interpolates** along long legs to ~1.5m spacing
  (configurable via `Planner(waypoint_spacing_m=)`, `None` restores raw cell
  centers). A filter-only version collapsed a 100m straight leg to 2 waypoints,
  leaving pure-pursuit's lookahead walk with nothing between ego and horizon.
- Hold-last-path fallback: A* returning `None` with a prior path now returns the
  last good path (`replanned=False`, "holding") instead of `[]`; with no history
  it still returns `is_valid=False`. With soft costs a true `None` is rare by
  design — this fires only on real blockage, not sampling noise.

**`docs/pipeline-decision-log.md`** — §18 records rationale + deferred items.

## Verification (all on the committed tree, this machine, no server)

- Headless imports: **8/9 OK** — all 5 `ps_scenarios`, `pedestrian_jumpout`,
  `chaotic_traffic`, `PipelineAutopilot` import and construct cleanly.
- Unit checks pass: 1-pt track → 1 mode; 5-pt bus track → 3 modes; far-goal plan
  61 points with max gap exactly 1.5m; `force_replan` path intact.
- `run_demo.py`: **40/40 ticks valid**, mean ~14ms, max ~63ms — well under the
  ~150ms closed-loop collapse threshold from `docs/architecture.md`.

## Known issues (pre-existing, not from this port)

- `framework/scenarios/traffic_stress.py` fails to import in this machine's
  `.venv` (`from carla.command import ...` → `No module named
  'carla.libcarla.command'`; `import carla` itself works). The file is untouched
  by this commit — broken wheel metadata in the local env, no code fix needed
  here. Re-check the import on the server env as a wheel-health signal.
- Version skew: this `.venv` has `carla==0.9.15`, but `requirements.txt` pins
  `0.9.16` and the documented server build is 0.9.16. CARLA requires exact
  client/server match — the server side must run its 0.9.16 env (or the `.whl`
  from the install's `PythonAPI/carla/dist/`, see `requirements.txt`).

## Next (server side, in order)

1. Pull `fsi-R1`, re-run this report's verification suite verbatim.
2. Smoke `highway_merge_slow_traffic` (coordinate-free), then
   `urban_intersection_no_signals` (verified spawn) — SIGINT-kill pattern,
   leftover-actor assert per `DESIGN_GUIDELINES.md` §6.
3. Coordinate capture for Town07/Town10HD placeholders (village, market, cattle).
4. Full scenario matrix → metrics/video/report. Thaw deferred planner items only
   on live evidence.
