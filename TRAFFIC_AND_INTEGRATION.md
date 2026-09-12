# Traffic generation + comparison-run integration

**Read this before touching this branch — it documents a pivot.** The
branch is named `summit-integration`, but SUMMIT itself was investigated
and dropped (see `docs/pipeline-decision-log.md` §11 for the full
reasoning). The short version: SUMMIT is a full CARLA source fork pinned
to CARLA 0.9.8, not a plugin — it can't attach to your running 0.9.16
server, and building it from source is a separate multi-hour Unreal
Engine build, not something to attempt this close to the deadline.
Chaotic/heterogeneous background traffic is generated instead by
`pipeline/traffic_chaos.py`, using CARLA's own Traffic Manager tuned
aggressively. No SUMMIT setup is needed anywhere in this branch.

## What's on this branch

Two comparison scripts, both closed-loop, both logging through the same
`MetricsRecorder`/`metrics_export.py` pipeline as `run_live.py`:

1. **`run_own_perception_plant2.py`** — our own perception
   (`pipeline/perception_fusion.py` + `pipeline/tracker.py`, real YOLO +
   LiDAR fusion + Kalman tracking) feeding PCLA's bundled **PlanT2**
   agent's planner. See `pipeline/plant2_adapter.py`'s module docstring
   for exactly how this works and its disclosed limitations.
2. **`run_pcla_transfuserv6.py`** — PCLA's fully end-to-end
   **TransFuser v6** agent, driving off its own sensors, no involvement
   of our pipeline at all. PCLA's standard, documented use case.

Both spawn the same aggressive background traffic
(`pipeline/traffic_chaos.py`) so the two runs are directly comparable.

**These two are not a strictly fair head-to-head** — PlanT2 is
object-level/planning-only, TransFuser v6 is sensor-based end-to-end.
Treat TransFuser v6 as a reference point, and the real story as
"PlanT2-with-our-perception vs. what PlanT2 would do with ground-truth
perception" if time allows adding that third run later (it's one line:
undo the `get_bounding_boxes` monkey-patch).

## What you need to do before running either script

### 1. Set up PCLA's own environment

PCLA has a large, separate dependency set (py-trees, its own torch/
timm/etc. pins) from this project's `requirements.txt` — don't try to
merge them into one environment.

```bash
cd external/PCLA
conda env create -f environment.yml
conda activate pcla   # or whatever environment.yml names it -- check the file's `name:` field
python download_weights.py   # pulls agent checkpoints, including plant2 and tfv6_regnet, from Zenodo -- this is a real download, several GB, start it before you need to run anything
python download_assets.py
```

Read `external/PCLA/README.md` for the exact current commands — this was
correct as of when this branch was built, but PCLA is a third-party repo
that can change.

### 2. Fill in CONFIG at the top of both scripts

Both scripts have a `CONFIG` section, same pattern as `run_live.py`:

- `MAP_NAME` — **must be identical in both scripts** for the comparison
  to mean anything (same map = same scene geometry).
- `EGO_SPAWN_POINT_INDEX` — same reasoning.
- `YOLO_MODEL_PATH` (own-perception script only) — path to the real
  fine-tuned YOLO checkpoint.
- `ROUTE_XML_PATH` — a real leaderboard-format route XML for `MAP_NAME`.
  `external/PCLA/sample_route.xml` is for a different town and will not
  work as-is; generate a real one for your map (see PCLA's README, or
  build a short XML from a sequence of waypoints on your route).
- `TRAFFIC_SEED` — **must match between the two scripts** for a fair
  comparison of the same traffic scene; vary it across repeated runs of
  the *same* script (per `metrics_export.py`'s ≥3-runs-per-scenario
  guidance already used elsewhere in this project).

### 3. Run

```bash
.venv\Scripts\python.exe run_own_perception_plant2.py
.venv\Scripts\python.exe run_pcla_transfuserv6.py
```

Both write `logs/metrics_<scenario_name>_<run_id>.csv` +
`_summary.json`, same as `run_live.py`. Run `metrics_export.py`
afterward to get the aggregated comparison numbers.

**Read `docs/pipeline-decision-log.md` §13** before interpreting
`total_latency_ms`/`replanning_latency_ms` from these two scripts —
neither runs our actual 7-stage pipeline, so those numbers mean
something different here than in `run_live.py`'s own runs. Don't merge
them into one figure across all three scenario types without accounting
for that.

## If something breaks

- **`get_bounding_boxes` monkey-patch not taking effect / PlanT2 still
  seems to see ground truth**: confirm it's patched on the *instance*
  (`pcla.agent_instance.get_bounding_boxes = ...`) AFTER `PCLA(...)`
  construction, not before — `PCLA.setup_agent()` builds a fresh
  `agent_instance` internally, so patching earlier has nothing to attach
  to.
- **PlanT2 agent errors on first tick about missing route data**: the
  agent's `_waypoint_planner` is built lazily inside `_init()`, which
  only runs on the *first* `run_step()` call. If it errors before that,
  check `ROUTE_XML_PATH` parses correctly for `MAP_NAME` first (PCLA's
  own `setup_route()` — called during `PCLA(...)` construction — should
  already have failed loudly if the route is bad).
- **`pipeline/plant2_adapter.py`'s label_raw output looks wrong** (PlanT2
  driving erratically even though our own perception looks fine in the
  logs): this adapter was written by reading PlanT2's source, never
  tested against a live run (see decision log §12, point 5). First debug
  step: temporarily log both the adapter's output AND
  `data_agent.get_bounding_boxes()`'s real ground-truth output side by
  side for the same tick, and diff them — the likely failure modes are
  the coordinate-frame sign flip (x=forward, y=right vs. our own
  y=left convention) or a missing `type_id` key on a "car"-classed
  entry (see the adapter's module docstring).
- **SUMMIT**: don't try to make it work. If a future need for actual
  SUMMIT-quality traffic comes up, the real fix is asking whoever owns
  the CARLA server to also stand up a *separate* SUMMIT-built simulator
  instance (built from `external/summit` against real UE4 source) and
  bridging scenario data between the two — a multi-day undertaking, not
  something to start under deadline pressure.

## Repo layout added by this branch

```
external/summit/                    # cloned, gitignored, unused (see above) -- harmless to leave or delete
external/PCLA/                      # cloned, gitignored -- set up its own conda env per step 1 above
pipeline/traffic_chaos.py           # SUMMIT substitute: CARLA Traffic Manager, tuned aggressive
pipeline/plant2_adapter.py          # our TrackedObject list -> PlanT2's label_raw format
run_own_perception_plant2.py        # comparison run (i)
run_pcla_transfuserv6.py            # comparison run (ii)
docs/pipeline-decision-log.md       # §11-13: full writeup of the SUMMIT pivot + PlanT2 investigation
```
