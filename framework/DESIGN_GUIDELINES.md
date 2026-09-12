# Design guidelines: extending `framework/`

Written for either a human or an AI agent to read **before** adding a new
`Scenario` or `Autopilot`, so it can be done correctly without first
reading `base.py`'s implementation. If you're about to write either, read
the relevant section below, then look at the real worked example named
in it.

---

## 1. The contract, in one picture

```
ScenarioRunner (base.py -- written once, don't modify per-scenario)
    │
    ├── owns and drives:  Scenario           (what happens in the world)
    │                     Autopilot          (what drives the ego)
    │
    └── every tick, builds ONE TickContext and hands it to both:

            world.tick()
                 │
                 ▼
            TickContext.gather()  -- fetches ego transform/velocity/
            │                        sensors ONCE, shared below
                 │
                 ├──► scenario.on_tick(ctx)      -- scripted events
                 │
                 └──► autopilot.compute(ctx, goal_xy) -- returns ControlCommand
                              │
                              ▼
                     ego_vehicle.apply_control(...)
```

A `Scenario` never touches driving logic. An `Autopilot` never touches
world-building (spawning actors, scripting events). If you find yourself
wanting to do either, that's a sign the thing you're writing should
actually be the other class, or that `TickContext` is missing a field it
should expose instead.

---

## 2. `TickContext` field reference

Every field below is computed **once per tick**, before `on_tick()` or
`compute()` run, and shared by both.

| Field | Type | Notes |
|---|---|---|
| `tick_count` | `int` | Increments every tick, starts at 1 |
| `sim_time_s` | `float` | CARLA's own sim clock (`world.get_snapshot().timestamp.elapsed_seconds`), not a manually accumulated counter |
| `world` | `carla.World` | Escape hatch for anything not worth caching (e.g. querying other actors) |
| `ego_vehicle` | `carla.Vehicle` | The actual ego actor |
| `ego_x`, `ego_y` | `float` | World-frame position, already extracted |
| `ego_yaw` | `float` | Radians |
| `ego_speed` | `float` | m/s, already computed from velocity |
| `rgb_array` | `np.ndarray \| None` | `(H, W, 3)` uint8 RGB, `None` until the camera's first frame arrives |
| `lidar_xyz` | `np.ndarray \| None` | `(N, 3)` float32, ego-local frame, `None` until the first sweep arrives |
| `seg_tags` | `np.ndarray \| None` | `(H, W)` int32 raw CARLA semantic tags, `None` until the first frame arrives |
| `sensors_ready` (property) | `bool` | `True` once all three sensor fields above are non-`None` |

**The one rule that matters most**: if a field you need already exists on
`ctx`, use it — don't call `ego_vehicle.get_transform()` (or similar)
again yourself. That redundant-RPC pattern (the same actor's transform
fetched 2-3 times per tick from different call sites) was a real,
measured problem in the code this framework replaced, and `TickContext`
exists specifically to fix it. It's easy to reintroduce by accident in a
new scenario — the temptation is always "I'll just grab the transform
again real quick."

---

## 3. Adding a `Scenario`

Worked example: `framework/scenarios/pedestrian_jumpout.py` (the
simpler one — read it alongside this section).

1. Subclass `framework.base.Scenario`.
2. Set the required class attributes: `EGO_SPAWN` (a `carla.Transform`)
   and `FINAL_GOAL` (an `(x, y)` world-frame tuple).
3. Set `SPECTATOR_TRANSFORM` if you want a specific camera angle — if
   omitted, the runner points the spectator at `EGO_SPAWN`.
4. Implement `spawn_actors(self, world, bp_lib, client)`: spawn whatever
   makes this scenario unique. **Wrap every spawned actor (or raw actor
   ID from a batch spawn) in `self.track(...)`** — e.g.
   `self.pedestrian = self.track(world.spawn_actor(...))`. This is what
   makes the runner clean it up automatically on shutdown. Forgetting it
   is the #1 way to leak actors between runs — if a second run ever
   fails to spawn because "actor already exists" or similar, this is the
   first thing to check.
   - Runs **before** the ego is spawned. If you need to avoid spawning
     on top of the ego, check against `self.EGO_SPAWN.location` (known
     statically, a plain `Transform`) — not a live ego actor, which
     doesn't exist yet at this point. (See `traffic_stress.py` for a
     real example of this.)
5. Implement `on_tick(self, ctx)` **only if** something needs to happen
   mid-run (a trigger distance, a timed event). Most scenarios won't
   need this at all — the base class default is a no-op.
   - Runs **before** `autopilot.compute()` each tick. So something this
     call changes (e.g. commanding a pedestrian to start moving) is
     reflected in the **next** tick's sensor data, not this one — don't
     expect the autopilot to react to an event in the same tick it fires.
6. Implement `cleanup_extra(self, client)` **only if** something beyond
   "destroy every tracked actor" is needed (e.g. restoring traffic
   lights, stopping AI walker controllers — see `traffic_stress.py`).
   Runs **before** tracked actors are destroyed, so anything it needs to
   act on (e.g. call `.stop()` on) is still alive.

Common pitfalls:
- Forgetting `self.track(...)` on an actor (see step 4).
- Doing something in `spawn_actors()` that depends on the ego already
  existing (it doesn't yet — see step 4's sub-note).
- Fetching an actor's transform more than once per tick when `ctx`
  already has what you need (see §2's rule) — this applies to actors
  *your* scenario tracks too (e.g. a scripted pedestrian's own position
  still needs its own per-tick query since only the ego is cached in
  `ctx`, but don't query it twice if you only need it once).

---

## 4. Adding an `Autopilot`

Worked example: `framework/autopilots/pipeline_autopilot.py` (wraps the
existing `pipeline/` stack — read it alongside this section).

1. Subclass `framework.base.Autopilot`.
2. Implement `setup(self, *, ego_camera, camera_intrinsic,
   camera_to_lidar_extrinsic, image_width, image_height, wheelbase_m,
   dt)` for anything that only needs to happen **once**, before the tick
   loop starts (building a model, a detector, etc.). Default is a no-op
   — skip overriding it if your autopilot doesn't need one-time setup
   (e.g. a simple rule-based controller).
   - **Must stay CARLA-connection-agnostic beyond what's handed to it
     here and through `TickContext` each tick.** Don't reach back into a
     `carla.Client`, don't spawn actors. If you find yourself wanting to,
     that logic belongs in a `Scenario`, not here.
3. Implement `compute(self, ctx, goal_xy) -> ControlCommand` (required).
   Called once per tick, only once `ctx.sensors_ready` is `True`.
   `goal_xy` is already computed for you (clamped to the planner-style
   60m-window assumption most classical planners need — see
   `carla_runtime.compute_local_goal`'s docstring if you're curious why).
4. Implement `debug_info(self) -> dict` **only if** you want the
   dashboard/latency summary to show something. Recognized (all
   optional) keys:
   - `"detections"`: `list[pipeline.types.Detection]`
   - `"planned_waypoints"`: `list[tuple[float, float]]`, **world frame**
   - `"timings"`: anything with a `.total_ms` attribute, or a plain
     float ms value — feeds the runner's end-of-run latency summary
   Default is `{}` — not every autopilot has something visualizable
   here (a black-box model might not produce explicit detections or a
   path at all), so this is deliberately optional and separate from
   `compute()`'s required return value, not a 4th thing `compute()` must
   also compute and return every tick.

---

## 5. Registering it

Add one line to `framework/run_scenario.py`'s `SCENARIOS` or
`AUTOPILOTS` dict, keyed by whatever name you want on the command line:

```python
SCENARIOS = {
    "pedestrian_jumpout": PedestrianJumpOut,
    "traffic_stress": TrafficStress,
    "my_new_scenario": MyNewScenario,   # <- add this
}
```

Nothing else needs to change.

---

## 6. Verification checklist before calling it done

1. **Syntax**: `python3 -c "import ast; ast.parse(open('framework/....py').read())"`.
2. **Headless import**: `conda run -n carla_env python -c "from
   framework.scenarios.my_new_scenario import MyNewScenario"` (or the
   autopilot equivalent) — catches import/reference errors without
   needing CARLA running yet.
3. **Bounded live smoke test** against a running CARLA server. Don't just
   background the process and forget it — use this exact pattern (a
   plain `timeout ... conda run ...` combination was found, the hard
   way, to leave an **orphaned process still ticking the world** in the
   background, silently corrupting a later "clean" run):
   ```bash
   cd /path/to/repo && {
     /path/to/carla_env/bin/python -u framework/run_scenario.py my_new_scenario \
       > /tmp/smoke.log 2>&1 &
     PY_PID=$!
     sleep 30
     kill -INT "$PY_PID"     # NOT plain `timeout` -- see above
     wait "$PY_PID"
   }
   ```
4. **Confirm zero leftover actors** in the world afterward:
   ```python
   world = carla.Client('localhost', 2000).get_world()
   leftover = [a for a in world.get_actors() if a.type_id.startswith(('vehicle.','walker.','sensor.'))]
   assert len(leftover) == 0
   ```
   A non-zero count almost always means a missing `self.track(...)` call
   (§3) or a `cleanup_extra()` that didn't run before an exception.

---

## 7. Explicit non-goals / anti-patterns

- **Don't modify `pipeline/` or `carla_runtime.py`** for something that's
  actually scenario- or autopilot-specific — extend `framework/` instead.
  Both of those stay general-purpose, reused by everything.
- **Don't have a `Scenario` make driving decisions**, and don't have an
  `Autopilot` spawn world actors or script events. That boundary is the
  entire reason this split exists — crossing it quietly defeats the
  point of either class being swappable.
- **Don't skip `self.track()`** on a spawned actor (§3) — it's the only
  thing standing between a clean shutdown and a leaked actor next run.
- **Don't re-fetch something `TickContext` already has** (§2) — it's a
  regression of a real, previously-measured inefficiency, not just a
  style nitpick.

---

## 8. PCLA-backed autopilots (`pcla_tfv6`, `own_perception_plant2`)

Two autopilots wrap [PCLA](https://github.com/MasoudJTehrani/PCLA), a
framework bundling 41 pretrained CARLA driving-agent checkpoints
(TransFuser v3–v6, PlanT/PlanT2, SimLingo, Roach, LAV, and more):

- **`pcla_tfv6`** (`framework/autopilots/pcla_transfuser_autopilot.py`,
  `Transfuserv6Autopilot`) — TransFuser v6 fully end-to-end. PCLA
  attaches and manages its own sensors internally; nothing from
  `pipeline/` or `TickContext`'s camera/LiDAR/segmentation fields is
  used. This is PCLA's own documented, tested use case
  (`external/PCLA/sample.py`), just wrapped behind `Autopilot`.
- **`own_perception_plant2`**
  (`framework/autopilots/own_perception_plant2_autopilot.py`,
  `OwnPerceptionPlanT2Autopilot`) — the SAME perception
  `PipelineAutopilot` uses (`GroundTruthDetector` + `perception_fusion.py`
  + `tracker.py`) feeding PCLA's bundled **PlanT2** agent's planner
  instead of this project's own A* planner. Works by monkey-patching
  `agent_instance.get_bounding_boxes` on the constructed PCLA agent — see
  `pipeline/plant2_adapter.py`'s module docstring for the full
  investigation of PlanT2's actual model interface and this injection
  point's disclosed limitations (class-vocabulary mismatch, no real 3D
  extents, no heading estimation, never tested against a live run).

**Why these two exist side by side**: `pipeline` (existing) and
`own_perception_plant2` share perception and differ only in planner —
the fair comparison for "is our own planner competitive with a learned
one." `pcla_tfv6` is a fully independent reference point, not a
like-for-like rival to either (different planning paradigm entirely —
sensor-based end-to-end vs. object-level).

### Setup required before selecting either

Neither is bundled by cloning this repo — PCLA is a separate,
gitignored external dependency:

```bash
git clone https://github.com/MasoudJTehrani/PCLA external/PCLA
cd external/PCLA
conda env create -f environment.yml     # separate env from carla_env -- see note below
conda activate <name from environment.yml's own name: field>
python download_weights.py              # pulls checkpoints incl. tfv6_regnet + plant2 -- several GB
python download_assets.py
```

**Which Python environment actually runs `framework/run_scenario.py
--autopilot pcla_tfv6`/`own_perception_plant2` matters**: PCLA needs
`py-trees` (pinned `py-trees==0.8.3` in its own `environment.yml`) plus
its own torch/timm pins, none of which are in `carla_env`'s
`requirements.txt`. Either install `py-trees` (and whatever else PCLA's
import chain needs) directly into `carla_env`, or point
`framework/run_scenario.py` at PCLA's own conda env instead — whichever
is less disruptive to `carla_env`'s existing, working state is the
right call to make at the time, not a rule fixed in advance here.

`--autopilot pipeline` needs **none** of the above — `run_scenario.py`
imports autopilots lazily (only the one actually selected), specifically
so PCLA not being set up never blocks the existing, working path.

### The two extension points this needed in `base.py`

Both are additive — every existing `Scenario`/`Autopilot` subclass keeps
working unmodified (verified: `pedestrian_jumpout`/`traffic_stress`
still import and construct cleanly after this change):

1. **`Autopilot.setup()`** gained three new keyword-only params —
   `client`, `ego_vehicle`, `route_xml_path` — all defaulting to `None`.
   PCLA's constructor (`PCLA(agent, vehicle, route, client)`) genuinely
   needs a live `carla.Client` and the ego actor itself, which nothing in
   the original contract exposed (by design — see §7's own rule about an
   `Autopilot` not reaching into a `carla.Client`). This is the one
   deliberate, disclosed exception to that rule, scoped narrowly to what
   PCLA's own constructor requires.
2. **`Autopilot.cleanup()`** (new, optional, default no-op) — called by
   `ScenarioRunner` in its `finally` block, before it destroys
   `ego_sensors`/`ego_vehicle` itself. Needed because `PCLA.cleanup()`
   destroys the ego vehicle and PCLA's own attached sensors internally —
   without this hook, nothing would ever call it. `ScenarioRunner`'s own
   subsequent ego/sensor destroy calls use `client.apply_batch()`
   (fire-and-forget, silent no-op on an already-destroyed actor), so the
   resulting double-destroy is safe, deliberate redundancy — not
   something to special-case away.

Also added: `Scenario.ROUTE_XML_PATH` (optional, default `None`) and
`framework/pcla_route.py`'s `build_route_xml()` — `ScenarioRunner`
auto-generates a minimal 2-waypoint route XML from
`EGO_SPAWN`/`FINAL_GOAL` when a scenario doesn't set one explicitly, so
nobody has to hand-author a leaderboard-format route file per scenario
just to use a PCLA-backed autopilot. Confirmed (not assumed) sufficient
by reading PCLA's own `route_parser.py`/`route_manipulation.py`: only
`x`/`y`/`z` per waypoint are ever read, and CARLA's own
`GlobalRoutePlanner` traces a full legal path between as few as two
coarse points — see that module's docstring for the exact reasoning.

### Chaotic background traffic (`chaotic_traffic` scenario)

`framework/scenarios/chaotic_mixin.py`'s `ChaoticTrafficMixin` wraps
`pipeline/traffic_chaos.py` (dense, aggressively-tuned Traffic
Manager-driven background traffic — the SUMMIT substitute; see
`docs/pipeline-decision-log.md` for why literal SUMMIT was dropped) as a
mixin any `Scenario` can add via `class X(ChaoticTrafficMixin,
Scenario)`. `framework/scenarios/chaotic_traffic.py`'s `ChaoticTraffic`
combines it with `PedestrianJumpOut`'s existing, already-verified hazard
— built specifically so the PCLA-backed autopilots have something closer
to real unregulated-traffic conditions to run against than an
otherwise-empty town. **Not** applied to `traffic_stress.py` — that
scenario already spawns its own background traffic via a different,
already-tested pattern; the two test different things (raw actor count
vs. traffic heterogeneity/aggressiveness) and stacking them would
double-spawn for no benefit.
