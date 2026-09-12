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
