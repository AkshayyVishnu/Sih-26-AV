# `framework/`

A scenario/autopilot runner built **alongside** `Simulation Files/
scenario_1.py` and `scenario_2.py` — not in place of them. Those two
scripts are untouched and still work exactly as before; this is a
parallel, independent way to run the same ideas with reusable, swappable
pieces, so adding scenario 3 (or a second driving algorithm) doesn't mean
copy-pasting a few hundred lines of CARLA boilerplate again.

## The two-axis mental model

- **`Scenario`** = what happens in the world (what actors get spawned,
  what scripted events happen mid-run). It never makes driving decisions.
- **`Autopilot`** = what drives the ego (given sensor data, returns a
  control command). It never spawns world actors or scripts events.

Any `Scenario` can be run with any `Autopilot` — that's the whole point.
Everything mechanical in between (CARLA connection, sync mode, spawning
the ego + its sensors, the tick loop, the debug dashboard, cleanup) is
handled once, by `ScenarioRunner`, in `base.py`.

## Running something

```bash
python framework/run_scenario.py pedestrian_jumpout
python framework/run_scenario.py traffic_stress
python framework/run_scenario.py pedestrian_jumpout --no-viz   # skip the pygame dashboard
```

Requires a running CARLA server (`~/CARLA_0.9.16/CarlaUE4.sh`) first,
same as the original scenario scripts.

## Adding a new scenario or autopilot

See **[`DESIGN_GUIDELINES.md`](./DESIGN_GUIDELINES.md)** — it's written
so either a human or an AI agent can extend this without reading
`base.py`'s implementation first.
