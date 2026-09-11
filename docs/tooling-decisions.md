# Tooling Decisions — SIH 2026, PS 26037

> **Status note:** this document predates confirmation of the NIT
> Warangal license's actual toolbox coverage. That's since been
> confirmed: RoadRunner specifically is **not** included, but nearly
> the entire rest of the MATLAB/Simulink catalog is. That opened two
> RoadRunner-free, MATLAB-native architectures — see
> `docs/tradeoffs-a-vs-b.md` for the live comparison between them, and
> `requirements.txt` for the current dependency manifest. The
> RoadRunner-fallback research below (Blender Driving Scenario Creator,
> SUMO, Driving Scenario Designer, OSM import) remains valid background
> for whichever path's scene-authoring step ends up needing it.

## Problem statement summary

**PS 26037 — "Adaptive Path Planning and Collision Avoidance for Autonomous
Vehicles on Unstructured Indian Roads"** (MathWorks, Robotics and Drones
theme, Software category). Teams must design and simulate an adaptive path
planning system for an autonomous vehicle operating in unstructured Indian
road conditions — perceiving mixed traffic (cars, auto-rickshaws, pushcarts,
pedestrians, animals) via a multi-sensor setup, predicting short-term agent
motion, and generating a real-time replanned, collision-free path. The
solution must be validated across at least five scenarios (unmarked village
road, unsignaled urban intersection, highway merge, dense market area,
sudden cattle-crossing) and deliver: (1) a working perception → prediction →
planning → decision → motion pipeline, (2) at least two detailed authored
scenes (village road, urban intersection) used to test all five scenarios,
and (3) results with metrics (replanning latency, path smoothness, scenario
completion rate), a technical report, and a demo video. The PS explicitly
suggests MathWorks tools (RoadRunner, Automated Driving Toolbox, Navigation
Toolbox, Stateflow, Vehicle Dynamics Blockset, Deep Learning Toolbox) but
does not mandate them.

## Options considered, and why each was kept or omitted

**Full MATLAB/Simulink + RoadRunner (the PS's suggested default)** — kept
*partially*. RoadRunner is retained for scene authoring of the two required
detailed scenes, contingent on license access. The full Simulink simulation
pipeline (perception/prediction/planning/decision/motion all in
MATLAB/Simulink) was omitted in favor of a Python/CARLA pipeline, for three
reasons: (a) team flexibility and familiarity with the Python/ML ecosystem,
(b) zero recurring cost versus MATLAB toolbox licensing, and (c) RoadRunner
entitlement specifically is not guaranteed to be enabled in time even with a
campus license (see Constraints).

**CARLA default Towns alone** — omitted as the sole road source. CARLA's
built-in maps (Town01–Town15) are clean, Western-style structured roads with
formal lane markings — they do not represent "unstructured Indian roads" and
would need to be substantially reworked regardless.

**OSM import into CARLA** — kept as **fallback tier 2**. Real Indian road
layouts (village roads, market streets, specific intersections) can be
pulled from OpenStreetMap and converted to OpenDRIVE via CARLA's `osm2odr`
tool, for free. Limitation: geometry only — no lane-marking/pothole/terrain
texture control, no clutter placement.

### Fallback tier list — if the RoadRunner license does not come through in time

Ranked by how close a substitute each is for RoadRunner's actual job (3D
road/scene authoring producing OpenDRIVE + FBX, consumed by CARLA):

1. **Blender Driving Scenario Creator** (free, open-source Blender add-on —
   `johschmitz/blender-driving-scenario-creator`) — chosen as the **primary
   fallback** for the two required detailed scenes. Exports OpenDRIVE (.xodr)
   + FBX geometry (the same file pair RoadRunner produces) and OpenSCENARIO
   (covering RoadRunner Scenario's trigger-authoring role too), with esmini
   preview and native CARLA compatibility.
   - *Pros*: real 3D terrain/road-mesh authoring, proper junction curve
     tools, free, closest single-tool replacement for RoadRunner.
   - *Cons*: smaller community than RoadRunner (fewer tutorials), no
     Indian-specific asset library (same limitation as every option here),
     requires someone on the team with Blender modeling skill.

2. **SUMO (netedit + netconvert) with CARLA co-simulation** — kept as a
   **complementary** choice, not a scene-authoring substitute. SUMO's
   network editor authors road topology (irregular junctions, lane counts)
   and converts to OpenDRIVE via `netconvert`; CARLA's official SUMO bridge
   lets SUMO drive background traffic microscopically while CARLA renders
   and runs the ego vehicle. Its real value here is traffic *behavior* —
   tunable aggressive lane-changing, following distance, and speed variance
   approximate Indian mixed-traffic disorder better than CARLA's default
   Traffic Manager can out of the box.
   - *Limitation*: purely topological — no terrain, elevation, or texture —
     must be paired with another tool for the visual scene itself.

3. **MATLAB Driving Scenario Designer** (ships with **Automated Driving
   Toolbox** — a separate, more commonly-covered license bucket than
   RoadRunner) — kept as a **secondary option**, only useful if some MATLAB
   integration is wanted. A 2D top-down scenario app: draw roads, export
   OpenDRIVE 1.4/1.5/1.6 + OpenSCENARIO 1.0, generate MATLAB/Simulink code
   directly.
   - *Omitted as primary*: it is a functional/abstract scenario tool
     (cuboid actors, centerline + width road definitions) built for testing
     things like ACC/lane-keep — not a 3D visual authoring tool. No
     terrain, texture, or clutter placement, so it cannot satisfy the PS's
     "detailed scene" language on its own.

4. **OSM import** — see above (fastest, real layouts, geometry-only).

**Cross-cutting note**: none of the fallback tools — nor RoadRunner itself —
ship Indian-specific assets (auto-rickshaw, pushcart, cattle, market
stalls). Manual asset dressing directly in CARLA/Unreal is required
regardless of which authoring tool is chosen.

**Other open-source AV simulators** (LGSVL — discontinued 2022; AirSim —
drone-oriented, less actively maintained for ground-vehicle driving) —
omitted. CARLA has the larger active community, a mature Python API, a
built-in Traffic Manager, and an official ROS2 bridge, all directly relevant
to this PS.

**Cloud GPU choices**:
- *Kaggle* — omitted for running CARLA itself: notebooks run sandboxed
  without Docker/privileged container access, which the standard headless
  CARLA deployment relies on. Kept only for offline perception-model
  training (free GPU-hours, no simulator needed there).
- *Google Colab* — kept as **secondary/dev-only**: workable for light
  CARLA testing via Xvfb+VNC community setups, but session limits (~12h)
  and no persistent storage make it unsuitable for a stable long run.
- *Google Cloud free trial* — omitted as primary: GPUs are explicitly
  blocked on a non-billable free-trial account; GPU quota can only be
  requested after upgrading to a paid billing account (the $300 credit
  still applies afterward).
- *RunPod / Vast.ai* — chosen as the **primary paid-but-minimal-cost
  route**: instant spin-up, no quota-approval wait, ~$0.27–0.40/hr for a
  24GB card (A5000/L4/4090-class), well-suited to a hackathon timeline.

**Remote viewing**:
- Plain sensor-to-video recording (CARLA camera sensor → mp4) — chosen as
  the **default workflow** for day-to-day dev and for producing the
  required demo video; works everywhere, zero extra setup.
- noVNC — kept as a debug-only fallback for occasional live peeks.
- Sunshine (host) + Moonlight (client) — chosen for interactive live
  viewing/demos, using GPU hardware encoding for low-latency streaming.

## Final decision

**CARLA** as the simulation engine (free, open-source, Python API) +
**RoadRunner** for the two required detailed scenes' road/scene authoring
*if* the NIT Warangal campus license clears RoadRunner's entitlement in
time, with the **Blender Driving Scenario Creator → SUMO → OSM import**
fallback tier used in that order otherwise. The Python stack in
`requirements.txt` stands in for Automated Driving Toolbox, Navigation
Toolbox, Deep Learning Toolbox, and Stateflow; CARLA's built-in PhysX-based
vehicle physics stands in for Vehicle Dynamics Blockset/Simulink bicycle
model.

## Prerequisites

- MathWorks/NIT Warangal campus license sign-in via the TAH portal
  (`mathworks.com/academia/tah-portal/national-institute-of-technology-warangal-31567629.html`)
  + RoadRunner entitlement specifically enabled by NITW's license
  administrator (a separate, manual step beyond just having the campus
  license).
- CARLA server installed, version matching the pinned client in
  `requirements.txt` exactly.
- An NVIDIA GPU with 6GB+ VRAM — local, or rented (RunPod/Vast.ai, or GCP
  after upgrading to a paid billing account with GPU quota approved).
- Python 3.8–3.10 environment.
- Docker, for headless CARLA deployment on a rented GPU box (optional but
  recommended for reproducibility).
- A RunPod or Vast.ai account with a payment method attached, or a Google
  Cloud account with billing upgraded to paid and GPU quota approved.
- Access to the Indian Driving Dataset (IDD) — linked directly in the
  official PS — for training/fine-tuning the perception model.
- Sunshine (host) + Moonlight (client) installed, only if doing live
  interactive viewing/demos rather than recorded video.
- If pursuing a fallback scene-authoring tool: Blender (with the
  `blender-driving-scenario-creator` add-on) and/or SUMO installed.

## Constraints

- CARLA is GPU-heavy at runtime; free notebook platforms (Kaggle, Colab)
  are unsuitable for running the simulator itself and useful only for
  offline model training.
- There are documented, unresolved community bugs combining custom
  imported maps with CARLA's ScenarioRunner (trigger framework sometimes
  fails to recognize a custom-imported world). Mitigation: rely on
  hand-rolled Python waypoint-trigger logic via the raw CARLA API as the
  primary, reliable path rather than depending on ScenarioRunner + a
  custom map together.
- Google Cloud's free trial explicitly blocks GPU access until the billing
  account is upgraded to paid — budget this step in early if using GCP.
- RoadRunner access depends on NITW's license administrator manually
  enabling the entitlement for the team — a timeline risk outside the
  team's direct control; don't block development on it (see fallback tier
  above).
- The CARLA Python client version must exactly match the CARLA server
  version — a version-drift risk to actively guard against when updating
  `requirements.txt` or the server install.
- No built-in CARLA actor assets exist for auto-rickshaws, pushcarts, or
  cattle — custom or re-skinned substitute 3D assets are required no matter
  which scene-authoring tool is used.
- Idea submission deadline: **20 September 2026** (per official PS
  metadata, as of the date this document was written).
