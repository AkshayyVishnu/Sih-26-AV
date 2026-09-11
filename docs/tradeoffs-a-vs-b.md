# Path A vs. Path B — Full Tradeoff Comparison

**Decision status: deferred.** This document compares, it does not recommend.
It captures the component-by-component analysis worked through while
evaluating how to build PS 26037's pipeline now that the NIT Warangal
license is confirmed to cover nearly the full MATLAB/Simulink toolbox
catalog, minus RoadRunner specifically.

- **Path A — Full native**: Simulink pipeline + Automated Driving
  Toolbox's own Unreal Engine 3D simulation, custom scenes hand-built in
  the free Unreal Editor.
- **Path B — Hybrid**: the same Simulink pipeline (Stateflow, Navigation
  Toolbox, Deep Learning Toolbox) driving a CARLA-simulated world over
  ROS Toolbox's CARLA ROS bridge, using CARLA's free OSM import for
  Indian road geometry.

Both share an identical "brain." They diverge only in how the "world" is
simulated and connected to it.

## What's common to A and B

| Component | Why it's used | Pros | Cons | Learning curve | Approx. time | Known failure points |
|---|---|---|---|---|---|---|
| **Simulink** (core pipeline) | PS explicitly names it as the required pipeline environment; ties every other block together into one executable model | Visual, modular, easy to swap components; built-in logging/scopes for metrics | Can get slow/heavy with many blocks; debugging signal-type mismatches is fiddly | Low if familiar with block-diagram tools; medium otherwise | Ongoing — the skeleton everything else is built into over the whole project | Solver step-size mismatches between subsystems running at different rates |
| **Stateflow** (decision logic) | Named explicitly in the PS; a state machine is the natural fit for NORMAL_DRIVE → OBSTACLE_DETECTED → EMERGENCY_BRAKE → REPLAN | Visual state/transition editor, easy to demo, clean state-history logs | Can become spaghetti with many states/guards if not planned up front | Low-medium — intuitive with a sketched diagram first | ~1 day for the core states, more for edge-case transitions | Guard-condition bugs (a transition fires on the wrong tick when two conditions are momentarily both true) |
| **Navigation Toolbox** (planning) | Built-in planners (`plannerHybridAStar`, `plannerRRT`, `plannerRRTStar`) suited to lane-free reasoning — configure, don't hand-code | Saves writing a planner from scratch; well-documented, tested | Tuning the built-in planners to feel "adaptive" to Indian-road chaos is real engineering, not just config | Low to get running; **high** to tune well | Few hours to a basic planner; **3-5+ days** of tuning — likely the single biggest time sink on either path | Planner freezing/timing out on dense obstacle fields — directly hurts the "replanning latency" metric |
| **Deep Learning Toolbox + Computer Vision Toolbox** (perception) | Object detection for cars/autos/pushcarts/pedestrians/animals from camera frames | Can import pretrained ONNX models, ready detector architectures, plugs into Simulink | Fine-tuning on IDD for auto-rickshaws/pushcarts/animals still takes real training time | Medium — basic DL familiarity needed, MATLAB apps lower the bar | 1-3 days for data prep + fine-tuning + validation | Model doesn't generalize from real IDD photos to simulated renders (domain gap) — test early |
| **Sensor Fusion and Tracking Toolbox** (prediction) | Multi-object tracking + short-term trajectory prediction | Built-in trackers (JPDA, multi-object) save writing a Kalman filter from scratch | Tuning association logic for erratic/non-lane-based motion is non-trivial | Medium | 1-2 days to wire up a baseline tracker | Track-switching errors when two objects cross paths under occlusion |

## Where A and B diverge

### Path A's unique piece — ADT's own Unreal Engine scene + custom-scene authoring

| | Detail |
|---|---|
| Why used | Built into Automated Driving Toolbox — no extra bridge/middleware; Simulink drives it directly via a native block (`Simulation 3D Scene Configuration`), no network layer |
| Pros | Tightly coupled, synchronous, no timing/sync risk since it's in-process; produces `.slx` artifacts judges can open directly |
| Cons | Weak/no built-in autonomous NPC traffic behavior — actor paths mostly hand-scripted rather than getting CARLA's Traffic Manager for free |
| Learning curve | **High** if nobody has touched a game engine — Unreal Editor's level-design workflow (static meshes, materials, lighting, blueprint scripting) is a separate skill from MATLAB |
| Approx. time | 3-5+ days to hand-build one convincing custom Indian road scene (village road or market street) — the main cost of Path A |
| Known failure points | The MathWorks Unreal interface support package is **version-pinned** to a specific Unreal Engine release — a mismatched UE version silently breaks the integration; also frame-rate/performance issues with heavy custom geometry |
| Vehicle physics | Handled by Vehicle Dynamics Blockset/Simscape *inside* Simulink — Unreal is the renderer only |

### Path B's unique pieces — CARLA + the ROS bridge

**Why ROS specifically:** CARLA and Simulink are two separate processes in
two different languages (CARLA's engine is C++/Python; Simulink is
MATLAB) with no native shared-memory or direct API link between them. ROS
is the middleware translation layer — CARLA ships an official
`ros-bridge` that republishes its sensor/vehicle data as standard ROS
topics, and MATLAB's ROS Toolbox (already licensed) lets Simulink blocks
subscribe/publish to those same topics. It's not the only way to connect
them (community projects talk to CARLA's raw Python API directly), but
it's the path MathWorks actually documents and supports, and the one the
license already covers.

| | Detail |
|---|---|
| Pros (CARLA itself) | Free, superior Traffic Manager for realistic disorderly NPC behavior, huge community/tutorial base, real Indian road geometry via free OSM import |
| Cons (CARLA itself) | Runs as a separate process — none of it produces MATLAB artifacts for judges to inspect directly |
| Learning curve (ROS bridge) | **Medium-high** — requires understanding ROS concepts (nodes/topics/message types) plus getting CARLA's bridge version, the ROS distro, and MATLAB's ROS Toolbox-supported version all agreeing |
| Approx. time (ROS bridge) | 1-2 days for basic topics flowing with no prior ROS experience; **can stretch several more days** for a stable, low-latency, correctly-timed closed loop — the riskiest time sink on Path B |
| Known failure points | Version mismatches (ROS1 vs ROS2, bridge version vs. ROS distro vs. MATLAB's supported version) are the #1 reported issue on MathWorks Answers for this exact setup; message-queue backpressure causing latency spikes that would corrupt the "replanning latency" metric; sim-time vs. wall-clock desync between Simulink's solver and CARLA's tick loop |
| Vehicle physics | Handled by CARLA's own built-in PhysX model — Simulink only computes and sends throttle/brake/steer commands. Vehicle Dynamics Blockset becomes largely redundant here except for offline controller tuning before deployment |

## Summary comparison

| | **Path A** | **Path B** |
|---|---|---|
| Matches PS's "in MATLAB and Simulink" line | Yes, fully | Yes — the pipeline itself is in Simulink |
| RoadRunner needed | No | No |
| Scene authoring | Hand-build in free Unreal Editor | CARLA + free OSM import (real Indian layouts) |
| Traffic AI richness | Weaker — not built for rich autonomous NPC traffic | Strong — CARLA's Traffic Manager |
| GPU need | Real, likely lighter than CARLA | CARLA-level (heavy) |
| Team-skill fit | Simulink + Unreal Editor comfort | Simulink + Python/CARLA + ROS, spanning three ecosystems |
| Timeline cost | Slow to a visual custom scene, fast logic iteration after (toolbox planners are drop-in) | Fast early skeleton for scene work, but 1-2+ days of ROS integration risk before any feature work is reliable |
| Scoring risk | Lowest deviation risk — produces native MathWorks artifacts | Middle — pipeline is native, but a fragile live cross-process bridge is a visible failure mode if it stutters during a demo |

**One-line framing:** Path A keeps everything in one program at the cost
of hand-building the scene yourself; Path B gets a richer, freer world at
the cost of a fragile cross-process bridge. Decision deferred.
