# Path A vs. Path B — Full Tradeoff Comparison

> **Status: decided — Path B (refined), now historical background.**
> The team went with Path B, and `docs/architecture.md` has since refined
> it substantially beyond what's captured here: within Path B, **direct
> Python interop (`py.*` calling CARLA's own Python API) replaced ROS as
> the primary Simulink↔CARLA connection**, with ROS Toolbox + `ros-bridge`
> demoted to a documented fallback — made after failure-mode research
> found nearly every integration bug in this stack lives inside
> `ros-bridge` itself. `docs/architecture.md` is the current source of
> truth for the actual build; this document's comparison reasoning is
> kept intact below since it's still the record of *why* Path B was
> chosen over Path A, with one section (Path B's ROS-bridge framing)
> corrected to reflect the later revision rather than left stale.

This document compares, it does not recommend — it captures the
component-by-component analysis worked through while evaluating how to
build PS 26037's pipeline now that the NIT Warangal license is confirmed
to cover nearly the full MATLAB/Simulink toolbox catalog, minus RoadRunner
specifically.

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

### Path B's unique pieces — CARLA + the connection to it

**Why ROS was the original plan, and why it's since been demoted:** CARLA
and Simulink are two separate processes in two different languages
(CARLA's engine is C++/Python; Simulink is MATLAB) with no native
shared-memory or direct API link between them — *something* has to
bridge them. ROS was the initial choice because CARLA ships an official
`ros-bridge` republishing sensor/vehicle data as ROS topics, and MATLAB's
ROS Toolbox (already licensed) lets Simulink subscribe/publish to those
topics — it's the path MathWorks documents most directly.

**This has since changed** (see the status note at the top): dedicated
failure-mode research for `docs/architecture.md` found that nearly every
documented integration bug in this combination lives *inside* `ros-bridge`
itself — most notably issue #758, where Simulink's own ROS subscriber
block crashes the bridge with `bad_alloc` on CARLA image topics, on
almost exactly this stack, unresolved upstream. MATLAB's native Python
interoperability (`py.*`/`pyrun`, calling CARLA's own Python API directly
from a MATLAB Function/System block) removes that entire middleware layer
— and with it, that entire category of risk — so it replaced ROS as
Path B's primary connection mechanism. ROS Toolbox + `ros-bridge` is kept
only as a documented fallback now. The table below describes CARLA itself
plus both connection options.

| | Detail |
|---|---|
| Pros (CARLA itself) | Free, superior Traffic Manager for realistic disorderly NPC behavior, huge community/tutorial base, real Indian road geometry via free OSM import (and, per later research, **SUMMIT** for authentically chaotic mixed traffic on real OSM locations) |
| Cons (CARLA itself) | Runs as a separate process — none of it produces MATLAB artifacts for judges to inspect directly |
| **Direct Python interop (now primary)** | No known integration bugs found (the flip side: also no prior art for this exact pairing — genuinely unexplored territory, self-benchmark everything). Removes `ros-bridge`'s entire documented bug class and its latency overhead from the critical path |
| ROS bridge (now fallback) — learning curve | **Medium-high** — requires understanding ROS concepts (nodes/topics/message types) plus getting CARLA's bridge version, the ROS distro, and MATLAB's ROS Toolbox-supported version all agreeing |
| ROS bridge (now fallback) — approx. time | 1-2 days for basic topics flowing with no prior ROS experience; **can stretch several more days** for a stable, low-latency, correctly-timed closed loop |
| ROS bridge (now fallback) — known failure points | Issue #758 (Simulink crashes the bridge, see above); version mismatches (ROS1 vs ROS2, bridge version vs. ROS distro vs. MATLAB's supported version) are the #1 reported issue on MathWorks Answers for this exact setup; message-queue backpressure causing latency spikes that would corrupt the "replanning latency" metric; sim-time vs. wall-clock desync between Simulink's solver and CARLA's tick loop; enabling the bridge with active RGB/depth cameras has been a recurring, unresolved multi-year community complaint about client slowdown |
| Vehicle physics | Handled by CARLA's own built-in PhysX model regardless of connection method — Simulink only computes and sends throttle/brake/steer commands. Vehicle Dynamics Blockset becomes largely redundant here except for offline controller tuning before deployment |

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
| Scoring risk | Lowest deviation risk — produces native MathWorks artifacts | Middle — pipeline is native, but the CARLA connection is a live cross-process link; direct Python interop (now primary) has no documented failure history to weigh against, unlike the ROS fallback it replaced, which had a specific known crash bug |

**One-line framing:** Path A keeps everything in one program at the cost
of hand-building the scene yourself; Path B gets a richer, freer world at
the cost of a cross-process link to manage. **Decision made: Path B**,
refined further in `docs/architecture.md` — see that document for the
concrete, current build.
