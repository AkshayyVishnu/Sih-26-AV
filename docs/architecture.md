# End-to-End Architecture — SIH 2026, PS 26037

## Purpose

This is the synthesis document: one concrete, end-to-end architecture
assembled from everything researched across `tooling-decisions.md`,
`tradeoffs-a-vs-b.md`, `sota-research.md`, `component-deep-dive.md`, and
`carla-ecosystem-catalog.md`, plus three new research threads (failure
modes, latency composition, low-level control) run specifically to close
gaps for this document. It is explicitly optimized around **minimizing
replanning latency** — the PS's own numbers show why this matters: a CARLA
closed-loop study found success rate **collapses from ~95% to under 10%**
once sensing-to-actuation delay crosses **~150-200ms** ([source](https://arxiv.org/html/2605.29138)).
That figure is this architecture's real design constraint — every stage
below is chosen with a running latency budget against that ceiling.

---

## Pipeline overview

```
World/Scene (CARLA)
   -> Sensors (camera + LiDAR, both required; radar optional)
   -> Perception (object detection + classical LiDAR-fused distance)
   -> Prediction (MoFlow, with a native trackingIMM parallel fallback)
   -> Planning (path/trajectory generation)
   -> Decision logic (Stateflow, hierarchical + parallel states)
   -> Low-level control (MPC/PID -> steer/throttle/brake)
   -> Vehicle motion (CARLA physics)
   -> [loop back to Sensors]
        \-> Metrics/logging (latency, smoothness, completion rate)
```

Everything from Sensors through Low-level control lives in Simulink,
satisfying the PS's "pipeline...in MATLAB and Simulink" requirement
(see `tradeoffs-a-vs-b.md`); CARLA is the world/renderer/physics engine,
connected via **direct Python interop** (MATLAB's native `py.*`/`pyrun`
calling CARLA's own Python API from a MATLAB Function/System block) as the
**primary** integration path — a deliberate revision from Path B's original
ROS-bridge-based plan, made after failure-mode research found that nearly
every documented integration bug in this stack lives inside `ros-bridge`
itself. ROS Toolbox + `ros-bridge` is kept as a **documented fallback**
(see the Cross-cutting section below) if the direct route hits an
unexpected wall — see that section for the honest tradeoff, including the
one real cost: this specific combination has no found prior art to check
against.

---

## Stage 1 — World / Scene

| | Detail |
|---|---|
| **Chosen** | CARLA + custom scenes authored via OSM import (`osm2odr`) or Blender Driving Scenario Creator, enriched with **SUMMIT** for the market-area/unsignaled-intersection scenarios specifically (`carla-ecosystem-catalog.md`) |
| **Why chosen** | RoadRunner remains contingent on the NITW license clearing (`tooling-decisions.md`); SUMMIT is the strongest free option found for solving both road geometry *and* realistic chaotic mixed-traffic behavior in one tool — plain `osm2odr` gives geometry only |
| **Alternatives omitted** | RoadRunner as primary (license uncertain, see `tooling-decisions.md`); CARLA default Towns alone (too structured, doesn't satisfy "unstructured Indian roads"); IDD-3D's own toolkit (confirmed a stub, `component-deep-dive.md`) |
| **Constraints** | SUMMIT built for CARLA 0.9.x/UE4 — verify version compatibility against your chosen CARLA build before committing. No India-specific CARLA map exists anywhere (confirmed after 9 search phrasings, `carla-ecosystem-catalog.md`) — you are building this from scratch, which is a genuine novelty claim, not a shortcut you missed |
| **Known pitfalls** | CARLA sync-mode + variable timestep is an explicit anti-pattern per CARLA's own docs — physics becomes unreliable if the timestep balloons beyond ~0.1s. **Traffic Manager must be set to sync mode too** if the world is — leaving it async while the world is sync is a common, unpredictable misconfiguration. Long evaluation runs are prone to segfaults on repeated `load_world` calls (`carla_garage` #116) and route-specific stream errors (`carla_garage` #113) — build an auto-restart/resume harness rather than assuming a single long run completes cleanly |

## Stage 2 — Sensors

| | Detail |
|---|---|
| **Chosen** | Camera + **LiDAR**, both required (revised — LiDAR was previously optional). Camera feeds perception (Stage 3); LiDAR feeds a **classical, non-learned** point-cloud distance/range estimate fused with camera detections via Sensor Fusion and Tracking Toolbox. Radar kept as a stretch addition, not day-one |
| **Why chosen** | Reconsidered after a direct question: monocular camera-only distance estimation to multiple objects is the *harder*, slower, inherently scale-ambiguous problem — it would need its own learned depth network (more latency, more fine-tuning risk). LiDAR gives real geometric ground truth for free in CARLA, and turning it into per-object distance is a **classical projection operation** (project each camera bounding box into the LiDAR frustum, take median/min point depth) — not a deep model, likely latency-neutral-to-positive versus the monocular-depth alternative it replaces. This also stops underusing Sensor Fusion and Tracking Toolbox, which the PS names explicitly for exactly this multi-sensor role, and directly serves "collision avoidance" in the PS's own title |
| **Alternatives omitted** | Camera-only (the original pick) — lighter, but forces either a learned monocular-depth network (extra latency/training risk) or accepting weak distance estimates, neither good for a PS titled around collision avoidance. Full 360°camera+LiDAR+radar (TransFuser v6/LEAD's best config) — heavier still; radar specifically deferred since its main value (Doppler velocity) is lower priority than LiDAR's direct range for the "distance between multiple objects" need |
| **Constraints** | IDD-3D's *learned* LiDAR-detection toolkit is still a stub — irrelevant here since the chosen approach is classical projection, not a learned point-cloud detector, so this doesn't block anything. More sensor data volume per tick either way — benchmark the added marshaling cost through the direct-Python path |
| **Known pitfalls** | CARLA's own camera-LiDAR projection example (`lidar_to_camera.py`) has a **documented open issue**: the extrinsic calibration matrix isn't constant across frames for a fixed sensor rig (issue #3795, `carla-ecosystem-catalog.md`) — test this specifically before relying on it for the fusion step. With the ROS fallback specifically: multiple sensors at identical tick rates can drift by one frame and get spuriously dropped by the bridge's frame-validation logic at low-to-moderate physics tick rates (`ros-bridge` #589) — mitigated only by running physics at very high frequency (1000Hz/0.001s delta), which has its own cost. With direct Python interop, this specific bug class doesn't apply, but multi-sensor frame alignment is still your own responsibility to get right |

## Stage 3 — Perception

| | Detail |
|---|---|
| **Chosen** | Your own YOLOv8/v11 fine-tuned on **DATS_2022** (downloadable now, includes cattle/goat/dog/camel/horse) + IDD 2D detection data (`component-deep-dive.md`) |
| **Why chosen** | BMD-45, the earlier top pick, turned out to be a fixed-CCTV detector — domain-mismatched to an ego-vehicle camera (`component-deep-dive.md`). This is the actually-unblocked path: both source datasets are downloadable today, no gating, no access-request cycle |
| **Alternatives omitted** | BMD-45 (domain mismatch, downgraded); DriveIndia (best per-class numbers found — auto-rickshaw 0.940, pushcart 0.391, animal 0.769 mAP50 — but still EULA-gated with unconfirmed turnaround, request access now as a possible upgrade, don't depend on it); IDD-3D (stub toolkit, not usable) |
| **Constraints** | **License risk**: if you use YOLOv12 specifically, its weights inherit AGPL-3.0 upstream despite Apache-2.0 wrapper licenses on datasets like BMD-45 — check which YOLO version you actually fine-tune before shipping |
| **Known pitfalls** | No inference-latency figure exists for any perception candidate researched — **you must self-benchmark on your target hardware**, not cite an estimated number. No dedicated, well-benchmarked India-specific animal-hazard detector exists at any venue — this class is entirely your own fine-tuning work, not something you missed finding |
| **Sensor fusion (new, follows from Stage 2's LiDAR addition)** | Camera-detected bounding boxes get a distance value by projecting them into the LiDAR point cloud (median/min depth within the box's frustum) and fusing via Sensor Fusion and Tracking Toolbox — classical, not a second learned model, so this doesn't add to the perception-latency unknown above beyond the projection step itself |

## Stage 4 — Prediction

**Re-audited against MATLAB-native alternatives (`trackingIMM`/`trackerJPDA`) and newer literature — retained.** Full comparison below.

| | Detail |
|---|---|
| **Chosen** | **MoFlow** (specifically the **IMLE student model**, not the teacher — say this explicitly in the report, since the 0.70ms figure is student-only, ~47x faster than the teacher's 33.20ms/100-step figure) as the real-time backbone; IDD-PeD's trajectory-only baselines (BiTraP/SGNet_CVAE — run **both**, not just one, since both are lightweight and the domain-gap evidence differs between them) as an India-grounded comparison/ensemble partner |
| **Why chosen** | MoFlow's confirmed **0.70ms/prediction** (IMLE student, RTX6000/A40) is the fastest figure found across the entire research effort. MIT license, generic agent-agnostic BEV input. **Audited against `trackingIMM`/`trackerJPDA`** (Sensor Fusion and Tracking Toolbox, already licensed, zero-Python-dependency): these are structurally **unimodal, single-step-ahead** trackers — forward-simulating them for a multi-second forecast converges toward one weighted-average path, unable to branch into "pedestrian goes left vs. right around the stalled auto-rickshaw" the way MoFlow's K multimodal samples can. This is a real capability gap, not just an unmeasured latency difference: pure linear/CV extrapolation baselines are documented losing 24-32% on ADE/FDE against even a 2018-era learned model (Social-GAN, [arXiv:1803.10892](https://arxiv.org/html/1803.10892)) |
| **Resolved: does MoFlow's `py.*` dependency undermine "pipeline in MATLAB/Simulink"?** | **No — confirmed non-issue.** The `py.*` mechanism used to call MoFlow is architecturally identical to the one already accepted for the CARLA connection (same `pyenv` setup, same call-and-return pattern) — there's no separate, riskier category for "calling a PyTorch model" versus "calling a simulator client." If anything, Deep Learning Toolbox's documented TensorFlow/PyTorch co-execution workflow gives MoFlow **more** precedent than the CARLA pairing has (which has none, per the Cross-cutting section). Stateflow, the planner, and the MPC block still own the scheduling loop — MoFlow's call is a bounded single-tick subroutine, no different in kind from calling a MEX file. The only legitimate residual argument is judging-narrative optics, not a technical one |
| **Alternatives omitted** | `trackingIMM`/`trackerJPDA` as a **full replacement** — rejected per the capability gap above, but **added as a parallel fallback**, see below. IDD-PeD's pose-branch models (MASK_PCPA etc.) — confirmed to require real architecture surgery to extend beyond pedestrians (MMPose's human-skeleton assumption doesn't transfer to two-wheelers/animals, `component-deep-dive.md`); TrajGNAS/SocialMOIF (architecture-only, no released code); MAVEN-T (arXiv:2604.10169, April 2026, 14.6ms on a Jetson AGX Orin — not a fair comparison against MoFlow's datacenter-GPU figure, no released code, no India relevance — watch-list only) |
| **New: native parallel fallback** | Run **`trackingIMM`** alongside MoFlow at near-zero extra cost (CPU-only, natively licensed, zero `py.*` risk) so Stateflow has a same-tick single-hypothesis prediction to fail over to if the MoFlow call ever times out or errors mid-run. This operationalizes "classical fallback stays load-bearing" concretely for this stage, the same way the classical planner already backs up the planning stage |
| **Constraints** | MoFlow never validated on Indian/unstructured traffic — needs fine-tuning (use the `--load_pretrained` flag to warm-start from teacher weights, and tune `--num_to_gen` down from the default K=20 toward K=6-10 if downstream mode-selection complexity matters, headroom exists either way at 0.70ms); **METEOR's license is genuinely disputed** (MIT per HuggingFace vs. CC BY 4.0 per paper) — verify directly before redistributing anything built from it; no ready-made converter exists turning METEOR's XML tracks into MoFlow/IDD-PeD training pairs — budget for writing this yourself |
| **Known pitfalls** | IDD-PeD's code repo has **no license file at all** (`null`) — a real redistribution risk for modified baseline code, separate from the dataset's stated CC BY 4.0; **action this** (open an issue / contact the authors), don't just note it. Two of IDD-PeD's dataset archives are confirmed **server-side corrupted** (`gp_set_0001-0007.tar`, issue #4) — **verify checksums now**, not just before depending on them |
| **Worth checking (cheap, unverified)** | Whether MoFlow's IMLE student — unlike the teacher, which needs an ODE solver Deep Learning Toolbox's importer doesn't support — is importable via `importNetworkFromPyTorch`. If it works, this eliminates the `py.*` dependency for prediction entirely rather than just reasoning around it being a non-issue. Not verified against the importer's supported-layer list this round; a concrete, low-cost thing to test, not a confirmed finding |

## Stage 5 — Planning

**Re-audited against a MATLAB-native Reinforcement Learning Toolbox planner — retained, RL Toolbox not added.**

| | Detail |
|---|---|
| **Chosen (primary)** | Navigation Toolbox's classical planners (`plannerHybridAStar`/`plannerRRT*`) — zero-training, license-safe, satisfies "in MATLAB and Simulink" literally |
| **Chosen (AI-enhancement option)** | **PlanT2** — object-level, planning-only transformer (assumes perception/prediction solved), the structurally closest AI match to a "planner node" role, more so than end-to-end vision agents (`carla-ecosystem-catalog.md`) |
| **Why chosen** | The classical planner carries zero training risk and is the safety net if anything else underperforms or runs out of integration time (`tradeoffs-a-vs-b.md`). PlanT2 is worth layering in specifically because it doesn't collapse perception+prediction+planning into one opaque model the way TransFuser v6/SimLingo/ORION do — it stays modular, matching the PS's explicit staged-pipeline requirement. **Audited against training a native RL Toolbox planner** (zero Python dependency at inference, unlike PlanT2) — rejected as a replacement: MathWorks' only comparable driving example (PPO for Automated Parking Valet) is explicitly a narrow, fully-known **static** parking maneuver controller, documented as unable to handle dynamic obstacles or unknown layouts — no precedent exists for an RL Toolbox *global path planner* in dynamic/unstructured conditions. Designing and training 5 separate reward functions for your 5 scenarios (no MathWorks template covers pedestrians, animals, or unmarked-lane geometry) is real, un-de-risked engineering risk for a hackathon timeline that the classical+PCLA combination doesn't carry |
| **Alternatives omitted** | **TransFuser v6/LEAD, SimLingo, ORION as the primary planner** — despite having the strongest completion-rate numbers found (TFv6/LEAD: Bench2Drive DS 95.0/SR 84.3%), these are end-to-end agents that fuse perception+prediction+planning into one model producing only a trajectory — using one as your *entire* pipeline would blur the distinct stages the PS's expected-solution text explicitly asks for. Kept as a **fallback/comparison agent** deployed via PCLA if the modular stack underperforms, not the primary architecture; CarPlanner/GoalFlow (no released training code, `sota-research.md`); **RL Toolbox planner** (see above — worth a one-line "future work" mention in the report, since the zero-Python-dependency story is real, just not worth the training-risk trade-off now); newer 2026 work (CLEAR, PlanRL, ParkingTransformer — all PyTorch-based, none in PCLA's supported-agent list, same integration cost as current picks, not a switch candidate) |
| **Reusable architectural template found** | MathWorks' **Automated Parking Valet** example (`plannerHybridAStar` + `vehicleCostmap`/`inflationCollisionChecker` + `HelperBehavioralPlanner` mission-sequencer) is a stronger template than previously credited — its costmap-based inflation and nonholonomic path generation map directly onto the **market-area** (tight-space, multi-obstacle) and **unmarked-road** (no lane constraints) scenarios specifically. It does **not** cover unsignaled intersections or cattle-crossing (dynamic, unpredictable-obstacle cases) — those remain original work regardless. **Highway Lane Following** is a controller/safety-layer donor only (its MPC tracking + watchdog-braking pattern), not a planning donor — it has no lane-change/merge decision logic despite the name |
| **Constraints** | Tuning the classical planner to feel genuinely "adaptive" to Indian-road chaos is real engineering — flagged in `component-deep-dive.md` as likely the single biggest time sink regardless of path chosen. **If the PCLA fallback activates**, budget for **two stacked layers of Python call overhead** — your CARLA bridge's own `py.*` calls plus PCLA's separate per-step `get_action()` call — when estimating that path's real-time margin |
| **Small improvements to adopt** | `plannerHybridAStar.MotionPrimitiveLength` (default `ceil(√2×mapCellSize)`) — tune per-scenario: larger for sparse maps (highway merge), smaller for dense ones (market area). `DirectionSwitchingCost` (default 0) — raise above 0 for market-area/tight scenarios to discourage reverse-thrashing. `plannerRRTStar.GoalBias` (default 0.05) and `MaxConnectionDistance` (default 0.1, MathWorks itself notes 0.3 "works well for some scenarios") — the defaults are flagged by MathWorks' own docs as likely too conservative for vehicle-scale planning, tune up. Pin **CARLA to 0.9.15** specifically if using PCLA — it's recommended over the newer 0.9.16 for agent compatibility. When selecting a PCLA fallback agent, default to the **lightest/fastest** one (TransFuser v3/PlanT, not SimLingo/LMDrive's heavier LLM-backed variants) for real-time margin unless a demo specifically wants the flashier agent |
| **Known pitfalls** | If you do deploy a PCLA-bundled agent as a fallback: TransFuser showed a **route-deviation bug on spawn** (PCLA issue #9, ~144° veer, collided with static vegetation, unresolved) and PlanT showed a **non-finite bounding-box crash** from certain static trigger actors (PCLA issue #11, unresolved) — test both before relying on them for a demo run |

## Stage 6 — Decision logic

**Re-audited against MathWorks' own driving examples and Stateflow best-practice docs — retained, but redesigned structurally.**

| | Detail |
|---|---|
| **Chosen** | Stateflow, but a **redesigned** chart (see below), not the flat 4-state original |
| **Why chosen** | Named explicitly by the PS; negligible latency contribution (state evaluation is microseconds, not a bottleneck in this pipeline) |
| **Honest correction, don't cite this as "matching MathWorks practice"** | Checked directly: **none** of MathWorks' own flagship driving examples (Automated Parking Valet, Highway Lane Following, Highway Lane Change Planner and Controller, Forward Collision Warning) actually use a Stateflow chart for their decision layer — they use scripted classes (`HelperBehavioralPlanner`), feedforward Simulink logic, or closed-form thresholds. Stateflow remains the right choice here (interpretability, code-gen traceability, and it's what the PS names) — but present it to judges/mentors as **your team's conscious choice**, not as following MathWorks' own pattern, because on the evidence gathered they don't do this for comparable decision layers |
| **Alternatives omitted** | A learned decision layer (e.g. reimplementing iPLAN's intent-aware MARL architecture as a custom RL Toolbox agent) — kept as a design-pattern reference only. A "classifier feeds the transition guard directly" hybrid was checked specifically against literature and MathWorks docs — **no confirmed precedent found either way** (closest is Elhafsi et al. 2023, where an LLM watches an FSM from *outside* rather than feeding a guard directly — the inverse structure). If you build this, present it as your own contribution, not a validated pattern |
| **Redesigned structure (small improvements, all traced to documented Stateflow mechanisms)** | (1) **Hierarchical decomposition**: nest obstacle types under `OBSTACLE_DETECTED`, adding a distinct sibling **`ANIMAL_ON_ROAD`** state — justified directly since the PS names cattle-crossing explicitly, and livestock typically warrants REPLAN/crawl-around rather than the harder EMERGENCY_BRAKE a startled pedestrian might need. (2) **Truth table** guard immediately inside `OBSTACLE_DETECTED`, combining `obstacleType`/`confidence`/`TTC`/`relativeVelocity` — MathWorks explicitly recommends truth tables for "mode switching," replacing a nested if/else transition label. (3) **`EMERGENCY_BRAKE` and `REPLAN` as parallel (AND) states**, not exclusive alternatives — a real vehicle often needs to brake *while* a new path computes, not one-or-the-other. (4) **A parallel top-level `SAFETY_SUPERVISOR` state** (the documented "Supervisor-Worker" AND-pattern) monitoring sensor dropout/watchdog timeout, able to force `EMERGENCY_BRAKE` from any current state without duplicating a guard on all four states individually. (5) **Debounce** the `NORMAL_DRIVE → OBSTACLE_DETECTED` transition with a temporal operator (e.g. `duration(obstacleConfirmed, 0.3)`) rather than a single-frame trigger — dust/glare/occlusion on unpaved roads make single-frame triggers unreliable. (6) **Gate `RESUME` on sustained clearance** (e.g. `before(1, obstacleFrame) && duration(clear, 2)`) rather than resuming the instant an obstacle drops out of view, to prevent chattering at sensor-range boundaries. (7) **History junction** on `NORMAL_DRIVE`/`RESUME` so resume returns to the specific interrupted sub-mode (lane-keep vs. overtake-in-progress) instead of a hard default reset |
| **Terminology correction** | `during` is **not** a temporal-logic operator — it's a state-action type (`entry:`/`during:`/`exit:`) that runs every active tick, a different mechanism from `after(N,event)`/`duration(C,sec)`-style temporal operators. Worth getting right before it propagates into the technical report |
| **Research-gap finding, worth stating in the report as-is** | 2025/2026 research on unstructured/mixed-traffic decision-making has moved almost entirely to RL/GNN/VLM approaches, not classical FSMs — a direct literature search for "state machine" + unstructured/cattle/animal-detection/pedestrian-intent-and-emergency-braking terms returned zero results. **No FSM-design research exists for this exact problem** — your structural design here has no research benchmark to validate against either way, which is itself worth stating plainly rather than implying one exists |
| **Constraints** | None specific to this stage beyond general Stateflow guard-condition bugs (a transition firing on the wrong tick when two conditions are momentarily both true) — a design discipline issue, not a tooling limitation |
| **Known pitfalls** | With direct Python interop, Stateflow's inputs come from your own function calls, not a ROS topic, so this specific issue doesn't apply — but verify your actual achieved call rate matches what you configured regardless. If you fall back to ROS: `/carla/ego_vehicle/vehicle_status` was found publishing at only **3Hz despite a configured 20Hz `fixed_delta_seconds`** in one reported case (`ros-bridge` #732, unresolved) — don't assume ROS topic inputs arrive at the rate you configured |

## Stage 7 — Low-level control

| | Detail |
|---|---|
| **Chosen** | MATLAB's **"Path Following Control System"** Simulink reference block (Model Predictive Control Toolbox — already licensed) |
| **Why chosen** | This is the strongest single finding from this round's new research: it's an **Adaptive MPC** block purpose-built for exactly this task (steering + acceleration from set velocity, road curvature, lateral deviation, yaw angle), and critically, it has a **"suboptimal solution" mode with a configurable max-iteration cap specifically to guarantee worst-case execution time** — directly addresses your stated latency-minimization goal in a way you actually control, unlike black-box PyTorch model latencies. Default sample time 0.1s (configurable), supports Simulink Coder/PLC Coder code generation for further real-time optimization ([source](https://www.mathworks.com/help/mpc/ref/pathfollowingcontrolsystem.html)) |
| **Alternatives omitted** | Plain PID (what CARLA's own reference agents, TransFuser, and HiP-AD all actually use internally — `BasicAgent`'s `VehiclePIDController`, TransFuser's dual-PID with published gains) — simpler and proven, but the comparison literature found (arXiv:2011.08729) places PID as the *least* accurate of the four standard options, with worse high-curvature tracking; Pure Pursuit (efficient but ignores heading error, "erratic" at trajectory endpoints); Stanley (better than Pure Pursuit but still purely kinematic, large initial reactive steering >50° noted as a limitation) |
| **Constraints** | MPC is the most computationally expensive of the four options per the same literature — mitigated here specifically by the suboptimal-mode latency guarantee; no quantitative smoothness-vs-controller-choice numbers were found anywhere (a real gap) — you'll need to measure this yourselves against the "path smoothness" metric rather than cite a paper figure |
| **Known pitfalls** | None controller-specific found; general task-overrun risk applies regardless of integration path if the control step can't complete within its sample time — MATLAB's own Simulink-ROS docs describe this as "overrun" handling (dropped tasks) even in the ROS-based route, so budget for it either way (see latency budget below) |

## Stage 8 — Vehicle motion

| | Detail |
|---|---|
| **Chosen** | CARLA's own built-in PhysX-based vehicle physics (Path B — see `tradeoffs-a-vs-b.md`) |
| **Why chosen** | No separate physics engine needed; Simulink only sends control commands and receives state feedback |
| **Alternatives omitted** | Vehicle Dynamics Blockset/Simscape (Path A's approach) — not needed here since CARLA already simulates this; Project Chrono for higher-fidelity dynamics — worth revisiting only if the default model proves insufficient, but note it **does not support collisions** (reverts to default physics on impact, `carla-ecosystem-catalog.md`) |
| **Constraints** | None beyond what's already documented for CARLA's physics substep tuning (see latency budget) |
| **Known pitfalls** | `fixed_delta_seconds` must not exceed `max_substep_delta_time × max_substeps` or physics accuracy silently degrades; substep delta should stay below ~0.01666s, ideally below 0.01s, for stability — this is a real tuning curve, not "lower is always better" (`carla_garage`/CARLA docs, this round's research) |

## Cross-cutting: Simulink ↔ CARLA integration

### Primary: direct Python interop

| | Detail |
|---|---|
| **Chosen** | MATLAB's native Python interoperability — `py.*`/`pyrun`/`pyrunfile` calling CARLA's own Python API (`pip install carla`) directly from a Simulink MATLAB Function/System block, stepped in lockstep with CARLA's **synchronous mode + fixed timestep** (CARLA's own docs recommend this for "slow client applications" like an external controller) |
| **Why chosen** | Revised from the original ROS-bridge-based plan after failure-mode research found that **nearly every documented integration bug in this stack lives inside `ros-bridge` itself** (see the ROS fallback's pitfalls below) — removing that middleware layer removes that entire category of risk, not just mitigates it. Also removes the ROS-bridge's own latency overhead and the Simulink-ROS pub/sub overhead from the critical path |
| **Alternatives omitted** | ROS Toolbox + `ros-bridge` as primary — demoted to fallback, see below |
| **Constraints** | **No found prior art for this exact combination** (Simulink → `py.*` → CARLA client) — unlike nearly everything else in this architecture, there's no existing writeup to check pitfalls against. MATLAB's own docs confirm the `py.*` mechanism exists and how to configure it (`pyenv`, matching Python version) but publish **no timing/overhead figures** for the call path itself — this must be self-benchmarked like several other stages already flagged |
| **Known pitfalls** | None documented (see constraint above) — this is genuinely unexplored territory for this specific pairing. Mitigate by testing the call path in isolation early (a minimal Simulink model calling `py.carla.Client(...)` and reading one sensor frame) before building the full pipeline on top of it |

### Fallback: ROS Toolbox + `ros-bridge`

Kept as a documented fallback if the direct-Python route hits an unexpected wall (e.g., `py.*` call overhead proves too high, or a MATLAB/Python version incompatibility blocks it).

| | Detail |
|---|---|
| **Chosen** | `ros-bridge` + `ros-carla-msgs` (matched message types, `carla-ecosystem-catalog.md`) |
| **Constraints** | Synchronous mode means the server blocks on your client's tick — a deliberate throughput-for-determinism tradeoff, same as the primary path |
| **Known pitfalls — the most consequential finding of the whole research round, and the actual reason this was demoted from primary** | `ros-bridge` issue **#758**: Simulink's ROS subscriber block **crashes the bridge with `bad_alloc`** when consuming CARLA image topics, on almost exactly this team's stack (MATLAB/Simulink + ROS2 bridge + CARLA), unresolved upstream. If you end up here, test this specific path first, in isolation, ideally single-machine before attempting a cross-OS (Windows client/Ubuntu bridge) setup, since that combination is a plausible aggravating factor in the reported case. Separately: enabling the bridge with active RGB/depth cameras has been a recurring, unresolved community complaint about client slowdown across multiple years of issues (#34→#192→#605→#589) — a structural bottleneck, not a one-off bug |

---

## End-to-end latency budget

| Stage | Figure | Source | Status |
|---|---|---|---|
| Perception (YOLO fine-tuned) | Not reported anywhere | — | **Must self-benchmark** on target hardware |
| Prediction (MoFlow, IMLE student) | **0.70 ms** | [arXiv:2503.09950](https://arxiv.org/html/2503.09950) | Confirmed, real hardware (RTX6000/A40) |
| Prediction fallback (`trackingIMM`, native) | Not published by MathWorks | — | Expected sub-ms to low-single-digit-ms (Kalman-family, matrix-algebra only, no GPU/Python round-trip) — **estimate, not a citation**, self-benchmark |
| Planning (Navigation Toolbox classical) | Not reported | — | Must self-benchmark; PlanT (if used) claims "5.3x faster than pixel-based baselines" but no absolute figure |
| Planning fallback (PCLA-bundled agent) | Not reported | — | Stacks **two** layers of Python call overhead (CARLA bridge `py.*` + PCLA's own `get_action()`) — budget conservatively if this path activates |
| Decision logic (Stateflow) | Negligible (µs-scale state evaluation) | — | Not a bottleneck |
| Low-level control (MATLAB Adaptive MPC) | Sample time 0.1s (configurable), **worst-case execution time bounded via suboptimal mode** | [MathWorks docs](https://www.mathworks.com/help/mpc/ref/pathfollowingcontrolsystem.html) | Confirmed guaranteed-bound mechanism exists; exact ms figure depends on your horizon/iteration-cap configuration |
| Direct Python interop overhead (primary integration path) | Not reported anywhere — no found precedent for this exact pairing | — | **Must self-benchmark**; likely lower than the ROS path since it removes a full middleware hop, but unconfirmed |
| *Reference only —* ROS bridge overhead (fallback path; Autoware-bridge figure, not vanilla ros-bridge) | avg **7.8ms**, max **&lt;15ms** per sensor hop | [arXiv:2402.11239](https://arxiv.org/html/2402.11239v1) | Order-of-magnitude reference only, and only relevant if you fall back to the ROS path |
| *Reference only —* Simulink ROS pub/sub pair overhead (fallback path; community-reported, unofficial) | **≥5ms per pair** | [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/242320-how-to-define-ros-publisher-and-subscriber-rate-in-simulink) | Anecdotal, fallback-path only |
| **Target ceiling** | **~150-200ms total** (success rate collapses beyond this in a comparable CARLA closed-loop study) | [arXiv:2605.29138](https://arxiv.org/html/2605.29138) | This is the number to design against |

**Bottom line**: the confirmed figures (MoFlow 0.70ms + MPC's bounded control step) leave headroom under the ~150-200ms ceiling regardless of integration path — **provided perception inference, planner tuning, and the direct-Python call overhead don't blow the budget**, none of which have a published number. Benchmark all three early; they're your actual risk, not the pieces with published figures.

---

## Metrics-to-architecture mapping

| PS metric | Primarily driven by | Secondary contributors |
|---|---|---|
| **Replanning latency** | Planning stage (self-benchmark required) + direct-Python interop overhead (self-benchmark required) + MPC control step | Perception/prediction inference time (MoFlow confirmed fast; perception unconfirmed) |
| **Path smoothness** | Low-level control choice (MATLAB Adaptive MPC, chosen specifically for this) | Planner output quality (jerky waypoints propagate downstream regardless of controller) |
| **Scenario completion rate** | Planning + decision logic working correctly across all 5 scenarios | Perception/prediction accuracy (a missed auto-rickshaw detection cascades into a completion failure) |

---

## Full submission-deliverable checklist

| PS-required deliverable | Produced by |
|---|---|
| Simulation model | The Simulink model assembled from Stages 1-8 above |
| Designed scenarios | Stage 1 (World/Scene) — CARLA + SUMMIT/OSM-authored village road, unsignaled intersection, highway merge, market area, cattle-crossing |
| Performance results (latency, smoothness, completion rate) | Stage-level logging per the latency budget table + your own benchmarking, per `component-deep-dive.md`'s metrics-alignment table |
| Technical report | This document + `tooling-decisions.md`/`tradeoffs-a-vs-b.md` for the "explains approach and design choices" requirement |
| Demonstration video | CARLA camera sensor recording of closed-loop runs across all 5 scenarios (per earlier GPU/viewing research) |
| Closed-loop validation | The full loop in the Pipeline overview above — perception→plan→control→CARLA physics→perception again, each tick, not an open-loop replay |

---

## Constraints summary (consolidated across all research)

- **Licensing**: YOLOv12 weights (AGPL-3.0 upstream), METEOR (disputed MIT/CC-BY-4.0), IDD-PeD code repo (no license file — action this, don't just track it), `carla_dataset_tools` (GPL-3.0), Bench2Drive (CC-BY-NC-ND, eval-only), Roach (CC-BY-NC, non-commercial) — audit all of these against your submission's actual usage before finalizing.
- **Sensor fusion**: LiDAR promoted from optional to required (Stage 2) — CARLA's own camera-LiDAR projection example has a documented open calibration-matrix-instability issue (issue #3795) — test this specifically, don't assume the reference example's extrinsics hold across frames.
- **Prediction/planning fallbacks**: `trackingIMM` (native) now backs up MoFlow; a PCLA-bundled agent still backs up the classical planner but stacks two layers of Python call overhead if activated — both fallbacks are unbenchmarked, budget time to test them, not just have them on paper.
- **Access gating**: DriveIndia (EULA-gated, unconfirmed turnaround — request now if pursuing it as a perception upgrade).
- **Hardware**: perception + planning latency both unconfirmed and must be self-benchmarked; SimLingo confirmed running at ~0.05x real-time on an RTX 4060 Ti in one report — don't assume paper-reported timings hold on your hardware for any PCLA-bundled fallback agent.
- **Version pins**: CARLA client/server version must match exactly; SUMMIT targets CARLA 0.9.x/UE4 specifically; the ROS fallback path has version-specific regressions (e.g. issue #533, fixed between 0.9.10.1 and 0.9.11) — pin and document your exact CARLA version everywhere.
- **Operational**: long closed-loop evaluation runs need an auto-restart/resume harness (repeated segfault reports across `carla_garage` and SimLingo); closed-loop scores are seed-sensitive — run ≥3 seeds before reporting a completion-rate number, per `carla_garage`'s own maintainer-authored common-mistakes guide.
- **Integration path**: direct Python interop is now primary specifically because nearly every found integration bug lives in `ros-bridge` — but it has **no found prior art for this exact pairing**, unlike everything else in this architecture. Treat its latency and stability as unverified until you've benchmarked it yourselves; the ROS fallback remains documented with its own known pitfalls if you need it.
- **Research gaps acknowledged, not silently dropped**: MATLAB-forum-specific integration reports and SIH/hackathon post-mortems could not be found this round (search quota was exhausted mid-research) — worth a manual follow-up search on MATLAB Central Answers directly before final integration.
