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
   -> Sensors (camera, optional LiDAR)
   -> Perception (object detection)
   -> Prediction (trajectory forecasting)
   -> Planning (path/trajectory generation)
   -> Decision logic (Stateflow mode-switching)
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
| **Chosen** | Camera-primary sensor rig (matches the lightest-footprint shortlisted models, e.g. SimLingo's single front camera); LiDAR optional, added only if pursuing IDD-3D-style perception or a heavier agent like TransFuser v6/LEAD's best config |
| **Why chosen** | Camera-only keeps sensor-processing/data-marshaling overhead and GPU load down. Direct Python interop removes the `ros-bridge` camera-serialization bottleneck that motivated this in the original ROS-based plan (issues #34/#192/#605/#589) — but camera-only is kept anyway since it's still the lighter, simpler rig regardless of integration path |
| **Alternatives omitted** | Full 360°camera+LiDAR+radar rig (TransFuser v6/LEAD's best-scoring config) — higher completion-rate ceiling, but heavier data-marshaling and GPU load either way; revisit only if camera-only underperforms |
| **Constraints** | If LiDAR is added, IDD-3D's toolkit is still a stub — you'd be building the point-cloud pipeline yourselves |
| **Known pitfalls** | With the ROS fallback specifically: multiple cameras at identical tick rates can drift by one frame and get spuriously dropped by the bridge's frame-validation logic at low-to-moderate physics tick rates (`ros-bridge` #589) — mitigated only by running physics at very high frequency (1000Hz/0.001s delta), which has its own cost. With direct Python interop, this specific bug class doesn't apply, but multi-sensor frame alignment is still your own responsibility to get right — test it early either way |

## Stage 3 — Perception

| | Detail |
|---|---|
| **Chosen** | Your own YOLOv8/v11 fine-tuned on **DATS_2022** (downloadable now, includes cattle/goat/dog/camel/horse) + IDD 2D detection data (`component-deep-dive.md`) |
| **Why chosen** | BMD-45, the earlier top pick, turned out to be a fixed-CCTV detector — domain-mismatched to an ego-vehicle camera (`component-deep-dive.md`). This is the actually-unblocked path: both source datasets are downloadable today, no gating, no access-request cycle |
| **Alternatives omitted** | BMD-45 (domain mismatch, downgraded); DriveIndia (best per-class numbers found — auto-rickshaw 0.940, pushcart 0.391, animal 0.769 mAP50 — but still EULA-gated with unconfirmed turnaround, request access now as a possible upgrade, don't depend on it); IDD-3D (stub toolkit, not usable) |
| **Constraints** | **License risk**: if you use YOLOv12 specifically, its weights inherit AGPL-3.0 upstream despite Apache-2.0 wrapper licenses on datasets like BMD-45 — check which YOLO version you actually fine-tune before shipping |
| **Known pitfalls** | No inference-latency figure exists for any perception candidate researched — **you must self-benchmark on your target hardware**, not cite an estimated number. No dedicated, well-benchmarked India-specific animal-hazard detector exists at any venue — this class is entirely your own fine-tuning work, not something you missed finding |

## Stage 4 — Prediction

| | Detail |
|---|---|
| **Chosen** | **MoFlow** (fine-tuned) as the real-time backbone; IDD-PeD's trajectory-only baselines (BiTraP/SGNet_CVAE) as an India-grounded comparison/ensemble partner |
| **Why chosen** | MoFlow's confirmed **0.70ms/prediction** (IMLE student, RTX6000/A40) is the fastest figure found across the entire research effort — critical given the latency ceiling. MIT license, generic agent-agnostic BEV input (works for any tracked class, not just pedestrians) |
| **Alternatives omitted** | IDD-PeD's pose-branch models (MASK_PCPA etc.) — confirmed to require real architecture surgery to extend beyond pedestrians (MMPose's human-skeleton assumption doesn't transfer to two-wheelers/animals, `component-deep-dive.md`); TrajGNAS/SocialMOIF (architecture-only, no released code) |
| **Constraints** | MoFlow never validated on Indian/unstructured traffic — needs fine-tuning; **METEOR's license is genuinely disputed** (MIT per HuggingFace vs. CC BY 4.0 per paper) — verify directly before redistributing anything built from it; no ready-made converter exists turning METEOR's XML tracks into MoFlow/IDD-PeD training pairs — budget for writing this yourself |
| **Known pitfalls** | IDD-PeD's code repo has **no license file at all** (`null`) — a real redistribution risk for modified baseline code, separate from the dataset's stated CC BY 4.0. Two of IDD-PeD's dataset archives are confirmed **server-side corrupted** (`gp_set_0001-0007.tar`, issue #4) — verify checksums before depending on them |

## Stage 5 — Planning

| | Detail |
|---|---|
| **Chosen (primary)** | Navigation Toolbox's classical planners (`plannerHybridAStar`/`plannerRRT*`) — zero-training, license-safe, satisfies "in MATLAB and Simulink" literally |
| **Chosen (AI-enhancement option)** | **PlanT2** — object-level, planning-only transformer (assumes perception/prediction solved), the structurally closest AI match to a "planner node" role, more so than end-to-end vision agents (`carla-ecosystem-catalog.md`) |
| **Why chosen** | The classical planner carries zero training risk and is the safety net if anything else underperforms or runs out of integration time (`tradeoffs-a-vs-b.md`). PlanT2 is worth layering in specifically because it doesn't collapse perception+prediction+planning into one opaque model the way TransFuser v6/SimLingo/ORION do — it stays modular, matching the PS's explicit staged-pipeline requirement |
| **Alternatives omitted** | **TransFuser v6/LEAD, SimLingo, ORION as the primary planner** — despite having the strongest completion-rate numbers found (TFv6/LEAD: Bench2Drive DS 95.0/SR 84.3%), these are end-to-end agents that fuse perception+prediction+planning into one model producing only a trajectory — using one as your *entire* pipeline would blur the distinct stages the PS's expected-solution text explicitly asks for. Kept as a **fallback/comparison agent** deployed via PCLA if the modular stack underperforms, not the primary architecture; CarPlanner/GoalFlow (no released training code, `sota-research.md`) |
| **Constraints** | Tuning the classical planner to feel genuinely "adaptive" to Indian-road chaos is real engineering — flagged in `component-deep-dive.md` as likely the single biggest time sink regardless of path chosen |
| **Known pitfalls** | If you do deploy a PCLA-bundled agent as a fallback: TransFuser showed a **route-deviation bug on spawn** (PCLA issue #9, ~144° veer, collided with static vegetation, unresolved) and PlanT showed a **non-finite bounding-box crash** from certain static trigger actors (PCLA issue #11, unresolved) — test both before relying on them for a demo run |

## Stage 6 — Decision logic

| | Detail |
|---|---|
| **Chosen** | Stateflow state machine (`NORMAL_DRIVE → OBSTACLE_DETECTED → EMERGENCY_BRAKE/REPLAN → RESUME`), as established in `tradeoffs-a-vs-b.md` |
| **Why chosen** | Named explicitly by the PS; negligible latency contribution (state evaluation is microseconds, not a bottleneck in this pipeline) |
| **Alternatives omitted** | A learned decision layer (e.g. reimplementing iPLAN's intent-aware MARL architecture as a custom RL Toolbox agent) — kept as a design-pattern reference only, not required since Stateflow already satisfies the PS's ask cheaply |
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

## Cross-cutting: ROS bridge (Simulink ↔ CARLA)

| | Detail |
|---|---|
| **Chosen** | `ros-bridge` + `ros-carla-msgs` (matched message types, `carla-ecosystem-catalog.md`), CARLA in **synchronous mode with a fixed timestep** — CARLA's own docs explicitly recommend this for "slow client applications" like an external MATLAB/Simulink controller |
| **Why chosen** | Synchronous mode is the only way to get determinism, which you need for reproducible demo/judging runs |
| **Constraints** | Synchronous mode means the server blocks on your client's tick — a deliberate throughput-for-determinism tradeoff |
| **Known pitfalls — the most consequential finding of this whole research round** | `ros-bridge` issue **#758**: Simulink's ROS subscriber block **crashes the bridge with `bad_alloc`** when consuming CARLA image topics, on almost exactly this team's stack (MATLAB/Simulink + ROS2 bridge + CARLA), unresolved upstream. **Test this specific path first, in isolation, before building anything else on top of it** — ideally single-machine before attempting a cross-OS (Windows client/Ubuntu bridge) setup, since that combination is a plausible aggravating factor in the reported case. Separately: enabling the bridge with active RGB/depth cameras has been a recurring, unresolved community complaint about client slowdown across multiple years of issues (#34→#192→#605→#589) — treat this as a structural bottleneck to design around, not a one-off bug you might avoid |

---

## End-to-end latency budget

| Stage | Figure | Source | Status |
|---|---|---|---|
| Perception (YOLO fine-tuned) | Not reported anywhere | — | **Must self-benchmark** on target hardware |
| Prediction (MoFlow, IMLE student) | **0.70 ms** | [arXiv:2503.09950](https://arxiv.org/html/2503.09950) | Confirmed, real hardware (RTX6000/A40) |
| Planning (Navigation Toolbox classical) | Not reported | — | Must self-benchmark; PlanT (if used) claims "5.3x faster than pixel-based baselines" but no absolute figure |
| Decision logic (Stateflow) | Negligible (µs-scale state evaluation) | — | Not a bottleneck |
| Low-level control (MATLAB Adaptive MPC) | Sample time 0.1s (configurable), **worst-case execution time bounded via suboptimal mode** | [MathWorks docs](https://www.mathworks.com/help/mpc/ref/pathfollowingcontrolsystem.html) | Confirmed guaranteed-bound mechanism exists; exact ms figure depends on your horizon/iteration-cap configuration |
| ROS bridge overhead (reference figure, Autoware-bridge, not vanilla ros-bridge) | avg **7.8ms**, max **&lt;15ms** per sensor hop | [arXiv:2402.11239](https://arxiv.org/html/2402.11239v1) | Order-of-magnitude reference only — not a guarantee for your own bridge config |
| Simulink ROS pub/sub pair overhead (community-reported, unofficial) | **≥5ms per pair** | [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/242320-how-to-define-ros-publisher-and-subscriber-rate-in-simulink) | Anecdotal, not an official benchmark — budget conservatively if chaining several topics |
| **Target ceiling** | **~150-200ms total** (success rate collapses beyond this in a comparable CARLA closed-loop study) | [arXiv:2605.29138](https://arxiv.org/html/2605.29138) | This is the number to design against |

**Bottom line**: the confirmed figures (MoFlow 0.70ms + ROS overhead ~10-30ms for 2-3 hops + MPC's bounded control step) leave comfortable headroom under the ~150-200ms ceiling — **provided perception inference and planner tuning don't blow the budget**, which is exactly the part nobody has published a number for. Benchmark those two first; they're your actual risk, not the pieces with published figures.

---

## Metrics-to-architecture mapping

| PS metric | Primarily driven by | Secondary contributors |
|---|---|---|
| **Replanning latency** | Planning stage (self-benchmark required) + ROS bridge overhead + MPC control step | Perception/prediction inference time (MoFlow confirmed fast; perception unconfirmed) |
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

- **Licensing**: YOLOv12 weights (AGPL-3.0 upstream), METEOR (disputed MIT/CC-BY-4.0), IDD-PeD code repo (no license file), `carla_dataset_tools` (GPL-3.0), Bench2Drive (CC-BY-NC-ND, eval-only), Roach (CC-BY-NC, non-commercial) — audit all of these against your submission's actual usage before finalizing.
- **Access gating**: DriveIndia (EULA-gated, unconfirmed turnaround — request now if pursuing it as a perception upgrade).
- **Hardware**: perception + planning latency both unconfirmed and must be self-benchmarked; SimLingo confirmed running at ~0.05x real-time on an RTX 4060 Ti in one report — don't assume paper-reported timings hold on your hardware for any PCLA-bundled fallback agent.
- **Version pins**: CARLA client/server version must match exactly; SUMMIT targets CARLA 0.9.x/UE4 specifically; `ros-bridge` has version-specific regressions (e.g. issue #533, fixed between 0.9.10.1 and 0.9.11) — pin and document your exact CARLA version everywhere.
- **Operational**: long closed-loop evaluation runs need an auto-restart/resume harness (repeated segfault reports across `carla_garage` and SimLingo); closed-loop scores are seed-sensitive — run ≥3 seeds before reporting a completion-rate number, per `carla_garage`'s own maintainer-authored common-mistakes guide.
- **Research gaps acknowledged, not silently dropped**: MATLAB-forum-specific integration reports and SIH/hackathon post-mortems could not be found this round (search quota was exhausted mid-research) — worth a manual follow-up search on MATLAB Central Answers directly before final integration.
