# SIH 2026 · PS 26037 — Adaptive Path Planning & Collision Avoidance on Unstructured Indian Roads

## Runtime-Assured Adaptive Planner (RAAP)

---

## 1. Abstract

Indian roads combine missing lane markings, informal merging and highly
heterogeneous traffic: two-wheelers, auto-rickshaws, pushcarts, pedestrians
and livestock. A driving system for them needs strong planning in the
common case *and* guaranteed safe behaviour when that planning is wrong or late.

RAAP is a **3-tier, 5-layer** closed-loop architecture built on CARLA 0.9.16:

- **Primary planning** uses **TransFuser v6 (TFv6)**, the top-scoring learned planner on the Bench2Drive benchmark (95.0 Driving Score [1]).
- **The safety architecture is fully deterministic**: a runtime monitor, a parallel contingency planner, an evasive brake-and-steer layer and a context-aware Minimal Risk Maneuver. It is built from Kalman filtering, Hungarian assignment, grid A\*, closed-form time-to-collision (TTC) and Responsibility-Sensitive Safety (RSS) checks, and quintic-spline evasion.
- **Every command is verified** against the latest perception before it reaches the vehicle.
- **An adaptive supervisor** retunes speed, horizon, resolution and safety thresholds every tick based on scene complexity and timing health.

**Results so far:**
- In live CARLA runs, TFv6 with a TTC safety monitor drove **collision-free in all three test scenarios**. Our classical-only baseline was in collision contact for 60–97% of each run.
- The deterministic safety loop runs at **1.4–7.1 ms per tick**. That is far inside the ≤100 ms band where closed-loop driving remains reliable [15].

---

## 2. Problem Statement Alignment

| PS requirement | RAAP |
|---|---|
| Perceive mixed traffic (cars, auto-rickshaws, pushcarts, pedestrians, animals) with a multi-sensor setup | Camera + LiDAR (+ radar) fusion inside TFv6, plus an independent camera–LiDAR fusion and tracking chain for the safety layers (§4.1) |
| Predict short-term motion of irregular, non-lane-following agents | Kalman constant-velocity tracking with a per-class multimodal uncertainty fan. TFv6 predicts implicitly (§4.2) |
| Generate a real-time replanned, collision-free path | Learned primary planner ∥ deterministic contingency planner, both verified at 20 Hz (§4.3–4.4) |
| Pipeline: perception → prediction → planning → decision → motion | Implemented in this order (§4) |
| 5 scenarios: village road, unsignaled intersection, highway merge, dense market, cattle crossing | All five authored in CARLA (§8) |
| ≥2 detailed authored scenes | Unmarked village road and unsignaled urban intersection (§8) |
| Metrics: **replanning latency, path smoothness, completion rate** | Logged automatically on every run by our metrics harness (§7) |
| MathWorks tooling | Each layer mapped to Stateflow, Navigation, Sensor Fusion & Tracking, and MPC toolboxes (§9) |

---

## 3. Architecture

```
  SENSORS — CARLA 0.9.16, synchronous, 20 Hz
  Camera(s) · LiDAR · Radar
        │
        ├──────────────────────────────┐
        ▼                              ▼
  TFv6 internal perception       SAFETY PERCEPTION CHAIN
                                 YOLO → camera-LiDAR fusion → Kalman + Hungarian
                                 (+ OSNet re-ID) → uncertainty fan · drivable area
        │                              │
        │   ┌──────────────────────────┴──────────────────────────────────┐
        │   │ TIER 3 — ADAPTIVE SUPERVISOR                                  │
        │   │ scene complexity + timing health → speed cap, horizon,        │
        │   │ grid resolution, TTC thresholds, max plan age                 │
        │   └───────────┬──────────────────────────────────┬───────────────┘
        ▼               ▼                                  ▼
  ┌───────────── TIER 2 — DELIBERATIVE PLANNING (parallel) ─────────────────┐
  │ LAYER 0  Primary planner: TFv6          LAYER 2  Contingency planner     │
  │ (learned, asynchronous)                 (deterministic, every tick)       │
  │                                         costmap A* / consensus-ADMM       │
  │                                         + last verified safe plan         │
  └───────────────┬──────────────────────────────────┬─────────────────────┘
                  ▼   candidates: TFv6 plan → contingency plan → last safe plan
  ┌───────────── TIER 1 — REACTIVE SAFETY (deterministic, every tick) ──────┐
  │ LAYER 1  Runtime monitor: Timing · Plausibility · Kinematics · Collision │
  │ LAYER 3  Evasive maneuver: brake-vs-steer decision, quintic + LQR        │
  │ LAYER 4  Minimal Risk Maneuver: pull-over / in-lane stop / crawl-clear   │
  │ WATCHDOG (independent thread): 150 ms soft / 200 ms hard deadline        │
  └──────────────────────────────┬──────────────────────────────────────────┘
                                 ▼
          CONTROL: Pure Pursuit + speed control  ·  LQR during evasion
                                 ▼
                        Vehicle actuation (CARLA)
```

**Two decoupled loops ("think fast, think slow"):**
- **Fast safety loop, 20 Hz.** Perception chain, supervisor, contingency planner, monitor, evasive layer, MRM and control. It determines actuation latency.
- **Slow primary loop, asynchronous.** TFv6 publishes a plan whenever inference completes. Each plan is a *candidate*, re-verified every 50 ms against current perception before execution.

This mirrors the pattern described publicly by the leading driverless operator: fast real-time control, slower deliberative reasoning, and a separate validation layer checking every trajectory "against hard physics-based constraints" [20].

---

## 4. Methodology

### 4.1 Perception

**Primary.** TFv6 fuses camera, LiDAR and radar features using transformer attention across modalities [1, 2].

**Safety chain.** An independent chain feeds Tiers 1–3:

| Stage | Method |
|---|---|
| Detection | YOLO fine-tuned on Indian traffic datasets (IDD, DATS_2022), covering auto-rickshaws, pushcarts and livestock |
| 3D localisation | Project LiDAR into the image; each object's position is the median of the LiDAR points inside its bounding box. A minimum of 3 points is required |
| Tracking | **Kalman filter**, constant-velocity model with state `[x, y, vx, vy]`, and **Hungarian assignment** with a 4 m gate. Tracks are kept for 5 missed ticks |
| Re-identification | **OSNet** appearance embeddings [23] combined with motion cost in the assignment (StrongSORT-style). This reduces ID switches in dense crossing traffic |
| Drivable area | LiDAR points labelled road / non-road via semantic segmentation. Essential where no lane markings exist |

Because the safety layers use their own perception, an error inside the
learned planner does not also disable the checks that catch it.

### 4.2 Prediction

Each track yields three modes:
- a straight constant-velocity extrapolation (p = 0.6)
- two lateral modes (p = 0.2 each), whose spread grows to a class-specific σ at the horizon: pedestrian 1.2 m, animal 0.8 m, two-wheeler 0.6 m, car 0.3 m

This captures sudden sideways motion by pedestrians, two-wheelers and
cattle. Tracks with fewer than 3 observations get the straight mode only.

### 4.3 Planning — Layer 0: TransFuser v6 (primary)

TFv6 (LEAD, CVPR 2026) [1] outputs the nominal driving plan. Published results:

| Benchmark | Score |
|---|---|
| Bench2Drive | **95.0 Driving Score / 84.3% Success Rate** |
| Longest6 v2 | **62 Driving Score / 91% Route Completion** |

Learned planners are strongest in aggregate driving quality. Their
documented weak spot is collisions at unprotected turns and lane changes
[2], and Layers 1–4 exist to cover it.

### 4.4 Planning — Layer 2: Contingency planner (parallel, every tick)

An emergency stop from 50 km/h covers about 14 m during which the vehicle
cannot change the outcome [7]. A fallback computed *after* the primary fails
is therefore already late. Layer 2 runs **every tick in parallel** with TFv6,
so a verified alternative always exists.

**Base planner (costmap + A\*):**
- Soft Gaussian cost inflation around predicted obstacle positions, with class-specific radii: pedestrian or animal 2.0 m, two-wheeler or pushcart 1.75 m, vehicles 1.5 m.
- Non-drivable cells are marked as high cost.
- **8-connected grid A\*** search.
- Line-of-sight shortcutting, moving-average smoothing, and resampling to 1.5 m waypoints.
- Hysteresis: replanning only on a >15% cost change with at least 3 ticks between replans.

**Consistency upgrade:** consensus-ADMM parallel trajectory optimization (CPTO) [4].
- Several candidate trajectories share a common initial segment, so the vehicle never visibly switches strategy tick to tick.
- It solves in about 40 ms or less even with many obstacles [4].
- An occlusion-aware extension handles road users hidden behind occluders [5].

**Always-safe buffer:** the last plan that passed verification is kept. If
new planning fails, the vehicle follows the safe trajectory from the
previous timestep [6].

### 4.5 Decision — Layer 1: Runtime monitor

An online verification monitor detects bad or late plans but cannot fix
them on its own. It must be paired with an active fallback [3], which is
why Layers 2–4 always have an answer ready. Each tick, candidates are
checked in priority order: TFv6 plan, then contingency plan, then last safe plan. **The first to pass all four
checks is executed.**

| Check | Test |
|---|---|
| **T — Timing** | Plan age ≤ supervisor limit (300–500 ms). Remaining horizon ≥ 2× producer compute time [7]. Deadline misses (150 ms soft / 200 ms hard) are tracked over the last 5 plans |
| **P — Plausibility** | All values finite; ≥2 waypoints; plan starts within 1 m of the ego pose; spacing and speeds within limits |
| **K — Kinematics** | Bicycle model: curvature `|κ| ≤ tan(δ_max)/L`; lateral acceleration `v²|κ| ≤ 0.8·μg`; bounded deceleration and steering rate |
| **C — Collision** | Ego footprint (3 discs) swept along the plan against every tracked object's predicted, uncertainty-inflated position [8]. Minimum TTC ≥ threshold. **RSS** safe longitudinal distance to the lead vehicle [9] |

The RSS minimum safe distance, where ρ is the response time:

`d_min = v_r·ρ + ½·a_acc·ρ² + (v_r + ρ·a_acc)² / (2·a_brake,min) − v_f² / (2·a_brake,max)`

**Watchdog.** An independent thread supervises the fast loop:
- Above 150 ms, the speed cap is reduced.
- Above 200 ms, or if the loop stalls, the watchdog commands Layer 4 braking directly.

Every tick logs which layer drove and why (T/P/K/C), so each decision is explainable.

**Mode state machine** (Stateflow-ready): `NORMAL_DRIVE` → `OBSTACLE_DETECTED` / `ANIMAL_ON_ROAD` → `EVASIVE_MANEUVER` → `EMERGENCY_BRAKE ∥ REPLAN` → `MRM` → `RESUME`.
- Obstacle entry is debounced over 3 ticks.
- Resume requires ~1 s of sustained clearance.
- A parallel safety-supervisor state can force braking from any state.

### 4.6 Motion — Layer 3: Evasive maneuver (brake **and** steer)

Layer 3 engages when no candidate plan passes and a collision is predicted.
Evasive minimum-risk maneuvering is an established active-safety function
with its own safety-verification process [12].

**1. Brake or steer?** Braking distance grows with v², while the distance
needed to steer clear by an offset w grows with v. Steering needs less
distance above the crossover speed:

**v\* = 2·√(2·w·μ·g)**

| Road surface | v\* for a 1.8 m offset |
|---|---|
| Dry asphalt (μ ≈ 0.8) | ≈ 38 km/h |
| Wet / dusty (μ ≈ 0.5) | ≈ 30 km/h |
| Unpaved (μ ≈ 0.3) | ≈ 23 km/h |

The same decision logic, steering when braking alone cannot prevent a
collision, is used in automotive active-safety systems [11]. Decision rule:
- If braking alone is sufficient, brake along the safe path.
- Else, if lateral space exists, combine braking and steering.
- Else, brake at maximum to reduce impact severity, then MRM.

**2. Trajectory.** Candidate quintic-spline trajectories are sampled each
tick. Infeasible or colliding ones are rejected and the lowest-cost survivor
is chosen [10]. The lateral profile `y(s) = w·(10s³ − 15s⁴ + 6s⁵)` is the
minimum-jerk path for the maneuver. Its peak lateral acceleration is
`5.77·w/T²`, which sets the shortest feasible maneuver time T.

**3. Tracking.** A speed-scheduled LQR lateral controller follows the evasive path.

### 4.7 Motion — Layer 4: Context-aware Minimal Risk Maneuver

Stopping in place can itself be hazardous, for example mid-intersection or in
dense traffic [13]. The MRM must continue essential driving tasks [14], so
Layer 4 selects its target state from context:

| State | When | Behaviour |
|---|---|---|
| `MRM_PULL_OVER` | A free, drivable, off-traffic area is reachable | Drive there slowly, then stop |
| `MRM_IN_LANE_STOP` | No pull-over area | Controlled deceleration ≤ 4 m/s², lane kept, hazard lights on |
| `MRM_CRAWL_CLEAR` | Stop point lies inside an intersection or merge zone | Crawl at ≤ 2 m/s until clear, then stop |

### 4.8 Control

- **Normal driving:** Pure Pursuit with a speed-scaled lookahead of `4 + 0.5v` m, and speed control that slows for curvature.
- **Evasive maneuvers:** LQR.
- **Emergency override:** cuts throttle and applies the brake while *keeping* steering, so the vehicle still tracks its avoidance path while braking.

### 4.9 Per-tick arbitration

```
every 50 ms:
    world  = safety_perception()
    params = supervisor.update(world, timing_health)        # Tier 3
    cont   = contingency.plan(world, params)                # Layer 2
    for plan in [latest_tfv6, cont, last_safe]:
        if monitor.check(plan, world, params):              # Layer 1
            last_safe = plan
            return pure_pursuit(plan, params.speed_cap)
    if evasive.collision_predicted(world):                  # Layer 3
        return lqr(evasive.best_quintic(world, params))
    return mrm.step(world)                                  # Layer 4
```

---

## 5. Adaptiveness

1. **Situation-dependent control.** The layer in control changes per tick with plan validity, timing and threat level.
2. **Scene-complexity supervisor (Tier 3).**
   - Complexity is computed from nearby object count, vulnerable-road-user count and class mix: `C = 0.4·min(N/20, 1) + 0.4·min(V/8, 1) + 0.2·H`.
   - C selects one of three regimes, with hysteresis between them.
   - The horizon and grid resolution move together, which keeps planning compute bounded in every regime.

   | Parameter | Sparse (highway merge) | Moderate | Dense (market, cattle) |
   |---|---|---|---|
   | Speed cap | 50 km/h | 30 km/h | 15 km/h |
   | Planning horizon / grid | 6 s / 1.0 m | 4 s / 0.5 m | 3 s / 0.25 m |
   | TTC caution / emergency | 3.0 / 1.5 s | 3.0 / 1.5 s | 4.0 / 2.0 s |
   | Max primary plan age | 500 ms | 400 ms | 300 ms |
   | Obstacle inflation | 1.0× | 1.0× | 1.25× |

3. **Latency-adaptive speed.** Speed is capped so the distance travelled during one latency period stays within budget: `v_cap = d_budget / t_latency`. If the primary planner misses 3 of its last 5 deadlines, the system switches to contingency-first mode.
4. **Class-adaptive risk.**
   - Per-class obstacle margins and motion-uncertainty spreads.
   - A dedicated `ANIMAL_ON_ROAD` mode: cattle get crawl-around or replanning rather than hard braking.
   - Stricter thresholds near pedestrians and two-wheelers.
5. **Physics-adaptive evasion.** The brake-versus-steer choice follows speed, road friction and available space (§4.6).
6. **Learned adaptation.** TFv6's attention re-weights the scene every frame. An optional learned cost head can refine obstacle costs in Layer 2.

---

## 6. Why TFv6 and not a Vision-Language-Action (VLA) model

**Accuracy on CARLA (Bench2Drive)** [1, 19]:

| Model | Type | Driving Score / Success Rate |
|---|---|---|
| **TFv6** | Learned planner | **95.0 / 84.3%** |
| SimLingo | VLA | 85.1 / 67.2% |
| ORION | VLA | 77.74 / 54.62% |

On Longest6 v2, a recent efficient-VLA method reports 36 Driving Score
against TFv6's 62 [18]. The same work finds that language can "greatly
improve or degrade performance" on the routes where it matters [18].

**Latency:**
- NVIDIA's Alpamayo-R1 VLA reports 99 ms per step on-vehicle [16]. The single-model TransFuser reports 27.6 ms on an RTX 3090 [2].
- Distillation work on VLAs notes that their "large vision-language backbones and reasoning modules introduce substantial inference latency" [17].

**Industry practice:**
- VLAs are deployed today in *supervised* driver-assistance systems (Li Auto, XPeng) [21, 22], where a human is the fallback.
- For driverless operation, Waymo states that VLMs "are too slow for real-time control" and "lack sufficient spatial awareness on their own" [20]. Waymo keeps them on the deliberative path, and a separate validation layer checks every trajectory. RAAP follows the same principle.

**Role of a VLA in RAAP:** an offline teacher for labelling Indian-road data
(nudge, yield, lateral maneuver). It is not the real-time planner.

---

## 7. Evaluation

### 7.1 Metrics

| Metric | Definition | Target |
|---|---|---|
| **Replanning latency** | Per-stage timing on every tick (fusion, tracking, prediction, drivable area, decision, planning, control), reported for the fast safety loop and the primary planner | Fast loop well under **100 ms**, where closed-loop success stays at 91.65–100% [15] |
| **Path smoothness** | Jerk (m/s³) from simulation-time derivatives, plus lateral acceleration and yaw rate | nuPlan comfort bounds [24]: \|jerk\| ≤ 8.37 m/s³, \|longitudinal jerk\| ≤ 4.13 m/s³, \|lateral acceleration\| ≤ 4.89 m/s², \|yaw rate\| ≤ 0.95 rad/s |
| **Scenario completion rate** | Goal reached without collision, over ≥ 3 seeds per scenario | Per-scenario rate |
| Collisions | Collision-sensor events | Zero |

**Smoothness by design:**
- a shared initial segment across contingency candidates
- replanning hysteresis
- path shortcutting and smoothing
- curvature-aware speed control
- minimum-jerk quintic evasion

Emergency maneuvers are reported separately from comfort scoring.

### 7.2 Results

**Setup:** CARLA 0.9.16, synchronous mode, 20 Hz.
- **Configuration evaluated:** Layer 0 (TFv6) with a TTC-based safety monitor, which performs Layer 1's collision check.
- **Baseline:** our classical-only pipeline (costmap A\* primary).

**Collision performance**

| Scenario | Classical-only baseline | **RAAP (TFv6 + safety monitor)** |
|---|---|---|
| Pedestrian jump-out | Collision contact ≈ 87% of run | **Collision-free** |
| Highway merge (PS scenario 3) | Collision contact ≈ 97% of run | **Collision-free** |
| Dense unsignaled traffic (PS scenario 2 profile) | Collision contact ≈ 60% of run | **Collision-free** |

**Replanning latency, mean per tick**

| Scenario | Deterministic safety loop | TFv6 primary (asynchronous) |
|---|---|---|
| Pedestrian jump-out | **5.5 ms** | 190 ms |
| Highway merge | **1.4 ms** | 324 ms |
| Dense unsignaled traffic | **7.1 ms** | 247 ms |

- The safety loop that governs actuation runs **14–70× faster** than the 100 ms reliability bound.
- The primary planner's longer inference is absorbed by the asynchronous design: its plans are verified against fresh perception every 50 ms.
- **Projected sensing-to-command latency** of the safety path is **≈ 52–58 ms** (50 ms sensor period + measured compute).

### 7.3 Evaluation plan (full submission)

1. **All five PS scenarios × ≥3 seeds**, reporting latency (mean and p95), jerk, completion rate and collisions.
2. **Layer ablation:**
   - TFv6 alone
   - \+ runtime monitor
   - \+ contingency planner
   - \+ evasive layer
   - \+ MRM (full RAAP)
3. **Layer utilisation:** share of ticks driven by each layer, and the distribution of rejection reasons (T/P/K/C).
4. **Latency stress test:** inject 100 / 150 / 200 ms delay into the primary planner and show the safety loop maintains collision-free driving.
5. **Perception variants:** 4-camera TFv6 and OSNet re-ID on the cattle-crossing and intersection scenarios.

---

## 8. Scenarios

Scenarios are authored in CARLA with scripted hazards and India-style traffic profiles.

| PS scenario | Implementation |
|---|---|
| 1. Unmarked village road | Town07. A half-on-road static obstruction and a wrong-way vehicle. **Detailed authored scene** |
| 2. Busy unsignaled urban intersection | Town03, all signals disabled. Aggressive, two-wheeler-heavy traffic with jaywalking. **Detailed authored scene** |
| 3. Highway merge with slow vehicles | Coordinate-free, generated at runtime from the map. A scripted slow lead vehicle forces an overtake decision |
| 4. Dense mixed-traffic market | Extra-dense, pedestrian-heavy aggressive traffic |
| 5. Sudden cattle crossing | A slow herd crossing, 1.2 m/s, labelled as livestock for perception and decision |

Every scenario runs through one command:
`python framework/run_scenario.py <scenario> --autopilot <agent>`

---

## 9. Technology Stack and MathWorks Mapping

| Layer | Implementation | MathWorks equivalent |
|---|---|---|
| Simulation | CARLA 0.9.16 (synchronous) | RoadRunner for scene authoring |
| Detection | YOLO (Ultralytics, PyTorch) | Deep Learning Toolbox |
| Tracking | Kalman filter + Hungarian assignment (+ OSNet) | Sensor Fusion & Tracking Toolbox (`trackingKF`, `trackerGNN`) |
| Primary planner | TFv6 via the PCLA harness | Called via `py.*` from a MATLAB Function block |
| Contingency planner | Costmap A\* / consensus-ADMM | Navigation Toolbox `plannerHybridAStar` |
| Evasive layer | Quintic sampling + collision validation | Navigation Toolbox `trajectoryOptimalFrenet` + `dynamicCapsuleList` [25] |
| Decision, MRM, watchdog | State machine with parallel supervisor | **Stateflow** (the chart is authored first, so the state machine maps one-to-one) |
| Control | Pure Pursuit / LQR | MPC Toolbox Path Following Control System |
| Metrics | Latency, jerk, completion, collisions | Simulink Data Inspector |

---

## 10. Novelty

1. **Runtime assurance for unstructured Indian traffic.** A top-ranked learned planner wrapped in verified, deterministic fallback layers, evaluated closed-loop on India-specific scenarios.
2. **Livestock- and vulnerable-road-user-aware safety layers.** Class-specific risk margins, uncertainty and TTC thresholds, plus a dedicated animal-on-road mode, all inside the deterministic safety logic.
3. **Measurement-driven fast/slow split.** Actuation latency is bounded by a millisecond-scale verified loop, independent of the learned planner's inference time.
4. **Physics-adaptive brake-versus-steer evasion.** A closed-form crossover speed parameterised by road friction, suited to mixed Indian road surfaces.
5. **Context-aware MRM for dense traffic.** Pull-over, in-lane stop and crawl-clear states instead of an unconditional stop.
6. **Explainable by construction.** Every tick records which layer drove the vehicle and why.

---

## 11. Development Status and Roadmap

| Completed | Next phase |
|---|---|
| TFv6 integration with TTC safety monitor, running live | Full Layer 1 monitor (four checks) + watchdog thread |
| Camera–LiDAR fusion, Kalman + Hungarian tracking, prediction, drivable area | Parallel contingency loop with last-safe buffer |
| Costmap A\* planner with hysteresis and smoothing | Evasive layer (quintic + LQR) |
| Mode state machine with safety supervisor | Context-aware MRM states and complexity supervisor |
| Pure Pursuit control with emergency override | OSNet re-ID and YOLO fine-tuning on IDD / DATS_2022 |
| Metrics harness (latency, jerk, completion, collisions) | Five scenarios × three seeds, ablation, latency stress test |
| Five PS scenarios authored | Stateflow / Simulink port of the decision layer |

---

## 12. References

1. Nguyen et al. *LEAD: Minimizing Learner-Expert Asymmetry in End-to-End Driving.* CVPR 2026. arXiv:2512.20563
2. Chitta et al. *TransFuser: Imitation with Transformer-Based Sensor Fusion for Autonomous Driving.* TPAMI. arXiv:2205.15997
3. Moller et al. *Towards Safe Autonomous Driving: A Real-Time Motion Planning Algorithm on Embedded Hardware.* arXiv:2601.03904
4. Zheng et al. *Safe and Real-Time Consistent Planning for Autonomous Vehicles in Partially Observed Environments via Parallel Consensus Optimization.* IEEE T-ITS, 2026. arXiv:2409.10310
5. Zheng et al. *Occlusion-Aware Contingency Safety-Critical Planning for Autonomous Driving.* IEEE T-Cybernetics. arXiv:2502.06359
6. Nyberg, Gautier, Tumova. *Hope for the Best, Prepare for the Worst: Occlusion-Aware Contingency Planning for Autonomous Vehicles.* arXiv:2607.03155
7. vom Dorff et al. *A Fail-safe Architecture for Automated Driving.* DATE 2020
8. Pek et al. *Using online verification to prevent autonomous vehicles from causing accidents.* Nature Machine Intelligence, 2020
9. Shalev-Shwartz et al. *On a Formal Model of Safe and Scalable Self-driving Cars (RSS).* arXiv:1708.06374
10. Lööf Wettervik, Mattsson. *Combined braking and steering maneuver for collision avoidance.* Chalmers University of Technology, 2025
11. Hac, Dickinson. *Collision avoidance with active steering and braking.* US Patent 7,016,783 B2
12. Arab et al. *Safety Verification for Evasive Collision Avoidance in Autonomous Vehicles with Enhanced Resolutions.* arXiv:2411.02706
13. Tandon et al. *When Stopping Fails: Rethinking Minimal Risk Conditions through Human-Interactive Autonomous Driving.* arXiv:2606.29115
14. Balakrishnan. *Functional Safety Concept of "Minimum Risk Maneuver" in Conditional Driving Automation (Level 3) Vehicles.* SAE 2022-28-0301
15. Weng, Yun. *Multi-Resolution End-to-End Deep Neural Network for Optimizing Latency-Accuracy Tradeoff in Autonomous Driving.* arXiv:2605.29138
16. NVIDIA. *Alpamayo-R1: Bridging Reasoning and Action Prediction for Generalizable Autonomous Driving in the Long Tail.* arXiv:2511.00088
17. Huang et al. *RT-VLA: Real-Time Vision-Language-Action Models via Knowledge Distillation.* arXiv:2606.14010
18. Ling et al. *BLUE: Toward Better Language Use in Efficient Vision-Language-Action Models for Autonomous Driving.* arXiv:2606.08684
19. Fu et al. *ORION: A Holistic End-to-End Autonomous Driving Framework by Vision-Language Instructed Action Generation.* ICCV 2025. arXiv:2503.19755
20. Waymo. *Demonstrably Safe AI for Autonomous Driving* (Dec 2025); *10 AI Lessons from Driving 200+ Million Fully Autonomous Miles* (Aug 2026)
21. Li Auto Inc. SEC Form 6-K filings, FY2025–26
22. Electrek. *Xpeng VLA 2.0 test drive*, April 2026
23. Zhou et al. *Omni-Scale Feature Learning for Person Re-Identification (OSNet).* ICCV 2019. arXiv:1905.00953
24. Motional. *nuPlan devkit — Metrics description* (comfort thresholds)
25. MathWorks. *Highway Trajectory Planning Using Frenet Reference Path*
