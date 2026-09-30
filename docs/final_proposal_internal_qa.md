# Final Proposal — SIH 2026 · PS 26037
## Runtime-Assured Adaptive Path Planning & Collision Avoidance for Unstructured Indian Roads

**Status of this document:** official proposal report. It supersedes
`docs/ideal-architecture.md`, `docs/combined-collision-avoidance-architecture.md`
and `docs/Collision_Avoidance_Architecture_Report.docx` where they differ.
Every number carries a provenance tag so that nothing can be mistaken for
more than it is:

| Tag | Meaning |
|---|---|
| **[M]** | Measured by us, in CARLA 0.9.16, with this repository's code |
| **[P]** | Published figure, from the cited source (verified against the source, 2026-09-30) |
| **[E]** | Our estimate or closed-form derivation. The derivation is shown and it is not a measurement |
| **[D]** | Design parameter we chose. It is tunable and not a result |

Claims that failed verification were removed or corrected; see
Appendix C (fact-check log).

---

## 1. Executive summary

Learned end-to-end planners are now the strongest drivers on open CARLA
benchmarks. TransFuser v6 (TFv6, from LEAD, CVPR 2026) reports **95.0 Driving
Score / 84.3% Success Rate on Bench2Drive [P]**. Two problems remain for
this problem statement. First, a single learned model has **no independent
backstop** when its output is wrong. Second, on our development hardware its
inference latency sits in the range where closed-loop driving is known to
collapse. We measured **190–324 ms mean per tick [M]**. The CARLA
latency study we build on reports **0–8.35% success at 200 ms delay [P]**.

Our solution keeps TFv6 as the primary planner and wraps it in a
**runtime-assured, adaptive, 3-tier / 5-layer architecture**. The safety
layers are **fully deterministic**: Kalman filtering, Hungarian assignment,
grid A\*, closed-form TTC/RSS checks, quintic-spline evasion and a
Stateflow-style state machine. No command reaches the actuators without
passing a deterministic check against the latest perception.

- **Tier 3 — Adaptive supervisor:** reads scene complexity and timing health, and tunes the parameters of Tiers 1–2 every tick.
- **Tier 2 — Deliberative planning:** Layer 0 (TFv6, learned) runs **in parallel** with Layer 2 (deterministic contingency planner). The contingency planner runs every tick, not only after a failure.
- **Tier 1 — Reactive safety:** Layer 1 (runtime monitor) accepts or rejects each plan. Layer 3 (evasive brake + steer) and Layer 4 (context-aware Minimal Risk Maneuver) sit below it.

**Evidence so far:**
- In three live scenarios, the TFv6-based agent with an independent TTC backstop had **0 collision events [M]**. Our earlier modular A\* pipeline had **601–1,212 collision-contact ticks per run [M]**.
- The deterministic stack that Layers 1–4 run on measured **1.4–7.1 ms mean per tick [M]**. That is 14–70× under the 100 ms "safe" band [P].

---

## 2. Problem statement and requirement traceability

PS 26037 asks for a simulated, closed-loop driving system for **unstructured
Indian roads**. These roads have mixed traffic (cars, buses, trucks, auto-rickshaws,
two-wheelers, bicycles, pedestrians, pushcarts, animals), missing lane
markings, informal merging and sudden direction changes. The system needs
multi-sensor perception (camera + LiDAR, radar optional), short-term
prediction of irregular agents, collision-free real-time replanning, and
validation on ≥5 scenarios. It must report **replanning latency, path
smoothness and scenario completion rate**, and deliver a report, a video
and ≥2 authored scenes. The pipeline is expected in MATLAB/Simulink.

| PS requirement | How this proposal meets it | Section |
|---|---|---|
| Multi-sensor perception | TFv6 fuses camera + LiDAR + radar [P]. The independent safety chain fuses camera detections with LiDAR by classical projection | §4.1, §4.2 |
| Diverse, non-lane-following agents incl. animals | Per-class risk tables, a distinct `ANIMAL_ON_ROAD` mode, OSNet appearance re-ID in the tracker | §4.1, §6 |
| Short-term prediction of irregular motion | Kalman constant-velocity model with a per-class lateral-uncertainty fan (3 modes). TFv6 predicts implicitly. A learned MoFlow predictor is wired but gated on a checkpoint | §4.1 |
| Collision-free path, real-time replanning | L0 ∥ L2 planning, every candidate checked by L1 at 20 Hz, L3/L4 below | §4 |
| **Replanning latency** | Per-stage `TickTimings` on every tick. Reported separately for the fast (safety) loop and the slow (TFv6) loop | §9.2 |
| **Path smoothness** | Jerk from simulation-time derivatives (m/s³), scored against published comfort thresholds | §9.1 |
| **Scenario completion rate** | Goal-radius completion plus collision sensor, ≥3 seeds per scenario | §9.1, §9.4 |
| 5 scenarios | Village road, unsignaled intersection, highway merge, dense market and cattle crossing are all authored. 3 still need live coordinate capture | §11 |
| MATLAB/Simulink | Toolbox mapping for every layer (Stateflow, Navigation, Sensor Fusion & Tracking, MPC) | §11.2 |

---

## 3. Solution overview

### 3.1 Architecture

```
  SENSORS (CARLA 0.9.16, synchronous, dt = 50 ms)
  Camera(s) + LiDAR (+ radar for TFv6)
        │
        ├────────────────────────────────────────────┐
        ▼                                            ▼
  TFv6 internal perception                  SAFETY PERCEPTION CHAIN (deterministic core)
  (inside Layer 0)                          YOLO 2D det → LiDAR projection fusion →
                                            Kalman CV + Hungarian (+ OSNet appearance) →
                                            per-class uncertainty fan; drivable-area map
        │                                            │
        │      ┌─────────────────────────────────────┴───────────────────────────┐
        │      │ TIER 3 — ADAPTIVE SUPERVISOR  (every tick)                       │
        │      │ scene complexity C ∈ [0,1] + timing health (m-of-k deadline misses)│
        │      │ → speed cap, horizons, grid resolution, TTC thresholds, max plan age│
        │      └───────────────┬───────────────────────────────┬──────────────────┘
        ▼                      ▼                               ▼
  ┌──────────────── TIER 2 — DELIBERATIVE PLANNING (parallel, not sequential) ─────┐
  │  LAYER 0  Primary: TFv6 (learned)       LAYER 2  Contingency (deterministic)     │
  │  async "slow loop"; publishes a plan    every fast tick: short-horizon costmap   │
  │  whenever inference completes           A* (implemented) → CPTO consensus-ADMM   │
  │                                         (upgrade); keeps last verified safe plan │
  └────────────────┬──────────────────────────────────────┬────────────────────────┘
                   ▼  candidates, priority order:  [TFv6 plan, contingency plan, last-safe plan]
  ┌──────────────── TIER 1 — REACTIVE SAFETY (deterministic, every tick) ──────────┐
  │  LAYER 1  Runtime monitor: T timing · P plausibility · K kinematics · C collision│
  │           first candidate passing all four checks is executed                    │
  │  LAYER 3  Evasive maneuver: TTC/RSS threat → closed-form brake-vs-steer →        │
  │           quintic-spline sampling → LQR tracking (only if NO candidate passes)   │
  │  LAYER 4  Minimal Risk Maneuver: context-aware pull-over / in-lane stop / crawl  │
  │  WATCHDOG (own thread): fast tick >150 ms → speed cap; >200 ms → direct MRM brake│
  └────────────────────────────────────────┬────────────────────────────────────────┘
                                           ▼
                        CONTROL: Pure Pursuit + P speed (normal, MRM)
                                 LQR lateral (evasive only)
                                           ▼
                                  CARLA vehicle actuation
```

### 3.2 Two loops, deliberately decoupled ("think fast / think slow")

| Loop | Rate | Contents | Measured cost |
|---|---|---|---|
| **Fast (safety) loop** | 20 Hz, every 50 ms tick [D] | Safety perception chain, Tier 3, Layer 2, Layer 1, Layer 3/4, control | 1.4–7.1 ms mean/tick for the deterministic stack [M] |
| **Slow (primary) loop** | Asynchronous. Publishes when inference finishes | TFv6 inference | 190–324 ms mean on dev hardware [M]. TransFuser reference: 27.6 ms single model on RTX 3090 [P] |

TFv6's latency is **not in the actuation path**. Its latest plan is only a
*candidate*, re-checked every 50 ms against the **current** world by
Layer 1. This matches the split the leading L4 operator describes publicly.
Waymo pairs fast real-time processing with slower deliberative reasoning,
plus "a separate… safety system that monitors every trajectory… against hard
physics-based constraints" [P].

### 3.3 How the 5 layers map onto the 3 tiers

| Tier | Layers | Nature |
|---|---|---|
| Tier 3 — Adaptive supervisor | (cross-cutting) | Deterministic rules |
| Tier 2 — Deliberative planning | L0 TFv6 ∥ L2 contingency | L0 learned, L2 deterministic |
| Tier 1 — Reactive safety | L1 monitor → L3 evasive → L4 MRM | Deterministic |

---

## 4. Methodology — layer by layer

### 4.1 Safety perception chain (feeds Tiers 1–3; independent of TFv6)

All components below except OSNet and YOLO are implemented in `pipeline/`.

| Stage | Method (deterministic unless stated) | Key parameters |
|---|---|---|
| 2D detection | YOLO (v8 or v11, **version to be fixed**; v12 carries an AGPL-3.0 risk) fine-tuned on IDD + DATS_2022. *Learned.* CARLA ground-truth boxes stand in until fine-tuning completes | — |
| Camera–LiDAR fusion | Project LiDAR points into the image and take the **median 3D point** inside each box. Needs ≥3 points, otherwise fusion is marked failed | `perception_fusion.py` |
| Tracking — motion | **Kalman filter**, constant-velocity model, state `[x, y, vx, vy]` | P₀×10, R×0.5, Q×0.1 [D] |
| Tracking — association | **Hungarian algorithm** (`linear_sum_assignment`) with a Euclidean gate. Tracks are dropped after 5 missed ticks; history holds 12 points | gate 4 m [D] |
| Tracking — appearance (new) | **OSNet** re-ID embedding. Association cost = λ·motion + (1−λ)·cosine appearance distance, as in DeepSORT/StrongSORT. *Learned feature, deterministic matching* | §6.3 |
| Prediction | Straight CV extrapolation (p = 0.6) plus two lateral modes (p = 0.2 each) whose spread grows to a per-class σ at the horizon (pedestrian 1.2 m, animal 0.8 m, two-wheeler 0.6 m, car 0.3 m). Straight-only if history < 3 points | `predictor.py` [D] |
| Drivable area | Project LiDAR points onto a semantic mask to label road vs non-road. CARLA ground-truth segmentation is a disclosed sim stand-in for a SegFormer-class model | `drivable_area.py` |

This chain is **independent of TFv6's own perception**. A TFv6 perception
miss therefore does not also blind the monitor. Production safety monitors
use diverse, separate sensing for exactly this reason. Our current SafetyEnvelope
uses CARLA ground truth as a sim-only stand-in, and says so.

### 4.2 Layer 0 — Primary planner: TransFuser v6

- **What it is:** TFv6 is the model from *LEAD: Minimizing Learner-Expert Asymmetry in End-to-End Driving* (Nguyen, Fauth, Jaeger, Dauner, Igl, Geiger, Chitta; CVPR 2026). TransFuser fuses camera and LiDAR features with transformer attention across modalities. LEAD's contribution is fixing asymmetries between what the expert that generated training data could see and know, and what the student model observes: visibility, uncertainty and navigational intent [P].
- **Sensors:** LEAD's best Bench2Drive configuration uses camera + LiDAR + radar [P]. Our PCLA wrapper exposes `tfv6_regnet` (default) and a `tfv6_4cameras` variant, among others. Keys are per our autopilot's docstring; verify them against PCLA's `agents.json`.
- **Published performance:**
  - Bench2Drive: **95.0 ± 0.7 DS / 84.3 ± 2.1 SR** [P]
  - Longest6 v2: **62 ± 1 DS / 91 ± 1 RC** [P]
  - LEAD does **not** publish TFv6 inference latency. The TransFuser paper reports **27.6 ms** single-model and **59.6 ms** for a 3-model ensemble on an RTX 3090 [P].
- **Known weakness this architecture targets:** the original TransFuser has "around 9× more vehicle collisions per kilometer than the expert". Collisions happen "primarily … during unprotected turns and lane changes" [P]. Its hand-tuned creeping heuristic cost "nearly 10 points in DS on the leaderboard" because it did not generalize [P]. So we add *external, physics-derived* layers rather than another heuristic inside the model.
- **Integration:** runs asynchronously (§3.2). Its output is a candidate plan, never a direct command.

### 4.3 Layer 1 — Runtime monitor: *how it checks* (Q1)

**Role:** a passive accept/reject gate. Moller et al. define Online
Verification as "an independent module [that] continuously checks the
feasibility and timing of planning outputs". They also note these
mechanisms "often remain passive" and must be "complemented by an active
safety component capable of providing a feasible fallback solution" [P].
That is why Layer 1 never stands alone: Layers 2–4 always have an answer
ready.

Every fast tick, each candidate (TFv6 plan → contingency plan → last-safe
plan) passes through four deterministic checks, in increasing order of cost.
The **first candidate that passes all four is executed**.

**T — Timing / freshness**
- `age = t_now − t_sensor(plan)` must be ≤ `max_plan_age`. This is 500 ms in sparse scenes and 300 ms in dense ones, set by the supervisor [D].
- The plan's remaining horizon must be ≥ 2× the producer's recent compute time. This follows vom Dorff et al.: the executed trajectory duration "should be at least twice as long as the time needed to acquire new data and calculate a new trajectory" [P].
- Deadline bookkeeping: a TFv6 inference > 150 ms is a *soft miss* and > 200 ms a *hard miss*. The last k = 5 outcomes feed the supervisor's m-of-k health signal (weakly-hard timing) [D].
- An independent **watchdog thread** checks the fast loop itself. Above 150 ms the speed cap drops. Above 200 ms, or if the process dies, the watchdog issues the Layer 4 braking command directly without waiting for the pipeline.

**P — Plausibility**
- All values are finite (no NaN/Inf).
- The plan has ≥ 2 waypoints.
- The first waypoint lies within ε = 1.0 m of the current ego pose after re-anchoring by odometry [D].
- Consecutive spacing is ≤ `v_max·Δt + margin`.
- Target speed is within [0, speed cap].

**K — Kinematic feasibility (bicycle model)**
- Discrete curvature at each interior waypoint uses the Menger formula: `κᵢ = 4·Area(pᵢ₋₁, pᵢ, pᵢ₊₁) / (|a|·|b|·|c|)`.
- Steering geometry: `|κᵢ| ≤ tan(δ_max)/L`, where `L` is the wheelbase.
- Lateral acceleration: `vᵢ²·|κᵢ| ≤ a_lat,max = η·μ·g`, with a headroom factor η = 0.8 [D].
- Longitudinal: `a_min ≤ Δvᵢ/Δtᵢ ≤ a_max`. Braking is bounded by ≈8 m/s² on dry asphalt, the value vom Dorff et al. use [P].
- Steering rate: `|Δδ/Δt| ≤ δ̇_max`, where `δ = atan(L·κ)`.

**C — Collision and safe distance (against the *latest* safety perception)**
- The ego footprint is approximated by 3 discs along its long axis. At each plan timestamp tⱼ, every tracked object's Kalman-predicted position is inflated by `r_obj + margin + σ_class(tⱼ)`. The plan passes if every ego disc is clear of every inflated object at every tⱼ. This follows the online-verification approach of checking intended trajectories against predicted occupancy of other road users (Pek et al., *Nature Machine Intelligence* 2020) [P].
- Minimum TTC along the plan must be ≥ `ttc_emergency` (1.5 s implemented; the supervisor raises it for vulnerable road users).
- **RSS longitudinal safe distance** to the in-path lead object (Shalev-Shwartz et al., 2017) [P]:
  `d_min = [ v_r·ρ + ½·a_acc,max·ρ² + (v_r + ρ·a_acc,max)² / (2·a_brake,min) − v_f² / (2·a_brake,max) ]₊`
  where ρ is the response time, `v_r` the rear (ego) speed and `v_f` the front (lead) speed.
- Non-drivable cells from the drivable-area map count as obstacles.

**Cost:** about `3 discs × N_waypoints × N_objects` distance tests. For 20
waypoints × 50 objects that is ≈3,000 vector operations, expected well under 1 ms in NumPy [E]. It must be measured once built (§11).

**Outcome:** `ACCEPT(plan)` or `REJECT(reason ∈ {T, P, K, C})`. The reason is logged every tick, so each fallback is explainable.

### 4.4 Layer 2 — Contingency planner (parallel, every tick)

**Why parallel:** in vom Dorff et al.'s worked example (50 km/h, 200 ms dead
time, 8.6 m/s² braking), an emergency full stop covers about **14 m "blindly"**,
with no further ability to steer the outcome. They call this "not acceptable
for fail-safe behavior" and propose duplicated Observer/Planner/Buffer pairs
(A and B) feeding a **Decider** [P]. A fallback computed only *after* the
primary fails is already late. Our Layer 2 is planner B and Layer 1 is the Decider.

**Implementation:**
1. **Now (implemented):** a short-horizon, conservative-speed run of our costmap planner (`pipeline/planner.py`):
   - soft Gaussian inflation around predicted obstacle points, with per-class radii (pedestrian or animal 2.0 m, two-wheeler or pushcart 1.75 m, car/bus/truck 1.5 m)
   - non-drivable cells rasterized as high cost
   - **8-connected grid A\***, Euclidean heuristic, step cost = distance + 50 × cell cost
   - line-of-sight shortcutting, 0.25/0.5/0.25 smoothing, and decimation to 1.5 m waypoint spacing
   - signature hysteresis: replan only on >15% cost change and ≥3 ticks since the last replan
2. **Upgrade (literature-backed):** **CPTO**, consistent parallel trajectory optimization (Zheng et al., *IEEE T-ITS* 2026) [P]:
   - uses consensus ADMM to solve several candidate trajectories in parallel as low-dimensional QPs with discrete-time barrier-function safety constraints
   - all candidates "share a common segment and diverge at a specific divergent point", so the vehicle does not visibly switch strategies from tick to tick
   - maximum optimization time "stabilizes at approximately 40 ms once the number of considered obstacles exceeds four"
   - as low as 1.22 ms for two trajectories [P]
   - the occlusion-aware extension adds reachable sets of phantom vehicles hidden behind occluders (Zheng et al., *IEEE T-Cybernetics*) [P]
3. **Always-available fallback:** we keep the last plan that passed Layer 1, following Nyberg, Gautier & Tumova (2026). If the new computation fails, the system "proceeds to the safe fallback trajectory determined in timestep t−1", so under their reachability assumptions "a safe trajectory always exists" [P].

**Honest caveat:** as a *standalone driver*, our A\* pipeline logged
601–1,212 collision-contact ticks per run [M] (§9.2). In this architecture
it (a) drives only when TFv6's plan is rejected, (b) is itself checked by
Layer 1 before execution, and (c) has Layers 3–4 below it. Its standalone
collision root cause is still under investigation (§12).

### 4.5 Layer 3 — Evasive maneuver (brake **and** steer)

**Engages when:** no candidate passes Layer 1 *and* a collision is predicted
within the braking horizon.

**Step 1 — Threat assessment (every tick):** per-object TTC, the RSS `d_min`,
and the free lateral space `w_free`, taken from the occupancy and drivable-area maps.

**Step 2 — Closed-form brake-vs-steer decision [E].** This uses a
first-order point-mass model with friction μ, dead time `t_d` and required
lateral offset `w`:
- Distance needed to stop: `d_brake = v·t_d + v² / (2μg)`
- Distance needed to clear the obstacle laterally: `d_steer = v·t_d + v·√(2w / (μg))`
- Steering needs less distance than braking when **v > v\* = 2·√(2·w·μ·g)**. The dead time cancels out.

| Surface (assumed μ) | v\* for w = 1.8 m |
|---|---|
| Dry asphalt, μ ≈ 0.8 | 10.6 m/s ≈ **38 km/h** |
| Wet / dusty, μ ≈ 0.5 | 8.4 m/s ≈ **30 km/h** |
| Loose gravel / unpaved, μ ≈ 0.3 | 6.5 m/s ≈ **23 km/h** |
| Ice, μ ≈ 0.15 | 4.6 m/s ≈ **17 km/h** |

Decision rule:
- If `d_available ≥ d_brake`: brake along the last safe path.
- Else, if `w_free ≥ w` and `d_available ≥ d_steer`: combined steer + brake inside the friction circle.
- Else: maximum braking to reduce impact severity, then Layer 4.

This is the decision logic of the Delphi patent US7016783B2: an emergency
lane change is performed when a collision "cannot be prevented by braking
only, but can be prevented by steering … and braking" [P].
*Indian-road note:* μ, `w` (which depends on class, e.g. a narrower
two-wheeler) and the maneuver limits must be calibrated for local surfaces.
The table above is illustrative, not calibrated.

**Step 3 — Trajectory generation (quintic splines):**
- Sample several quintic trajectories each tick, varying the maneuver duration T, the offset w and the deceleration. Reject any that are dynamically infeasible or colliding, then pick the lowest-cost survivor. This is the Spline-based algorithm of Lööf Wettervik & Mattsson (Chalmers/Aptiv, 2025), which "samples several quintic spline trajectories every time step" and selects "the safest trajectory … with a cost function" [P].
- With zero initial lateral velocity and acceleration, the lateral profile is `y(s) = w·(10s³ − 15s⁴ + 6s⁵)`, where `s = t/T`. This is the minimum-jerk profile for these boundary conditions.
- Its peak lateral acceleration is `5.77·w/T²` [E]. Feasibility therefore requires `T ≥ √(5.77·w / (η·μ·g))`. For example, w = 1.8 m, μ = 0.8, η = 0.8 gives T ≥ 1.29 s [E].
- The thesis also found that its rule-based and spline-based variants "in some test scenarios choose different combinations of braking and steering". The right split is scenario-dependent, which is why we sample rather than fix a threshold [P].

**Step 4 — Tracking:** an LQR lateral controller on the error states (lateral error, heading error and their rates). Gains are gain-scheduled by speed and computed offline, so the online cost is one matrix-vector product per tick. Pure Pursuit stays in charge of normal driving.

### 4.6 Layer 4 — Context-aware Minimal Risk Maneuver

**Why not "just stop":** Tandon et al. (2026) show that "stopping alone does
not guarantee safe integration into human-governed roadway systems"; stopped
AVs can obstruct traffic and emergency operations [P]. SAE 2022-28-0301
(Balakrishnan, presented at SAE India, Bangalore) frames the MRM as a set of
*Dynamic Driving Tasks that must still be performed* [P]. This matters even
more in dense Indian traffic, where a sudden stop invites rear-end and
two-wheeler conflicts.

**States, chosen deterministically [D]:**
- `MRM_PULL_OVER`: a free, drivable, off-traffic region is reachable. Track to it with Pure Pursuit at reduced speed, then stop.
- `MRM_IN_LANE_STOP`: no pull-over is available. Decelerate in a controlled way at ≤ 4 m/s² (within the nuPlan comfort bound of −4.05 m/s² [P]), keep the lane along the last safe path, and turn on hazard lights.
- `MRM_CRAWL_CLEAR`: the stop point would lie inside an intersection or merge conflict zone. Crawl at ≤ 2 m/s along the last safe path until clear, then switch to the in-lane stop.

**Exit:** return to normal driving only after sustained clearance. We reuse
the implemented 20-tick gate (≈1 s at 20 Hz) and also require all four
Layer-1 checks to be healthy.

### 4.7 Control and the mode state machine

- **Control:**
  - Pure Pursuit with lookahead `4.0 + 0.5·v` m, curvature law `κ = 2y/L²`, and a P speed controller that slows for curvature (`controller.py`, implemented).
  - LQR during evasive maneuvers only.
  - Emergency override: throttle 0 and brake 1 while *preserving steering*. This is implemented, both in `pipeline.py` and in the SafetyEnvelope.
- **State machine** (extends `decision_logic.py`; written chart-first so it redraws 1:1 in Stateflow):
  - `NORMAL_DRIVE` (L0 plan accepted)
  - `OBSTACLE_DETECTED` or `ANIMAL_ON_ROAD` (debounced 3 ticks; L2 favored, VRU thresholds)
  - `EVASIVE_MANEUVER` (L3)
  - `EMERGENCY_BRAKE ∥ REPLAN` (parallel states)
  - `MRM` (L4)
  - `RESUME` (sustained clearance, history return)
  - A parallel `SAFETY_SUPERVISOR` (the watchdog) can force `EMERGENCY_BRAKE`/`MRM` from any state.

### 4.8 Per-tick arbitration (pseudo-code)

```
every fast tick (50 ms):
    world  = safety_perception()                       # §4.1
    params = supervisor.update(world, timing_health)   # Tier 3, §5
    cont   = contingency.plan(world, params)           # Layer 2
    for plan in [latest_tfv6(max_age=params.max_age), cont, last_safe]:
        ok, reason = monitor.check(plan, world, params)   # Layer 1: T, P, K, C
        log(reason)
        if ok:
            last_safe = plan
            return pure_pursuit.track(plan, params.speed_cap)
    if evasive.collision_predicted(world):             # Layer 3
        return lqr.track(evasive.best_quintic(world, params))
    return mrm.step(world)                             # Layer 4

watchdog thread (independent):
    fast tick > 150 ms  -> speed_cap *= 0.5
    fast tick > 200 ms or loop dead -> send MRM brake directly
```

---

## 5. Adaptiveness — how the system adapts (Q2)

Adaptation happens at six levels, all deterministic and explainable except the last.

1. **Per-tick layer arbitration.** The controlling layer changes with the situation: TFv6 when its plan is valid and fresh, the contingency planner when it is not, evasive maneuvers when a collision is imminent, and MRM when nothing is valid. Every switch has a logged reason (T/P/K/C).

2. **Scene-complexity supervisor (Tier 3).** Every tick, over objects within 30 m [D]:
   - N = number of tracked objects
   - V = number of vulnerable road users (pedestrians, animals, two-wheelers, bicycles, pushcarts)
   - H = normalized class-mix entropy
   - `C = 0.4·min(N/20, 1) + 0.4·min(V/8, 1) + 0.2·H`, a value in [0, 1] [D]
   - Regimes: sparse when C < 0.3, moderate when 0.3–0.6, dense when > 0.6. Enter/exit thresholds differ by ±0.05 so the regime does not chatter [D].

   Initial parameter settings, to be tuned [D]:

   | Parameter | Sparse (e.g. highway merge) | Moderate | Dense (e.g. market, cattle) |
   |---|---|---|---|
   | Speed cap | 50 km/h | 30 km/h | 15 km/h |
   | Contingency horizon / grid resolution | 6 s / 1.0 m | 4 s / 0.5 m | 3 s / 0.25 m |
   | TTC obstacle / emergency | 3.0 / 1.5 s | 3.0 / 1.5 s | 4.0 / 2.0 s |
   | Max TFv6 plan age | 500 ms | 400 ms | 300 ms |
   | Inflation scale | 1.0× | 1.0× | 1.25× |

   The horizon and resolution move together, so the A\* search space stays bounded by construction [E]:
   - Sparse: ~100 m ahead at 1.0 m resolution → ≈10⁴ cells
   - Dense: ~12.5 m ahead at 0.25 m resolution → ≈2.5×10³ cells

3. **Latency-adaptive speed.** The distance travelled during one latency period, `v·t_lat`, is capped at `d_budget` (e.g. 3 m [D]), so `v_cap = d_budget / t_lat,p95`. This ties speed directly to measured timing health. If TFv6 misses ≥ 3 of its last 5 deadlines, the supervisor also shifts to contingency-first mode.

4. **Class-adaptive risk.**
   - Per-class inflation radii and lateral-uncertainty fans (implemented).
   - A dedicated `ANIMAL_ON_ROAD` mode, because the PS names cattle crossing. Animals typically warrant crawl-around or replanning rather than a hard stop.
   - Stricter TTC thresholds when VRUs are present.

5. **Physics-adaptive evasion.** The brake-vs-steer choice depends on speed, friction and available lateral space (v\* in §4.5), not on a fixed rule.

6. **Learned adaptation (Layer 0).** TFv6's attention re-weights the scene every frame. An optional learned cost head (a 7→32→16→2 MLP in `cost_head.py`) can shape Layer 2's inflation and danger costs. Without weights it falls back exactly to the hand-tuned table.

---

## 6. How the design reduces collisions (Q3)

Each layer catches a different failure class.

| Failure | Caught by | Mechanism |
|---|---|---|
| Normal driving errors of a heuristic planner | L0 TFv6 | Learned multimodal planner, top Bench2Drive score [P]. In our runs: 0 collision events vs 601–1,212 contact ticks for the old A\* pipeline [M] |
| Learned plan is wrong (e.g. unprotected turns and lane changes, TransFuser's documented weak spot [P]) | L1 collision check vs independent perception | TFv6's plan is rejected before execution |
| Plan is late (TFv6 at 190–324 ms [M]) | L1 timing check + L2 in parallel | A fresh deterministic alternative exists every 50 ms, with no waiting |
| No alternative planner output | L2 last-safe buffer | The previously verified plan is always executable [P] |
| Braking physically cannot avoid impact | L3 evasive steering | Above v\*, steering needs less distance than braking [E], per the Delphi patent logic [P] |
| Everything above fails | L4 MRM | Controlled, context-aware safe state instead of an unconditional stop |
| Pipeline stall or crash | Watchdog | Independent thread issues MRM braking |

### 6.1 Camera coverage (TransFuser)

LEAD's best Bench2Drive result uses a wide camera configuration + LiDAR + radar [P].
Wider coverage matters in Indian traffic, where two-wheelers filter
past on both sides and animals enter from the roadside. We will run the
`tfv6_4cameras` variant on the side-approach scenarios (intersection,
cattle crossing) and A/B test it against `tfv6_regnet`. We claim no number
until that has been measured.

### 6.2 Independent perception for the safety layers

Layers 1–4 consume their own camera–LiDAR fusion and Kalman/Hungarian
tracks, not TFv6's internals. A perception miss inside TFv6 therefore does
not silently disable the checks that would catch its consequences.

### 6.3 OSNet re-identification in the tracker

- **What it is:** OSNet (Zhou, Yang, Cavallaro, Xiang; ICCV 2019) is a lightweight re-ID CNN that learns features at multiple scales. It is available as plug-in appearance weights in the StrongSORT and BoxMOT trackers [P].
- **Why it helps:** our tracker associates on motion distance only. That is its documented weak point in dense, crossing traffic, where tracks can swap identities. Adding appearance cost to the Hungarian step reduces ID switches.
- **Collision mechanism:** fewer ID switches mean Kalman velocities don't reset. That keeps TTC, RSS distances and predicted occupancy accurate, which drives both false alarms and missed hazards in Layers 1 and 3.
- **Caveats:**
  - Public OSNet weights are trained for *person* re-ID. Vehicles and animals need vehicle re-ID weights or fine-tuning.
  - It adds one small CNN pass per detection crop, which must be profiled against the fast-loop budget.
  - No measured gain yet.

---

## 7. Why not a VLA as the primary planner (Q4) — cross-verified

**1. Latency.**
- NVIDIA's Alpamayo-R1 VLA reports **99 ms** latency in on-vehicle tests [P]. The paper text we accessed does not name the hardware. That is 3.6× the published TransFuser single-model time (27.6 ms, RTX 3090 [P]); the hardware differs, so this is indicative only.
- A distillation study built on SimLingo states that VLA "large vision-language backbones and reasoning modules introduce substantial inference latency and thereby prevent their deployment" [P]. Its student needed a **44.8× (vision-only) / 7.9× (vision + language)** speed-up over SimLingo [P].
- Our own TFv6 already measures 190–324 ms on dev hardware [M]. A multi-billion-parameter VLA would be slower still on the same machine [E].

**2. Accuracy on the benchmark our PS runs on (CARLA / Bench2Drive).**

| Model | Type | Bench2Drive DS / SR |
|---|---|---|
| TFv6 (LEAD) | Learned, not VLA | **95.0 / 84.3** [P] |
| HiP-AD | Not VLA | 86.8 / 69.1 [P] |
| SimLingo | VLA; CARLA challenge-winning family | 85.1 / 67.2 [P] |
| ORION | VLA, ICCV 2025 | 77.74 / 54.62 [P] |

The BLUE VLA-efficiency work reports **76.2% SR** on Bench2Drive and
**36 DS on Longest6 v2**, against TFv6's 62 [P]. BLUE also found that
language "matters on only a small fraction of routes, but on those routes
it can greatly improve or degrade performance" [P]. That is an extra failure
mode for no average gain.

**3. Industry — the honest picture.** VLAs **are** in production in 2025–26,
so we do **not** claim otherwise:
- Li Auto began full-scale deployment of its "VLA Driver" model in September 2025 [P].
- XPeng rolled out VLA 2.0 in China in March 2026, with Volkswagen as its first external customer [P].

However:
- These are **supervised driver-assistance** systems with a human as the fallback. This PS requires autonomous collision avoidance with no human in the loop.
- The driverless leader, Waymo, says VLMs "are too slow for real-time control" and "lack sufficient spatial awareness on their own" [P].
- Waymo uses them for deliberative reasoning. Real-time control runs on a fast path, and a **separate onboard validation layer** checks every trajectory "against hard physics-based constraints" [P]. That is structurally our architecture: fast learned planner, deterministic monitor, fallback layers.

**4. Open-source reproducibility.**
- Alpamayo-R1 weights are public (a 10B checkpoint on Hugging Face) [P]. It is evaluated in NVIDIA's own closed-loop simulation and road tests, not on CARLA or Bench2Drive [P], so there is no like-for-like number to check.
- TFv6 is open (MIT), CARLA-evaluated, and already running live in our harness [M].

**Where a VLA still helps:** as an **offline teacher or labeler**. Its
reasoning taxonomy (nudge, yield, lateral maneuver) is a template for
labeling Indian-road data. It is not the runtime planner.

---

## 8. Novelty

Scoped to what our literature search supports:

1. **A runtime-assured learned planner for unstructured Indian traffic in closed loop.** We found no prior CARLA agent evaluated on the PS's five Indian-road scenarios. We also found no work combining a Bench2Drive-leading learned planner with a parallel contingency planner, evasive maneuvering and a context-aware MRM for this setting.
2. **VRU- and livestock-aware deterministic safety.** Per-class risk (inflation, uncertainty, TTC) and a dedicated `ANIMAL_ON_ROAD` mode sit in the *safety* layers, not only in a learned model.
3. **A measurement-driven fast/slow split.** The actuation path's latency is set by a measured deterministic loop (1.4–7.1 ms [M]). The learned planner's measured latency (190–324 ms [M]) is consumed asynchronously and verified against current perception.
4. **Friction- and speed-adaptive brake-vs-steer decisions**, with a closed-form crossover (§4.5) to be calibrated for Indian surfaces.
5. **An MRM designed for Indian traffic density.** Crawl-clear and pull-over states replace the unconditional stop that current MRC definitions default to.
6. **Explainability by construction.** Every tick logs which layer drove and why (T/P/K/C reason codes, supervisor regime).

---

## 9. Evaluation

### 9.1 Metrics (as the PS names them) and targets

| PS metric | Definition in our harness (`pipeline/metrics.py`) | Target |
|---|---|---|
| **Replanning latency** | Per-stage `TickTimings` (fusion, tracking, prediction, drivable area, decision, planning, control), total ms per tick. Warm-up ticks are excluded. Reported separately for the fast loop (sensing → validated command) and the slow loop (TFv6 inference) | Fast loop: mean and p95 well under **100 ms**. Weng & Yun report 91.65–100% success at ≤100 ms delay and 0–8.35% at 200 ms [P] |
| **Path smoothness** | Jerk `‖Δa‖/dt` in m/s³. `dt` is the **fixed simulation step**, not wall-clock (a bug we caught and fixed), plus lateral acceleration and yaw rate | nuPlan comfort thresholds [P]: \|jerk\| ≤ 8.37 m/s³, \|longitudinal jerk\| ≤ 4.13 m/s³, \|lateral acceleration\| ≤ 4.89 m/s², longitudinal acceleration ∈ [−4.05, 2.40] m/s², \|yaw rate\| ≤ 0.95 rad/s. Evasive/MRM ticks are reported separately, since emergency maneuvers are exempt from comfort |
| **Completion rate** | Reached the goal radius without a disqualifying collision, over ≥3 seeds per scenario | Report per scenario ± spread |
| Collisions | Collision-sensor events per tick | 0 |

### 9.2 Results to date [M]

**Setup:** CARLA 0.9.16 in synchronous mode, three live scenarios. The
seed count and GPU model were not recorded in the source table; both must
be added (§9.4).

**Scenario mapping:**
- `highway_merge` → PS scenario 3.
- `traffic_stress` → the traffic profile reused by PS scenario 2 (unsignaled intersection).
- `pedestrian_jumpout` → the sudden-hazard pattern behind PS scenario 5 and unmarked pedestrian crossings.

**Collision events per run [M]**

| Scenario | Old modular A\* pipeline | **TFv6 + independent TTC backstop** | CARLA BehaviorAgent (privileged reference) |
|---|---|---|---|
| pedestrian_jumpout | ~1,212 | **0** | 0 |
| highway_merge | ~948 | **0** | 0 |
| traffic_stress | ~601 | **0** | 0 |

How to read this table:
- **What "collisions" counts:** the number of ticks carrying a collision-sensor event, which measures *contact duration*, not separate crashes. At dt = 0.05 s, 1,212 ticks ≈ 61 s of a 70 s run. This suggests one or a few early impacts with **no recovery**, not a thousand crashes [E]. It is still a failing result for the old pipeline.
- **The TFv6 row** ran with `SafetyEnvelope`: a TTC ≤ 1.5 s brake override that reads CARLA ground truth, standing in for Layer 1's collision check. It is not raw TFv6.
- **BehaviorAgent** reads privileged map and actor ground truth. It is an upper-bound reference like LEAD's expert, not a deployable competitor.
- **Run lengths differed:** 18–93 s of simulated time depending on the run, so per-run totals are not normalized.

**Replanning latency per tick, mean [M]**

| Scenario | Deterministic stack (runs Layers 1–4) | TFv6 inference (Layer 0, slow loop) |
|---|---|---|
| pedestrian_jumpout | **5.5 ms** | 190 ms |
| highway_merge | **1.4 ms** | 324 ms |
| traffic_stress | **7.1 ms** | 247 ms |

Reading:
- The safety loop is **14–70× under** the 100 ms safe band.
- TFv6 on this hardware is in the 150–200+ ms degradation zone [P]. It is therefore consumed asynchronously and never directly actuated (§3.2).
- **Important:** CARLA synchronous mode pauses the world while the client computes. TFv6's zero collisions therefore do **not** show it is safe at 190–324 ms in real time. The architecture is what keeps real-time behavior safe.

### 9.3 Projected sensing-to-command latency of the safety path [E]

- Components: sensor frame period (50 ms) + deterministic compute (1.4–7.1 ms mean [M]) + Layer 1/3 checks (estimated < 1 ms, to be measured).
- Total: **≈ 52–58 ms mean.** That is inside the ≤100 ms band where Weng & Yun observed 91.65–100% success [P].
- This is a projection. The p95 and max latencies must be measured once Layers 1–4 are built.

### 9.4 Evaluation plan (to complete before submission)

1. Run all **5 PS scenarios × ≥3 seeds** and record the GPU model, seeds and wall-clock budget.
2. **Ablation ladder** on identical seeds:
   - TFv6 raw
   - \+ L1 (with the ground-truth backstop replaced by the real safety perception chain)
   - \+ L2
   - \+ L3
   - \+ L4 (full system)

   Report collisions, completion, jerk, and latency (mean and p95) for each rung.
3. Measure the **fraction of ticks per layer** (how often each layer drives) and **reject-reason histograms** (T/P/K/C).
4. **Latency-injection test:** add 100/150/200 ms artificial delay to TFv6 in asynchronous mode and show that the fast loop keeps collisions at zero. This reproduces Weng & Yun's stress axis on our system.
5. **Path smoothness:** jerk, lateral acceleration and yaw rate against the nuPlan thresholds, excluding evasive and MRM ticks.
6. Pedestrian and cattle scenarios with `tfv6_4cameras` vs `tfv6_regnet`, and with vs without OSNet (tracking ID switches plus downstream collisions).

---

## 10. Why this is better — evidence-backed comparison

| Alternative | Its documented problem | Our answer |
|---|---|---|
| Our earlier modular A\* pipeline as the driver | 601–1,212 collision-contact ticks per run [M] | TFv6 drives normally (0 events [M]). A\* is demoted to a checked contingency |
| TFv6 alone | No backstop. TransFuser has 9× the expert's vehicle collisions [P]. 190–324 ms latency on our hardware [M] | L1 verification, L2 parallel alternative, L3/L4 below, and an asynchronous slow loop |
| Braking-only AEB as the sole safety net | Braking distance grows with v² while steering distance grows ∝ v. Above v\*, braking cannot avoid what steering can [E], per US7016783B2 [P] | L3 combined brake + steer with a closed-form decision |
| "Just stop" as the MRC | Stopped AVs obstruct traffic and emergency operations [P] | Context-aware MRM states |
| Hand-tuned heuristic inside the learned model | Cost TransFuser ~10 DS on unseen maps [P] | External, physics-derived, benchmark-independent layers |
| VLA as primary | Slower [P], lower Bench2Drive scores [P], and driverless operators keep them off the real-time control path [P] | TFv6 primary. A VLA is optional as an offline teacher |
| Privileged rule-based agent (BehaviorAgent) | Needs ground-truth map and actors, so it is not deployable | Uses only onboard sensing (sim stand-ins disclosed) |

---

## 11. Implementation status and MATLAB/Simulink mapping

### 11.1 Status

The "Where" column gives file and branch. Many components live on
`framework-summit-integration` / `fsi-R3-moflow`, not `main`.

| Component | Status | Where |
|---|---|---|
| TFv6 via PCLA (L0) | **Running live** (§9.2) | `framework/autopilots/pcla_transfuser_autopilot.py` |
| TTC SafetyEnvelope (sim stand-in for L1-C) | **Running live** (ground truth) | `framework/safety_envelope.py` |
| Fusion, Kalman + Hungarian tracker, CV predictor, drivable area | **Implemented** | `pipeline/*.py` |
| Costmap A\* + hysteresis + smoothing (L2 core) | **Implemented** | `pipeline/planner.py` |
| Mode FSM + supervisor | **Implemented** | `pipeline/decision_logic.py` |
| Pure Pursuit + emergency override | **Implemented** | `pipeline/controller.py`, `pipeline.py` |
| Metrics harness (latency, jerk, collisions, completion) | **Implemented** | `pipeline/metrics.py` |
| 5 PS scenarios | **Authored**. 3 need live coordinate capture | `framework/ps_scenarios/` |
| L1 monitor (T/P/K/C) + watchdog thread | To build (NumPy, small) | — |
| L2 parallel loop + last-safe buffer | To build (reuses `planner.py`). CPTO is optional | — |
| L3 evasive (decision + quintic sampling + LQR) | To build | — |
| L4 context-aware MRM states | To build (extends the FSM) | — |
| Tier 3 complexity supervisor | To build | — |
| OSNet appearance association | To build (BoxMOT/torchreid) | — |
| YOLO fine-tune (IDD + DATS_2022) | Pending. Ground-truth detector stands in | — |

### 11.2 MATLAB/Simulink mapping (PS expects a MATLAB/Simulink pipeline)

| Layer | MathWorks equivalent |
|---|---|
| Mode FSM, L4 MRM, watchdog | **Stateflow** (chart authored first; parallel states for the supervisor) |
| Tracking | Sensor Fusion and Tracking Toolbox (`trackingKF`, `trackerGNN`) / Automated Driving Toolbox `multiObjectTracker` |
| L2 contingency | Navigation Toolbox `plannerHybridAStar` |
| L3 quintic evasive sampling + collision validation | Navigation Toolbox **`trajectoryOptimalFrenet`** (quintic polynomial connections) + **`dynamicCapsuleList`** (validating many trajectories against dynamic obstacles) [P] |
| Control | Model Predictive Control Toolbox (Path Following Control System block) or an LQR / Stanley lateral controller |
| L0 TFv6 | Called through `py.*` from a MATLAB Function block. This is one crossing per tick |

Status: the mapping is designed, and the running implementation is Python.
The Stateflow redraw is mechanical because the FSM was authored chart-first.

---

## 12. Risks and limitations (disclosed)

- **The eval evidence is early.** It covers three scenarios, the seed count is unrecorded, run durations are unequal, and the hardware is unrecorded (§9.2).
- **TFv6's result used a ground-truth TTC backstop.** The deployable version replaces this with the independent perception chain.
- **Synchronous CARLA hides wall-clock latency.** Real-time safety claims rest on the fast loop and the planned latency-injection test (§9.4).
- **The old A\* pipeline's collision root cause is undiagnosed.** It matters because the same planner serves as the Layer 2 contingency.
- **Sim stand-ins.** Detection, segmentation and the SafetyEnvelope use CARLA ground truth today. CARLA has no livestock blueprint, so walkers are relabeled as livestock.
- **Evasive thresholds are illustrative.** μ, `w` and actuator limits need calibration for Indian surfaces and lane-free gaps.
- **OSNet weights are person-trained.** Vehicle and animal re-ID needs other weights.
- **CPTO timing is published on unspecified hardware.** It must be re-measured before we adopt it.

---

## 13. References (verified 2026-09-30)

1. Nguyen, Fauth, Jaeger, Dauner, Igl, Geiger, Chitta. *LEAD: Minimizing Learner-Expert Asymmetry in End-to-End Driving.* CVPR 2026. [arXiv:2512.20563](https://arxiv.org/abs/2512.20563)
2. Chitta et al. *TransFuser: Imitation with Transformer-Based Sensor Fusion for Autonomous Driving.* TPAMI. [arXiv:2205.15997](https://arxiv.org/abs/2205.15997)
3. Moller, Tungka, Jürgens, Betz. *Towards Safe Autonomous Driving: A Real-Time Motion Planning Algorithm on Embedded Hardware.* [arXiv:2601.03904](https://arxiv.org/abs/2601.03904)
4. Zheng, Yang, Zheng, Wang, Ma. *Safe and Real-Time Consistent Planning for Autonomous Vehicles in Partially Observed Environments via Parallel Consensus Optimization.* IEEE T-ITS 27(5), 2026. [arXiv:2409.10310](https://arxiv.org/abs/2409.10310)
5. Zheng, Yang, Zheng, Peng, Wang, Ma. *Occlusion-Aware Contingency Safety-Critical Planning for Autonomous Driving.* IEEE T-Cybernetics. [arXiv:2502.06359](https://arxiv.org/abs/2502.06359)
6. Nyberg, Gautier, Tumova. *Hope for the Best, Prepare for the Worst: Occlusion-Aware Contingency Planning for Autonomous Vehicles.* [arXiv:2607.03155](https://arxiv.org/abs/2607.03155)
7. vom Dorff, Böddeker, Kneissl, Fränzle. *A Fail-safe Architecture for Automated Driving.* DATE 2020. [PDF](https://past.date-conference.com/proceedings-archive/2020/pdf/0215.pdf)
8. Pek, Manzinger, Koschi, Althoff. *Using online verification to prevent autonomous vehicles from causing accidents.* Nature Machine Intelligence 2(9), 2020. [Link](https://www.nature.com/articles/s42256-020-0225-y)
9. Shalev-Shwartz, Shammah, Shashua. *On a Formal Model of Safe and Scalable Self-driving Cars (RSS).* [arXiv:1708.06374](https://arxiv.org/abs/1708.06374)
10. Lööf Wettervik, Mattsson. *Combined braking and steering maneuver for collision avoidance.* MSc thesis, Chalmers, 2025. [PDF](https://odr.chalmers.se/server/api/core/bitstreams/cc485e27-6110-4442-84ca-c6779981bfce/content)
11. Hac, Dickinson (Delphi). *Collision avoidance with active steering and braking.* US7016783B2. [Google Patents](https://patents.google.com/patent/US7016783B2/en)
12. Arab et al. *Safety Verification for Evasive Collision Avoidance in Autonomous Vehicles with Enhanced Resolutions.* [arXiv:2411.02706](https://arxiv.org/abs/2411.02706)
13. Tandon, Tapia Lopez, Blennemann, Trivedi, Greer. *When Stopping Fails: Rethinking Minimal Risk Conditions through Human-Interactive Autonomous Driving for Safe Transportation Systems.* [arXiv:2606.29115](https://arxiv.org/abs/2606.29115)
14. Balakrishnan. *Functional Safety Concept of "Minimum Risk Maneuver" in Conditional Driving Automation (Level 3) Vehicles.* SAE 2022-28-0301. [SAE](https://saemobilus.sae.org/papers/functional-safety-concept-minimum-risk-maneuver-conditional-driving-automation-level-3-vehicles-2022-28-0301)
15. Weng, Yun. *Multi-Resolution End-to-End Deep Neural Network for Optimizing Latency-Accuracy Tradeoff in Autonomous Driving.* [arXiv:2605.29138](https://arxiv.org/abs/2605.29138)
16. NVIDIA. *Alpamayo-R1: Bridging Reasoning and Action Prediction for Generalizable Autonomous Driving in the Long Tail.* [arXiv:2511.00088](https://arxiv.org/abs/2511.00088)
17. Huang, Hua, Zhou, Sural, Rajkumar. *RT-VLA: Real-Time Vision-Language-Action Models via Knowledge Distillation.* [arXiv:2606.14010](https://arxiv.org/abs/2606.14010)
18. Ling, Yang, Yang, Huang. *BLUE: Toward Better Language Use in Efficient Vision-Language-Action Models for Autonomous Driving.* [arXiv:2606.08684](https://arxiv.org/abs/2606.08684)
19. Fu et al. *ORION: A Holistic End-to-End Autonomous Driving Framework by Vision-Language Instructed Action Generation.* ICCV 2025. [arXiv:2503.19755](https://arxiv.org/abs/2503.19755)
20. Waymo. *Demonstrably Safe AI for Autonomous Driving* (Dec 2025) and *10 AI Lessons from Driving 200+ Million Fully Autonomous Miles* (Aug 2026). [Blog 1](https://waymo.com/blog/2025/12/demonstrably-safe-ai-for-autonomous-driving/), [Blog 2](https://waymo.com/blog/2026/08/10ailessons/)
21. Li Auto. SEC Form 6-K filings, FY2025–26 (VLA Driver deployment, Sep 2025; MindVLA, GTC 2026). [SEC](https://www.sec.gov/Archives/edgar/data/1791706/000110465926041742/tm2611553d1_ex99-3.pdf)
22. Electrek. *Xpeng VLA 2.0 test drive* (Apr 2026). [Link](https://electrek.co/2026/04/29/xpeng-vla-2-test-drive-tesla-not-alone-full-self-driving/)
23. Zhou, Yang, Cavallaro, Xiang. *Omni-Scale Feature Learning for Person Re-Identification (OSNet).* ICCV 2019. [arXiv:1905.00953](https://arxiv.org/abs/1905.00953). StrongSORT: [GitHub](https://github.com/dyhBUPT/StrongSORT)
24. nuPlan devkit. *Metrics description* (comfort thresholds). [Docs](https://nuplan-devkit.readthedocs.io/en/latest/metrics_description.html)
25. MathWorks. *Highway Trajectory Planning Using Frenet Reference Path* (`trajectoryOptimalFrenet`, `dynamicCapsuleList`). [Docs](https://www.mathworks.com/help/nav/ug/highway-trajectory-planning-using-frenet.html)

---

## Appendix A — Raw run log (complete, for Q&A)

This is the full table exactly as logged, including a metric we deliberately
do not headline.

| Scenario | Autopilot | Collisions/run (contact ticks) | Final distance to goal | Mean latency/tick | Simulated time covered |
|---|---|---|---|---|---|
| pedestrian_jumpout | pipeline (old A\*) | ~1,212 | 44.5 m | 5.5 ms | 70 s |
| pedestrian_jumpout | carla_behavior_agent | 0 | 3.98 m | not instrumented | 93 s |
| pedestrian_jumpout | pcla_tfv6 (+ GT TTC backstop) | 0 | 50.0 m | 190 ms | 34 s |
| highway_merge | pipeline (old A\*) | ~948 | 247.8 m | 1.4 ms | 49 s |
| highway_merge | carla_behavior_agent | 0 | 18.5 m | not instrumented | 51 s |
| highway_merge | pcla_tfv6 (+ GT TTC backstop) | 0 | 174.2 m | 324 ms | 18 s |
| traffic_stress | pipeline (old A\*) | ~601 | 7.9 m | 7.1 ms | 50 s |
| traffic_stress | carla_behavior_agent | 0 | 14.7 m | not instrumented | 52 s |
| traffic_stress | pcla_tfv6 (+ GT TTC backstop) | 0 | 44.6 m | 247 ms | 25 s |

**Why "final distance to goal" is not headlined:**
- TFv6 ran for only 18–34 s of simulated time, against 49–93 s for the other agents. The runs appear bounded by wall-clock time while TFv6 is slower per tick, so its distance-to-goal is confounded by getting less simulated time to make progress.
- The old pipeline's smaller distance in two scenarios was achieved while in collision contact.
- BehaviorAgent uses privileged ground truth.
- The fix is equal-simulated-time runs with a completion-rate metric (§9.4), not a different number.

**Why BehaviorAgent latency is "not instrumented":** the logged 0 ms reflects
that its compute is not timed through `TickTimings`. It does not mean the
agent takes zero time.

---

## Appendix B — Diagram edits for "System Architecture and Workflow"

- **Box 2 (Perception & Tracking):**
  - Add "Kalman CV + Hungarian + OSNet re-ID" and "independent safety chain".
  - Label YOLO as "v8/v11 (TBD)" until the version is fixed.
- **Box 3 (Adaptive Planning Core):** keep the three tiers but restructure them.
  - **Tier 3:** "Adaptive Supervisor — scene complexity + timing health → speed cap, horizon, resolution, TTC thresholds, max plan age".
  - **Tier 2:** two side-by-side boxes, "L0 TFv6 (async)" ∥ "L2 Contingency: costmap A\* / CPTO + last-safe plan". **Remove PlanT2.** It is an untested comparison adapter, not part of this design.
  - **Tier 1:** "L1 Runtime Monitor (Timing · Plausibility · Kinematics · Collision/RSS)" → "L3 Evasive (brake-vs-steer, quintic + LQR)" → "L4 Context-aware MRM", plus a "Watchdog 150/200 ms" badge.
- **Box 4 (Vehicle Control):** add "LQR (evasive only)" and "MRM speed profile" next to Pure Pursuit.
- **Tech stack panel:**
  - Planning: "PCLA + TFv6" (drop PlanT2).
  - Control & Safety: "Runtime monitor, TTC + RSS, quintic evasive + LQR, MRM, Watchdog".
  - Label MATLAB/Simulink "toolbox mapping / port target", not "in use".

## Appendix C — Fact-check log

These corrections were made to earlier drafts and inputs.

| Claim as received | Finding | Action |
|---|---|---|
| "VLA not SOTA / not used in industry" | **False for industry:** Li Auto (Sep 2025) and XPeng VLA 2.0 (Mar 2026) ship VLA-based driver assistance. True on Bench2Drive, where TFv6 95.0 > SimLingo 85.1 > ORION 77.74 | Reframed §7 around supervised L2 vs driverless, Waymo's real-time statement, and benchmark scores |
| The monitor's four checks come from Moller et al. | Moller defines Online Verification as feasibility + timing checks only | Plausibility and collision checks are our design; the collision check follows Pek et al. |
| "~14 m pass blindly before a reactive system starts responding" (vom Dorff) | The 14 m is the **entire emergency stopping distance** at 50 km/h (200 ms dead time, 8.6 m/s²), covered without further control | Corrected in §4.4 |
| CPTO "23–29 ms average, 40–47 ms max" | Verified only "max ≈ 40 ms beyond 4 obstacles" and "min 1.22 ms (2 trajectories)". The average was not found | Only verified figures used |
| Alpamayo-R1 99 ms "on RTX 6000 Pro Blackwell" | 99 ms verified. The hardware is not stated in the accessible text | Hardware claim dropped |
| TFv6 "70 ms/frame on RTX 2080 Ti (LEAD Table 6)" and "95.2 / 86.8" (in our `ppt-report.md`) | LEAD contains **no** latency figure. Table 5 gives 95.0 ± 0.7 / 84.3 ± 2.1 | Use the Table 5 figures. Fix `ppt-report.md` |
| "SimLingo > 1 s/frame" (`ppt-report.md`) | No verifiable absolute figure found. RT-VLA gives relative speed-ups only | Relative speed-ups used instead |
| Arab & Khaleghi prove braking-fails-steering-works scenarios | The abstract covers hazard analysis and verification of evasive MRM, not that claim | Claim re-based on the Delphi patent and our own derivation |
| Crossover "42 km/h dry, 18 km/h ice" (patent family) | Not found in the patent text | Replaced by our derivation: ≈38 km/h (μ = 0.8) and ≈17 km/h (μ = 0.15) for a 1.8 m offset |
| SAE 2022-01-0096 (quintic + LQR evasive steering) | The paper could not be located or verified | Removed. Quintic sampling cited to the Chalmers thesis; LQR is a standard method |
| TFv6 "navigation targets as tokens instead of a GRU; dense path + target speed" | Not verifiable from the accessible LEAD text | Removed |
| Eval: TFv6 "0 collisions" | That run had a ground-truth TTC backstop enabled by default, in synchronous CARLA | Disclosed in §9.2 |
| Eval: "~1,212 collisions" | Counts contact ticks, not distinct crashes | Disclosed in §9.2 |
