# Combined Collision-Avoidance Architecture (v2)

Merges `docs/ideal-architecture.md`'s 3-tier structure with the findings in
`docs/DETERMINISTIC_FALLBACK_SOTA_RESEARCH.md` and
`docs/Collision_Avoidance_Architecture_Report.docx`. The 3-tier shape is
unchanged (Tier 3 supervisory → Tier 2 deliberative → Tier 1 reactive safety);
each tier is enriched with the concrete deterministic mechanisms the two
research passes surfaced. This is the reference to redraw the
"System Architecture and Workflow" diagram from — see §6 for the specific box
changes.

The single biggest structural change from `docs/ideal-architecture.md` v1:
**the deterministic fallback (Tier 1 / contingency planner) must run every
tick, in parallel with Tier 2, not be invoked after Tier 2 is judged to have
failed.** This is the `docx` report's central claim (vom Dorff et al.: ~14m
travels "blind" before a reactive-only system starts braking) and it changes
Tier 2 from one planner into two parallel ones arbitrated by a runtime
monitor.

---

## 1. Architecture overview

```
                          ┌──────────────────────────────────────────┐
                          │  TIER 3 — Supervisory                     │
                          │  Scene complexity (count/class-mix/spread)│
                          │  + system health/timing-budget signal     │
                          │  runs at perception rate, every tick      │
                          └───────────────┬────────────────────────────┘
                                           │ tunes horizon, resolution,
                                           │ deadline, cost weights,
                                           │ contingency-vs-primary mix
                                           ▼
 Sensors ─▶ Perception ─▶ Tracking ─▶ ┌────────────────────────────────────────┐
  (Camera+   (Fusion:    (Kalman +    │  TIER 2 — Adaptive deliberative layer   │
   LiDAR)     projection)  Hungarian) │                                          │
                                      │  ┌─────────────────┐  ┌────────────────┐│
                                      │  │ 2a. PRIMARY      │  │ 2b. CONTINGENCY││
                                      │  │ planner          │  │ planner        ││
                                      │  │ costmap A* +      │  │ CPTO/ADMM or   ││
                                      │  │ optional learned  │  │ DWA-class,     ││
                                      │  │ cost-head (danger,│  │ deterministic, ││
                                      │  │ inflation radius)│  │ ALWAYS running ││
                                      │  │ deadline-bounded  │  │ parallel, not  ││
                                      │  │ (anytime search)  │  │ on-demand      ││
                                      │  └─────────┬─────────┘  └───────┬────────┘│
                                      │            │  both outputs feed  │        │
                                      │            ▼                     ▼        │
                                      │       ┌──────────────────────────────┐   │
                                      │       │  RUNTIME MONITOR (passive)    │   │
                                      │       │  feasibility + deadline check │   │
                                      │       │  picks 2a if valid+on-time,   │   │
                                      │       │  else hands off down          │   │
                                      │       └───────────────┬────────────────┘   │
                                      └───────────────────────┼────────────────────┘
                                                                │ path (or handoff signal)
                                                                ▼
             ┌─────────────────────────────────────────────────────────────────────┐
             │ TIER 1 — Reactive safety layer (independent execution context)        │
             │                                                                        │
             │  WATCHDOG (own thread/process, two-stage: 150ms soft / 200ms hard) ───┐│
             │  TTC guard (kinematics-only) ── RSS safe-distance check (kinematics)  ││
             │           │                              │                            ││
             │           ▼                              ▼                            ││
             │     DriveMode state machine: NORMAL_DRIVE → OBSTACLE_DETECTED/         ││
             │     ANIMAL_ON_ROAD → EVASIVE_MANEUVER → EMERGENCY_BRAKE ⇄ REPLAN →     ││
             │     MRM (context-aware) → RESUME                                       ││
             │                                                                        ││
             │  EVASIVE MANEUVER: quintic-polynomial brake+steer trajectory + LQR     ││
             │  MRM: context-aware target (pull-out / controlled decel), not "stop"   ││
             └─────────────────────────────────────┬──────────────────────────────────┘
                                                     ▼
                                     Pure Pursuit + speed control (Vehicle control)
```

---

## 2. Tier 3 — Supervisory layer

Unchanged in role from `docs/ideal-architecture.md`: computes a scene-
complexity signal (object count, class mix, spatial spread) from
perception/tracking output and feeds it into Tier 2's parameters (horizon,
resolution, deadline, cost-weighting) every tick.

**Addition**: Tier 3 also carries a **system-health/timing-budget signal**
derived from the Tier-1 watchdog (§4), not just scene complexity — e.g.
"Tier 2's last N ticks have been within budget" vs. "degraded." This lets
Tier 2 pre-emptively favor the contingency planner's output or shrink its own
search horizon under sustained latency pressure, rather than waiting for a
hard deadline miss each time. Matches the "weakly-hard" timing framing from
the SOTA research doc (§3, watchdog section) — a bounded pattern of misses
is a legitimate degrade-early trigger, not just a single miss.

---

## 3. Tier 2 — Adaptive deliberative layer (now two parallel sub-planners + a monitor)

### 2a. Primary planner — deterministic search core, optionally learned cost shaping
This is what's actually implemented today (`pipeline/planner.py`):
- **Grid costmap** with soft Gaussian obstacle inflation, built from
  Kalman-filtered tracked-object predictions.
- **8-connected grid A\*** search (heuristic = Euclidean distance to goal,
  step cost = movement + inflated cell cost × weight) — the deterministic
  search backbone. Per-class inflation radii are hand-tuned by default
  (pedestrian 2.0m, car 1.5m, etc.); optionally replaced by a small learned
  MLP (`cost_head.py`, 7→32→16→2) mapping per-point features (class, speed,
  TTC, lateral spread, history length, mode probability, ego speed) to
  `(inflation_radius, danger)`, with a hard fallback to the hand-tuned table
  if no trained weights are loaded — so the "learned" part is strictly an
  optional cost-shaping layer on top of a deterministic search, never a
  replacement for it.
- **Signature-based hysteresis replanning**: replans only when a scalar
  costmap-cost signature changes >15% and ≥3 ticks have passed since the
  last replan — bounds worst-case planning latency, avoids chasing LiDAR
  sampling noise.
- **New addition — deadline-bounded (anytime) search**: A* should return the
  best path found so far if it hasn't converged by a wall-clock budget
  (e.g. 100ms), rather than running to completion or failing outright. This
  is the concrete mechanism behind Tier 3's "never silently miss the control
  deadline" property (`ideal-architecture.md` §1) and directly closes the gap
  identified in the SOTA research doc (currently the 150ms check is
  log-only).

### 2b. Contingency planner — always-on, deterministic, parallel (new)
Per the `docx` report's central finding: a fallback computed *after* the
primary is judged unsafe arrives too late. So a second, cheap, fully
deterministic planner runs **every tick, in parallel with 2a**, not on
demand:
- **CPTO-style parallel consensus optimization (ADMM)** — computes multiple
  candidate trajectories in parallel, sharing a common near-term segment so
  behavior doesn't flip-flop between candidates. Reported 23-29ms avg /
  40-47ms max with up to 6 tracked obstacles — comparable to or cheaper than
  the primary planner's own budget, so running it unconditionally doesn't
  blow the tick budget.
- **Cheaper fallback for constrained hardware**: Dynamic Window Approach or
  a potential-field method — O(n) in obstacle count, bounded worst-case
  compute, operates directly on current tracked positions/velocities (no
  prediction, no costmap search). Use this if CPTO/ADMM is too heavy for the
  target embedded hardware.
- **"Yesterday's safe answer is always ready"** (Nyberg et al.): the
  contingency planner's output from the *previous* tick is always held as a
  valid, immediately-executable fallback trajectory — so even if this tick's
  contingency computation is itself running late or the scene is occluded,
  there is never a tick with zero valid deterministic alternative.

### Runtime monitor — passive arbiter (new)
A cheap (~sub-ms), passive check: is 2a's output feasible (collision-free
against the current costmap, kinematically valid) and on-time? If yes, use
it. If no (infeasible or late), hand off to 2b's output. If 2b is *also*
invalid or a collision is judged imminent (TTC below threshold), hand off
further down to Tier 1's evasive/MRM layer directly — this monitor does not
itself compute a replacement, only decides which already-computed candidate
to use, which is why it can be sub-millisecond.

---

## 4. Tier 1 — Reactive safety layer (expanded: watchdog + TTC/RSS + evasive + MRM)

Everything in this tier must be computable from **current tracked-object
kinematics only** (position + velocity from the Kalman filter) — no
prediction, no costmap, no search — so its own worst-case compute time is
bounded and independent of scene complexity. It must also run on an
**independent execution context** (separate thread/process, ideally a
separate core) from Tiers 2/3, so it still fires if the main pipeline stalls
entirely, not just runs late.

### 4.1 Watchdog (new — closes the gap in the current `decision_logic.py`)
- Runs on its own timer, independent of `Pipeline.tick()`'s synchronous call
  chain (today `decision_logic.step()` is called *inside* the same tick as
  fusion/tracking/prediction/planning — if any of those stalls, the watchdog
  never runs that tick either; this must change).
- **Two-stage**: first timeout (150ms) → force handoff to Tier 2b's
  contingency output (cheap, already-computed, no new search needed);
  second timeout (200ms) or a sustained run of misses → force Tier 1's own
  evasive/MRM state directly, bypassing Tier 2 entirely for that tick.

### 4.2 TTC guard (already implemented, `decision_logic.py`)
Deterministic time-to-collision: `dist / max(closing_speed, floor)` per
tracked object, floor-guarded against near-zero closing speed. Debounced
entry (3 ticks) and a sustained-clearance gate (~20 ticks) before resuming,
so a single noisy detection doesn't trip mode changes.

### 4.3 RSS-style safe-distance check (new)
A second, independent deterministic check alongside TTC: minimum safe
following/lateral distance as a **closed-form function** of both vehicles'
speeds, a fixed reaction time τ, and max braking/acceleration capability
(Mobileye/Intel RSS formula). Runs in parallel with TTC as a second vote —
TTC catches "closing fast," RSS catches "already too close for physics to
save this regardless of closing rate," which TTC alone can miss when speeds
are momentarily matched.

### 4.4 Evasive maneuver layer (new — the component that actually improves collision avoidance, not just system robustness)
Per the `docx` report: braking alone has a hard physical floor (stopping
distance × friction); there exist real scenarios (governed by relative
speed, gap, vehicle width) where no braking profile stops in time but a
lateral maneuver still can, above a friction-dependent crossover speed
(≈42 km/h dry pavement in the cited analysis, ≈18 km/h on ice — **needs
re-derivation for Indian road surfaces before being trusted, not directly
usable**).
- **Quintic (5th-order) polynomial trajectory** around the obstacle —
  chosen specifically because position/velocity/acceleration boundary
  conditions can be pinned at both ends, producing a jerk-minimized swerve
  rather than an abrupt steering input.
- Tracked by an **LQR controller** (replaces Pure Pursuit only for the
  duration of the evasive maneuver — Pure Pursuit is tuned for normal
  waypoint following, not a jerk-bounded emergency swerve).
- Triggered from the `DriveMode` state machine when TTC/RSS cross the
  emergency threshold *and* the closed-form braking-only stopping distance
  exceeds the available gap (i.e., braking alone provably can't make it) —
  this is a deterministic go/no-go decision, not a learned one.

### 4.5 Minimal Risk Maneuver — redefined, context-aware (new; replaces "just brake to a stop")
Per the `docx` report and SAE 2022-28-0301: an unconditional stop can itself
be a hazard (mid-intersection, no shoulder, in the path of following
traffic) — especially relevant for Indian traffic density, called out
explicitly in the `docx` report's own adaptation notes. MRM is therefore a
**state**, not a single action:
- Preferred target: nearest safe pull-out (if the drivable-area classifier
  and costmap can identify one) with lane-keeping maintained via Pure
  Pursuit at reduced speed.
- Fallback-of-fallback: controlled deceleration in-lane (not an instant
  full-brake) with hazard signaling.
- Only engaged when **both** Tier 2b's contingency planner and Tier 1's own
  evasive-maneuver layer have failed to produce a valid trajectory — it is
  the last resort, not the default emergency response.

### 4.6 State machine (existing `decision_logic.py`, extended)
```
NORMAL_DRIVE → OBSTACLE_DETECTED / ANIMAL_ON_ROAD (debounced entry)
             → EVASIVE_MANEUVER (new state: TTC/RSS emergency + braking-insufficient)
             → EMERGENCY_BRAKE ⇄ REPLAN (existing: parallel, not exclusive)
             → MRM (new state: both 2b and evasive layer exhausted; context-aware target)
             → RESUME (existing: sustained clearance, returns to prior mode)
```
The parallel `SAFETY_SUPERVISOR` watchdog check (already exists as
`SafetyStatus`) is retained and extended to also watch Tier 2's timing, per
§4.1 — it can force `EMERGENCY_BRAKE` or `EVASIVE_MANEUVER` from any state,
same as today.

---

## 5. Deterministic methods used — explicit inventory

| Stage | Method | Deterministic? | Where |
|---|---|---|---|
| Fusion | Camera-LiDAR projection + median position in bbox | Yes (geometry) | `perception_fusion.py` |
| Tracking (motion estimate) | **Kalman filter**, constant-velocity model, state=[x,y,vx,vy] | Yes | `tracker.py` |
| Tracking (association) | **Hungarian algorithm** (`linear_sum_assignment`) on Euclidean gating distance | Yes | `tracker.py` |
| Prediction | Constant-velocity extrapolation + fixed per-class lateral-uncertainty fan (3 modes) | Yes | `predictor.py` |
| Tier 2a search | **A\*** (8-connected grid, soft-inflated costmap), deadline-bounded (new) | Yes (search algorithm itself; cost *inputs* optionally learned) | `planner.py` |
| Tier 2a replan trigger | Cost-signature delta threshold + min-tick hysteresis | Yes | `planner.py` |
| Tier 2b contingency | **Consensus ADMM (CPTO-style)** or **Dynamic Window Approach** | Yes | new |
| Tier 2 arbitration | Runtime monitor: feasibility + deadline check | Yes | new |
| Tier 1 collision check | **Time-to-collision (TTC)**: dist/closing-speed, floor-guarded | Yes | `decision_logic.py` |
| Tier 1 collision check (2nd vote) | **RSS closed-form safe-distance formula** | Yes | new |
| Tier 1 evasive trajectory | **Quintic polynomial** path + **LQR** tracking | Yes | new |
| Tier 1 watchdog | Two-stage timer, independent execution context | Yes | new |
| Path smoothing | Line-of-sight shortcutting, 0.25/0.5/0.25 moving-average smoothing, fixed-spacing decimation | Yes | `planner.py` |
| Control | **Pure Pursuit** (geometric curvature law) + proportional speed control | Yes | `controller.py` |
| MRM control | Pure Pursuit onto a conservative pull-out/decel target (not a separate controller) | Yes | new |

Nothing in this entire table is ML — the only learned component anywhere in
the combined design is the optional cost-head MLP shaping Tier 2a's
inflation/danger *inputs*, which itself degrades to the hand-tuned table with
zero behavior change if unavailable. Every safety-relevant decision (Tier 1
in full, and Tier 2's search/arbitration/replan-trigger logic) is
closed-form or classical-algorithmic.

---

## 6. Diagram changes needed (for the "System Architecture and Workflow" image)

Mapped to the original uploaded diagram's boxes:

- **Box 3 "Adaptive Planning Core"** needs to become 4 sub-boxes instead of
  3: Tier 3 (unchanged, add "+ timing-budget signal"), Tier 2 split into
  **two side-by-side boxes** (2a Primary A\*/learned-cost planner, 2b
  Contingency ADMM/DWA planner, always running in parallel) feeding a small
  **Runtime Monitor** box, then Tier 1 expanded into its own multi-part box:
  Watchdog → TTC/RSS guard → Evasive Maneuver (quintic+LQR) → MRM
  (context-aware), replacing the current single "Reactive Safety Layer"
  block.
- Remove or clearly caveat the **PlanT2** box under Tier 2 — it's an
  experimental comparison adapter (`plant2_adapter.py`), never tested live,
  not part of the production planning path; don't present it as the core
  Tier-2 engine.
- Remove or caveat the **MATLAB/Simulink** entry in the tech-stack panel —
  no `.m`/`.slx` files exist in the repo; the pure-Python path is what's
  actually built (per `docs/two-path-strategy.md`).
- Confirm **YOLO version** before labeling it "YOLOv11" specifically — docs
  currently say "YOLOv8/v11," undecided, with an AGPL-license caveat on v12.
- Vehicle Control box gains a note that MRM routes through the same Pure
  Pursuit controller but at a different (conservative) target, and that
  the evasive maneuver uses LQR instead of Pure Pursuit for that one
  maneuver's duration.
