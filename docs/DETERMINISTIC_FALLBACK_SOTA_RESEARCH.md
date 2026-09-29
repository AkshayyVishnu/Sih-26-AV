# Deterministic (Non-ML) Fallback — State of the Art Research

Research pass done 2026-09-30, scoped to: what does the field actually use for a
**deterministic, non-ML fallback** that can be triggered by a hard latency
ceiling (our pipeline's 150ms/200ms tick-latency threshold, see
`docs/architecture.md` and `pipeline/pipeline.py`'s `TickTimings` warning)?
This sits under Tier 1 ("Reactive safety layer") of `docs/ideal-architecture.md`
— that document already specifies the *shape* (independent, runs every tick,
decoupled from Tier 2/3's own judgment); this doc collects what the literature
and industry actually build to fill that shape, with no ML involved.

Not yet implemented in code — this is research to inform a future
implementation. See "Gap in current pipeline" at the end for what's missing.

---

## 1. The governing pattern: Simplex / Runtime Assurance architecture

This is the canonical architecture for "complex adaptive controller + simple
verified fallback, with a deterministic switch between them."

**Structure:**
- **Complex Subsystem** — the sophisticated (possibly ML-based, possibly just
  too complex to fully verify) controller. In our system: Tier 2/3, the
  costmap A*/learned-cost-head planner.
- **Safety Subsystem / Baseline Controller** — a simple, high-integrity
  controller providing similar-but-reduced functionality, simple enough to be
  exhaustively verified or formally proven safe.
- **Decision Module** — monitors the Complex Subsystem's outputs (and, in the
  variant relevant to us, its *timing*). If outputs are missing, stale, or
  violate a safety precondition, it deterministically disconnects the Complex
  Subsystem and hands control to the Safety Subsystem.

**Key property**: the actual safety case is carried entirely by the Decision
Module + Safety Subsystem, which are kept simple enough to certify. The
Complex Subsystem can be arbitrarily sophisticated (or wrong, or slow)
because it is never trusted directly — it's trusted only through the monitor.

**Recent extensions** (2025–2026):
- *Synergistic Simplex* adds bidirectional communication between layers
  (the safety layer can inform the complex layer, not just override it).
- *Perception Simplex* extends the pattern specifically to obstacle-detection
  faults (i.e., a monitor over the perception stage, not just planning/control).
- *Mission-Level Runtime Assurance* applies Simplex-style supervision across
  an entire driving mission, not just a single control loop.
- *Sℒ1-Simplex* specifically targets safe velocity regulation for
  self-driving vehicles using this pattern.

**Sources:**
- [Using simplicity to control complexity](https://www.researchgate.net/publication/3247757_Using_simplicity_to_control_complexity)
- [Synergistic Simplex: Cooperative Runtime Assurance for Safety-Critical Systems](https://arxiv.org/html/2605.08190v1)
- [Perception Simplex: Verifiable Collision Avoidance in Autonomous Vehicles Amidst Obstacle Detection Faults](https://arxiv.org/pdf/2209.01710)
- [Mission-Level Runtime Assurance Framework for Autonomous Driving](https://arxiv.org/pdf/2606.06996)
- [The Simplex architecture (figure/overview)](https://www.researchgate.net/figure/The-Simplex-architecture-The-Decision-Module-and-Baseline-Controller-are-pre-certified_fig1_316276041)
- [Sℒ1-Simplex: Safe Velocity Regulation of Self-Driving Vehicles under Perception Uncertainty](https://dl.acm.org/doi/10.1145/3564273)
- [Dynamic Simplex: Balancing Safety and Performance in Autonomous Systems](https://arxiv.org/pdf/2302.09750)
- [The Use of the Simplex Architecture to Enhance Safety in Deep-Learning-Powered Autonomous Systems](https://arxiv.org/pdf/2509.21014)
- [Safety architecture for autonomous vehicles (patent)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10962972)

---

## 2. Deterministic decision content for the fallback trigger/maneuver

### 2.1 Responsibility-Sensitive Safety (RSS) — closed-form safe-distance formulas

Developed by Mobileye/Intel. Formalizes "safe driving" as a **provable
mathematical theorem**, not a learned or heuristic judgment:

- Defines minimum longitudinal/lateral safe distance as a closed-form function
  of both vehicles' speeds, a fixed reaction time τ, and each vehicle's
  max braking/acceleration capability.
- Compliance with the RSS formula is claimed to guarantee collision avoidance
  as a mathematical theorem, under its stated assumptions.
- Used in practice as an **independent checker/veto layer** — not necessarily
  the primary planner, but a cheap, always-computable rule that can override
  it.
- **Stated limitations** (matter for our fallback design): RSS assumes the AD
  system never malfunctions and has perfect perception (all road users
  perfectly detected) — i.e., RSS alone does not cover perception failure or
  system stalls, only "is this planned trajectory geometrically safe given
  what we currently believe about the world." Robust-RSS variants extend it
  to handle stochastic/disturbed dynamics.

**Sources:**
- [Mobileye RSS](https://www.mobileye.com/technology/responsibility-sensitive-safety/)
- [Extending Responsibility-Sensitive Safety for the Assessment of Offloaded Autonomous Driving Services](https://arxiv.org/pdf/2606.07067)
- [Robust responsibility-sensitive safety: Noise disturbed adaptive cruise control](https://www.sciencedirect.com/science/article/abs/pii/S0167691125000039)
- [A Formally Verified Fail-Operational Safety Concept for Automated Driving](https://arxiv.org/pdf/2011.00892)
- [1. Responsibility-Sensitive Safety](https://arxiv.org/pdf/2206.03418)
- [On a Formal Model of Safe and Scalable Self-driving Cars](https://arxiv.org/pdf/1708.06374)

### 2.2 Rule-based, TTC-threshold Automatic Emergency Braking (AEB)

The standard automotive pattern, and the closest existing analogue to our own
`decision_logic.py`'s `_min_ttc` guard:

- Compute **time-to-collision (TTC)** purely from kinematics: distance /
  closing speed, for each tracked object.
- Compare against a fixed threshold; if below it, trigger braking. No search,
  no prediction network — pure arithmetic on current tracked state.
- Frequently combined with fixed **safety-distance rules** (the 3-, 4-, and
  5-second following-distance rules) as a second, independent deterministic
  check layered on top of TTC.
- Acts as a **supervisory safety layer**: continuously evaluates collision
  risk and overrides the nominal (possibly ML-driven) control output whenever
  a predefined threshold is violated and no safe corrective maneuver exists
  within the available time horizon.
- Regulatory reference point: FMVSS AEB rulemaking (US) formalizes minimum
  required AEB behavior for production vehicles, independent of any ADS
  software stack — i.e., this exact deterministic-rule pattern is already a
  regulatory floor, not just a research idea.

**Sources:**
- [Automatic emergency braking using a time-to-collision threshold based on target acceleration (patent)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11724673)
- [Modelling safety distance rule-based automatic emergency braking](https://www.sciencedirect.com/science/article/abs/pii/S0020025526001167)
- [Research on Longitudinal Active Collision Avoidance of Autonomous Emergency Braking Pedestrian System (AEB-P)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6864679/)
- [Emergency collision avoidance strategy based on steering and differential braking](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9805451/)
- [Adaptive emergency braking for highway safety: a simulation framework](https://link.springer.com/article/10.1007/s43684-026-00140-5)
- [Dual-AEB: Synergizing Rule-Based and Multimodal LLMs for Effective Emergency Braking](https://arxiv.org/pdf/2410.08616) — note: hybrid/LLM-augmented, cited here only for the rule-based-baseline half being the actual fallback in that design
- [Federal Register: FMVSS — Automatic Emergency Braking Systems for Light Vehicles](https://www.federalregister.gov/documents/2024/05/09/2024-09054/federal-motor-vehicle-safety-standards-automatic-emergency-braking-systems-for-light-vehicles)

### 2.3 Reactive geometric local planners as the fallback *path* (not just brake-in-place)

When the fallback needs to do more than stop dead (e.g., still needs to
steer around something already in the lane), the standard non-ML fallback
planner is a **reactive, bounded-time geometric method**, not a scaled-down
search:

- **Dynamic Window Approach (DWA)**: parameterizes local motion by
  achievable (translational, rotational) velocity pairs given the vehicle's
  dynamic constraints, scores each against obstacle clearance/goal
  progress/speed, picks the best in one bounded pass. O(n) in obstacle count,
  no learned components, no multi-step search tree.
- **Potential-field methods**: obstacles generate repulsive fields, goal
  generates an attractive field, resulting velocity command is the gradient.
  Even cheaper than DWA; some recent variants blend the two (gradient-field
  DWA) to get potential-field cheapness with DWA's dynamic feasibility.
- These operate directly on **current tracked-object position/velocity only**
  — no trajectory prediction, no A* over a costmap — which is exactly what
  makes them safe to trust in the tick where the main pipeline blew its
  latency budget: their own worst-case compute time is bounded and known in
  advance, unlike A* over a costmap whose size can vary.
- Search results did not surface a paper specifically framed as "DWA as the
  deadline-miss fallback for a costmap-A* primary planner" — this is a
  reasonable extrapolation from DWA's known role as a fast local planner
  layered under slower global planners in mobile robotics generally, not a
  directly-cited claim.

**Sources:**
- [The Dynamic Window Approach to Collision Avoidance](https://www.researchgate.net/publication/3344494_The_Dynamic_Window_Approach_to_Collision_Avoidance)
- [Dynamic window based approaches for avoiding obstacles in moving obstacle environments](https://www.sciencedirect.com/science/article/abs/pii/S0921889018309746)
- [Gradient Field-Based Dynamic Window Approach for Collision Avoidance](https://arxiv.org/html/2504.03260v1)
- [Enhancing Obstacle Avoidance in Dynamic Window Approach via Dynamic Parameters](https://doi.org/10.3390/act14050207)
- [Dynamic window based approach to mobile robot motion control](http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_ICRA_2007/data/papers/1765.pdf)

### 2.4 Minimal Risk Maneuver (MRM) / Minimal Risk Condition (MRC)

The formal SAE/ISO-level naming for "the thing you deterministically fall
back to when the primary system can't guarantee a valid output in time":

- **Minimal Risk Condition**: a low-risk operating condition an ADS
  automatically resorts to when the system fails, or when a human operator
  fails to take over the driving task appropriately.
- **Fault Tolerant Time Interval (FTTI)**: the maximum time the system may
  remain in a faulted/degraded state before it must have reached the MRC.
  This is the formal analogue of our 150ms/200ms tick-latency ceiling — the
  literature frames the latency budget as an FTTI, and the fallback's job is
  to reach MRC within it.
- The transition itself is described as an immediate mode-shift: on detecting
  a deadline/fault condition, the system immediately switches to MRM mode
  rather than attempting to finish or retry the faulted computation.
- Practical metrics used to evaluate an MRM implementation: detection
  latency, fallback-initiation latency, time-to-reach-MRC, the realized stop
  zone (geometric footprint of the stopping maneuver), and MRM success
  probability. These would be reasonable to log directly from our own
  `TickTimings`-style instrumentation once a real fallback exists.
- Distinct from **teleoperated takeover**: research notes teleoperation
  latencies above ~500ms make remote control itself unsafe, which is exactly
  why an *onboard*, deterministic MRM is treated as essential rather than
  optional — remote human fallback cannot cover the latency regime we care
  about (150–200ms).

**Sources:**
- [Minimum Risk Maneuver Fallback Strategy for Autonomous Vehicles: Design and Experimental Validation](https://www.researchgate.net/publication/394365957_Minimum_Risk_Maneuver_Fallback_Strategy_for_Autonomous_Vehicles_Design_and_Experimental_Validation)
- [Contingency Planning for Safety-Critical Autonomous Vehicles: A Review and Perspectives](https://arxiv.org/pdf/2601.14880)
- [The Development of Teleoperated Driving to Cooperate with the MRM](https://doi.org/10.3390/automation6030026)
- [Re-imagining ISO 26262 in the Age of Autonomous Vehicles](https://arxiv.org/html/2606.07437v1)

---

## 3. The timing/watchdog mechanism (the actual latency-trigger machinery)

This is the part most directly relevant to "deterministically trigger a
fallback if latency > 150/200ms" — separate from *what* the fallback does
(section 2) is *how reliably it gets triggered* (this section).

- **Independent execution context.** The standard implementation places the
  watchdog and the MRM/fail-safe controller on a **separate execution
  context** from the main perception/planning pipeline — a lockstep core, a
  separate thread/process, or a dedicated microcontroller. The reason is
  specific: this lets the watchdog fire even if the main pipeline *hangs or
  stalls mid-computation*, not just when it merely finishes slowly and
  returns a late result. A watchdog implemented as "check elapsed time after
  the call returns" (our current `pipeline.py` pattern) cannot catch a stall
  — it can only catch and log a slow-but-completed tick.
- **Two-stage watchdog.** A common embedded-systems pattern: first timeout
  triggers fail-safe outputs (force a safe control state, e.g. begin
  deceleration); a second, longer timeout — if the fault hasn't cleared —
  triggers a harder response (full stop / system reset). This avoids a
  single "cliff" behavior and gives a graduated response.
- **Deadline-miss early detection.** Rather than waiting for the full period
  to elapse, some designs detect that a deadline *will* be missed partway
  through a task (e.g., from a DAG task's progress) and switch to fallback
  proactively, shaving reaction time.
- **Graceful degradation design process.** Rather than one binary
  fallback-on/fallback-off, define explicit degradation tiers, each mapped to
  a specific failure/latency condition (perception slow vs. planner slow vs.
  full stall), each with its own appropriately-conservative response, rather
  than always jumping to the most conservative maneuver regardless of which
  stage actually missed its budget. This is presented as improving the
  "automated driving continuation rate" — i.e., avoiding over-conservative
  full stops for degradations that don't warrant them.
- **Weakly-hard timing analysis.** Formal frameworks exist (e.g. SAW) for
  reasoning about systems that are allowed to miss *some* deadlines as long
  as misses don't exceed a bounded pattern (e.g., "no more than m misses in
  any k consecutive ticks") — relevant if a single 200ms tick shouldn't
  itself be catastrophic but a *run* of them should trigger fallback.

**Sources:**
- [Deadline Miss Early Detection Method for DAG Tasks](https://drops.dagstuhl.de/storage/00lipics/lipics-vol298-ecrts2024/LIPIcs.ECRTS.2024.8/LIPIcs.ECRTS.2024.8.pdf)
- [SAW: A Tool for Safety Analysis of Weakly-hard Systems](https://arxiv.org/pdf/2005.07159)
- [Approaching Current Challenges in Developing a Software Stack for Fully Autonomous Driving](https://arxiv.org/pdf/2504.12813)
- [Graceful Degradation Design Process for Autonomous Driving System Design](https://link.springer.com/content/pdf/10.1007/978-3-030-26601-1_2.pdf)
- [Reliability Analysis of Gracefully Degrading Automotive Systems](https://arxiv.org/html/2305.07401)
- [Watchdog Timer for Fault Tolerance in Embedded Systems](https://www.iieta.org/download/file/fid/153646)
- [SENSORAY: Single and Multistage Watchdog Timers (whitepaper)](https://www.sensoray.com/downloads/whitepaper_watchdogtimer.pdf)
- [Watchdog timer — overview](https://en.wikipedia.org/wiki/Watchdog_timer)

---

## 4. Synthesis — recommended shape for our system

Putting sections 1–3 together, the SOTA pattern for exactly our stated goal
("deterministically trigger a fallback if latency exceeds 150/200ms") is:

1. **An independent watchdog**, not a post-hoc check inside the same
   synchronous `tick()` call — so it can fire even on a stall, not just a
   slow-but-completed tick.
2. **A trigger condition** that is a hard, cheap, kinematics-only check —
   RSS-style safe-distance or TTC-threshold — computed from current tracked
   positions/velocities directly, bypassing prediction and the costmap
   planner entirely, so its own worst-case compute time is bounded and known.
3. **A fallback maneuver** that is either (a) a fixed deceleration-to-stop
   (MRM's default, simplest, always safe) or (b) a reactive DWA/potential-
   field local planner if a swerve is needed and time allows — never a
   reduced-size version of the same A*/costmap search, since that search's
   worst-case time isn't what caused the problem in the first place.
4. **Two-stage response**, matching graceful-degradation practice: a first
   threshold (150ms) degrades to a conservative-but-still-moving fallback;
   a second, harder threshold (200ms, or sustained misses) escalates to full
   stop.

## 5. Gap in the current pipeline (as of this research pass)

- `pipeline/pipeline.py`'s `Pipeline.tick()` only **logs a warning** when
  `timings.total_ms > 150` — no control-output change results from it. The
  150/200ms ceiling exists today purely as an observability threshold, not
  an actuation trigger.
- `pipeline/decision_logic.py` (`fsi-R3-moflow` branch only, not on `main`)
  has a `SafetyStatus.check()` that is the closest existing analogue to a
  watchdog (checks tick-gap and sensor health "regardless of mode"), but it
  is invoked **synchronously inside the same `tick()` call** as fusion /
  tracking / prediction / planning. If any of those stages itself stalls
  past budget, `decision_logic.step()` is never reached that tick at all —
  so it does not yet satisfy the "independent execution context" property
  that the watchdog literature treats as load-bearing (section 3, first
  bullet).
- No RSS-style or TTC-only fallback path exists that bypasses prediction/A*
  entirely; the only "fallback" behavior currently in the planner
  (`Planner.plan`'s "hold last good path" logic in `planner.py`) is triggered
  by A* failing to find a path, not by exceeding a latency budget, and it
  still requires that tick's A* run to have been attempted.
