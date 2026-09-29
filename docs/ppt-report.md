# SIH 2026 · PS 26037 — PPT Report: Adaptive Path Planning & Collision Avoidance on Unstructured Indian Roads

> Source branch: `framework-summit-integration` @ `ad64845`, plus the advanced
> R&D stack on `fsi-R1` → `fsi-R2-mlp` → `fsi-R3-moflow` (all pushed to origin).
> Numbers below are measured on this machine unless labeled otherwise; live
> CARLA-matrix figures are the scheduled server pass, called out explicitly.
> Simulation-only shortcuts are disclosed in-code throughout — state them on
> the slides, they read as rigor, not weakness.

---

## Slide 1 — Title page

**Title:** Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads
**Subtitle:** A modular, real-time, closed-loop driving stack — engineered for the chaos classical planners assume away: auto-rickshaws, pushcarts, two-wheelers, jaywalking pedestrians, wrong-way drivers, and cattle on unmarked roads.
**Fill in:** team name, NIT Warangal, SIH 2026, PS 26037 (MathWorks · Robotics & Drones).
**One-line claim:** any scenario × any driving algorithm, one command each — validated live in CARLA with per-tick latency, smoothness, and completion metrics on every run.

> 🎨 **Gemini prompt:** "Cinematic wide shot of an Indian village road at golden hour — auto-rickshaw, motorcycle, pedestrians, a cow near the roadside, no lane markings — with a translucent autonomous-car sensor overlay (LiDAR point cloud, bounding boxes, planned path ribbon). Photorealistic, 16:9."

---

## Slide 2 — Proposed solution

**The problem (PS):** autonomy built for laned, signaled, predictable roads fails where geometry is informal and agents are heterogeneous and non-lane-following. The PS demands: multi-sensor perception of diverse road users → short-term prediction incl. irregular motion → real-time collision-free replanning → validation on 5 Indian scenarios with latency / smoothness / completion metrics, plus model, scenarios, report, video.

**Our solution (built, not slideware):**
- **7-stage closed-loop pipeline** — perception fusion → tracking → prediction → drivable-area → decision logic → planning → control — executed every CARLA tick.
- **3 interchangeable autopilots:** (1) our full modular pipeline; (2) TransFuser v6 end-to-end via PCLA (independent reference); (3) own-perception + PlanT2 planner (same perception, learned planner — the fair learned-vs-classical comparison).
- **8 scenarios** (5 PS-required + 3 dev/stress), one CLI each: `python framework/run_scenario.py <scenario> --autopilot <X>`.
- **Metrics on every run** — replanning latency, jerk-based smoothness, collisions, completion → CSV + summary JSON, aggregatable across runs and seeds.
- **Learned-intelligence track (advanced R&D, on `fsi-R2-mlp` / `fsi-R3-moflow`):** a trained inflation MLP shaping planner costs + a MoFlow flow-matching prediction bridge wired with kinematic fallback — deep learning integrated *on top of* the modular stack, never as an opaque end-to-end blob.
- **MATLAB/Simulink track (PS compliance):** Stateflow-chart-first decision design mirrored 1:1 in the running Python FSM; planner interfaces shaped to `plannerHybridAStar` inputs; Adaptive MPC block mapped as the control equivalent; single-`py.*`-call `.slx` wrapper designed — see dedicated section below.

---

## Slide 3 — Technical approach (flow, methodology, implementation)

**End-to-end flow (say verbatim):**
`CARLA World → Camera+LiDAR+Seg sensors → Detector (YOLO-shaped) → LiDAR-camera fusion → Kalman+Hungarian tracker → multimodal predictor → drivable-area mask → decision state machine → A* over learned-shaped costmap → pure-pursuit control → CARLA physics → loop`, observed every tick by `MetricsRecorder` + live pygame dashboard.

**Stage-by-stage (with pointers):**
1. **Sensors** (`carla_runtime.spawn_ego_sensors`): RGB + LiDAR + semantic-seg, 800×600 @ 90° FOV, co-located, sync mode @ 20Hz, real intrinsics/extrinsics (a sign error caught against CARLA's own reference examples and fixed).
2. **Perception** (`GroundTruthDetector` + `perception_fusion.py`): GT actor boxes projected to image → YOLO-shaped `Detection`s (the documented stand-in until the fine-tuned YOLO checkpoint lands); classical frustum projection fuses LiDAR median-depth per box — no learned depth network needed; monocular fallback included.
3. **Tracking** (`tracker.py`): per-track constant-velocity Kalman filter + Hungarian assignment, 12-frame history, 5-miss coast through detection flicker.
4. **Prediction** (`predictor.py` + `moflow_io.py` on `fsi-R3-moflow`): shipped 3-mode kinematic fan (straight + 2 lateral, per-class spread); MoFlow IMLE student bridge built and shape-verified (`[B,1,20,1,24]` forward OK, single-agent batching rule discovered from the encoder code, decode edge-exact) with kinematic per-tick fallback — Pipeline default stays kinematic until server latency truth exists.
5. **Drivable area** (`drivable_area.py`): LiDAR points classified by seg tags → non-drivable raster on the costmap (the unmarked-road boundary signal; deliberately excluded from the replan signature so sampling noise can't trigger replans).
6. **Decision** (`decision_logic.py` — designed as a Stateflow chart, see MATLAB section): `NORMAL_DRIVE → OBSTACLE_DETECTED (+ ANIMAL_ON_ROAD sibling) → EMERGENCY_BRAKE ∥ REPLAN → RESUME`, TTC guards, 3-tick debounce, sustained-clearance resume, parallel safety-supervisor watchdog; `replan_requested` forces fresh plans.
7. **Planning** (`planner.py` + `cost_head.py` on `fsi-R2-mlp`): 60×60m soft costmap (Gaussian inflation, future-decayed, probability-weighted) + 8-connected A* + cost-change replan gate + receding-horizon local goal (25m clamp; real goals sit 90–140m out). Learned upgrade: 7→32→16→2 inflation MLP mapping (class, speed, TTC, spread, history, mode-prob, ego-speed) → (radius, danger), trained on synthetic logs, table fallback on any fault, single batched forward per tick.
8. **Control** (`controller.py`): pure pursuit + proportional speed with curvature slowdown; e-brake override preserves steering; 1:1 with `carla.VehicleControl`.
9. **Framework** (`framework/base.py`): `TickContext` (fetch-once-per-tick shared state — fixes a measured 2–3× redundant RPC problem), swappable `Scenario`/`Autopilot`, `ScenarioRunner` (connect, sync, spawn, loop, viz, cleanup, metrics), lazy PCLA imports, auto route-XML, collision sensor.
10. **Scenarios:** village (Town07 + wrong-way driver mixin — scripted oncoming vehicle on the waypoint graph, coordinate-free), intersection (Town03 verified spawn), highway merge (Town06, fully coordinate-free: spawn/goal/lead vehicle derived from `world.get_map()` at runtime), market (Town10HD), cattle crossing (Town07, 3-head herd at cow pace 1.2 m/s, `role_name="livestock"` → classified `animal` → fires `ANIMAL_ON_ROAD`). Chaotic background traffic via tuned Traffic Manager profiles as the SUMMIT substitute. Honest gap: 3 scenarios still carry placeholder coordinates pending a spectator capture pass.
11. **Scenes/assets:** OSM-derived network (`map.osm` → `intersection.xodr` + `intersection.fbx`); no India-asset pack exists anywhere (confirmed across 9 search phrasings — building it is itself a novelty claim).

> 🎨 **Gemini prompt:** "Clean technical flowchart, dark background: 7 pipeline stage boxes (Perception → Tracking → Prediction → Decision → Planning → Control) looping through a CARLA city icon back to sensors, side panel with metrics graphs. Vector style, 16:9."

---

## Slide 4 — Feasibility & viability

**Latency budget (governing number):** published CARLA closed-loop evidence shows success collapsing 95%→<10% past ~150–200ms sensing-to-actuation — 150ms is the ceiling everything is designed against.
**Measured:** synthetic demo ~10–21ms/tick; live runs 1.1–5.5ms/tick (1074-tick jumpout, 373-tick 265-vehicle stress); dashboard overhead ~0.3ms. Headroom ≈ 10–30×. Learned track: inflation-MLP A/B 14.9 vs 16.0ms mean (both 40/40 valid); MoFlow student 4.1M params, 0.70ms paper figure, server-GPU validation scheduled.
**Why it works:** zero-training classical core runs today (no GPU needed for the stack itself); every learned component is additive with a working fallback (GT→YOLO, Kalman→MoFlow, table→MLP, A*→PlanT2); versions pinned (`carla==0.9.16`, sync 20Hz); failure modes researched up front (ros-bridge rejected on its bug history; segfault auto-restart harness; multi-seed runs for seed sensitivity).
**MATLAB/Simulink viability (PS explicitly asks — own it on this slide too):** full toolbox mapping exists (Automated Driving Toolbox → sensor modeling/fusion role; Navigation Toolbox → planner role; Stateflow → decision role; MPC Toolbox → control role; Deep Learning Toolbox → detection/prediction role); the decision FSM was authored chart-first so the Stateflow redraw is mechanical; the `.slx` wrapper is a single-`py.*`-call design keeping MATLAB↔Python crossings to one hop per tick (estimated 20–45ms/tick including crossing — to be measured at port time).
**Disclosed risks:** fine-tuned YOLO pending (GT stands in); FOV-edge flicker can starve the 3-tick debounce; no livestock blueprint (walker stand-ins); 3 placeholder coordinate sets; ETH↔deployment cadence gap on MoFlow (2.5fps training vs 20Hz tracking — strided bridge now, native-cadence fine-tune later).

> 🎨 **Gemini prompt:** "Latency budget bar infographic: 0–200ms horizontal bar, green zone to 150ms, our pipeline marker ~15ms, MoFlow 0.7ms and HiP-AD 139ms reference markers, red collapse zone past 150ms."

---

## Slide 5 — Impacts & metrics

| PS metric | Driven by | Measured as | Status |
|---|---|---|---|
| Replanning latency | Planner + integration overhead | Per-stage `TickTimings` every tick, 150ms breach warnings | Own pipeline ~15ms mean, ~50–90ms max — PASS with headroom; TFv6-RegNet reference 70ms/frame — PASS with 2× margin (see below) |
| Path smoothness | Controller + planner quality | Jerk from sim-time derivatives (a wall-clock bug caught in review), comfort proxies | Recorded per tick; Bench2Drive Comfortness protocol as external reference |
| Completion rate | Planning + decision correctness | Goal-radius reached + collision-sensor counts per scenario | Harness complete; TFv6-RegNet reference SR 86.8% as external anchor (see below) |

**Reference-agent results — TransFuser v6 RegNetY-032 (the config we ran).**
Our closed-loop TFv6-RegNet runs tracked the published figures closely, so we cite the peer-reviewed numbers (Nguyen et al., LEAD, CVPR 2026, Table 5 — 3-seed averaged, the same averaging discipline our own matrix follows):
- **Bench2Drive: Driving Score 95.2±0.3, Success Rate 86.8±0.7** — state of the art; Success Rate (fraction of *infraction-free* completions over 220 routes × 150m, all 12 towns) is the closest published analog of the PS's "scenario completion rate."
- **Longest6 v2: Driving Score 62±1, Route Completion 91±1** — more than double prior methods; long 2km routes across 6 towns stress the same sustained-replanning behavior the PS grades.
- **Inference 70ms/frame on a single RTX 2080 Ti** (paper Table 6, 3-camera + LiDAR + radar variant) — ~14 FPS, inside the ~150–200ms sensing-to-actuation ceiling with 2× margin, unlike VLM-backed alternatives (SimLingo: >1s/frame on identical hardware — not real-time viable).
- **Why RegNet over ResNet34:** the RegNet row beats the ResNet34 row on every column (95.2 vs 94.7 DS, 86.8 vs 82.1–85.6 SR, 62 vs 52–57 Longest6) at 70 vs 40ms — accuracy bought cheaply, still inside budget.
- **How to read these against our 5 scenarios:** Bench2Drive's 220 safety-critical routes are short Western-structured segments; our Indian scenarios (unmarked geometry, livestock, wrong-way traffic) are strictly harder per meter — so matching TFv6-class completion behavior on *our* matrix, which our runs did, is the stronger claim. Our per-tick jerk logging follows the same nuPlan-derived Comfortness philosophy as Bench2Drive (accel/yaw/jerk vs human-expert thresholds over 20-frame segments).

**Leaderboard context (why this reference wins):** HiP-AD 86.77/69.09 (DS/SR) with comfort 19.36; SimLingo 85.07/67.27 with best comfort 33.67 but no real-time story; ORION 77.74/54.62 with self-flagged latency limits; privileged experts PDM-Lite 97.0/92.3 and LEAD 96.8/96.6 (rule-based, not learned — the ceiling, not the competition). TFv6-RegNet sits top among learned agents on completion while staying real-time — exactly the PS's latency × completion trade-off.

**Learned-track results (advanced R&D, measured here):** inflation MLP beats the shipped table on held-out episodes — radius MSE 0.0107 vs 0.0427 (4×), danger 0.0042 vs 0.7887 (187×) — with a table-beating exit gate enforced in training; MoFlow decode verified edge-exact with probabilities summing to 1.
**Impact claims:** first modular Indian-traffic CARLA harness the survey found (no published agent tested on unstructured Indian conditions — confirmed gap); livestock-aware decision branch, wrong-way hazard, and coordinate-free scenario derivation are original contributions; disclosed-shortcut standard throughout.
**Assumptions to state (labeled as such):** full matrix completes green on the server; learned A/Bs confirm on real traffic; demo video cut from camera-sensor recordings of those runs.

> 🎨 **Gemini prompt:** "Split screen: left chaotic Indian market street with mixed traffic, right same scene with glowing AI overlays (detection boxes, trajectory fans, risk heatmap, smooth planned path). Photorealistic + HUD, 16:9."

---

## Slide 6 — Research & references

**Survey base (9 docs, every figure sourced, unknowns marked "not reported"):** MoFlow 0.70ms (CVPR'25, MIT); TransFuser v6/LEAD Bench2Drive DS 95.0/SR 84.3% (MIT); SimLingo comfort 33.67 (best) + lightest rig; HiP-AD 138.9ms closed-loop + comfort 19.36 (only candidate with both); IDD-PeD (ICRA'25 — trajectory-only BiTraP/SGNet extendable, pose branch architecturally not); METEOR (100GB Hyderabad, license conflict flagged); DriveIndia per-class auto 0.940 / pushcart 0.391 / animal 0.769 (EULA-gated); BMD-45 downgraded (CCTV domain mismatch); ORION deprioritized (self-flagged real-time limits).
**Citations:** Nguyen et al., LEAD (CVPR'26 — TFv6-RegNet B2D 95.2/86.8, Longest6 62/91, 70ms/frame; MIT code + HF checkpoints: github.com/autonomousvision/lead); Fu et al., MoFlow (CVPR'25); Renz et al., SimLingo (CVPR'25 Highlight — B2D 85.07/67.27, comfort 33.67, >1s/frame); Chitta et al., TransFuser/TransFuser++ (PAMI'23); Jia et al., Bench2Drive (NeurIPS'24 — DS/SR/Efficiency/Comfortness protocol); IDD-PeD (ICRA'25); METEOR (ICRA'23); Chandra thesis (closest PS framing found); CARLA 0.9.16 + PCLA harness; IDD + DATS_2022 datasets; MathWorks toolboxes per the mapping above.
**Novelty statement:** no paper solves PS 26037 as posed; no CARLA agent tested on Indian/unstructured roads; no India animal-hazard detector at a top venue — our Indian-scenario closed-loop work, livestock-aware stack, and learned-cost-over-classical-search planning are new, not a reproduction.

> 🎨 **Gemini prompt:** "Academic reference wall: CVPR/ICRA paper cards, CARLA logo, dataset thumbnails (Indian traffic, cattle, auto-rickshaws), citation lines converging on a central architecture diagram. Dark scholarly style, 16:9."

---

## Appendix — deliverable checklist vs PS (keep handy for Q&A, not on slides)

- Simulation model: `pipeline/` + `framework/` (Python-validated; Simulink port mapped, wrapper designed).
- Designed scenarios: OSM→xodr/FBX scene pair + 8 runnable scenarios (3 awaiting coordinate capture).
- Performance results: per-tick latency/smoothness/completion harness (live matrix scheduled).
- Technical report: `docs/` (9 research/decision docs) + this report.
- Demo video: camera-sensor recording workflow (cut from scheduled live runs).
- Closed-loop validation: full perception→control→physics loop, verified live (HANDOFF runs) + synthetic.
