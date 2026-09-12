# CARLA Ecosystem Catalog — SIH 2026, PS 26037

## Purpose & scope

This is the ecosystem-breadth complement to the paper-driven `docs/sota-research.md`
and `docs/component-deep-dive.md` — a non-paper-filtered sweep of **every
open-source component connected to CARLA** found via its official GitHub
orgs, community lists, and forum resources. Items already documented
elsewhere are **not repeated** here — see the Cross-reference index at the
bottom. Everything below is new ground from this round.

---

## Core infrastructure & tooling

| Component | What it does | License | Integration read |
|---|---|---|---|
| **`scenario_runner`** | Traffic-scenario engine — Python API + OpenSCENARIO 1.x/2.0 support, built-in scenario library, `no_rendering_mode.py` 2D viewer | MIT | Scripts the CARLA-side scenarios your Simulink/Stateflow logic reacts to via ROS topics — no direct MATLAB hook, consumed downstream |
| **`leaderboard`** (1.0/2.0/2.1) | Route XML files, eval harness, `autonomous_agent.py` sensor-interface contract (≤8 cameras, 2 LiDAR, 4 radar, GNSS/IMU), Docker submission pipeline. 2.1 (current, Mar 2025) changed infraction scoring to linear — **not score-comparable to 2.0** | MIT | Map your ROS bridge's sensor topics 1:1 to this contract if you want leaderboard-standard routes to be runnable against your stack |
| `leaderboard-agents` | Official baseline reference agents (human/ROS/ROS2/performance/log), Docker-wrapped | MIT | Useful as a "do-nothing" baseline to sanity-check your harness |
| `carla_route_generator` (autonomousvision) | Qt GUI for interactively creating/editing Leaderboard 2.0 route/scenario XML | MIT | Faster than hand-editing XML for your 5 scenario routes |
| `Bench2Drive-Leaderboard` (autonomousvision) | Community results *table* tracking 60+ published methods by driving score — not an eval harness itself | — | Reference for where your fine-tuned model would rank |
| **`ros-carla-msgs`** | Official ROS message definitions underlying `ros-bridge` | MIT | **High value** — match your ROS Toolbox subscriber/publisher message types exactly to these rather than hand-rolling parsers |
| CARLA–Autoware bridges (5 community forks: NEWSLabNTU, rohanNkhaire, guardstrikelab, Robotics010, HoYongLee98) | Translate CARLA↔Autoware(ROS2) message schemas; official `carla-simulator/carla-autoware` is **deprecated** in favor of these | Not individually confirmed | Low direct relevance (you don't need Autoware itself), but the message-translation *pattern* is directly reusable for your own ROS Toolbox design |
| `DReyeVR` | VR + eye-tracking human-driver research extension | MIT | **Stale** (last commit Nov 2023, CARLA 0.9.13/UE4.26 only) — skip, not autonomy-relevant |
| RL/gym wrappers: `gym-carla`, `macad-gym`, `CARLA-GymDrive`, `johnMinelli/carla-gym`, `janwithb/carla-gym-wrapper`, `carla-rl-gym/carla-rl`, `angelomorgado/CARLA-RL-Agents`, official `rllib-integration` | Python-side Gym/RLlib training harnesses for CARLA | Mostly MIT; several stale (gym-carla targets CARLA 0.9.6) | Relevant only for offline policy pretraining (Python/RLlib → ONNX export → Deep Learning Toolbox import) — none speak ROS/Simulink natively for live training |
| CARLA built-in **No Rendering Mode** | 2D top-down map view, boosts server FPS by skipping GPU rendering | Bundled w/ CARLA | Good for headless CI runs, frees GPU for MATLAB/Simulink inference |
| `carla-birdeye-view` (deepsense-ai) | 8-channel one-hot BEV feature map or RGB visualization, CNN-ready | MIT | Candidate input encoding for a Deep Learning Toolbox planning network, or exportable as an Image/OccupancyGrid ROS message for Navigation Toolbox costmaps |
| CARLA built-in **Recorder** | Deterministic scenario logging + variable-speed replay (~200MB/hr @ 100 vehicles) | Bundled | Iterate on Stateflow/Simulink logic against a fixed replayed scenario without re-running full traffic sim each time |
| `carla_dataset_tools` (KevinLADLee) | Multi-sensor synchronized dataset export → KITTI/YOLOv5/Argoverse formats, depends on `ros-bridge` | **GPL-3.0 — copyleft, check compatibility before bundling** | Useful for offline Deep Learning Toolbox perception training via `kittiDataset`-style importers, but mind the license |
| `sachinkum0009/carla-multi-sensor-fusion`, `joedlopes/carla-simulator-multimodal-sensing` | Camera+LiDAR fusion utilities, ROS-oriented | Not confirmed | Reference implementations; prefer mirroring CARLA's own documented sensor intrinsics/extrinsics (`ref_sensors.md`) directly into Sensor Fusion and Tracking Toolbox objects instead |
| **Scenic** (BerkeleyLearnVerify) | Probabilistic scenario-description language/compiler, official CARLA interface | **BSD-3-Clause** | Strong complement to `scenario_runner` for generating many randomized concrete test cases per scenario type from one spec |
| **RSS** (`ad-rss-lib` via CARLA's `RssSensor`/`RssRestrictor`) | Formally-grounded safe-distance/action evaluation | **LGPL-2.1**, Linux-only, statically linked | Could back a Stateflow safety-supervisor layer conceptually — no direct MATLAB binding found, would need a custom bridge |
| Project Chrono integration | Higher-fidelity multi-physics vehicle dynamics | Open-source (Chrono) | Only worth it if Navigation Toolbox path-tracking needs dynamics beyond CARLA's default PhysX model — **note: does not support collisions** |
| `carla-simulator/carla-agentic-tools` | Official **MCP server** exposing CARLA procedures (Python API, scenario_runner, leaderboard, ROS2, Scenic) as AI-agent-callable skills | MIT | Adopt for your own dev workflow (drive CARLA setup/scenario runs via Claude Code) — not part of the shipped AD pipeline |
| `carla-digitaltwins` | Official UE5 plugin importing real OSM data into interactive 3D digital twins (GDAL + Mitsuba) | Not confirmed | Overlaps with Blender Driving Scenario Creator/`osm2odr` but is UE5-native and official — worth a look if the team ends up on UE5 |
| `carla-studio` | Official Qt desktop app: setup, vehicle import, **sensor calibration GUI**, simulation control | Not confirmed (license file exists, type unverified) | Useful dev-convenience tool |
| `RobotecGPULidar` (CARLA fork) | GPU-accelerated (CUDA/RTX) LiDAR simulation, ROS2/PCL/Unity/O3DE/Gazebo integration | Not confirmed | Alternative LiDAR sensor implementation if CARLA's default LiDAR model proves insufficient |
| `imitation-learning` (official) | Reference CIL agent from the original CARLA paper (Codevilla et al., ICRA 2018) | MIT | Historical baseline only — TF 1.x, legacy CARLA 0.8.2, not competitive with TransFuser-family models already recommended |

---

## Agents & benchmarks (PCLA-bundled families not yet individually researched)

PCLA bundles 41 checkpoints across 18 families (confirmed via `agents.json`). Previously deep-dived: TransFuser v6/LEAD, HiP-AD, SimLingo, ORION, CaRL. Remaining:

| Agent | Venue/Year | Output | Benchmark (best available figure) | License | Integration read |
|---|---|---|---|---|---|
| **NEAT** | ICCV 2021 | Waypoints + semantics (attention-field architecture) | Exact DS/RC table not extractable — cite arXiv:2109.04456 directly | MIT | Old (2021) camera-only IL agent; wrap same as other PCLA agents |
| **InterFuser** (full deep-dive) | CoRL 2022 | Waypoints + interpretable safety maps (density/traffic-rule), post-processed by an internal **safety controller** | Public CARLA Leaderboard (2022): DS 76.18, RC 88.23, IS 0.84. **Not evaluated on Bench2Drive** | Apache-2.0 | Camera+LiDAR, heavier footprint than TransFuser-family. Its internal safety-controller concept parallels a Stateflow decision layer, but isn't separably reusable — only the final waypoint output is exposed |
| **LAV** | CVPR 2022 | Joint perception+prediction+planning+control, trained on *all* observed vehicles' data | Won 2021 CARLA Challenge; public Leaderboard DS 61.85 | Apache-2.0 | Needs LiDAR + multi-camera — heavier than pure-camera options |
| **LBC** | CoRL 2019 | Two-stage distilled IL (privileged → image-based) | NoCrash Town01/02 breakdown available in repo README | MIT | Oldest/lightest, weakest benchmark of the group — best as a cheap baseline for comparison, not primary candidate |
| **WoR** (World on Rails) | ICCV 2021 Oral | Control from camera, via non-reactive-world RL distillation | "+25% over prior SOTA using 40x less data" (exact table not extracted) | MIT | Same author lineage as LAV/LBC, easy to wrap |
| **PlanT / PlanT2** | CoRL 2022 / newer | **Privileged, object-level, planning-only** transformer — assumes perception solved | PlanT: matches expert DS on Longest6 at 5.3x speed of pixel-based baselines. PlanT2: claims SOTA on Longest6 v2/Bench2Drive (exact numbers not extracted) | MIT | **Structurally the closest match to a "planner node"** in a ROS-bridge architecture — pair with a separate perception stack (or CARLA ground truth) and let it do pure behavior planning. Worth prioritizing for architecture study |
| **Roach** | ICCV 2021 | Continuous control, distilled from an RL "coach" | Claims 78% NoCrash-dense success — **but a later paper (arXiv:2306.07957) found its leaderboard-SOTA claim erroneous** (evaluated on a benchmark missing safety-critical scenarios) | **CC-BY-NC 4.0** — non-commercial, more restrictive than the rest | Flag both the license and the debunked-claim caveat if citing it |
| **ThinkTwice** | CVPR 2023 | Scalable decoder: coarse trajectory → Look Module → refine | Town05 Long DS 65.0±1.7; Bench2Drive DS 62.44 (SR 31.23% or 28.14% depending on source — cite paper directly) | Apache-2.0 | Heavier compute (iterative Look/Predict/Refine loop) — benchmark latency yourself |
| **LMDrive** | CVPR 2024 | **LLM-driven** closed-loop control from camera+LiDAR+natural-language instructions | LangAuto-Tiny: DS 70.40, RC 74.92, IS 0.935 (degrades under instruction perturbation) | Apache-2.0 | Directly relevant if Stateflow needs to consume natural-language high-level directives — but 7B-param LLM backbone, heaviest compute of the group, plan for GPU-hosted inference only |
| **MindDrive** | 2025-2026 (self-reported dates) | VLA model, LoRA "Decision Expert" + "Action Expert," online-RL-trained | Bench2Drive: 0.5B DS 78.04/SR 55.09%; 3B DS 80.59/SR 58.26% — **self-reported, not third-party verified** | Apache-2.0 | Newest/highest-scoring VLA in the bundle by its own numbers; verify independently before treating as authoritative |
| **Autoware v1 agent** (PCLA integration) | — | Not a neural model — full ROS2 Autoware stack in Docker, bridged via Zenoh (`evshary/autoware_carla_launch`); PCLA's adapter is a thin pass-through | N/A | Per Autoware's own | **Directly relevant as a reference architecture** — this is the one bundled agent doing exactly what your team is planning (external ROS2 stack, Docker-isolated, bridged to CARLA). Study `autoware_agent.py`/`config.yaml` as a template, including its documented CARLA↔Autoware coordinate-frame (flipped y-axis) gotcha |
| **Bench2Drive** (benchmark suite itself) | NeurIPS 2024 D&B | 44 scenario types × 12 towns, 220 validation routes, official multi-ability metrics (Driving Score, Success Rate, **Efficiency**, **Comfortness**) | — | **CC-BY-NC-ND** — non-commercial, no-derivatives | Treat as **evaluation-only** — compute metrics offline against it, don't embed in a shipped pipeline given the ND license |
| **Town13** | Official Leaderboard 2.0 map | 20 validation routes, ~12.39km avg, unseen-town generalization test | Documented generalization gap between Town13-Train and Town13-Validation (carla_garage/"Hidden Biases" line of work) | MIT (CARLA's standard asset license) | Not a separate tool — an official map, useful as a generalization stress-test once your own Indian scenes exist |
| **Longest6 / Longest6 v2** | TransFuser repo / CaRL paper | 36 routes (6 longest per town, Towns 1-6) | v2 scores **not comparable** to v1 (harder: Leaderboard-2.0 scenario logic, background traffic up to 80km/h); CaRL scores 64 DS on v2 | Inherits hosting repo's license | Route-definition assets living inside `transfuser`/`carla_garage` repos, not standalone packages |

---

## Maps, assets & scenario-authoring tools

### The headline finding

**No India-specific or unstructured-road CARLA map/scenario project exists anywhere** — confirmed after 9 distinct search phrasings. The single closest adjacent result, **Team TwinX (SIH 2025 winner, same PS lineage)**, built their Indian-road digital twin using **MATLAB/Simulink/RoadRunner**, not CARLA, and published no repo. **This is a genuine, citable gap — a legitimate novelty claim for your submission**, not something to keep searching for.

### Best lead for authentic mixed-traffic behavior: SUMMIT

| Component | What it does | License | Integration read |
|---|---|---|---|
| **SUMMIT** (AdaCompNUS) | CARLA-derived simulator purpose-built for dense, unregulated urban traffic — ingests **any** real-world OSM location + SUMO road networks, uses a GAMMA motion-prediction model to generate heterogeneous, interactive, unregulated traffic (mixed vehicles + pedestrians) automatically | Mixed: MIT (code), CC-BY (assets), LGPL-2.1 (RSS), ISC, BSL-1.0 | **The single strongest find of this whole round for your market-area and unsignaled-intersection scenarios** — pull a real Indian town from OSM and get realistic chaotic traffic without hand-scripting NPC behavior. Built for CARLA 0.9.x/UE4 — verify compatibility against whatever CARLA version you standardize on |

### Map geometry sources

| Source | What it offers | License | Integration read |
|---|---|---|---|
| CARLA **Town07** (official) | Rural setting, narrow roads, few signals, many unsignalized crossings | CC-BY | Closest built-in stand-in for an unmarked village road / unsignalized intersection — usable directly or as a geometry base to crop/re-texture |
| CARLA **Town12/Town13** (official) | Large-scale maps with rural regions — unmarked dirt roads, single-lane interurban roads, farmland | CC-BY | Good source geometry for the village-road scenario; large enough to carve a sub-region from |
| CARLA 0.10.0 **off-road mine map** | Unpaved roads, heavy-vehicle traffic (new, Dec 2024, UE5.5) | MIT/CC-BY | Not Indian-styled, but a texture/dirt-road-surface asset donor |
| `Real2sim` (Oxford OATML) | Academic technique for OSM→CARLA-town automation with LiDAR-realistic noise (NeurIPS 2020 workshop) | **No public code found** | Technique reference only, not installable |

### Asset packs (vehicles, pedestrians, props)

| Item | Status | License | Integration read |
|---|---|---|---|
| Auto-rickshaw/tuk-tuk 3D models | Raw models exist (e.g. Sketchfab "Autorikshaw - Indian Tuk Tuk," free) — **none pre-rigged for CARLA** | Varies per listing | Requires following CARLA's `tuto_A_add_vehicle` FBX-import pipeline yourself |
| Animals | No CARLA-specific asset found; closest is **styloo's "Animals Asset Pack"** (itch.io, includes a cow, free, commercial use OK) | Personal+commercial OK, no resale | CARLA has no native "animal" actor class — spawn as a static prop or simple scripted walker per CARLA's custom-assets tutorial |
| `wielgosz-info/carla-pedestrians` | Retargets real pedestrian motion (OpenPose+SMPL from JAAD) onto CARLA walker skeletons for realism | MIT (repo), **but dependencies OpenPose/SMPL/JAAD carry academic/non-commercial restrictions** | Useful for a less-robotic dense-market pedestrian crowd — check dependency licenses carefully |
| Market/stall/cart props | No CARLA-specific pack; generic low-poly assets exist on Sketchfab/itch.io/Fab | Varies per listing | Manual import as static CARLA props per the custom-assets tutorial |

### Traffic/weather diversity tools

| Tool | What it does | License | Integration read |
|---|---|---|---|
| **Scenic** | Probabilistic scenario spec language, official CARLA backend | BSD-3 | Write one program per scenario type (e.g. "cattle crossing" with randomized animal position + approach speed) and sample many concrete test cases including weather |
| SUMMIT's GAMMA model | Diversified unregulated-traffic behavior generation | Same as SUMMIT above | Functional alternative to Traffic Manager for heterogeneous, undisciplined traffic |
| `traffic-generation-editor` (official, QGIS plugin) | Visual placement of vehicles/pedestrians/props + weather conditions on an OpenDRIVE map, exports OpenSCENARIO | MIT | GUI complement to Scenic — better for small curated demo runs than large-N randomized generation |

### OpenDRIVE/OpenSCENARIO authoring beyond Blender Driving Scenario Creator

| Tool | What it is | License | Integration read |
|---|---|---|---|
| `esmini` | OpenSCENARIO **player/runtime**, not an authoring tool — includes `odrviewer` + replayer | MPL-2.0 | Lightweight preview/validation of `.xosc`/`.xodr` files before loading into full CARLA/UE runtime |
| **`scenariogeneration`** (pyoscx) | Python library to **programmatically generate** linked OpenDRIVE+OpenSCENARIO files, integrates with esmini for preview | MPL-2.0 | Strong fit for parametrically scripting your 5 scenarios (e.g., N merge-angle variants) rather than hand-modeling each in Blender |
| `OpenScenarioEditor` (ebadi) | Simple `.xosc` GUI editor built on esmini | Not confirmed | Lightweight scenario-tweaking, not a full authoring tool |
| `carla-map-editor` (official) | Post-processes RoadRunner/hand-authored FBX+XODR to add traffic lights/signs, re-exports XODR | MIT (code) / CC-BY (assets) | Directly useful for your unsignaled-intersection scenario (deliberately omit signals) and highway-merge speed-sign placement |
| `brifsttar/OpenDRIVE` | Unreal Engine plugin for in-engine OpenDRIVE editing | Not confirmed | Native-feeling alternative to Blender since CARLA itself runs on Unreal |
| `OpenDriveOnlineEditor`, `odrviewer` | Viewers only (editing not implemented / core engine closed-source) | — | Low value beyond quick `.xodr` sanity-checks |

**Not found**: an "awesome-carla" list (`Amin-Tgz/awesome-CARLA` exists but is tutorial/paper-focused, no new tool/asset/map entries beyond what's already covered); the forum.carla.org "Download community maps" thread exists but its server refused every fetch attempt — worth checking manually, not resolved by this search.

---

## Bottom line for the PS's 5 scenarios

- **Village road (unmarked)**: crop/reuse Town07/Town12/Town13 geometry, or feed a real Indian OSM extract through **SUMMIT**.
- **Unsignaled urban intersection / dense market**: **SUMMIT** is the best lead — real OSM ingestion + unregulated heterogeneous-agent behavior — layer market-stall props and Scenic-driven pedestrian density on top.
- **Highway merge**: standard CARLA/OpenDRIVE authoring via `scenariogeneration` or Blender Driving Scenario Creator is sufficient, no special asset needed.
- **Cattle-crossing event**: no ready CARLA asset exists — hand-import the itch.io cow model as a walker/prop, script the trigger via Scenic or `scenariogeneration`/OpenSCENARIO.
- **The India-specific CARLA gap is real and confirmed** — a legitimate differentiator to state directly in your submission.

---

## Cross-reference index

Already documented in the earlier research rounds — **not repeated in this catalog**:

- `docs/sota-research.md` / `docs/component-deep-dive.md`: TransFuser v3–v6/LEAD, HiP-AD, SimLingo/CarLLava, ORION, CaRL, PCLA (deployment mechanics), `carla_garage`, Fail2Drive, DriveE2E, TaCarla, IDD-PeD, MoFlow, BMD-45, DriveIndia, IDD-3D, AutoNUE/IDD, DATS_2022, METEOR, IndiGo, OnSiteVRU.
- `docs/tooling-decisions.md`: Blender Driving Scenario Creator, SUMO co-simulation (`netconvert_carla.py`), CARLA's `osm2odr`, RoadRunner (MathWorks licensing).
- Checked and confirmed **not CARLA-based** (near-misses worth noting so they aren't mistakenly revisited): `autonomousvision/navsim` (nuPlan/OpenScene-based), `autonomousvision/tuplan_garage` (nuPlan-based).

## Updated integration notes

Nothing here overturns `docs/component-deep-dive.md`'s recommended stack (MoFlow for prediction, TransFuser v6/LEAD or SimLingo via PCLA for the planning front-end, your own YOLOv8/v11 fine-tuned on DATS_2022 for perception). What this round **adds** to that plan:

1. **Scene authoring**: adopt **SUMMIT** as a serious alternative/complement to plain `osm2odr` import for the market-area and intersection scenarios — it solves both geometry *and* realistic chaotic traffic behavior in one tool, which `osm2odr` alone does not.
2. **Message compatibility**: pull in **`ros-carla-msgs`** directly rather than hand-rolling ROS message parsing on the MATLAB side.
3. **Architecture reference**: study **PCLA's Autoware v1 integration** (`autoware_agent.py`) as a concrete template for your own ROS2-bridge-in-Docker approach.
4. **Planner-role candidate**: **PlanT2** is worth a closer look specifically because it's structurally an object-level, planning-only agent — the cleanest conceptual match to a "planner node" in your architecture, more so than the end-to-end vision agents already recommended.
5. **Scenario scripting**: use **Scenic** and/or **`scenariogeneration`** for parametric, randomized generation of your 5 scenarios' concrete test instances, rather than hand-authoring every variant.
6. **License housekeeping**: two new flags — `carla_dataset_tools` (GPL-3.0) and Bench2Drive itself (CC-BY-NC-ND, eval-only) — add these to whatever license-tracking your team keeps alongside the ones already flagged in `docs/component-deep-dive.md` (METEOR's MIT/CC-BY-4.0 conflict, IDD-PeD's unlicensed code repo).
