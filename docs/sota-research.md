# SOTA Literature Research — SIH 2026, PS 26037

## Scope & method

Surveyed via parallel web research (three focused sweeps: perception/prediction,
open-source CARLA driving agents, planning/decision-making) across CVPR 2025,
CVPR 2026 (including its Workshop on Autonomous Driving and DriveX), and
adjacent top venues (ICRA, ICCV, CoRL, IROS, RA-L) where CVPR itself had
nothing relevant — flagged explicitly wherever a result isn't CVPR. Every
compute figure below is sourced from the paper/repo's own reporting; anything
not confirmed is marked **"not reported"** rather than estimated. This is
measured against a compute ceiling of **~1800 legitimate GPU-hours** across
Kaggle+Colab (individual team-member accounts, not the multi-account approach
flagged earlier as a Kaggle ToS problem) plus rented GPUs (RunPod/Vast.ai) as
a scalable option where a specific candidate justifies the cost.

---

## Perception

| Model/Dataset | Venue/Year | What it does | Open-source status | Compute to reproduce/fine-tune | Integration path (Path B) | Constraints & tradeoffs | Verdict |
|---|---|---|---|---|---|---|---|
| **BMD-45** | CVPR 2026 Findings | 480K boxes / 45K images, Bengaluru CCTV, 14 vehicle classes incl. auto-rickshaw | Dataset + baseline weights (YOLOv12-S/X, RT-DETRv2, D-FINE, RF-DETR) on HuggingFace | A6000/A100, 100 epochs, batch 16 — exact hours not reported | Fine-tune pretrained weights, run as PyTorch ROS detection node publishing boxes into the ROS bridge | CCTV top-down viewpoint ≠ ego-vehicle camera angle — needs fine-tuning on CARLA-rendered ego-view frames | **Fine-tune as base** — best available starting weights for Indian vehicle classes |
| **DriveIndia** | ITSC 2025 (not CVPR) | 66,986 images, 24 classes incl. auto-rickshaw + pushcart, urban/rural/highway India | Dataset pending public release (TiHAN-IIT Hyderabad); no confirmed training-code repo | Not reported | Fine-tuning dataset once public | Release timing uncertain — check status before committing to it | **Fine-tune as base**, pending public release |
| **IDD-3D** | WACV 2023 (not CVPR) | 12k LiDAR frames, ~223k 3D boxes, 17 categories, unstructured Indian traffic | Code + data released | Not reported | PointPillars/CenterPoint baselines as a LiDAR-detection ROS node (needs LiDAR added to ego vehicle in CARLA) | Only relevant if your sensor suite includes LiDAR (PS names it as an option) | **Fine-tune as base**, LiDAR-dependent |
| **AutoNUE / IDD (core)** | ECCV'18/ICCV'19 workshops, ongoing | Foundational unstructured-India segmentation/detection benchmark | Public dataset + toolkit | Not reported | Source of pretrained segmentation models (drivable-area) for ONNX import into Deep Learning Toolbox | Older baseline, superseded by newer datasets above for detection specifically | **Use as base** for segmentation/drivable-area only |
| Animal (cattle) road-hazard detection | — | No CVPR/peer-reviewed paper found targeting this specifically for India | Only general cattle-breed-ID work and a Roboflow community dataset exist | N/A | N/A | Genuine open gap | **Not feasible to reuse — build this class yourselves** into your BMD-45/DriveIndia fine-tuning |

---

## Prediction

| Model | Venue/Year | What it does | Open-source status | Compute | Integration path | Constraints & tradeoffs | Verdict |
|---|---|---|---|---|---|---|---|
| **IDD-PeD** | ICRA 2025 (not CVPR) | Pedestrian intention/trajectory prediction, built explicitly on IDD's unstructured/unsignalized Indian traffic; benchmarks 5 baselines, quantifies up to 15% performance drop vs. structured-traffic datasets | Code **+ pretrained checkpoints** released | Not reported (GPU); only "32GB CPU RAM" noted | Run baseline (SGNet/BITRAP etc.) as a standalone ROS pedestrian-prediction node, publishing predicted paths to Simulink/Stateflow | Pedestrian-only out of the box — would need extending to two-wheelers/animals for full PS scope | **Use as base — strongest single candidate found across the entire survey** |
| **MoFlow** | CVPR 2025 (main) | One-step flow-matching multi-agent trajectory forecasting, fast/low-latency | Code + pretrained checkpoints (NBA, ETH-UCY, SDD) | Not reported | Standalone ROS node; fast enough for real-time replanning loop | Trained on structured Western pedestrian scenes — needs fine-tuning on IDD-PeD-style data | **Fine-tune as base** — good speed complement to IDD-PeD |
| TrajGNAS | CVPR 2025 WAD | Heterogeneous multi-agent trajectory prediction via GNN architecture search | No code released | Not reported | Reimplementation only | Architecture-only | **Pattern only, not feasible as drop-in** |
| SocialMOIF | CVPR 2025 (main) | Group + individual pedestrian intention fusion | No code released | Not reported | Reimplementation only | Architecture-only | **Pattern only** |
| IndiGo (two-wheeler dataset) | Discover Robotics 2025 | Multi-modal two-wheeler dataset, Bhubaneswar, unstructured traffic | Access status unconfirmed | Not reported | Potential fine-tuning data if access confirmed | Directly relevant to PS's two-wheeler mention, but usability blocked pending confirmation | **Watch-list — confirm access before relying on it** |

---

## Planning & decision-making

| Model | Venue/Year | What it does | Open-source status | Compute | Integration path | Constraints & tradeoffs | Verdict |
|---|---|---|---|---|---|---|---|
| **CarPlanner** | CVPR 2025 | RL auto-regressive trajectory planner + rule-based safety selector + emergency-stop fallback, beats IL/rule-based on nuPlan | No code released | **2×RTX 3090, 50 epochs** (wall-clock not stated) | Reimplement the mode-selector + safety-gate concept as a custom MATLAB RL Toolbox agent | Architecture-only, but concept is directly portable to Stateflow + RL Toolbox design | **Pattern only — reimplement, don't wait for code** |

| **DiffusionDrive** | CVPR 2025 (Highlight) | Truncated diffusion model, real-time multimodal trajectory generation, 45 FPS on RTX 4090 | Code + weights released (NAVSIM/nuScenes) | Not reported | Architectural reference / PyTorch side-channel | Structured-road/lane-graph input assumption — limited direct reuse for unmarked roads without re-engineering input representation | **Fine-tune as base only with rework**, or pattern-only |


| GoalFlow | CVPR 2025 | Goal-driven flow-matching trajectory generation | Weights released, **training code not released** | Not reported | Inference/evaluation reference only | Can't fine-tune without training code | **Reference only, not a fine-tuning base** |


| Lane-free MCTS (NN-guided) | arXiv 2026 (not CVPR) | MCTS + NN guidance for lane-free single-agent driving | Not confirmed open-source | Not reported | Pattern for MCTS + RL Toolbox decision layering | Motivated by European lane-free-traffic research, not India, and unconfirmed release | **Pattern only** |
    
| **iPLAN** | CoRL 2023 (not CVPR) | Decentralized MARL, intent-aware collision avoidance for heterogeneous traffic (Behavioral + Instant Incentive decomposition) | Full training code released (MIT), no pretrained checkpoints | Not reported | **Best RL-pattern candidate** — reimplement architecture as custom MATLAB RL Toolbox agent | Built on `highway-env`, not CARLA — needs re-implementation of the environment interface, not a drop-in | **Architectural base for a custom RL Toolbox agent** |
| B-GAP | IROS/RA-L 2022 (not CVPR) | DRL policy navigating aggressive-driver behaviors (vehicles only) | Training code released, no weights | Not reported | Same re-implementation path as iPLAN | Vehicle-only — no pedestrian/animal handling despite "heterogeneous" framing | **Pattern only, vehicle-collision-avoidance layer** |
| DenseCAvoid / Frozone | ICRA 2020 / ~2020-21 (GAMMA/UMD) | Anticipatory dense-crowd navigation / freezing-robot-problem avoidance | Not verified in this pass | Not reported | Conceptual reference for market-area/intersection scenarios | Unverified release status | **Pattern only, needs direct repo check before relying on it** |

---

## Open-source end-to-end / modular CARLA driving agents

| Model | Venue/Year | Output format | Open-source status | Compute | Integration path | Constraints & tradeoffs | Verdict |
|---|---|---|---|---|---|---|---|
| **TransFuser v6 / LEAD** | arXiv 2025, CVPR26 (pending) | Waypoints/trajectory (BEV planner) | Code, weights, training code, **8,930-route dataset** (HuggingFace) all released | **4×H100, <24h train**; data collection <1 day on a 64-GPU cluster | PyTorch ROS node, wrap CARLA ros-bridge camera/LiDAR topics → publish trajectory to Simulink/Stateflow | Not tested on non-Western/unstructured roads — needs your own fine-tuning data | **Use as base — strong fit, needs real GPU rental (H100-class) for full retrain, lighter for fine-tuning only** |
| **HiP-AD** | ICCV 2025 | Waypoints (multi-granularity planning query) | Code, pretrained (stage-2) weights, training code released | **8×RTX 4090, Stage1 ~14h + Stage2 ~46h ≈ 60h total** | Same ROS-node pattern | **Trains on consumer-grade 4090s** — the most compute-realistic full-retrain option found in this entire survey | **Use as base — best compute-fit for your budget** |
| **SimLingo (CarLLava)** | CVPR 2025 (Spotlight) | Waypoints + language VQA/commentary/instruction heads | Code, weights, training code, 3.3M-sample dataset | Not reported for final model | ROS node; language channel is a bonus for Stateflow explainability | Adds VLM inference latency (InternVL2-1B+Qwen2-0.5B) — check real-time budget | **Fine-tune as base**, watch latency |
| **ORION** | ICCV 2025 | Trajectory (generative, LLM-reasoning-aligned) | Code, checkpoints, training code, Chat-B2D dataset released | Not reported | ROS node | Heavier (LLM-in-the-loop) — check inference latency for real-time ROS use | **Fine-tune as base**, watch latency |
| CaRL | CoRL 2025 (not CVPR) | **Raw control** (steer/throttle-brake), not trajectory | Code + weights + training code, includes Roach/PlanT/Think2Drive reproductions | 8×A100 (40GB) + 108 CPU cores, ~1 week for 300M samples | Reactive controller only, or repurpose intermediate BEV features | Raw-control output is a **poor fit for the modular pipeline** the PS requires | **Not recommended as primary** — output format conflicts with PS's modular-stages requirement |
| **PCLA** (deployment framework) | FSE 2025 (ACM demo) | N/A — wrapper | Code released; bundles **36 pretrained agents** (SimLingo, TransFuser v3-v6, CaRL, InterFuser, PlanT, NEAT, WoR, LBC, LAV, etc.) + weights via Zenodo | N/A | Deploys any bundled Leaderboard agent as a standalone Python agent, decoupled from the Leaderboard codebase, tested on CARLA 0.9.16 | None significant | **Adopt as the deployment harness** — fastest path to standing up any agent above behind your ROS interface |

---

## Indian-road-specific prior work

The dedicated, closest-match body of research to PS 26037, consolidated from all three sweeps:

| Work | Venue/Year | What it is | Open-source | Relevance |
|---|---|---|---|---|
| **METEOR dataset** | arXiv 2021 / ICRA 2023 (GAMMA/UMD, Hyderabad) | 100GB, 1000+ clips, 2M+ frames, 13M+ boxes, up to 40 agents/frame, rare behaviors, rural unmarked roads | Dataset + code released | Most relevant real-world unstructured-Indian-traffic data source found; good for CARLA scenario authoring or robustness benchmarking |
| **TraPHic** | **CVPR 2019** | Trajectory prediction in dense/heterogeneous traffic — one of the few genuinely-CVPR, India-motivated papers | Check repo directly | Prediction, not planning, but CVPR-venue and directly on-theme |
| **IDD-PeD** | ICRA 2025 | See Prediction table above | Code + checkpoints | Standout actionable resource |
| **IDD-3D** | WACV 2023 | See Perception table above | Code + data | 3D unstructured-scene perception |
| **DriveIndia** | ITSC 2025 | See Perception table above | Dataset pending | 24-class Indian detection |
| **BMD-45** | CVPR 2026 Findings | See Perception table above | Data + weights | Bengaluru CCTV heterogeneous-vehicle detection |
| Rohan Chandra, UMD PhD Thesis (2022) | — | *"Towards Autonomous Driving in Dense, Heterogeneous, and Unstructured Traffic"* — title virtually mirrors PS 26037; aggregates the GAMMA lab's prior perception/prediction/planning work | Public thesis | **Closest single framing to this PS found anywhere** — but a synthesis of prior papers, not one new unified planner |
| IndiVNet | Scientific Reports 2025 | Region-adaptive segmentation for Indian unstructured roads | Check directly | Drivable-area segmentation |
| R-3D-YOLOv3 | MDPI | Indian roadway animal detection (perception only, no planning integration) | Check directly | Closest thing to animal-detection prior art, still isolated |

**Global South framing**: an explicit search for planning/ML research framed around "Global South" or "developing country" autonomous driving surfaced no CVPR/ML papers — only transportation-policy literature stating outright that *"no significant work has been carried out to date on the potential deployment of autonomous vehicles in the Global South."* This independently confirms the gap is real, not just under-searched.

---

## Explicit gap-check verdict (from all three research sweeps, independently confirmed)

- **No published paper — CVPR or otherwise — solves PS 26037 as posed.** The closest match is the GAMMA/UMD (Manocha/Chandra) body of work, which is split across perception, prediction, and planning papers rather than one unified adaptive real-time replanner validated on scenarios like yours.
- **No CARLA agent, of any kind, has been tested on or adapted for Indian/unstructured road conditions.** Confirmed independently by the CARLA-agents sweep (Fail2Drive, DriveE2E, TaCarla all still Western-structured) — this is a genuine, unaddressed gap, meaning your Indian-scenario CARLA work would be novel, not a reproduction.
- **No CVPR-venue paper specifically detects animals-on-road for India.** You'll need to build this class into your own fine-tuning data regardless of which base model you pick.
- **Compute numbers are almost never reported** by the papers themselves — HiP-AD, CarPlanner, TransFuser v6/LEAD, and CaRL are the exceptions with real figures; budget your own reproduction against those four as reference points, not the unreported ones.

---

## Recommended integration stack

Given the compute ceiling (~1800 GPU-hours across Kaggle+Colab, rented GPUs available for anything that specifically justifies it) and the Path B architecture (`docs/tradeoffs-a-vs-b.md`: Simulink core + CARLA via ROS bridge):

1. **Perception**: Fine-tune **BMD-45**'s released YOLOv12/RT-DETR/RF-DETR weights on your own CARLA-rendered Indian scenes (ego-view, not BMD-45's CCTV angle) — add the animal class yourselves, since no existing model has it. Run as a PyTorch ROS detection node.
2. **Prediction**: Use **IDD-PeD**'s released code and checkpoints as the base — the strongest single match in the entire survey, real Indian unstructured-traffic data, code and weights both available. Extend beyond its pedestrian-only scope to cover two-wheelers/animals for full PS coverage. Optionally pair with **MoFlow** (fine-tuned) if IDD-PeD's baselines are too slow for your real-time loop.
3. **Planning/trajectory front-end**: Deploy **HiP-AD** via **PCLA** as your trajectory-generating model — it's the most compute-realistic full-retrain option found (8×RTX 4090, ~60h total), directly producing waypoints that hand off cleanly to your Simulink/Stateflow decision layer. **TransFuser v6/LEAD** is the stronger but heavier alternative (needs H100-class rental) if HiP-AD underperforms on your scenes.
4. **Decision logic**: Keep Stateflow as the top-level mode-switcher (as already planned), informed by the trajectory/predictions above. If you want a learned component here specifically, use **iPLAN**'s intent-aware MARL architecture as a design pattern for a custom MATLAB RL Toolbox agent — not a drop-in, since it's `highway-env`-based, but directly portable as an architecture.
5. **Novelty/validation framing**: Build your own Indian-condition CARLA scenes (village road, market area) informed by **METEOR**'s real Hyderabad data, and stress-test your fine-tuned stack the way **Fail2Drive** stress-tests standard CARLA agents — this gives you a defensible, literature-grounded "gap we're closing" narrative for judges, consistent with the confirmed finding that no existing open-source system has been tested this way.
6. **Classical fallback stays load-bearing, not obsolete**: Navigation Toolbox's classical planners (`docs/tradeoffs-a-vs-b.md`) remain the safe, zero-training core if any of the above underperforms or integration time runs short — the AI models above are upgrades layered on top of, not replacements for, the already-agreed baseline architecture.

---

## What was excluded, and why

- **CarPlanner, GoalFlow's fine-tuning path, TrajGNAS, SocialMOIF, U2Diff** — architecture-only or weights-only with no training code; would require full reimplementation and training from scratch, not a compute-realistic use of the budget. Kept only as design-pattern references.
- **CaRL** — technically strong (real compute figures, real training code) but its raw-control output format conflicts with the PS's required modular perception/prediction/planning/decision-logic structure; using it as primary would risk the same "not really modular" scoring concern raised earlier for end-to-end approaches.
- **iPLAN, B-GAP, DenseCAvoid, Frozone** — genuinely good architectural references for heterogeneous-agent collision avoidance, but built on `highway-env`/custom particle environments, not CARLA — excluded as direct-integration candidates, kept as patterns to reimplement inside a custom RL Toolbox agent.
- **Proprietary/closed models** (e.g., Wayve LINGO-class systems) — not evaluated at all; no released weights or code exist to assess.
- **IndiGo, OnSiteVRU, PINNS** — flagged as watch-list only; data-access status unconfirmed as of this research pass, don't commit engineering time until confirmed.
