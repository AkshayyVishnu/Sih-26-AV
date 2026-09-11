# Component Deep-Dive — SIH 2026, PS 26037

## Purpose

This extends `docs/sota-research.md`'s shortlist with implementation-level
depth, oriented specifically around the PS's three named grading metrics —
**replanning latency, path smoothness, scenario completion rate** — plus the
other required deliverables (simulation model, designed scenarios, technical
report, demonstration video, closed-loop validation). Every figure below is
sourced from a direct fetch of the candidate's own paper/repo/model card;
anything not found after a genuine attempt is marked **"not reported"**
rather than estimated, carrying forward the same sourcing discipline as
`docs/sota-research.md`.

**Headline change from the previous round**: BMD-45, `sota-research.md`'s
top perception pick, turns out to be a **fixed-CCTV-camera detector**, not an
ego-vehicle detector — a domain mismatch serious enough to downgrade it. The
recommended stack at the bottom of this document reflects that and a few
other updates; everything unchanged is stated explicitly too.

---

## Perception

| Candidate | Latency | Fine-tuning specifics | License | Accuracy/robustness metric | Updated verdict |
|---|---|---|---|---|---|
| **BMD-45** | Not reported anywhere (dataset card, model card, paper) | COCO-JSON + HF `ImageFolder` layout; configs+weights (2.73GB) on HF, but **no training script or `requirements.txt` in the repo** | Dataset CC BY 4.0; code Apache-2.0 **but the YOLOv12 weights it ships inherit AGPL-3.0 upstream** — real redistribution risk | 83.8% mAP in-domain vs. 33.6% cross-domain (UA-DETRAC) — a strong quantified domain-gap proxy | **Downgraded** — fixed CCTV/oblique-angle cameras, not ego-vehicle forward-facing view. Useful only as a 14-class India-vehicle taxonomy source, not a drop-in ego-camera detector |
| **DriveIndia** | Not reported | YOLO `.txt` format, 53,586/6,700/6,700 split; base checkpoint/epochs not disclosed | **Gated EULA**, non-commercial — still not publicly downloadable (form → sign EULA → manual approval, turnaround unknown) | Auto-rickshaw mAP50 **0.940**, pushcart mAP50 **0.391** (hard class), animal mAP50 **0.769** — exactly the PS's named object classes | **Request access now if pursuing** — don't treat as a guaranteed fallback given unconfirmed turnaround time |
| **IDD-3D** | Not reported | **Repo is a stub** — README literally says "coming soon," no training scripts, no data-format spec despite being public since WACV 2023 | CC BY-SA 4.0 (secondary-sourced only, code-level LICENSE file not found) | Not extractable (PDF tables inaccessible as text) | **Not currently usable** — deprioritize entirely unless the team adds a LiDAR channel later and the kit matures |
| **AutoNUE/IDD core** | Not reported | Label-prep/eval toolkit only (Cityscapes-style layout, 4-level IDD taxonomy) — **not a training pipeline** | **`license: null`** on the code repo (confirmed via GitHub API) — do not redistribute without contacting authors; IDD dataset itself reportedly CC BY-NC-SA per secondary sources only, primary site unreachable this pass | Not found (paper PDF unreadable as text) | **Tooling only** — use for label remapping/class-taxonomy, train your own model separately on the remapped data |
| Animal/cattle detection | — | — | — | — | **Gap confirmed smaller than assumed but still no strong dedicated India-specific paper.** Best actually-downloadable option: **DATS_2022** (Mendeley, no gating, 45 classes incl. cattle/goat/dog/camel/horse) |

**Cross-cutting perception finding**: none of the five candidates report a usable inference-latency number from any primary source — this is the single biggest gap against the "replanning latency" metric at the perception stage. **You must benchmark your chosen checkpoint yourselves** on your actual target hardware; do not cite an estimated number in your report.

**Updated best-available path**: fine-tune your own YOLOv8/v11 on **DATS_2022 + IDD 2D detection data** (both genuinely downloadable today, unlike BMD-45's domain-mismatched weights or DriveIndia's gated access) — this is a concrete, unblocked path rather than the previous round's BMD-45 recommendation.

---

## Prediction

| Candidate | Latency | Input/output | License | Smoothness/quality metric | Updated verdict |
|---|---|---|---|---|---|
| **IDD-PeD** | Not reported anywhere (repo or paper) | Pedestrian-specific: RGB + bbox + occlusion + **17-point pose (via MMPose)** + behavioral annotations → intention (classification) or trajectory (BiTraP/SGNet_CVAE/MTN/PIEPredict) | Dataset CC BY 4.0 (paper-stated); **code repo has no LICENSE file at all** (`null`) — real risk for redistributing modified baseline code | PCPA intention: AUC 0.86→0.71, F1 0.77→0.33 (PIE→IDD-PeD, −15/−44 pts); Traj MSE: PIETraj 559→2181, SGNet 88→310 — quantifies the exact structured-vs-unstructured domain gap this PS is about | **Use as base, but only its trajectory-only baselines** (BiTraP/SGNet_CVAE) — these are agent-type-agnostic at the input level. The pose-branch models are **not** extendable to two-wheelers/animals without real architecture surgery (MMPose's human-skeleton assumption, pedestrian-specific behavior taxonomy) — confirmed from the actual model code, not guessed |
| **MoFlow** | **0.70 ms/prediction** (IMLE student, RTX6000/A40) vs. 33.20 ms (teacher) | Generic BEV coordinate history (8–12 frames) → K=20 multimodal future trajectories — **not pedestrian-specific**, works for any tracked agent class | **MIT** — clean | min20ADE/FDE: NBA 0.71/0.87, ETH-UCY avg 0.20/0.32 | **Strongest real-time candidate found across the entire survey.** Never validated on Indian/unstructured traffic — needs fine-tuning, but its generic input format makes it the more flexible base of the two |
| **METEOR** (data source) | N/A | XML bbox+GPS tracks, 16 agent categories, 17 behavior types, Hyderabad | **License conflict**: MIT per HuggingFace card vs. CC BY 4.0 per the paper — **verify directly before redistributing anything derived from it** | N/A | Usable as fine-tuning/validation data for MoFlow or IDD-PeD's trajectory-only baselines, but **no existing conversion script produces ready-made training pairs** — `xml2coco.py`/`xml2rawframe.py` target detection/classification, not trajectory forecasting. Budget for writing this converter yourselves |
| **IndiGo** | — | — | — | — | **Confirmed inaccessible** — paywalled Springer article, no dataset link, download URL, or repo found anywhere despite a genuine attempt. Downgrade from "watch-list" to **not usable** |
| **OnSiteVRU** | — | — | CC BY-NC-SA 4.0 (non-commercial) | — | **Confirmed accessible** — public Kaggle download link stated directly in the paper. Shanghai, not India, but structurally relevant mixed-traffic trajectories (motor vehicles, e-bikes, bicycles) |

**Extending IDD-PeD beyond pedestrians — the concrete answer**: this requires real architecture changes, not relabeling, for two specific reasons found in the actual repo/paper: (1) the pose branch is built on MMPose's human-skeleton keypoints, which have no equivalent for a two-wheeler or animal; (2) the behavioral taxonomy (crossing intention, carrying-object, crosswalk usage) is semantically pedestrian-specific. The trajectory-only baselines (BiTraP, SGNet_CVAE) skip the pose branch entirely and are the realistic path to extend, provided you build an extended-agent-type training set (e.g., via the METEOR conversion above).

---

## Planning / End-to-end CARLA agents

| Candidate | Closed-loop latency | Bench2Drive DS / SR | Comfort metric | License | Sensors | In PCLA? | Updated verdict |
|---|---|---|---|---|---|---|---|
| **HiP-AD** | **138.9 ms / 7.2 FPS** (RTX 3090) | **86.77 / 69.09%** | **Comfortness 19.36, Efficiency 203.12** | Apache-2.0 | 6 cameras, 640×352, no LiDAR | **No** — not bundled, needs a custom wrapper | Best combination of *reported* latency + comfort metric of anything surveyed — but the only one requiring you to build your own deployment wrapper |
| **TransFuser v6 / LEAD** | Not reported | **95.0±0.7 / 84.3±2.1%** (best of all candidates); Longest6 v2 DS 62/RC 91; Town13 (unseen) DS 4.04/RC 39.70 | Not reported | **MIT** | 360° camera + LiDAR + Radar (best config) | **Yes** — `tfv6_*` variants | Strongest completion-rate numbers found anywhere, cleanest license, and already wrapped in PCLA — but no latency figure, and needs the heavier full sensor rig for its best score |
| **SimLingo** | Not reported | 85.07±0.95 / 67.27±2.11%; Leaderboard 2.0 DS 6.87 (much harder benchmark) | **Comfortness 33.67±5.72 — best comfort number found in the whole survey** | Apache-2.0 | **Vision-only, single front camera** — lightest sensor rig of any candidate | **Yes** — `simlingo_simlingo` | Best comfort metric + lightest sensor requirement + already in PCLA — a strong all-around pick despite no latency figure |
| **ORION** | **Explicitly not reported — paper itself flags real-time VLM compute complexity as a limitation** | 77.74 / 54.62% | Not reported | Apache-2.0 | Camera-only, 640×640 | **Yes** — `orion_base` | Weakest real-time story of the four (self-acknowledged), and needs 32GB VRAM FP32 / 17GB FP16 — **deprioritize for a real-time hackathon demo** |
| iPLAN / CarPlanner (recheck) | — | — | — | — | — | No change from prior round — iPLAN still has no CARLA integration, CarPlanner still has no released code |

**PCLA mechanics, now fully concrete**: it does **not** manage the CARLA server itself (you start `CarlaUE4.sh` and hand it a `carla.Client` yourself); its Python API is genuinely minimal:
```python
from PCLA import PCLA
pcla = PCLA(agent, vehicle, route, client)   # agent = string id from agents.json
while running:
    ego_action = pcla.get_action()
    vehicle.apply_control(ego_action)
pcla.cleanup()
```
Tested against CARLA 0.9.16 (0.9.15 recommended for best agent compatibility). Apache-2.0, clean license. **41 bundled agent variants**, including 3 of your 4 shortlisted models (TransFuser v6/LEAD, SimLingo, ORION) — HiP-AD is the one exception requiring a hand-built wrapper.

---

## Metrics-alignment table

Directly mapping the strongest candidates to the PS's three named grading metrics:

| Metric | Best evidence found | Candidate |
|---|---|---|
| **Replanning latency** | 0.70 ms/prediction (prediction stage) | MoFlow |
| | 138.9 ms / 7.2 FPS closed-loop (planning stage, confirmed on real hardware) | HiP-AD |
| | Everything else | **Not reported — must self-benchmark on your own target hardware before citing any figure** |
| **Path smoothness** | Comfortness 33.67±5.72 (Bench2Drive sub-metric) | SimLingo |
| | Comfortness 19.36, Efficiency 203.12 | HiP-AD |
| | Everything else (TFv6/LEAD, ORION, all perception/prediction candidates) | **Not reported** |
| **Scenario completion rate** | Driving Score 95.0±0.7 / Success Rate 84.3±2.1% (Bench2Drive) | TransFuser v6/LEAD |
| | Driving Score 86.77 / Success Rate 69.09% | HiP-AD |
| | Driving Score 85.07 / Success Rate 67.27% | SimLingo |
| | Driving Score 77.74 / Success Rate 54.62% | ORION |
| | Domain-gap proxy: 83.8% in-domain vs 33.6% cross-domain mAP (perception stage) | BMD-45 |
| | Per-class accuracy proxy: auto-rickshaw 0.940, pushcart 0.391, animal 0.769 mAP50 | DriveIndia (pending access) |

Note none of Bench2Drive's "Driving Score"/"Success Rate" or CARLA Leaderboard's "Route Completion" are literally the PS's "scenario completion rate" — they're the closest published analogs, useful for setting expectations and comparison baselines, but your own scenario-completion metric (did the vehicle complete each of your 5 specific Indian scenarios without collision) will need to be measured directly in your own closed-loop runs regardless of which base model you adopt.

---

## Updated recommended stack

Revising `docs/sota-research.md`'s closing recommendation where these deeper findings actually change it:

1. **Perception — changed.** Drop BMD-45 as the primary pick (domain-mismatched, fixed-camera). Fine-tune your own YOLOv8/v11 on **DATS_2022** (downloadable now, includes animal classes) plus IDD 2D detection data (via AutoNUE's label-remapping tooling only, training your own model separately given the toolkit's `license: null`). Request DriveIndia access now as a stronger future upgrade if it clears in time — it has the best per-class numbers for exactly the PS's named object types.
2. **Prediction — refined, not reversed.** MoFlow (0.70ms, MIT license) is now the clearer real-time backbone given its confirmed sub-millisecond latency and generic agent-agnostic input format. Use IDD-PeD specifically for its trajectory-only baselines (BiTraP/SGNet_CVAE) as a India-grounded fine-tuning comparison/ensemble partner, not its pose-branch models. Both will need fine-tuning on METEOR-derived or your own CARLA-generated data — budget for writing the METEOR→training-pairs converter, since none exists today.
3. **Planning front-end — sharpened.** **TransFuser v6/LEAD via PCLA** remains the top pick given its clearly best completion-rate numbers, cleanest license (MIT), and zero-wrapper-effort integration (already bundled). **SimLingo via PCLA** is the strongest alternative if you value its best-in-survey comfort metric and lightest sensor rig (single camera, easier CARLA setup) over raw completion rate. **HiP-AD** is worth the extra wrapper-building effort only if its confirmed real hardware latency number (138.9ms/7.2FPS) is decisive for your real-time budget — it's the only candidate with both a real latency figure and a real comfort figure together. **Deprioritize ORION** — its own paper flags real-time infeasibility.
4. **Decision logic — unchanged.** Stateflow as the top-level mode-switcher, with iPLAN's intent-aware MARL architecture kept as a design pattern for a custom RL Toolbox agent if pursued, per `docs/tradeoffs-a-vs-b.md`.
5. **Classical fallback — unchanged and still load-bearing.** Navigation Toolbox's classical planners remain the zero-training safety net if any AI component above underperforms or integration time runs short.

---

## Remaining unknowns

- **Closed-loop latency for TransFuser v6/LEAD, SimLingo, and ORION** — none report this despite reporting Driving Score/Success Rate. You will need to benchmark these yourselves once deployed via PCLA.
- **IDD-PeD baseline latency** — not reported anywhere; same self-benchmarking requirement.
- **AutoNUE/IDD's actual dataset license** — the primary site (insaan.iiit.ac.in) was unreachable this pass; the CC BY-NC-SA claim is secondary-sourced only and needs direct verification before your report cites it.
- **METEOR's license** — genuine conflict between its HuggingFace card (MIT) and its paper (CC BY 4.0), unresolved; verify directly against the actual dataset repository metadata before redistributing anything derived from it.
- **DriveIndia's access turnaround time** — EULA-gated, approval timeline not stated anywhere; request access immediately if you want any chance of using it before the deadline.
- **A dedicated, well-benchmarked, India-specific animal/cattle road-hazard detector** — confirmed still not to exist at a strong venue; DATS_2022 remains the practical, already-downloadable fallback.
