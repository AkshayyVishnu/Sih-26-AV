# Model Acquisition Plan — SIH 2026, PS 26037

## Purpose

`docs/architecture.md` names a concrete model for every AI-relevant
pipeline stage. This document answers the practical next question: **for
each one, do you fetch a pretrained checkpoint and use it as-is, fetch one
and fine-tune it, or train something from scratch — and in what order**,
given a hackathon timeline. Everything below traces back to a source
already verified in `sota-research.md`, `component-deep-dive.md`, or
`carla-ecosystem-catalog.md` — no new claims are introduced here.

**The headline answer**: nothing in this architecture requires training
from scratch. Every AI component has a usable pretrained starting point.
The real work is fine-tuning three of them, and even that's optional for
one (IDD-PeD, usable as-is if you stay within its pedestrian scope).

---

## Classification per component

| Component | Classification | Base checkpoint source | Fine-tuning data (if applicable) |
|---|---|---|---|
| **Perception — YOLOv8/v11** | **FETCH + FINE-TUNE** | Ultralytics COCO-pretrained weights, auto-downloaded via `pip install ultralytics` | **DATS_2022** (Mendeley, open download, no gating) + IDD 2D detection data (via AutoNUE's label toolkit — access to the underlying IDD data at insaan.iiit.ac.in was unreachable during research, verify directly) |
| **Prediction — MoFlow** | **FETCH + FINE-TUNE** | Pretrained checkpoint (ETH-UCY/SDD/NBA) from [github.com/DSL-Lab/MoFlow](https://github.com/DSL-Lab/MoFlow) | India-relevant tracks derived from **METEOR** — requires writing a conversion script first (none exists today, see Prerequisites below) |
| **Prediction — IDD-PeD baselines** | **FETCH-ONLY** (pedestrian scope) / **FETCH + FINE-TUNE** (if extended) | Released checkpoints at [github.com/Ruthvik9/IDD-PeD](https://github.com/Ruthvik9/IDD-PeD) — already trained on real Indian unstructured-traffic data | Only needed if extending beyond pedestrians to two-wheelers/animals — confirmed to require architecture changes, not just relabeling (`component-deep-dive.md`) |
| **Planning — Navigation Toolbox classical planners** | **Not a model** | N/A — configuration/tuning of `plannerHybridAStar`/`plannerRRT*`, already licensed | N/A |
| **Planning — PlanT2 / TransFuser v6/LEAD / SimLingo (enhancement/fallback)** | **FETCH-ONLY** to start; fine-tuning is a stretch goal | PCLA's `download_weights.py` — pulls any of the 41 bundled checkpoints | Custom Indian scenes, only once the modular pipeline is proven end-to-end |
| **Decision logic — Stateflow** | **Not a model** | N/A — hand-designed state machine | N/A |
| **Low-level control — MATLAB Adaptive MPC block** | **Not a model** | N/A — configuration of an already-licensed Simulink block | N/A |
| **World/scene software** (CARLA, SUMMIT, `osm2odr`, Blender Driving Scenario Creator) | **Not a model** — software to install | CARLA: carla.org official download. SUMMIT: [github.com/AdaCompNUS/summit](https://github.com/AdaCompNUS/summit) | N/A |

**Explicitly confirmed: nothing requires train-from-scratch.** This was checked directly during the earlier SOTA research — the compute required to train something like UniAD/VAD from zero (~3,000+ T4-equivalent hours, per `sota-research.md`'s conversation-level analysis) was the reason that route was ruled out from the start. Every component actually chosen in `architecture.md` already has a usable pretrained base.

---

## Recommended order of operations

**Step 1 — Get the pipeline moving end-to-end with FETCH-ONLY checkpoints everywhere, before fine-tuning anything.**
Use stock YOLO (COCO classes only), MoFlow's stock checkpoint, IDD-PeD's released checkpoint, and a PCLA-bundled agent as a placeholder planner. The goal is mechanical validation — does perception→prediction→planning→control→CARLA actually close the loop — not accuracy. This directly de-risks the biggest unknown surfaced in the failure-mode research (the CARLA↔Simulink integration itself, `architecture.md`'s Cross-cutting section) before any time goes into fine-tuning something that might not even reach a working pipeline.

**Step 2 — Fine-tune perception first.**
Highest leverage: every downstream stage depends on correct detections. Start from DATS_2022 specifically because it's open and downloadable right now — don't wait on DriveIndia's EULA approval (unconfirmed turnaround) to start this step, even though DriveIndia has better per-class numbers for your exact object classes (auto-rickshaw 0.940, pushcart 0.391, animal 0.769 mAP50, per `component-deep-dive.md`). Request DriveIndia access in parallel as a possible upgrade later, not a blocker now.

**Step 3 — Fine-tune MoFlow, contingent on the METEOR conversion script.**
This is real engineering work, not a download — METEOR ships XML bbox+GPS tracks, and no existing tooling converts them into MoFlow's expected (history, future) coordinate-pair training format (`component-deep-dive.md`). Budget real time for this script specifically; it's the one piece of "fetch" work in this whole plan that's actually a build task.

**Step 4 — Only if time permits: fine-tune a PCLA-bundled fallback agent on your custom scenes.**
This is explicitly a stretch goal per `architecture.md`'s Stage 5 guidance — the modular pipeline (Steps 1-3) is the primary architecture; a fine-tuned end-to-end agent is a comparison/fallback, not the thing you should be depending on to work.

**Why this order**: front-loading fetch-only checkpoints proves the integration works before you invest in fine-tuning — if the direct-Python (or ROS-fallback) integration turns out to be the real blocker, as the failure-mode research suggests is plausible, you want to discover that on day one with stock models, not after a week spent fine-tuning something you can't yet plug in.

---

## Prerequisites per step

| Step | GPU needed? | Data access status | Related constraint (see `architecture.md` for full detail) |
|---|---|---|---|
| 1 — Stock pipeline | Yes, for running inference on stock checkpoints (any modest GPU) | None — all checkpoints are open-download | Direct-Python interop overhead unbenchmarked — test this specifically in Step 1 |
| 2 — YOLO fine-tune | Yes, for training (Kaggle/Colab/rented GPU per earlier compute discussion) | DATS_2022 open now; IDD access unconfirmed; DriveIndia EULA-gated | YOLOv12 specifically carries an AGPL-3.0 upstream license flag — check which YOLO version before shipping |
| 3 — MoFlow fine-tune | Yes, for training | METEOR open (HuggingFace) but **license genuinely disputed** (MIT vs CC-BY-4.0) — verify before redistributing anything built from it | Conversion script must be written first — this is the actual bottleneck, not GPU time |
| 4 — Fallback agent fine-tune | Yes | Your own custom-scene recordings | PCLA-bundled agents have their own individual license terms (Roach: CC-BY-NC; most others MIT/Apache) — check per-agent before use |

---

## Decision checkpoints / fallback triggers

- **If DATS_2022-fine-tuned perception underperforms noticeably** (no fixed threshold exists — compare qualitatively against DriveIndia's published per-class numbers as a reference ceiling), escalate the DriveIndia access request rather than continuing to iterate on DATS_2022 alone.
- **If the METEOR conversion script proves too time-consuming to finish**, ship MoFlow's stock (un-fine-tuned) checkpoint and disclose that limitation explicitly in the technical report — an honest, documented limitation is a safer choice than an untested rush job or blocking the whole pipeline on it.
- **If direct-Python interop (architecture.md's primary integration path) proves unstable or too slow in Step 1**, fall back to the documented ROS route immediately — don't discover this after fine-tuning is already underway.
- **If a PCLA-bundled fallback agent shows the same instability documented in research** (e.g., TransFuser's route-deviation bug, PlanT's non-finite-bbox crash, both in `architecture.md`'s Stage 5 pitfalls), switch to a different bundled agent rather than debugging an already-flagged unresolved upstream issue.
