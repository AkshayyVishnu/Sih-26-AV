# The Ideal Adaptive Planning Architecture for Indian Traffic

This document describes, from first principles, what the *ideal*
path-planning architecture for autonomous driving on unstructured Indian
roads looks like — not a ranking of existing approaches, a design.

## 1. The design thesis

The defining property of Indian traffic isn't "high density." It's
**heterogeneity that varies continuously in space and time**. The same
vehicle, on the same route, can pass through an open highway stretch, a
tight unmarked village lane, a chaotic unsignaled intersection, a dense
market alley, and a sudden animal crossing — each one a fundamentally
different planning problem, sometimes minutes apart. A fixed-parameter
planner — classical or learned — is tuned for one operating point. It
either wastes compute being over-careful on the open highway, or risks
its own latency budget when the scene suddenly gets complex.

The ideal architecture is therefore built around two non-negotiable
properties, satisfied simultaneously, every tick:

1. **Adaptive compute allocation** — deliberate harder where the scene
   is complex, deliberate less where it's simple, continuously and in
   real time, not via a parameter tuned once for an "average" road.
2. **A hard latency guarantee** — never silently miss the control
   deadline, no matter how the scene changes or how wrong the adaptive
   layer's own judgment turns out to be in a given tick.

Everything below exists to satisfy both at once, without trading one for
the other.

## 2. Architecture overview

```
                         ┌─────────────────────────────┐
                         │   TIER 3 -- Supervisory      │
                         │   Density / complexity signal │
                         │   (object count, class mix,   │
                         │    spatial spread) -- runs at  │
                         │    perception rate, every tick │
                         └───────────┬─────────────────┘
                                     │ tunes horizon, resolution,
                                     │ deadline, cost weights
                                     ▼
  Sensors ──▶ Perception ──▶ Tracking ──▶ ┌─────────────────────────┐
                                          │ TIER 2 -- Adaptive        │
                                          │ deliberative planner      │
                                          │ -- variable horizon        │
                                          │ -- variable resolution     │
                                          │ -- deadline-bounded search │
                                          │ -- confidence-weighted     │
                                          │    obstacle costing        │
                                          └───────────┬───────────────┘
                                                       │ best path found
                                                       │ by the deadline
                                                       ▼
             ┌────────────────────────────────────────────────────┐
             │ TIER 1 -- Reactive safety layer                     │
             │ independent TTC watchdog, runs every tick,          │
             │ regardless of Tier 2's progress or output           │
             └───────────────────────────┬──────────────────────────┘
                                          ▼
                                     Vehicle control
```

Three tightly coupled tiers, each running at its own natural rate, each
with one job.

## 3. Tier 1 — Reactive safety layer

**Job**: guarantee a bounded worst-case reaction time, unconditionally.

- A minimal, independent time-to-collision monitor, running every single
  tick, watching the world directly rather than trusting the
  deliberative planner's own belief about the world.
- It does not care what Tier 2 is doing. If Tier 2 is mid-search,
  running on a stale plan, or has simply misjudged the scene, Tier 1 can
  still command a full stop in time.
- This is what makes adaptiveness *safe to attempt*: Tiers 2 and 3 are
  free to be aggressive (short deadlines, coarse grids, wide prediction
  gaps under load) because a wrong guess there degrades smoothness or
  optimality, never safety. Safety is decoupled from planning quality by
  construction, not by hope that the planner behaves.

## 4. Tier 2 — Adaptive deliberative layer

**Job**: produce the best path it can, within a time budget that itself
adapts to how hard the current scene actually is.

**Density-conditioned planning horizon.** The number of future
timesteps predicted and planned is not a fixed constant — it is a
function of local scene complexity:
- Dense, cluttered, low-speed scenes (market alley, tight village lane,
  a jam at an intersection): fewer, coarser timesteps. Smaller search
  space, lower latency, exactly when congestion would otherwise blow the
  tick budget — and a shorter horizon is also the *right* answer here,
  since far-future prediction is least reliable in exactly this kind of
  scene anyway.
- Sparse, high-speed scenes (open highway, quiet village road): more,
  finer timesteps. There's compute budget to spare, and a longer,
  smoother horizon genuinely improves ride quality and gives more
  advance notice of a highway merge or a slow lead vehicle.

**Multi-resolution spatial planning.** The same adaptive-compute
principle, applied to the planning grid itself: coarsen cell resolution
in dense/cluttered scenes (fewer cells to search, cheaper per-step
cost), refine it in open or precision-sensitive scenes (finer control
where it's affordable and where narrow gaps actually matter).

**Deadline-bounded, anytime search.** The path search runs against a
hard wall-clock budget, not "search until optimal." If the budget is
reached before the search completes, the best path found *so far* is
returned and used immediately — there is no failure mode where the
planner simply doesn't produce an answer in time. This is the concrete,
always-on version of "have a fallback if it takes too long": not an
exception handler bolted on afterward, a property the search algorithm
is built to have from the start.

**Confidence-weighted obstacle costing.** Every tracked object is costed
not just by its class, but by how confident and stable its current
detection actually is. An occluded pedestrian at the edge of a market
crowd, or a two-wheeler flickering in and out of a cluttered scene, gets
treated more cautiously than a clean, stable detection of the same
class — the planner's caution scales with its own uncertainty, in real
time, rather than assuming every detection is equally trustworthy.

## 5. Tier 3 — Density-aware supervisory layer

**Job**: be the thing that actually watches the scene and turns Tier 2's
knobs — without this tier, "adaptive" is just a static config choice
someone made once, not a real-time property.

- Continuously computes a local complexity signal from the same
  perception/tracking output everything else already has: object count
  within range, class mix (a knot of two-wheelers behaves very
  differently from one truck), and spatial spread (clustered vs. spread
  out).
- Feeds that signal directly into Tier 2's parameters every tick:
  horizon length, grid resolution, search deadline, and cost-weighting
  aggressiveness all move together as one coherent response to "how
  complex is this scene right now," not as four independently-tuned
  knobs that might drift out of sync with each other.
- Runs at perception's own fast cadence, while Tier 2's *full* replan
  only fires when the scene has actually changed enough to warrant one.
  This decouples three different, legitimate rates that a naive
  architecture conflates into one: how often the vehicle senses, how
  often it fully re-deliberates, and how fast it must be able to brake —
  letting the system spend its compute exactly where the complexity
  actually is, tick to tick.

## 6. Why this shape, specifically, is the ideal one for this problem

- **Adaptiveness is architectural, not incidental.** The core fact about
  unstructured Indian roads is that the right operating point keeps
  changing mid-route — an architecture that treats "how much to
  deliberate" as a first-class, continuously-updated signal (Tier 3
  driving Tier 2) is solving the actual problem, not a fixed-scenario
  approximation of it.
- **Safety never competes with adaptiveness.** Because Tier 1's guarantee
  holds unconditionally, every aggressive choice available to Tiers 2/3
  (shorter deadlines, coarser grids, sparser horizons under load) only
  ever risks smoothness or optimality — never collision risk. This is
  what makes it safe to actually *be* aggressive when the scene demands
  speed over polish.
- **It degrades gracefully, not silently.** An anytime search with a
  hard deadline always has an answer to give; a supervisory signal that
  shrinks the horizon under load produces a smaller, more conservative
  plan rather than a late one. There is no path through this
  architecture where the vehicle simply has nothing to act on.
- **Every decision is explainable.** Why the horizon shrank, why the
  search stopped early, why a detection was costed cautiously, why the
  brake fired — each is a direct, inspectable consequence of one tier's
  own explicit signal, not a hidden byproduct of one opaque model's
  internal state. That matters for debugging today and for any future
  safety case.
