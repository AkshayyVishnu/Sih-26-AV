"""
SafetyEnvelope -- an independent, ground-truth-based time-to-collision
monitor that can override an Autopilot's raw control output with an
emergency brake, without touching that autopilot's own decision-making
at all. The standard real-world AV pattern for wrapping a learned or
otherwise opaque driving policy: not a replacement for the policy's own
judgement, a backstop for its rare bad decisions specifically under the
kind of chaotic traffic this project's scenarios are built to stress
(framework/ps_scenarios/chaotic_traffic.py,
urban_intersection_no_signals.py, the wrong-way vehicle mixed into
unmarked_village_road.py).

WHY THIS EXISTS, CONCRETELY: PCLA's bundled agents (TransFuser v6,
PlanT2) don't expose their internal object detections or confidence
through PCLA's public API at all (see
framework/autopilots/pcla_transfuser_autopilot.py's own debug_info()
docstring for why reaching into agent internals was judged too fragile)
-- there is nothing to hook into even if we wanted to introspect the
model's own reasoning. This sidesteps that entirely: it doesn't need to
know anything about how the wrapped autopilot thinks, only whether the
WORLD currently looks dangerous, independently.

WHY GROUND TRUTH, NOT A SEPARATE REAL SENSOR SUITE: a real production
safety monitor would use its own independent sensor pipeline (a second,
diverse set of sensors, ideally from a different vendor/modality than
the primary stack, precisely so a shared blind spot doesn't take out
both the primary policy AND its watchdog at once). Querying CARLA's own
ground truth directly is a simulation-only stand-in for that -- the same
disclosed shortcut already used throughout this project
(carla_runtime.GroundTruthDetector, pipeline/drivable_area.py's
ground-truth segmentation), stated plainly rather than dressed up as
more independent than it actually is in this simulated form.

NOT a reuse of pipeline/decision_logic.py's full state machine -- that
class is a hierarchical mode-switching system built specifically to
drive PipelineAutopilot's own planner/controller loop (debounce timers,
ANIMAL_ON_ROAD/OBSTACLE_DETECTED sub-states, a SAFETY_SUPERVISOR
watchdog for pipeline latency/crashes -- none of which apply to a
black-box PCLA agent). This is deliberately smaller and single-purpose:
one number (minimum TTC across nearby ground-truth actors) and one
action (override to full brake below a threshold, with hysteresis so it
doesn't chatter).

A REAL BUG WAS FOUND AND FIXED IN pipeline/decision_logic.py WHILE
BUILDING THIS: comparing this class's TTC math (correctly ego-relative
from the start, since it queries actor/ego locations directly) against
decision_logic.py's own _min_ttc surfaced that the latter computed
distance from WORLD ORIGIN, not from the ego -- invisible on the
synthetic demo (fake ego always at the origin) but would have silently
broken every mode-switching decision on the first real CARLA run. Fixed
at the source (pipeline/decision_logic.py now takes ego_x/ego_y) rather
than left as a latent bug -- see docs/pipeline-decision-log.md.

USAGE: construct one SafetyEnvelope per Autopilot instance in its
setup(), call `check(world, ego_vehicle)` once per tick in compute()
before returning, then `wrap_control(raw_control)` to get the (possibly
overridden) ControlCommand to actually return. See
framework/autopilots/pcla_transfuser_autopilot.py for a worked example.
Reusable as-is for any other Autopilot that wants the same backstop
(e.g. own_perception_plant2_autopilot.py) -- not wired in there yet,
deliberately scoped to what was asked for this round.

NEVER tested against a live CARLA server in this environment -- the TTC/
hysteresis logic itself was verified with hand-built mock actors (no
live CARLA needed), not against real actor kinematics.
"""
from __future__ import annotations

import math

import carla

from carla_runtime import classify_actor
from pipeline.types import ControlCommand


class SafetyEnvelope:
    def __init__(
        self,
        critical_ttc_s: float = 1.5,
        release_ttc_s: float = 2.5,
        max_range_m: float = 40.0,
    ):
        """
        critical_ttc_s: override to a full emergency brake once the
        minimum time-to-collision across all nearby ground-truth actors
        drops below this.
        release_ttc_s: don't hand control back to the wrapped autopilot
        until TTC recovers above THIS (strictly greater than
        critical_ttc_s) -- a small hysteresis gap so the override doesn't
        chatter on/off tick-to-tick right at the threshold. Same
        anti-chatter reasoning pipeline/decision_logic.py's own
        sustained-clearance gate uses, applied here in miniature (one
        threshold gap, not a multi-tick debounce timer -- this monitor is
        deliberately simpler than that state machine).
        max_range_m: ignore actors farther than this -- keeps the
        ground-truth scan cheap and avoids reacting to something too far
        away to be a real near-term hazard.
        """
        if release_ttc_s <= critical_ttc_s:
            raise ValueError(
                f"release_ttc_s ({release_ttc_s}) must be strictly greater than "
                f"critical_ttc_s ({critical_ttc_s}), or the hysteresis gap collapses to zero."
            )
        self.critical_ttc_s = critical_ttc_s
        self.release_ttc_s = release_ttc_s
        self.max_range_m = max_range_m
        self._override_active = False
        self.last_min_ttc = float("inf")
        self.last_hazard_class: str | None = None

    def check(self, world: carla.World, ego_vehicle: carla.Vehicle) -> tuple[float, str | None]:
        """Computes and caches this tick's minimum TTC across nearby
        ground-truth actors (vehicles + pedestrians, same actor filter
        carla_runtime.GroundTruthDetector already uses). Call once per
        tick, BEFORE wrap_control(). Returns (min_ttc_s, hazard_class)
        for logging/debug_info -- hazard_class is whatever
        carla_runtime.classify_actor() returns for the closest-in-time
        hazard (None if nothing nearby at all).

        Same cheap, disclosed simplification as
        pipeline/decision_logic.py's own _min_ttc: approximates TTC as
        distance / max(closing_speed, floor) -- doesn't account for
        relative heading (a vehicle converging at an angle looks the same
        as one dead ahead). Good enough for a coarse "is something
        dangerously close" backstop, not a substitute for real
        trajectory-intersection checking.
        """
        ego_transform = ego_vehicle.get_transform()
        ego_loc = ego_transform.location
        ego_velocity = ego_vehicle.get_velocity()
        ego_speed = math.hypot(ego_velocity.x, ego_velocity.y)

        actors = list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("walker.pedestrian.*"))

        min_ttc = float("inf")
        hazard_class = None
        for actor in actors:
            if actor.id == ego_vehicle.id:
                continue

            actor_loc = actor.get_transform().location
            dist = ego_loc.distance(actor_loc)
            if dist > self.max_range_m:
                continue

            actor_velocity = actor.get_velocity()
            actor_speed = math.hypot(actor_velocity.x, actor_velocity.y)
            closing_speed = max(ego_speed - actor_speed, 0.5)  # floor avoids div-by-~0, same as decision_logic.py
            ttc = dist / closing_speed

            if ttc < min_ttc:
                min_ttc = ttc
                hazard_class = classify_actor(actor)

        self.last_min_ttc = min_ttc
        self.last_hazard_class = hazard_class
        return min_ttc, hazard_class

    def wrap_control(self, control: ControlCommand) -> ControlCommand:
        """Applies hysteresis using the most recent check() result and
        returns either `control` unchanged or an emergency-brake
        override. MUST be called after check() this same tick -- it
        doesn't call check() itself, so an override decision is always
        based on data the caller explicitly gathered, not a hidden
        re-scan.

        Steering is PRESERVED on override (same choice
        pipeline/pipeline.py's own emergency-brake path already makes)
        so the vehicle can still track/swerve along whatever path it was
        already on while braking, not lock straight into whatever it's
        braking for.
        """
        if self._override_active:
            if self.last_min_ttc >= self.release_ttc_s:
                self._override_active = False
        else:
            if self.last_min_ttc < self.critical_ttc_s:
                self._override_active = True

        if self._override_active:
            return ControlCommand(throttle=0.0, steer=control.steer, brake=1.0)
        return control

    @property
    def override_active(self) -> bool:
        return self._override_active
