"""
ChaoticTraffic -- framework/scenarios/pedestrian_jumpout.py's exact,
already-validated obstruction+hidden-pedestrian hazard, PLUS dense,
aggressively-tuned background traffic
(framework/scenarios/chaotic_mixin.py's ChaoticTrafficMixin, the
SUMMIT-substitute -- see docs/pipeline-decision-log.md for why literal
SUMMIT was dropped). Built specifically to exercise the PCLA-backed
autopilots (pcla_tfv6 / own_perception_plant2) under something closer to
real unregulated Indian-road traffic than an otherwise-empty town, not
just a clean single-hazard test.

Reuses PedestrianJumpOut's EGO_SPAWN/FINAL_GOAL/SPECTATOR_TRANSFORM
(known-good, previously verified live -- see HANDOFF.md) rather than
picking new coordinates -- no reason to introduce a fresh, unverified
route just to add background traffic on top.
"""
from __future__ import annotations

import carla

from framework.scenarios.chaotic_mixin import ChaoticTrafficMixin
from framework.scenarios.pedestrian_jumpout import PedestrianJumpOut


class ChaoticTraffic(ChaoticTrafficMixin, PedestrianJumpOut):
    # Cooperative multiple inheritance: ChaoticTrafficMixin.spawn_actors()/
    # cleanup_extra() each call super(...), which the MRO resolves to
    # PedestrianJumpOut's versions -- so both the chaotic background
    # traffic AND the obstruction/pedestrian hazard get spawned/cleaned
    # up, in that order (chaotic traffic first, then the hazard actors).
    # See chaotic_mixin.py's own docstring for why the mixin goes first
    # in the class bases.
    pass
