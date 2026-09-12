"""
DenseMarketMixedTraffic -- PS scenario #4: "a dense market area with
mixed traffic."

Town10HD, CARLA's compact downtown map -- denser building/street layout
than a sprawling town gives higher PERCEIVED density for the same actor
count, closest stock analog to a market's tight quarters (per the
explicit instruction to use stock CARLA towns/assets only). FALLBACK, if
unavailable: Town05, a busier city grid.

Reuses framework/scenarios/chaotic_mixin.py's ChaoticTrafficMixin (same
map-agnostic shared infra framework/ps_scenarios/
urban_intersection_no_signals.py also reuses) -- the two-wheeler-biased,
aggressive, jaywalking-pedestrian traffic mix IS the "mixed traffic"
this scenario needs, no new traffic logic required. Tuned here for
higher pedestrian/two-wheeler density specifically: a literal market
wants foot-traffic and two-wheeler density more than raw car count, so
NUM_CHAOTIC_WALKERS is raised well above the mixin's own default (30) --
50 -- while vehicle count stays at the mixin's default.

No scripted hazard needed on top -- the density itself is the test
(can the planner hold a safe, continuously-replanned path through
genuinely crowded, unpredictable, mixed-speed traffic).

Lower-priority, NOT included here (flagged as a disclosed follow-up, not
silently dropped): visual stall-like prop dressing via stock
`static.prop.*` CARLA blueprints. Left out because which specific prop
blueprints exist needs a live `world.get_blueprint_library().
filter("static.prop.*")` check -- not something to hardcode
speculatively without confirming it live first. Purely cosmetic either
way -- doesn't affect the planner-relevant behavior this scenario tests.

COORDINATES ARE PLACEHOLDERS. Town10HD has never been loaded in this
repo before -- capture real ones via
Simulation Files/carla_print_coordinates.py (edit MAP_NAME to
"Town10HD"), aiming for the map's densest street section.
"""
from __future__ import annotations

import carla

from framework.base import Scenario
from framework.scenarios.chaotic_mixin import ChaoticTrafficMixin


class DenseMarketMixedTraffic(ChaoticTrafficMixin, Scenario):
    MAP_NAME = "Town10HD"  # fallback: "Town05" if unavailable -- confirm live first

    NUM_CHAOTIC_WALKERS = 50  # override the mixin's default of 30 -- a market wants foot-traffic density
    # NUM_CHAOTIC_VEHICLES left at the mixin's default (40) -- vehicle
    # count isn't what needs raising here, pedestrian/two-wheeler density is.

    # TODO: capture via carla_print_coordinates.py on Town10HD --
    # placeholders, aim for the densest downtown street section.
    EGO_SPAWN = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=1.5),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    FINAL_GOAL = (100.0, 0.0)  # TODO: replace with a real point through the dense area
    SPECTATOR_TRANSFORM = None

    # No spawn_actors()/cleanup_extra() override needed -- ChaoticTrafficMixin's
    # own (chaining into Scenario's abstract no-op) already does everything
    # this scenario needs.
