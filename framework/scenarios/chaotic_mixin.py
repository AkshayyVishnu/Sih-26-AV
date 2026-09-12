"""
ChaoticTrafficMixin -- a mixin any framework.base.Scenario can inherit to
get dense, aggressively-tuned background traffic
(pipeline/traffic_chaos.py, the SUMMIT-substitute -- see
docs/pipeline-decision-log.md for why SUMMIT itself was dropped) without
duplicating spawn/cleanup logic per scenario.

Usage: `class MyScenario(ChaoticTrafficMixin, Scenario):` -- mixin FIRST
in the MRO so its spawn_actors()/cleanup_extra() run and call
super().spawn_actors(...)/super().cleanup_extra(...) to chain into
whatever the concrete Scenario itself defines (see chaotic_traffic.py for
a worked example combining this with scripted actors of its own).

Deliberately NOT applied to framework/scenarios/traffic_stress.py --
that scenario already spawns its own CARLA-standard background traffic
via a different, already-tested spawn pattern; stacking both would
double-spawn and isn't needed (traffic_stress.py exists to stress-test
sheer actor COUNT, this mixin exists to approximate Indian-road traffic
CHAOS/heterogeneity -- different goals, don't conflate them).
"""
from __future__ import annotations

import carla

from framework.base import Scenario
from pipeline.traffic_chaos import destroy_chaotic_traffic, spawn_chaotic_traffic


class ChaoticTrafficMixin:
    """Mix in BEFORE Scenario in the class bases. Reads
    NUM_CHAOTIC_VEHICLES / NUM_CHAOTIC_WALKERS / CHAOTIC_TRAFFIC_SEED
    class attributes (override in the concrete scenario if needed) --
    kept as plain class attributes, not constructor kwargs, so a scenario
    combining this with its own __init__ doesn't have to thread extra
    args through by hand.
    """
    NUM_CHAOTIC_VEHICLES = 40
    NUM_CHAOTIC_WALKERS = 30
    CHAOTIC_TRAFFIC_SEED = 42
    CHAOTIC_TRAFFIC_TM_PORT = 8000

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        # avoid_locations=[EGO_SPAWN.location]: this runs BEFORE the ego
        # is spawned (see Scenario.spawn_actors()'s own docstring), so
        # the only thing to protect against overlap is the known, static
        # EGO_SPAWN transform -- same reasoning
        # framework/scenarios/traffic_stress.py already applies for its
        # own background vehicles.
        self._chaotic_traffic_actors = spawn_chaotic_traffic(
            client, world,
            num_vehicles=self.NUM_CHAOTIC_VEHICLES,
            num_walkers=self.NUM_CHAOTIC_WALKERS,
            seed=self.CHAOTIC_TRAFFIC_SEED,
            traffic_manager_port=self.CHAOTIC_TRAFFIC_TM_PORT,
            avoid_locations=[self.EGO_SPAWN.location],
        )
        # Track every spawned ID so ScenarioRunner's generic
        # "destroy every self._tracked_actors entry" cleanup covers these
        # too, in addition to this mixin's own cleanup_extra() below
        # (which handles walker-controller .stop() specifically --
        # destroying a controller without stopping it first is the kind
        # of leak framework/DESIGN_GUIDELINES.md already warns about).
        for actor_id in (
            self._chaotic_traffic_actors["vehicles"]
            + self._chaotic_traffic_actors["walkers"]
            + self._chaotic_traffic_actors["controllers"]
        ):
            self.track(actor_id)

        super().spawn_actors(world, bp_lib, client)  # chain into the concrete Scenario's own actors, if any

    def cleanup_extra(self, client: carla.Client) -> None:
        # destroy_chaotic_traffic() stops walker controllers, then
        # issues DestroyActor for everything -- redundant with (but
        # harmless alongside) ScenarioRunner's own generic
        # self._tracked_actors destroy pass afterward (client.apply_batch
        # is fire-and-forget; destroying an already-destroyed actor is a
        # silent no-op, same reasoning as the PCLA-autopilot cleanup()
        # docstrings use).
        world = client.get_world()
        destroy_chaotic_traffic(client, world, self._chaotic_traffic_actors)
        super().cleanup_extra(client)
