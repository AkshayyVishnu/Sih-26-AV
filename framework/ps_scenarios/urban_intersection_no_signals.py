"""
UrbanIntersectionNoSignals -- PS scenario #2: "a busy urban intersection
without signals."

Town03, reusing framework/scenarios/traffic_stress.py's own real,
already-live-verified EGO_SPAWN/FINAL_GOAL coordinates directly (zero new
coordinate-capture risk -- see that file's own docstring/HANDOFF.md for
confirmation it was run live). Combines two already-tested pieces rather
than inventing new plumbing:

1. The traffic-light freeze/restore block, inlined here in the exact
   form already proven live in traffic_stress.py's spawn_actors()/
   cleanup_extra() -- NOT factored into a shared mixin. Deliberate
   choice: this is the only other call site for that logic; stacking a
   second mixin alongside ChaoticTrafficMixin below would add MRO/
   super()-chaining complexity for one reuse, versus ~10 lines of
   already-proven code duplicated once.
2. framework/scenarios/chaotic_mixin.py's ChaoticTrafficMixin (imported
   from the EXISTING scenarios package -- it's map-agnostic shared infra,
   not scenario-specific, so reusing it across framework/scenarios/ and
   framework/ps_scenarios/ is intentional, not a layering violation) for
   two-wheeler-biased, aggressive, jaywalking-pedestrian traffic --
   deliberately swapped in over traffic_stress.py's own stock-composition
   300-vehicle mix, since the PS's "busy... intersection" framing fits a
   dense, heterogeneous, undisciplined traffic mix better than a
   same-style vehicle flood.
"""
from __future__ import annotations

import carla

from framework.base import Scenario
from framework.scenarios.chaotic_mixin import ChaoticTrafficMixin


class UrbanIntersectionNoSignals(ChaoticTrafficMixin, Scenario):
    MAP_NAME = "Town03"

    # Real, already-live-verified coordinates -- reused directly from
    # framework/scenarios/traffic_stress.py (EGO_SPAWN, FINAL_GOAL,
    # SPECTATOR_TRANSFORM all identical). Not a placeholder.
    EGO_SPAWN = carla.Transform(
        carla.Location(x=9.0, y=-77.7, z=1.5),
        carla.Rotation(pitch=0.0, yaw=270.0, roll=0.0),
    )
    FINAL_GOAL = (-13.5, -156.84)  # the same "target intersection" traffic_stress.py aims at
    SPECTATOR_TRANSFORM = carla.Transform(
        carla.Location(x=-13.5, y=-156.84, z=20.0),
        carla.Rotation(pitch=-30.0, yaw=54.0, roll=0.0),
    )

    # ChaoticTrafficMixin's own defaults (40 vehicles, 30 walkers) are
    # used as-is -- no override needed, this scenario's "busy" quality
    # comes from that mixin's existing two-wheeler-biased aggressive
    # tuning, not from raising the numbers further.

    def __init__(self):
        super().__init__()
        self.all_lights: list = []

    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        # Freeze lights BEFORE spawning traffic -- same order and the
        # same "needs one tick to settle first" reasoning already proven
        # in traffic_stress.py.
        world.tick()  # let CARLA's internal Traffic Light Manager finish initializing first
        self.all_lights = list(world.get_actors().filter("traffic.traffic_light"))
        print(f"Disabling ALL {len(self.all_lights)} traffic light actors in the map...")
        for tl in self.all_lights:
            tl.set_state(carla.TrafficLightState.Off)
            tl.freeze(True)
        world.tick()  # apply the light-state change

        # Chains into ChaoticTrafficMixin.spawn_actors(), which spawns
        # the two-wheeler-biased aggressive traffic and then chains into
        # Scenario's own (abstract, no-op-for-this-class) spawn_actors().
        super().spawn_actors(world, bp_lib, client)

    def cleanup_extra(self, client: carla.Client) -> None:
        if self.all_lights:
            print("Restoring all frozen traffic lights...")
            for tl in self.all_lights:
                tl.freeze(False)
        # Chains into ChaoticTrafficMixin.cleanup_extra(), which destroys
        # the background traffic.
        super().cleanup_extra(client)
