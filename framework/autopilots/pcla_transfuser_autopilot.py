"""
Transfuserv6Autopilot -- wraps PCLA's bundled, fully end-to-end
TransFuser v6 agent behind the framework.base.Autopilot interface. PCLA
manages its own sensors, perception, planning, and control internally;
this project's own pipeline/ and TickContext's camera/LiDAR/segmentation
fields are NOT used at all here. See own_perception_plant2_autopilot.py
for the alternative that DOES use this project's own perception.

Registered as "pcla_tfv6" in framework/run_scenario.py.

SAFETY ENVELOPE: wraps TFv6's raw output with
framework/safety_envelope.py's SafetyEnvelope -- an independent,
ground-truth-based time-to-collision monitor that can override to a full
emergency brake (steering preserved) if TFv6's own decision leaves the
ego dangerously close to something, WITHOUT touching TFv6's own
reasoning at all. See that module's own docstring for the full
reasoning (why ground truth, why not decision_logic.py's state machine,
the real bug it surfaced and fixed in pipeline/decision_logic.py along
the way). Set `enable_safety_envelope=False` at construction to run TFv6
raw, unwrapped -- useful for an A/B comparison of TFv6-alone vs.
TFv6+safety-envelope on the same chaotic scenario.

REQUIRES (not bundled by this file): external/PCLA cloned (gitignored --
`git clone https://github.com/MasoudJTehrani/PCLA external/PCLA`), its
own conda environment set up per external/PCLA/README.md
(environment.yml + download_weights.py for the tfv6_regnet checkpoint --
several GB, budget real time for this download), and py-trees installed
in whichever Python environment actually runs this (PCLA's own
dependency, separate from carla_env's requirements.txt -- see
framework/DESIGN_GUIDELINES.md's PCLA section).

KNOWN INEFFICIENCY, disclosed not hidden: ScenarioRunner always spawns
the standard RGB/LiDAR/segmentation sensor rig (via
carla_runtime.spawn_ego_sensors) before ANY autopilot's setup() runs --
that spawn order is shared/fixed across every autopilot (see
framework/base.py's Scenario/Autopilot contract). Those three sensors
are never read by this autopilot -- PCLA attaches and manages its own
separate sensor set internally, per the loaded agent's own sensors()
method. This costs some extra render time (a few more cameras rendering
per tick) but is not a correctness bug; making ScenarioRunner's sensor
spawn conditional per-autopilot was judged not worth the added
complexity this close to the deadline. Flagged here for anyone chasing
"why is this autopilot slower than PipelineAutopilot at idle."

NEVER run against a live CARLA server in this environment -- syntax/
import-checked only, same standard as run_live.py at the time it was
written.
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np

# external/PCLA must be importable before `from PCLA import PCLA` --
# PCLA.py itself also inserts its own directory at the front of sys.path
# on import (for its many internal absolute imports), but that only
# happens AFTER Python has already found PCLA.py once -- this insert is
# what lets the initial `from PCLA import PCLA` succeed at all.
_PCLA_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "external", "PCLA",
)
if _PCLA_DIR not in sys.path:
    sys.path.insert(0, _PCLA_DIR)

from PCLA import PCLA  # noqa: E402

from framework.base import Autopilot, TickContext  # noqa: E402
from framework.safety_envelope import SafetyEnvelope  # noqa: E402
from pipeline.types import ControlCommand  # noqa: E402


class Transfuserv6Autopilot(Autopilot):
    def __init__(self, agent_key: str = "tfv6_regnet", enable_safety_envelope: bool = True):
        """agent_key: any key from external/PCLA/agents.json's "tfv6"
        entry -- "tfv6_regnet" (default), "tfv6_resnet", "tfv6_4cameras"
        (wider camera coverage -- worth trying for scenarios with
        traffic approaching from multiple directions, e.g. this
        project's chaotic-traffic scenarios), "tfv6_noradar",
        "tfv6_visiononly", "tfv6_notown13". See agents.json for the
        authoritative current list -- these are the KEYS give_path.py
        splits on, not the longer checkpoint-folder names in that file's
        "config" values.

        enable_safety_envelope: True (default) wraps TFv6's output with
        SafetyEnvelope (see this file's module docstring). Set False to
        run TFv6 raw/unwrapped -- for an A/B comparison of the two on the
        same scenario.
        """
        self.agent_key = agent_key
        self.pcla = None
        self._last_action_ms = 0.0
        self.safety = SafetyEnvelope() if enable_safety_envelope else None

    def setup(
        self,
        *,
        ego_camera,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        image_width: int,
        image_height: int,
        wheelbase_m: float,
        dt: float,
        client=None,
        ego_vehicle=None,
        route_xml_path: str | None = None,
    ) -> None:
        if client is None or ego_vehicle is None or route_xml_path is None:
            raise RuntimeError(
                "Transfuserv6Autopilot requires client/ego_vehicle/route_xml_path -- "
                "these are supplied automatically by ScenarioRunner.run() (see "
                "framework/base.py's Autopilot.setup() docstring). Seeing this means "
                "something called setup() directly instead of going through ScenarioRunner."
            )
        print(f"Loading PCLA agent '{self.agent_key}' (this can take a while for a cold model load)...")
        self.pcla = PCLA(self.agent_key, ego_vehicle, route_xml_path, client)
        print(f"PCLA agent '{self.agent_key}' ready.")

    def compute(self, ctx: TickContext, goal_xy: tuple[float, float]) -> ControlCommand:
        # goal_xy is unused -- PCLA is fully self-contained (its own
        # sensors, its own route/goal via route_xml_path given at
        # setup()). ctx IS used now (for the safety envelope's ground-
        # truth scan) even though TFv6 itself never sees it.
        t0 = time.perf_counter()
        vehicle_control = self.pcla.get_action()
        self._last_action_ms = (time.perf_counter() - t0) * 1000

        control = ControlCommand(
            throttle=vehicle_control.throttle,
            steer=vehicle_control.steer,
            brake=vehicle_control.brake,
        )

        if self.safety is not None:
            self.safety.check(ctx.world, ctx.ego_vehicle)
            control = self.safety.wrap_control(control)

        return control

    def debug_info(self) -> dict:
        # No 'detections'/'planned_waypoints' -- TransFuser v6's internal
        # representations aren't exposed by PCLA's public API, and
        # reaching into agent-instance internals for visualization was
        # judged not worth the fragility (PCLA reloads a fresh agent
        # module per run via importlib -- see PCLA.py's setup_agent() --
        # so internal attribute names aren't a contract to depend on).
        info = {"timings": self._last_action_ms}
        if self.safety is not None:
            info["decision_mode"] = "TFV6_SAFETY_OVERRIDE" if self.safety.override_active else "TFV6_NORMAL"
            # Not official MetricsRecorder fields -- these two ride along
            # in debug_info() for anyone logging/printing it directly;
            # only 'decision_mode' (above) is a key MetricsRecorder's
            # record_tick() actually reads (see framework/base.py).
            info["min_ttc_s"] = self.safety.last_min_ttc
            info["hazard_class"] = self.safety.last_hazard_class
        else:
            info["decision_mode"] = "TFV6_UNWRAPPED"
        return info

    def cleanup(self) -> None:
        """IMPORTANT: PCLA.cleanup() also destroys the ego vehicle itself
        and PCLA's own attached sensors (see external/PCLA/PCLA.py's
        cleanup(): 'Destroy the vehicle after sensors are cleaned up').
        ScenarioRunner's own subsequent ego_vehicle/ego_sensors destroy
        calls use client.apply_batch() (fire-and-forget, no exception on
        an already-destroyed actor) -- this double-destroy is safe,
        deliberate redundancy with ScenarioRunner's own teardown, not a
        conflict to work around by skipping ScenarioRunner's cleanup.
        """
        if self.pcla is not None:
            self.pcla.cleanup()
            self.pcla = None
