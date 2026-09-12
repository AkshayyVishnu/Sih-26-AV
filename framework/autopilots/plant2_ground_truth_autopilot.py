"""
PlanT2GroundTruthAutopilot -- PlanT2 running with its OWN, UNMODIFIED
get_bounding_boxes() (CARLA ground truth: exact position/velocity/class,
zero latency, zero noise), fully bundled/unmodified via PCLA. NO
injection, NO involvement of this project's own perception at all --
the direct opposite of own_perception_plant2_autopilot.py.

WHY THIS EXISTS, GIVEN own_perception_plant2_autopilot.py ALREADY
FEEDS PLANT2 SOMETHING: it establishes a CEILING. own_perception_plant2
holds the planner fixed (PlanT2) and swaps in this project's own,
imperfect perception (tracking jitter, no heading estimate, a fixed
class vocabulary, see pipeline/plant2_adapter.py's disclosed
limitations) -- the gap between THIS autopilot's results and
own_perception_plant2's results, on the identical scenario, tells you
exactly how much performance is being lost to that perception's
imperfections specifically, versus PlanT2's planning quality itself.
Neither number alone answers that question; the two together do.

NOT the same thing as pcla_tfv6, despite both being "fully bundled,
unmodified PCLA agents" -- TransFuser v6 still has to perceive from real
camera/LiDAR pixels; this autopilot doesn't perceive at all, it reads
perfect simulator state directly. It is the MORE privileged of the two,
not a duplicate reference point.

Mechanically identical to Transfuserv6Autopilot (single bundled PCLA
agent, optional SafetyEnvelope wrapper) -- implemented as a thin
subclass reusing 100% of that class's setup()/compute()/debug_info()/
cleanup() logic, not a copy, so a fix made to one automatically applies
to both. Exists as its own class (not just a different `agent_key`
argument passed to Transfuserv6Autopilot at the call site) specifically
so framework/base.py's ScenarioRunner.run() -- which builds each run's
MetricsRecorder scenario_name from `type(self.autopilot).__name__` --
produces a distinctly-named metrics file for this configuration, without
the caller having to separately track and pass agent_key through to
that naming logic.

Registered as "plant2_ground_truth" in framework/run_scenario.py.

REQUIRES: same PCLA setup as pcla_transfuser_autopilot.py (see that
file's docstring), plus the plant2 checkpoint specifically.

NEVER run against a live CARLA server in this environment -- syntax/
import-checked only.
"""
from __future__ import annotations

from framework.autopilots.pcla_transfuser_autopilot import Transfuserv6Autopilot

_PLANT2_AGENT_KEY = "plant2_plant2"  # external/PCLA/agents.json's plant2 entry (currently one variant)


class PlanT2GroundTruthAutopilot(Transfuserv6Autopilot):
    def __init__(self, agent_key: str = _PLANT2_AGENT_KEY, enable_safety_envelope: bool = True):
        super().__init__(agent_key=agent_key, enable_safety_envelope=enable_safety_envelope)

    def debug_info(self) -> dict:
        # Same shape as the parent's debug_info(), just relabeled --
        # "TFV6_*" decision_mode strings would be actively misleading for
        # a PlanT2 run's logs/CSV.
        info = super().debug_info()
        mode = info.get("decision_mode", "")
        info["decision_mode"] = mode.replace("TFV6_", "PLANT2_") if mode else "PLANT2_UNWRAPPED"
        return info
