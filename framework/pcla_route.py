"""
Auto-generates a minimal PCLA/leaderboard-format route XML from a
Scenario's own EGO_SPAWN + FINAL_GOAL, so nobody has to hand-author a
route file per scenario just to run a PCLA-backed autopilot
(pcla_transfuser_autopilot.py, own_perception_plant2_autopilot.py).

WHY THIS IS SAFE / WHAT IT ACTUALLY NEEDS: read directly from PCLA's own
route-loading code (external/PCLA/leaderboard_codes/route_parser.py +
route_manipulation.py):
  - `RouteParser` only reads `x`, `y`, `z` off each `<waypoint>` element
    (`route_parser.py`'s `parse_routes_file`) -- `pitch`/`roll`/`yaw`
    attributes are required to be PRESENT (well-formed XML) but their
    VALUES are never read for route purposes. This module still writes
    real yaw/pitch/roll for readability/debugging, but only x/y/z matter
    functionally.
  - `interpolate_trajectory()` (`route_manipulation.py`) takes just the
    coarse waypoint list and calls CARLA's own `GlobalRoutePlanner` to
    trace a legal, road-network-aware path between consecutive points.
    So a route needs only TWO waypoints -- start and end -- for PCLA to
    produce a full, drivable dense route; CARLA's planner does the rest.

This means: any Scenario's EGO_SPAWN (a carla.Transform) and FINAL_GOAL
(an (x, y) world-frame tuple) is already 100% of what's needed. Nothing
here is a guess about the route itself (turn-by-turn correctness is
CARLA's GlobalRoutePlanner's job, not this module's).

NEVER tested against a live CARLA/PCLA run in this environment -- syntax/
structure verified against route_parser.py's actual read logic, not
against a real interpolate_trajectory() call.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import carla


def build_route_xml(
    spawn_transform: carla.Transform,
    goal_xy: tuple[float, float],
    out_path: str,
    route_id: str = "auto",
    town: str = "auto",
) -> str:
    """Writes a 2-waypoint route XML (spawn -> goal) to out_path, in the
    exact element/attribute shape route_parser.py expects
    (<route id=... town=...><waypoint x=... y=... z=... pitch=... roll=...
    yaw=.../></route>). Returns out_path for convenience.

    goal_xy's z is set to spawn_transform's z (roads are locally near-flat
    at this scale; CARLA's GlobalRoutePlanner snaps to the nearest actual
    road waypoint internally regardless -- see this module's docstring).

    town: PCLA's RouteParser reads this attribute (`route.attrib['town']`)
    but PCLA.py's own setup_route() never checks it against the actually-
    loaded world -- it's bookkeeping, not a load instruction. Left as
    "auto" (a label) unless the caller wants a specific value recorded.
    """
    route_el = ET.Element("route", {"id": route_id, "town": town})

    start = spawn_transform.location
    start_yaw = spawn_transform.rotation.yaw
    ET.SubElement(route_el, "waypoint", {
        "x": str(start.x), "y": str(start.y), "z": str(start.z),
        "pitch": "0.0", "roll": "0.0", "yaw": str(start_yaw),
    })

    goal_x, goal_y = goal_xy
    ET.SubElement(route_el, "waypoint", {
        "x": str(goal_x), "y": str(goal_y), "z": str(start.z),
        "pitch": "0.0", "roll": "0.0", "yaw": str(start_yaw),  # yaw at the goal isn't read (see docstring); reuse start's
    })

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    tree = ET.ElementTree(route_el)
    ET.indent(tree, space="\t")
    tree.write(out_path, encoding="UTF-8", xml_declaration=True)
    return out_path
