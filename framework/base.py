"""
framework/base.py -- the shared engine: TickContext, Scenario, Autopilot,
ScenarioRunner. Every concrete scenario (framework/scenarios/*.py) is
just a Scenario subclass; every driving algorithm
(framework/autopilots/*.py) is just an Autopilot subclass. Nothing here
knows about a specific scenario's actors or a specific algorithm's
internals -- that's the whole point. See DESIGN_GUIDELINES.md before
extending either.

Read this file's classes top to bottom for the full contract; the
guidelines doc is the "how do I use this" version of what's here.
"""
from __future__ import annotations

import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

import carla
import numpy as np

from carla_runtime import (
    build_camera_intrinsic,
    build_camera_to_lidar_extrinsic,
    carla_image_to_rgb_array,
    carla_lidar_to_xyz,
    carla_segmentation_to_tags,
    compute_local_goal,
    spawn_ego_sensors,
)
from framework.pcla_route import build_route_xml
from pipeline.metrics import MetricsRecorder
from pipeline.pipeline import TickTimings
from pipeline.types import ControlCommand, EgoState

try:
    from viz import Dashboard
except ImportError:  # pragma: no cover -- pygame/display not available
    Dashboard = None


# ============================================================
# TickContext -- built ONCE per tick by ScenarioRunner, shared by
# scenario.on_tick(), autopilot.compute(), and the dashboard. Exists
# specifically to avoid re-fetching the same CARLA state (e.g. the ego's
# transform) from multiple call sites in the same tick -- a real,
# measured issue found while tracing scenario_2.py's data flow before
# this refactor existed.
# ============================================================

@dataclass
class TickContext:
    tick_count: int
    sim_time_s: float
    world: carla.World
    ego_vehicle: carla.Vehicle
    ego_x: float
    ego_y: float
    ego_yaw: float          # radians
    ego_speed: float        # m/s
    rgb_array: np.ndarray | None    # None until the camera's first frame arrives
    lidar_xyz: np.ndarray | None
    seg_tags: np.ndarray | None

    @classmethod
    def gather(cls, world: carla.World, ego_vehicle: carla.Vehicle, latest: dict, tick_count: int) -> "TickContext":
        transform = ego_vehicle.get_transform()
        velocity = ego_vehicle.get_velocity()
        return cls(
            tick_count=tick_count,
            sim_time_s=world.get_snapshot().timestamp.elapsed_seconds,
            world=world,
            ego_vehicle=ego_vehicle,
            ego_x=transform.location.x,
            ego_y=transform.location.y,
            ego_yaw=np.radians(transform.rotation.yaw),
            ego_speed=float(np.hypot(velocity.x, velocity.y)),
            rgb_array=carla_image_to_rgb_array(latest["rgb"]) if latest["rgb"] is not None else None,
            lidar_xyz=carla_lidar_to_xyz(latest["lidar"]) if latest["lidar"] is not None else None,
            seg_tags=carla_segmentation_to_tags(latest["seg"]) if latest["seg"] is not None else None,
        )

    @property
    def sensors_ready(self) -> bool:
        return self.rgb_array is not None and self.lidar_xyz is not None and self.seg_tags is not None


# ============================================================
# Scenario -- "what actors do you spawn, and does anything scripted
# happen mid-run?" Nothing about CARLA connection/sync-mode/sensor/
# cleanup mechanics belongs in a subclass of this -- see
# DESIGN_GUIDELINES.md if you're about to write one.
# ============================================================

class Scenario(ABC):
    MAP_NAME = "Town03"
    EGO_BLUEPRINT = "vehicle.tesla.model3"
    VEHICLE_WHEELBASE_M = 2.8
    EGO_SPAWN: carla.Transform          # required, set by the subclass
    FINAL_GOAL: tuple[float, float]     # required, set by the subclass
    SPECTATOR_TRANSFORM: carla.Transform | None = None  # optional; runner falls back to EGO_SPAWN if unset
    ROUTE_XML_PATH: str | None = None   # optional -- a leaderboard-format route XML for PCLA-backed autopilots
                                          # (framework/autopilots/pcla_*.py). Leave unset and the runner
                                          # auto-generates one from EGO_SPAWN + FINAL_GOAL (see
                                          # framework/pcla_route.py) -- set this explicitly only if a
                                          # hand-authored route with specific intermediate waypoints matters
                                          # for this scenario. Autopilots that don't need a route (PipelineAutopilot)
                                          # ignore this entirely -- it costs nothing for scenarios that don't use PCLA.
    GOAL_REACHED_RADIUS_M = 3.0          # used by the default is_complete() below -- how close to FINAL_GOAL
                                          # counts as "reached it" for the PS's scenario-completion-rate metric
    MAX_TICKS: int | None = None         # optional hard stop (in ticks) if a scenario should end deterministically
                                          # rather than run until Ctrl+C or is_complete() -- default None preserves
                                          # today's run-until-stopped behavior for every existing scenario

    def __init__(self):
        self._tracked_actors: list = []

    def track(self, actor):
        """Wrap every world.spawn_actor(...) (or a raw actor ID from a
        batch spawn) call in spawn_actors() with this, so the runner can
        destroy it automatically on shutdown -- e.g.
        `self.pedestrian = self.track(world.spawn_actor(...))`.
        Forgetting this is the #1 way to leak actors between runs.
        Accepts either an actor object or a raw actor ID (both work with
        carla.command.DestroyActor -- confirmed against the installed
        carla package's own command.pyi stub), so batch-spawned actors
        (which only return IDs) and plain spawn_actor() results (which
        return objects) can both go through this same helper.
        """
        self._tracked_actors.append(actor)
        return actor

    @abstractmethod
    def spawn_actors(self, world: carla.World, bp_lib: carla.BlueprintLibrary, client: carla.Client) -> None:
        """Spawn whatever makes this scenario unique. Track each actor
        via self.track(...) so it gets cleaned up automatically. Runs
        BEFORE the ego is spawned -- if you need to avoid spawning on top
        of the ego, use self.EGO_SPAWN.location (known statically), not a
        live ego actor (it doesn't exist yet)."""

    def on_tick(self, ctx: TickContext) -> None:
        """Optional: scripted per-tick logic (a trigger distance, a timed
        event, ...). Runs BEFORE autopilot.compute() each tick -- so
        something this call changes (e.g. a pedestrian starting to move)
        is reflected in the NEXT tick's sensor data, not this one.
        Default: nothing happens -- most scenarios won't need to
        override this at all."""

    def cleanup_extra(self, client: carla.Client) -> None:
        """Optional: anything beyond destroying tracked actors -- e.g.
        restoring frozen traffic lights, stopping walker AI controllers.
        Called BEFORE tracked actors are destroyed, so actors this needs
        (e.g. to call .stop() on) are still alive when it runs."""

    def is_complete(self, ctx: TickContext) -> bool:
        """Optional: has this run finished successfully? Checked every
        tick, after autopilot.compute() -- returning True ends the run
        cleanly (ScenarioRunner records completed=True in its metrics
        summary; see pipeline/metrics.py) rather than requiring Ctrl+C.

        Default: True once the ego is within GOAL_REACHED_RADIUS_M of
        FINAL_GOAL (straight-line distance) -- reasonable for any
        scenario whose FINAL_GOAL genuinely marks "done" (true of all
        three scenarios shipped today: pedestrian_jumpout, traffic_stress,
        chaotic_traffic). Override this if a scenario's real completion
        condition is something else (e.g. "survived N ticks without a
        collision" for a pure stress test where reaching a specific point
        isn't really the point) -- don't stretch FINAL_GOAL's meaning to
        fit a scenario it doesn't actually describe.
        """
        distance = np.hypot(ctx.ego_x - self.FINAL_GOAL[0], ctx.ego_y - self.FINAL_GOAL[1])
        return bool(distance <= self.GOAL_REACHED_RADIUS_M)


# ============================================================
# Autopilot -- the swappable "brain." PipelineAutopilot wraps today's
# pipeline/Pipeline unchanged; anything else implementing this same
# interface can be swapped in without touching a single scenario file.
# See DESIGN_GUIDELINES.md if you're about to write one.
# ============================================================

class Autopilot(ABC):
    def setup(
        self,
        *,
        ego_camera: carla.Sensor,
        camera_intrinsic: np.ndarray,
        camera_to_lidar_extrinsic: np.ndarray,
        image_width: int,
        image_height: int,
        wheelbase_m: float,
        dt: float,
        client: carla.Client | None = None,
        ego_vehicle: carla.Vehicle | None = None,
        route_xml_path: str | None = None,
    ) -> None:
        """Called once, after sensors are spawned, before the tick loop
        starts. Default no-op -- an autopilot that doesn't need any of
        this (e.g. a wrapper around CARLA's own built-in autopilot) can
        skip overriding it.

        `client`, `ego_vehicle`, `route_xml_path` are the one deliberate
        exception to "must not reach back into a CARLA client" below --
        added specifically for PCLA-backed autopilots
        (framework/autopilots/pcla_*.py), since PCLA's own constructor
        (`PCLA(agent, vehicle, route, client)`) needs exactly these three
        plus route access, and PCLA manages its own sensor attachment
        internally (it is not something TickContext can hand over
        piecemeal). `route_xml_path` is auto-generated by ScenarioRunner
        from the scenario's EGO_SPAWN/FINAL_GOAL if the scenario didn't
        set ROUTE_XML_PATH explicitly (see framework/pcla_route.py) --
        an autopilot doesn't need to know which case it is. All three are
        keyword-only with a None default so every existing Autopilot
        subclass's setup() override keeps working unchanged; only an
        autopilot that actually needs them has to accept them.

        Beyond these three, still must not reach back into a CARLA client
        or spawn its own actors for anything else -- everything else it
        needs to drive comes through here and through TickContext each
        tick. If you find yourself wanting more than client/ego_vehicle/
        route from CARLA directly, that's a sign the logic belongs in a
        Scenario, not here.
        """

    @abstractmethod
    def compute(self, ctx: TickContext, goal_xy: tuple[float, float]) -> ControlCommand:
        """Called once per tick, only once ctx.sensors_ready is True."""

    def debug_info(self) -> dict:
        """Optional: whatever this autopilot wants the dashboard/latency
        summary/metrics recording to show this tick. Recognized (all
        optional) keys:
          - 'detections' (list[Detection])
          - 'planned_waypoints' (list[tuple[float,float]], WORLD frame)
          - 'timings' (an object with a .total_ms attribute -- e.g.
            pipeline.pipeline.TickTimings -- or a plain float ms value)
          - 'replanned' (bool, default False if absent)
          - 'path_valid' (bool, default True if absent)
          - 'decision_mode' (str, default this autopilot's class name if
            absent -- e.g. PipelineAutopilot exposes
            pipeline.decision_logic's actual state name; an autopilot
            with no comparable concept just lets this default)
        The last three feed pipeline/metrics.py's MetricsRecorder (see
        ScenarioRunner.run()) for the PS's replanning-latency/path-
        smoothness/completion-rate metrics -- not every autopilot has a
        real answer for them (a black-box model might not expose a
        "replanned" concept at all), which is exactly why they're
        optional with sensible defaults rather than required.
        Default: {} -- not every autopilot has something visualizable
        here, so this is deliberately optional, not part of compute()'s
        required return value.
        """
        return {}

    def cleanup(self) -> None:
        """Optional: release anything this autopilot owns that ISN'T a
        CARLA actor spawned through Scenario.track() or ScenarioRunner's
        own ego/sensor spawn (both already cleaned up automatically).
        Default no-op -- most autopilots (e.g. PipelineAutopilot) own
        nothing beyond in-process Python objects that garbage-collect
        normally and don't need this. Exists specifically for
        PCLA-backed autopilots (framework/autopilots/pcla_*.py): PCLA
        attaches its OWN sensors to the ego vehicle internally (separate
        from ScenarioRunner's standard rig) and its own cleanup() also
        destroys the ego vehicle itself -- see those autopilots'
        cleanup() docstrings for why that's safe, deliberate redundancy
        with ScenarioRunner's own ego/sensor teardown, not a conflict to
        avoid. Called by ScenarioRunner in its `finally` block, BEFORE
        it destroys ego_sensors/ego_vehicle itself.
        """


# ============================================================
# ScenarioRunner -- the engine. Written once; every scenario/autopilot
# combination runs through this unchanged.
# ============================================================

class ScenarioRunner:
    def __init__(
        self,
        scenario: Scenario,
        autopilot: Autopilot,
        *,
        enable_visualization: bool = True,
        sensor_width: int = 800,
        sensor_height: int = 600,
        sensor_fov_deg: float = 90.0,
        fixed_delta_seconds: float = 0.05,
        host: str = "localhost",
        port: int = 2000,
    ):
        self.scenario = scenario
        self.autopilot = autopilot
        self.enable_visualization = enable_visualization and Dashboard is not None
        if enable_visualization and Dashboard is None:
            print("[framework] pygame/viz.py not available -- running without the dashboard.")
        self.sensor_width = sensor_width
        self.sensor_height = sensor_height
        self.sensor_fov_deg = sensor_fov_deg
        self.fixed_delta_seconds = fixed_delta_seconds
        self.host = host
        self.port = port

    def run(self):
        client = carla.Client(self.host, self.port)
        client.set_timeout(10.0)

        world = None
        ego_vehicle = None
        ego_sensors: list = []
        dashboard = None
        completed, reason = False, "stopped before reaching goal"  # set before try -- finally references
                                                                     # both even if an exception hits before
                                                                     # the loop starts

        # scenario_name includes the AUTOPILOT too, not just the
        # scenario -- the whole point of having 3 registered autopilots
        # is comparing them on the SAME scenario, so metrics_export.py
        # needs to see them as separate rows, not merged into one
        # scenario's numbers.
        metrics = MetricsRecorder(
            scenario_name=f"{type(self.scenario).__name__}_{type(self.autopilot).__name__}",
            dt=self.fixed_delta_seconds,
        )

        try:
            print(f"Loading {self.scenario.MAP_NAME}...")
            world = client.load_world(self.scenario.MAP_NAME)

            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.fixed_delta_seconds
            world.apply_settings(settings)

            bp_lib = world.get_blueprint_library()
            self.scenario.spawn_actors(world, bp_lib, client)
            print(f"Scenario actors spawned ({len(self.scenario._tracked_actors)} tracked).")

            ego_bp = bp_lib.find(self.scenario.EGO_BLUEPRINT)
            ego_bp.set_attribute("role_name", "ego")
            ego_vehicle = world.spawn_actor(ego_bp, self.scenario.EGO_SPAWN)
            print(f"Spawned ego vehicle ({self.scenario.EGO_BLUEPRINT}).")

            mount = carla.Transform(carla.Location(x=1.5, z=2.4))
            ego_sensors, latest = spawn_ego_sensors(
                world, ego_vehicle, mount, self.sensor_width, self.sensor_height, self.sensor_fov_deg,
            )
            camera_intrinsic = build_camera_intrinsic(self.sensor_width, self.sensor_height, self.sensor_fov_deg)

            # Collision sensor -- not part of carla_runtime.spawn_ego_sensors()
            # deliberately: run_live.py's own (pre-framework) collision
            # sensor was always spawned inline at the call site rather than
            # folded into that shared helper, and this keeps the same
            # convention. Appended to ego_sensors so the existing generic
            # stop+destroy cleanup below covers it too, nothing extra needed.
            collision_bp = bp_lib.find("sensor.other.collision")
            collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego_vehicle)
            collision_sensor.listen(lambda event: metrics.record_collision(event.other_actor.type_id))
            ego_sensors.append(collision_sensor)

            # Route XML: use the scenario's own if it set one, otherwise
            # auto-generate a 2-waypoint route from EGO_SPAWN/FINAL_GOAL
            # (see framework/pcla_route.py's docstring for why 2 waypoints
            # is enough -- CARLA's own GlobalRoutePlanner fills in the
            # rest). Cheap (a few KB XML write) and harmless for
            # autopilots that never look at it -- PipelineAutopilot's
            # setup() below just doesn't have a route_xml_path parameter
            # doing anything with the value.
            route_xml_path = self.scenario.ROUTE_XML_PATH
            if route_xml_path is None:
                logs_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs", "routes")
                route_xml_path = build_route_xml(
                    self.scenario.EGO_SPAWN, self.scenario.FINAL_GOAL,
                    out_path=os.path.join(logs_dir, f"{type(self.scenario).__name__}_auto_route.xml"),
                )

            self.autopilot.setup(
                ego_camera=ego_sensors[0],
                camera_intrinsic=camera_intrinsic,
                camera_to_lidar_extrinsic=build_camera_to_lidar_extrinsic(),
                image_width=self.sensor_width,
                image_height=self.sensor_height,
                wheelbase_m=self.scenario.VEHICLE_WHEELBASE_M,
                dt=self.fixed_delta_seconds,
                client=client,
                ego_vehicle=ego_vehicle,
                route_xml_path=route_xml_path,
            )

            spectator = world.get_spectator()
            spectator.set_transform(self.scenario.SPECTATOR_TRANSFORM or self.scenario.EGO_SPAWN)

            dashboard = Dashboard() if self.enable_visualization else None
            viz_active = self.enable_visualization

            world.tick()  # let actors physically appear before the loop starts
            print("Simulation is ready. Starting control loop...")

            tick_count = 0
            while True:
                world.tick()
                tick_count += 1
                ctx = TickContext.gather(world, ego_vehicle, latest, tick_count)

                self.scenario.on_tick(ctx)

                if not ctx.sensors_ready:
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                    continue

                goal_xy = compute_local_goal(ctx.ego_x, ctx.ego_y, *self.scenario.FINAL_GOAL)
                control = self.autopilot.compute(ctx, goal_xy)
                ego_vehicle.apply_control(carla.VehicleControl(
                    throttle=control.throttle, steer=control.steer, brake=control.brake,
                ))

                debug = self.autopilot.debug_info()

                # Normalize 'timings' into a TickTimings for
                # MetricsRecorder: PipelineAutopilot already provides a
                # real one (7 named stages); an autopilot that only
                # reports one number (both PCLA-backed autopilots) has
                # that number placed in planning_ms with everything else
                # zero, so total_ms still comes out right -- see
                # docs/pipeline-decision-log.md's entry on why this
                # doesn't mean the same thing across autopilots.
                raw_timings = debug.get("timings")
                if isinstance(raw_timings, TickTimings):
                    timings_for_metrics = raw_timings
                else:
                    total = float(raw_timings) if raw_timings is not None else 0.0
                    timings_for_metrics = TickTimings(
                        fusion_ms=0.0, tracking_ms=0.0, prediction_ms=0.0,
                        drivable_area_ms=0.0, decision_ms=0.0, planning_ms=total, control_ms=0.0,
                    )

                accel = ego_vehicle.get_acceleration()
                distance_to_final_goal = float(np.hypot(
                    ctx.ego_x - self.scenario.FINAL_GOAL[0], ctx.ego_y - self.scenario.FINAL_GOAL[1],
                ))
                metrics.record_tick(
                    tick=ctx.tick_count, timings=timings_for_metrics,
                    replanned=debug.get("replanned", False),
                    path_valid=debug.get("path_valid", True),
                    decision_mode=debug.get("decision_mode", type(self.autopilot).__name__),
                    speed_mps=ctx.ego_speed,
                    accel_x=accel.x, accel_y=accel.y,
                    distance_to_goal_m=distance_to_final_goal,
                )

                if dashboard is not None and viz_active:
                    ego_state = EgoState(ctx.ego_x, ctx.ego_y, ctx.ego_yaw, ctx.ego_speed, *goal_xy)
                    viz_active = dashboard.update(
                        ctx.rgb_array, ctx.seg_tags, ctx.lidar_xyz,
                        debug.get("detections", []), control,
                        debug.get("planned_waypoints", []), ego_state, ctx.sim_time_s,
                    )

                if self.scenario.is_complete(ctx):
                    completed, reason = True, "reached goal"
                    print(f"\nScenario complete (within {self.scenario.GOAL_REACHED_RADIUS_M}m of FINAL_GOAL) "
                          f"after {ctx.tick_count} ticks.")
                    break
                if self.scenario.MAX_TICKS is not None and ctx.tick_count >= self.scenario.MAX_TICKS:
                    completed, reason = False, f"hit MAX_TICKS={self.scenario.MAX_TICKS} without completing"
                    print(f"\n{reason}.")
                    break

        except KeyboardInterrupt:
            print("\nCancelled by user. Bye!")
            reason = "manually stopped"

        finally:
            # Run first, while world/client are still fully alive -- see
            # Autopilot.cleanup()'s docstring for why PCLA-backed
            # autopilots need this (they own resources ScenarioRunner's
            # own generic ego/sensor teardown below doesn't know about).
            self.autopilot.cleanup()

            if dashboard is not None:
                dashboard.close()

            if world is not None:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)

            if ego_sensors:
                for sensor in ego_sensors:
                    sensor.stop()
                client.apply_batch([carla.command.DestroyActor(x) for x in ego_sensors])
                print(f"Stopped and destroyed {len(ego_sensors)} ego sensor(s).")

            if ego_vehicle is not None:
                client.apply_batch([carla.command.DestroyActor(ego_vehicle)])
                print("Destroyed ego vehicle.")

            self.scenario.cleanup_extra(client)

            if self.scenario._tracked_actors:
                client.apply_batch([carla.command.DestroyActor(x) for x in self.scenario._tracked_actors])
                print(f"Destroyed {len(self.scenario._tracked_actors)} scenario actor(s).")

            time.sleep(0.5)

            # Writes logs/metrics_<ScenarioClass>_<AutopilotClass>_<run_id>.csv
            # + _summary.json (replanning latency, path-smoothness/jerk,
            # collision count, completion) -- run metrics_export.py
            # afterward to aggregate across runs/autopilots. No-ops
            # harmlessly (logs a warning, returns {}) if zero ticks were
            # ever recorded (e.g. CARLA connection failed before the loop
            # started).
            metrics.finalize(completed=completed, reason=reason)

            print("Done.")
