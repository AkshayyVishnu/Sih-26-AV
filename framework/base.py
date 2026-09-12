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
    ) -> None:
        """Called once, after sensors are spawned, before the tick loop
        starts. Default no-op -- an autopilot that doesn't need any of
        this (e.g. a wrapper around CARLA's own built-in autopilot) can
        skip overriding it. Must not reach back into a CARLA client or
        spawn its own actors -- everything it needs to drive comes
        through here and through TickContext each tick."""

    @abstractmethod
    def compute(self, ctx: TickContext, goal_xy: tuple[float, float]) -> ControlCommand:
        """Called once per tick, only once ctx.sensors_ready is True."""

    def debug_info(self) -> dict:
        """Optional: whatever this autopilot wants the dashboard/latency
        summary to show this tick. Recognized (all optional) keys:
        'detections' (list[Detection]), 'planned_waypoints'
        (list[tuple[float,float]], WORLD frame), 'timings' (an object
        with a .total_ms attribute, or a plain float ms value). Default:
        {} -- not every autopilot has something visualizable here (a
        black-box model might not produce explicit detections/paths at
        all), so this is deliberately optional, not part of compute()'s
        required return value.
        """
        return {}


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
        tick_latencies_ms: list[float] = []

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
            self.autopilot.setup(
                ego_camera=ego_sensors[0],
                camera_intrinsic=camera_intrinsic,
                camera_to_lidar_extrinsic=build_camera_to_lidar_extrinsic(),
                image_width=self.sensor_width,
                image_height=self.sensor_height,
                wheelbase_m=self.scenario.VEHICLE_WHEELBASE_M,
                dt=self.fixed_delta_seconds,
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
                timings = debug.get("timings")
                if timings is not None:
                    total_ms = timings.total_ms if hasattr(timings, "total_ms") else float(timings)
                    tick_latencies_ms.append(total_ms)

                if dashboard is not None and viz_active:
                    ego_state = EgoState(ctx.ego_x, ctx.ego_y, ctx.ego_yaw, ctx.ego_speed, *goal_xy)
                    viz_active = dashboard.update(
                        ctx.rgb_array, ctx.seg_tags, ctx.lidar_xyz,
                        debug.get("detections", []), control,
                        debug.get("planned_waypoints", []), ego_state, ctx.sim_time_s,
                    )

        except KeyboardInterrupt:
            print("\nCancelled by user. Bye!")

        finally:
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

            if tick_latencies_ms:
                warm = tick_latencies_ms[5:] or tick_latencies_ms
                print(f"\n--- pipeline latency summary ({len(tick_latencies_ms)} ticks, "
                      f"warmed-up mean of last {len(warm)}) ---")
                print(f"Mean: {np.mean(warm):.2f}ms  Max: {np.max(warm):.2f}ms  Min: {np.min(warm):.2f}ms")

            print("Done.")
