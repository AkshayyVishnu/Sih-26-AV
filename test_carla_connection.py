"""
Incremental CARLA remote-connection diagnostic. Run this the moment you
get remote access, BEFORE trying to wire the real pipeline in -- it
tests exactly the stages pipeline/pipeline.py actually needs (camera,
LiDAR, segmentation, control), one at a time, so a failure tells you
precisely which stage broke instead of a confusing crash somewhere deep
in the real pipeline.

Fill in HOST/PORT below, then:
    .venv\\Scripts\\python.exe test_carla_connection.py
"""
from __future__ import annotations

import sys
import time

import numpy as np

# --- FILL THESE IN -------------------------------------------------------
HOST = "REPLACE_WITH_FRIENDS_IP"  # e.g. "192.168.1.42" (LAN) or their public IP
PORT = 2000
TIMEOUT_S = 10.0
# ---------------------------------------------------------------------------


def _header(title: str):
    print(f"\n{'='*70}\n{title}\n{'='*70}")


def _ok(msg: str):
    print(f"  [PASS] {msg}")


def _fail(msg: str):
    print(f"  [FAIL] {msg}")


def stage_a_connect():
    _header("Stage A: Basic connection")
    try:
        import carla
    except ImportError:
        _fail("`carla` package not installed in this environment. Run: pip install carla==<version matching server>")
        sys.exit(1)

    try:
        client = carla.Client(HOST, PORT)
        client.set_timeout(TIMEOUT_S)
        client_ver = client.get_client_version()
        server_ver = client.get_server_version()
        _ok(f"Connected. Client version: {client_ver}, Server version: {server_ver}")
        if client_ver != server_ver:
            print(f"  [WARNING] Client/server version MISMATCH ({client_ver} vs {server_ver}) -- "
                  f"this is a known hard-failure risk (docs/architecture.md constraints). "
                  f"pip install carla=={server_ver} to fix.")
        return client
    except Exception as e:
        _fail(f"Could not connect to {HOST}:{PORT} -- {type(e).__name__}: {e}")
        print("  Checklist: is CARLA actually running on your friend's machine? "
              "Is the IP/port correct? Is a firewall blocking inbound connections on that port? "
              "Are you both actually reachable (same LAN, or VPN/port-forward set up for remote)?")
        sys.exit(1)


def stage_b_world(client):
    _header("Stage B: World access")
    try:
        world = client.get_world()
        map_name = world.get_map().name
        _ok(f"Got world. Current map: {map_name}")
        actors = world.get_actors()
        vehicles = actors.filter("vehicle.*")
        _ok(f"World has {len(actors)} actors total, {len(vehicles)} vehicles.")
        return world
    except Exception as e:
        _fail(f"Could not access world -- {type(e).__name__}: {e}")
        sys.exit(1)


def stage_c_ego_vehicle(world, client):
    _header("Stage C: Find or spawn an ego vehicle")
    import carla

    vehicles = world.get_actors().filter("vehicle.*")
    if len(vehicles) > 0:
        ego = vehicles[0]
        _ok(f"Using existing vehicle: {ego.type_id} (id={ego.id})")
        return ego, False  # False = we didn't spawn it, don't destroy it later

    print("  No existing vehicle found -- attempting to spawn a test vehicle.")
    try:
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("vehicle.*")[0]
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            _fail("No spawn points available on this map.")
            sys.exit(1)
        ego = world.spawn_actor(vehicle_bp, spawn_points[0])
        _ok(f"Spawned test vehicle: {ego.type_id} (id={ego.id}) -- REMEMBER: this script destroys it at the end.")
        return ego, True  # True = we spawned it, safe to destroy later
    except Exception as e:
        _fail(f"Could not spawn a vehicle -- {type(e).__name__}: {e}")
        sys.exit(1)


def stage_d_camera(world, ego):
    _header("Stage D: RGB camera sensor")
    import carla

    result = {"image": None}

    def on_image(image):
        result["image"] = image

    try:
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", "800")
        bp.set_attribute("image_size_y", "600")
        transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(bp, transform, attach_to=ego)
        camera.listen(on_image)

        _wait_for(result, "image", label="camera frame")

        if result["image"] is not None:
            img = result["image"]
            arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
            _ok(f"Received RGB frame: {img.width}x{img.height}, array shape {arr.shape}")
        camera.stop()
        camera.destroy()
        return True
    except Exception as e:
        _fail(f"Camera sensor failed -- {type(e).__name__}: {e}")
        return False


def stage_e_lidar(world, ego):
    _header("Stage E: LiDAR sensor")
    import carla

    result = {"cloud": None}

    def on_lidar(data):
        result["cloud"] = data

    try:
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        transform = carla.Transform(carla.Location(x=0.0, z=2.4))
        lidar = world.spawn_actor(bp, transform, attach_to=ego)
        lidar.listen(on_lidar)

        _wait_for(result, "cloud", label="LiDAR sweep")

        if result["cloud"] is not None:
            data = result["cloud"]
            points = np.frombuffer(data.raw_data, dtype=np.float32).reshape((-1, 4))  # x,y,z,intensity
            _ok(f"Received LiDAR sweep: {points.shape[0]} points, shape {points.shape}")
            print(f"  Sample point (x,y,z,intensity): {points[0] if len(points) else 'none'}")
        lidar.stop()
        lidar.destroy()
        return True
    except Exception as e:
        _fail(f"LiDAR sensor failed -- {type(e).__name__}: {e}")
        return False


def stage_f_segmentation(world, ego):
    _header("Stage F: Semantic segmentation camera")
    import carla

    result = {"seg": None}

    def on_seg(image):
        result["seg"] = image

    try:
        bp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
        bp.set_attribute("image_size_x", "800")
        bp.set_attribute("image_size_y", "600")
        transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        seg_cam = world.spawn_actor(bp, transform, attach_to=ego)
        seg_cam.listen(on_seg)

        _wait_for(result, "seg", label="segmentation frame")

        if result["seg"] is not None:
            img = result["seg"]
            # RAW tags live in the red channel -- do NOT call image.convert(CityScapesPalette)
            # or you lose the raw tag IDs pipeline/drivable_area.py needs.
            arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
            tags = arr[:, :, 2]  # BGRA order in CARLA's raw buffer -- red channel is index 2
            unique_tags = np.unique(tags)
            _ok(f"Received segmentation frame: {img.width}x{img.height}")
            print(f"  Unique tag IDs present in this frame: {sorted(unique_tags.tolist())}")
            print(f"  IMPORTANT: confirm which of these correspond to 'Road'/'RoadLine' against "
                  f"carla.CityObjectLabel for YOUR CARLA version -- pipeline/drivable_area.py currently "
                  f"assumes Road=7, RoadLine=6 as a default, unverified guess.")
        seg_cam.stop()
        seg_cam.destroy()
        return True
    except Exception as e:
        _fail(f"Segmentation camera failed -- {type(e).__name__}: {e}")
        return False


def stage_g_control(ego):
    _header("Stage G: Vehicle control")
    import carla

    try:
        v_before = ego.get_velocity()
        speed_before = np.hypot(v_before.x, v_before.y)

        ego.apply_control(carla.VehicleControl(throttle=0.6, steer=0.0))
        time.sleep(1.5)

        v_after = ego.get_velocity()
        speed_after = np.hypot(v_after.x, v_after.y)

        _ok(f"Applied throttle=0.6 for 1.5s. Speed before: {speed_before:.2f} m/s, after: {speed_after:.2f} m/s")
        if speed_after <= speed_before:
            print("  [WARNING] Speed didn't increase -- check the vehicle isn't blocked, "
                  "or that physics/autopilot isn't overriding manual control.")

        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        return True
    except Exception as e:
        _fail(f"Control command failed -- {type(e).__name__}: {e}")
        return False


def _wait_for(result_dict: dict, key: str, label: str, timeout_s: float = 5.0):
    start = time.time()
    while result_dict[key] is None and (time.time() - start) < timeout_s:
        time.sleep(0.05)
    if result_dict[key] is None:
        _fail(f"Timed out waiting for {label} ({timeout_s}s) -- sensor may not be ticking "
              f"(check world is not paused, or synchronous mode needs a manual world.tick()).")


def main():
    if HOST == "REPLACE_WITH_FRIENDS_IP":
        print("Edit HOST at the top of this file first -- set it to your friend's actual IP.")
        sys.exit(1)

    client = stage_a_connect()
    world = stage_b_world(client)
    ego, we_spawned_it = stage_c_ego_vehicle(world, client)

    results = {
        "camera": stage_d_camera(world, ego),
        "lidar": stage_e_lidar(world, ego),
        "segmentation": stage_f_segmentation(world, ego),
        "control": stage_g_control(ego),
    }

    _header("Summary")
    for stage, passed in results.items():
        print(f"  {stage}: {'PASS' if passed else 'FAIL'}")

    if we_spawned_it:
        print("\nCleaning up the test vehicle this script spawned...")
        ego.destroy()


if __name__ == "__main__":
    main()
