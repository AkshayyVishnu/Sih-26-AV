# Setup checklist — running this against a real CARLA instance

You have the CARLA server and a fine-tuned YOLO model. This repo has the
rest of the pipeline (LiDAR fusion, tracking, drivable-area, prediction,
decision logic, planning, control) already built and tested against
synthetic data. Here's exactly what needs to change to run it for real.

## 1. Environment setup

```bash
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

**Check your CARLA server version first** — run `client.get_server_version()`
or check however you launched it. `requirements.txt` currently pins
`carla==0.9.16`. If your server is a **different** version:
- If it's on PyPI for your platform, change the pin and reinstall.
- If not (this happened once already — 0.9.15 wasn't available via pip
  for this platform), install from the `.whl` file bundled inside your
  own CARLA install at `PythonAPI/carla/dist/`.

Also install your own YOLO model's dependencies if they're not already
in `requirements.txt` (e.g. a specific `ultralytics` version your model
was trained with).

## 2. Fill in `run_live.py`'s CONFIG section — nothing here is a working default

| Value | What it needs |
|---|---|
| `YOLO_MODEL_PATH` | Path to your fine-tuned `.pt` checkpoint |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` / `CAMERA_FOV_DEG` | Must match whatever camera resolution/FOV your scene actually uses |
| `CAMERA_MOUNT` / `LIDAR_MOUNT` / `SEG_CAMERA_MOUNT` | Real sensor mount positions on your ego vehicle. If these three sensors are NOT co-located, you need a separate extrinsic per pair, not the shared `CAMERA_TO_LIDAR_EXTRINSIC` |
| `VEHICLE_WHEELBASE_M` | Your ego vehicle blueprint's real wheelbase — wrong value makes the Pure Pursuit steering geometry systematically wrong |
| `GOAL_X`, `GOAL_Y` | A real **CARLA world-frame** destination coordinate (e.g. a waypoint on your route) — NOT a "meters ahead" offset. See the frame note below. |

## 3. One thing already fixed for you, worth knowing about

`pipeline/pipeline.py` transforms LiDAR-derived object positions from
the sensor's local frame into CARLA's world frame internally, so
tracking/planning stay consistent as the vehicle actually moves. This
was a real bug caught before you ever got this repo — the synthetic
test demo never moved its fake ego vehicle, so the bug was invisible
until real motion was considered. You don't need to do anything for
this — just know `GOAL_X`/`GOAL_Y` must be **world-frame**, matching
`ego.get_transform().location.x/y`, not an ego-relative offset.

## 4. Verify the semantic segmentation tag IDs match your server version

`pipeline/drivable_area.py` uses `Road=1`, `RoadLine=24` — confirmed
correct for CARLA 0.9.16 specifically (CARLA's own docs note these tag
IDs changed between versions). If your server isn't 0.9.16, re-check
this against `carla.readthedocs.io/en/<your-version>/ref_sensors/`
before trusting drivable-area output.

## 5. Run it

```bash
.venv\Scripts\python.exe run_live.py
```

It expects an ego vehicle already present in the world (spawned by
whoever built your scene) — it attaches sensors to the first vehicle it
finds. Ctrl+C to stop; sensors and synchronous mode are cleaned up
automatically.

## 6. If something goes wrong, diagnose stage-by-stage first

Run `test_carla_connection.py` (fill in `HOST`/`PORT` at the top) before
debugging `run_live.py` directly — it checks connection, world access,
vehicle, camera, LiDAR, segmentation, and control **individually**, so a
failure tells you exactly which stage broke instead of a confusing crash
somewhere inside the full pipeline.

## Known risks to watch for (already documented, not new)

- Client/server version mismatch is a hard-failure risk, not a warning
- CARLA's own documented issue: camera-LiDAR extrinsic calibration isn't
  guaranteed constant across frames (issue #3795) — if fused distances
  look wrong or jump around, check this first
- The planner's costmap is a fixed 60m×60m window centered on the ego —
  a goal outside that range silently fails to find a path
- Full details: `docs/pipeline-decision-log.md` and `docs/architecture.md`
