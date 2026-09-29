# Commands to run the pipeline

## One-time setup

```bash
python -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Confirm your CARLA server version matches `carla==0.9.16` in
`requirements.txt` first — see `SETUP.md` if it doesn't.

## Test without any CARLA connection (synthetic data)

```bash
.venv\Scripts\python.exe run_demo.py
```

## Test the CARLA connection itself, stage by stage

Edit `HOST` at the top of `test_carla_connection.py` first, then:

```bash
.venv\Scripts\python.exe test_carla_connection.py
```

## Run the real pipeline against a live CARLA instance

Fill in the CONFIG section at the top of `run_live.py` first (YOLO model
path, sensor mounts, wheelbase, goal coordinates — see `SETUP.md`), then:

```bash
.venv\Scripts\python.exe run_live.py
```

Ctrl+C to stop.

## Check results after any run

```bash
dir logs\*.log
```

Latest log = most recent run's full decision trail (fusion, tracking,
predictions, decisions, replanning, latency per tick).
