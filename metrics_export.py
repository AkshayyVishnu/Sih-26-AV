"""
Aggregates metrics across multiple closed-loop runs (from
pipeline/metrics.py's per-run summary.json files) into the report-ready
numbers: replanning latency, path smoothness, and scenario completion
rate -- per scenario, across however many runs you've collected.

Closed-loop results are seed-sensitive (per carla_garage's own
common-mistakes guide, already cited in docs/architecture.md) -- this
script warns explicitly if a scenario has fewer than 3 runs rather than
silently reporting a single-run number as if it were final.

Run: .venv\\Scripts\\python.exe metrics_export.py
"""
from __future__ import annotations

import glob
import json
import os
from collections import defaultdict

import numpy as np

LOGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def load_all_summaries() -> list[dict]:
    paths = glob.glob(os.path.join(LOGS_DIR, "metrics_*_summary.json"))
    summaries = []
    for p in paths:
        with open(p) as f:
            summaries.append(json.load(f))
    return summaries


def aggregate_by_scenario(summaries: list[dict]) -> dict:
    by_scenario = defaultdict(list)
    for s in summaries:
        by_scenario[s["scenario_name"]].append(s)

    report = {}
    for scenario, runs in by_scenario.items():
        n_runs = len(runs)
        n_completed = sum(1 for r in runs if r["completed"])
        mean_latencies = [r["replanning_latency_ms"]["mean"] for r in runs]
        max_latencies = [r["replanning_latency_ms"]["max"] for r in runs]
        mean_jerks = [r["path_smoothness_jerk_mps3"]["mean"] for r in runs
                       if r["path_smoothness_jerk_mps3"]["mean"] is not None]
        collision_counts = [r["collision_count"] for r in runs]

        entry = {
            "num_runs": n_runs,
            "completion_rate_pct": round(100.0 * n_completed / n_runs, 1) if n_runs else None,
            "replanning_latency_ms": {
                "mean_of_means": round(float(np.mean(mean_latencies)), 2) if mean_latencies else None,
                "worst_max": round(float(np.max(max_latencies)), 2) if max_latencies else None,
            },
            "path_smoothness_jerk_mps3": {
                "mean_of_means": round(float(np.mean(mean_jerks)), 3) if mean_jerks else None,
            },
            "total_collisions_across_runs": sum(collision_counts),
            "runs_with_collision": sum(1 for c in collision_counts if c > 0),
        }

        if n_runs < 3:
            entry["WARNING"] = (
                f"Only {n_runs} run(s) recorded -- closed-loop results are seed-sensitive "
                f"(carla_garage's common-mistakes guide, docs/architecture.md). "
                f"Collect >=3 runs before reporting this as a final number."
            )

        report[scenario] = entry

    return report


def main():
    summaries = load_all_summaries()
    if not summaries:
        print(f"No metrics_*_summary.json files found in {LOGS_DIR} -- run run_live.py first to generate some.")
        return

    n_scenarios = len(set(s["scenario_name"] for s in summaries))
    print(f"Loaded {len(summaries)} run(s) across {n_scenarios} scenario(s).\n")

    report = aggregate_by_scenario(summaries)
    print(json.dumps(report, indent=2))

    out_path = os.path.join(LOGS_DIR, "aggregated_report.json")
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nWritten to {out_path} -- these are your report-ready numbers.")


if __name__ == "__main__":
    main()
