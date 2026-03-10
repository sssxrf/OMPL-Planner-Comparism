#!/usr/bin/env python3
"""
Analyze OMPL tour benchmark outputs.

Works with the newer summary schema:
  planner,env_seed,run,seed,obstacle_seed,resample_attempt,termination_mode,time_limit_per_leg,
  success,solve_time_s,simplify_time_s,total_time_s,total_length_se3,total_length_xyz

Also supports the legacy schema:
  planner,env_seed,run,seed,obstacle_seed,resample_attempt,success,total_time_s,total_length
"""

import argparse
import csv
import math
import os
import statistics
from collections import Counter
from typing import Dict, List


def fmean(values: List[float]) -> float:
    return statistics.mean(values) if values else math.nan


def load_rows(summary_csv: str) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    with open(summary_csv, newline="") as f:
        reader = csv.DictReader(f)
        fields = set(reader.fieldnames or [])
        is_new = "total_length_se3" in fields

        for r in reader:
            success = int(r["success"])
            row: Dict[str, object] = {
                "planner": r["planner"],
                "env_seed": int(r["env_seed"]),
                "run": int(r["run"]),
                "seed": int(r["seed"]),
                "obstacle_seed": int(r["obstacle_seed"]),
                "resample_attempt": int(r["resample_attempt"]),
                "success": success,
                "termination_mode": r.get("termination_mode", "legacy"),
                "time_limit_per_leg": float(r.get("time_limit_per_leg", "nan")),
                "solve_time_s": float(r.get("solve_time_s", r["total_time_s"])),
                "simplify_time_s": float(r.get("simplify_time_s", "0.0")),
                "total_time_s": float(r["total_time_s"]),
            }

            if is_new:
                row["total_length_se3"] = (
                    float(r["total_length_se3"]) if success else math.nan
                )
                row["total_length_xyz"] = (
                    float(r["total_length_xyz"]) if success else math.nan
                )
            else:
                legacy_len = float(r["total_length"]) if success else math.nan
                row["total_length_se3"] = legacy_len
                row["total_length_xyz"] = math.nan

            rows.append(row)

    return rows


def analyze(summary_csv: str) -> None:
    rows = load_rows(summary_csv)
    if not rows:
        raise RuntimeError(f"No rows in {summary_csv}")

    planners = sorted({r["planner"] for r in rows})
    envs = sorted({r["env_seed"] for r in rows})

    print(f"summary: {summary_csv}")
    print(f"rows: {len(rows)}")
    print(f"planners: {', '.join(planners)}")
    print(f"envs: {envs}")
    print()

    print("counts per planner:")
    for p in planners:
        n = sum(1 for r in rows if r["planner"] == p)
        s = sum(int(r["success"]) for r in rows if r["planner"] == p)
        print(f"  {p}: {s}/{n} success ({s/n:.3f})")
    print()

    has_xyz = any(not math.isnan(float(r["total_length_xyz"])) for r in rows)
    print("overall solved-only means:")
    for p in planners:
        g = [r for r in rows if r["planner"] == p and int(r["success"]) == 1]
        times = [float(r["total_time_s"]) for r in g]
        solve = [float(r["solve_time_s"]) for r in g]
        simp = [float(r["simplify_time_s"]) for r in g]
        se3 = [float(r["total_length_se3"]) for r in g if not math.isnan(float(r["total_length_se3"]))]
        line = (
            f"  {p}: mean_total_time={fmean(times):.6f}s "
            f"(solve={fmean(solve):.6f}s, simplify={fmean(simp):.6f}s), "
            f"mean_len_se3={fmean(se3):.6f}"
        )
        if has_xyz:
            xyz = [float(r["total_length_xyz"]) for r in g if not math.isnan(float(r["total_length_xyz"]))]
            line += f", mean_len_xyz={fmean(xyz):.6f}"
        print(line)
    print()

    print("per-env solved-only means:")
    for e in envs:
        print(f"  env {e}:")
        for p in planners:
            g = [r for r in rows if r["planner"] == p and r["env_seed"] == e and int(r["success"]) == 1]
            if not g:
                print(f"    {p}: no successes")
                continue
            t = fmean([float(r["total_time_s"]) for r in g])
            lse3 = fmean([float(r["total_length_se3"]) for r in g if not math.isnan(float(r["total_length_se3"]))])
            line = f"    {p}: mean_time={t:.6f}s, mean_len_se3={lse3:.6f}"
            if has_xyz:
                lxyz = fmean([float(r["total_length_xyz"]) for r in g if not math.isnan(float(r["total_length_xyz"]))])
                line += f", mean_len_xyz={lxyz:.6f}"
            print(line)
    print()

    # Winner counts by (env, run)
    speed_wins = Counter()
    se3_wins = Counter()
    xyz_wins = Counter()
    for e in envs:
        runs = sorted({r["run"] for r in rows if r["env_seed"] == e})
        for run in runs:
            g = [r for r in rows if r["env_seed"] == e and r["run"] == run and int(r["success"]) == 1]
            if not g:
                continue

            tmin = min(float(r["total_time_s"]) for r in g)
            for r in g:
                if abs(float(r["total_time_s"]) - tmin) < 1e-12:
                    speed_wins[str(r["planner"])] += 1

            se3_vals = [float(r["total_length_se3"]) for r in g if not math.isnan(float(r["total_length_se3"]))]
            if se3_vals:
                lmin = min(se3_vals)
                for r in g:
                    v = float(r["total_length_se3"])
                    if not math.isnan(v) and abs(v - lmin) < 1e-12:
                        se3_wins[str(r["planner"])] += 1

            xyz_vals = [float(r["total_length_xyz"]) for r in g if not math.isnan(float(r["total_length_xyz"]))]
            if xyz_vals:
                xmin = min(xyz_vals)
                for r in g:
                    v = float(r["total_length_xyz"])
                    if not math.isnan(v) and abs(v - xmin) < 1e-12:
                        xyz_wins[str(r["planner"])] += 1

    print("winner counts across (env, run):")
    print("  fastest total_time_s:", dict(speed_wins))
    print("  shortest total_length_se3:", dict(se3_wins))
    if has_xyz:
        print("  shortest total_length_xyz:", dict(xyz_wins))
    print()

    # Fairness consistency checks
    env_run_to_obs = {}
    env_run_to_attempt = {}
    env_run_to_seed = {}
    for r in rows:
        k = (int(r["env_seed"]), int(r["run"]))
        env_run_to_obs.setdefault(k, set()).add(int(r["obstacle_seed"]))
        env_run_to_attempt.setdefault(k, set()).add(int(r["resample_attempt"]))
        env_run_to_seed.setdefault(k, set()).add(int(r["seed"]))

    obs_nunique = sorted({len(v) for v in env_run_to_obs.values()})
    attempt_nunique = sorted({len(v) for v in env_run_to_attempt.values()})
    seed_nunique = sorted({len(v) for v in env_run_to_seed.values()})
    print("consistency checks (nunique per env/run):")
    print(f"  obstacle_seed: {obs_nunique}")
    print(f"  resample_attempt: {attempt_nunique}")
    print(f"  seed: {seed_nunique}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--summary_csv",
        default="results/tour_summary.csv",
        help="Path to tour_summary.csv",
    )
    args = ap.parse_args()

    if not os.path.exists(args.summary_csv):
        raise FileNotFoundError(args.summary_csv)
    analyze(args.summary_csv)


if __name__ == "__main__":
    main()
