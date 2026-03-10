#!/usr/bin/env python3
"""
Export benchmark CSVs into docs/data/<target>/ and create a manifest.json
for the GitHub Pages interactive viewer.
"""

import argparse
import json
import os
import re
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Dict, Tuple


PATH_RE = re.compile(r"^path_(?P<planner>.+)_env(?P<env>\d+)_run(?P<run>\d+)_seed(?P<seed>\d+)\.csv$")
BOX_RE = re.compile(r"^four_rooms_boxes_env(?P<env>\d+)\.csv$")


def build_manifest(results_dir: Path) -> Tuple[dict, Dict[Path, Path]]:
    files_to_copy: Dict[Path, Path] = {}
    boxes = {}
    paths = defaultdict(lambda: defaultdict(dict))
    planners = set()
    envs = set()

    for p in sorted(results_dir.glob("four_rooms_boxes_env*.csv")):
        m = BOX_RE.match(p.name)
        if not m:
            continue
        env = int(m.group("env"))
        envs.add(env)
        boxes[str(env)] = p.name
        files_to_copy[p] = Path(p.name)

    # Keep first lexicographic file if duplicates exist for same planner/env/run.
    seen = {}
    for p in sorted(results_dir.glob("path_*_env*_run*_seed*.csv")):
        m = PATH_RE.match(p.name)
        if not m:
            continue
        planner = m.group("planner")
        env = int(m.group("env"))
        run = int(m.group("run"))
        seed = m.group("seed")
        key = (planner, env, run)
        if key in seen:
            continue
        seen[key] = p

        planners.add(planner)
        envs.add(env)
        paths[str(env)][str(run)][planner] = {
            "seed": seed,
            "file": p.name,
        }
        files_to_copy[p] = Path(p.name)

    runs = {}
    for env in sorted(paths.keys(), key=int):
        run_ids = sorted(paths[env].keys(), key=int)
        runs[env] = [int(r) for r in run_ids]

    summary_file = results_dir / "tour_summary.csv"
    if summary_file.exists():
        files_to_copy[summary_file] = Path(summary_file.name)

    manifest = {
        "source_results_dir": str(results_dir),
        "planners": sorted(planners),
        "envs": sorted(envs),
        "boxes": boxes,
        "runs": runs,
        "paths": paths,
        "summary_csv": "tour_summary.csv" if summary_file.exists() else "",
    }
    return manifest, files_to_copy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--results_dir",
        default="ompl_cpp_bench/4roomversion/results/fixed_v2_first_solution_20260308",
        help="Path to a benchmark result folder",
    )
    ap.add_argument(
        "--docs_dir",
        default="docs",
        help="Path to docs folder used by GitHub Pages",
    )
    ap.add_argument(
        "--target",
        default="latest",
        help="Subfolder under docs/data/ (default: latest)",
    )
    args = ap.parse_args()

    results_dir = Path(args.results_dir).resolve()
    docs_dir = Path(args.docs_dir).resolve()
    target_dir = docs_dir / "data" / args.target

    if not results_dir.exists():
        raise FileNotFoundError(f"results_dir not found: {results_dir}")

    manifest, files_to_copy = build_manifest(results_dir)

    if target_dir.exists():
        shutil.rmtree(target_dir)
    target_dir.mkdir(parents=True, exist_ok=True)

    for src, rel_dst in files_to_copy.items():
        dst = target_dir / rel_dst
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    with open(target_dir / "manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(f"Export complete: {target_dir}")
    print(f"Files copied: {len(files_to_copy)}")
    print("Next: open docs/index.html locally or publish docs/ via GitHub Pages.")


if __name__ == "__main__":
    main()
