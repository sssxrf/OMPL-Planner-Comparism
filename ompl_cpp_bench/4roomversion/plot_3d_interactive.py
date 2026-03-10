#!/usr/bin/env python3
"""
Interactive 3D trajectory viewer for OMPL 4-room benchmarks.

Features:
- Rotate/zoom/pan in 3D with the mouse (matplotlib 3D interaction).
- Switch environment and run via on-figure buttons.
- Toggle planner visibility via checkboxes.

Expected files in --outdir:
- four_rooms_boxes_env<env>.csv
- path_<PLANNER>_env<env>_run<run>_seed<seed>.csv
"""

import argparse
import csv
import glob
import os
import re
from collections import defaultdict
from typing import Dict, List, Tuple

DEFAULT_PLANNERS = ["RRTConnect", "PRMstar", "RRTstar", "BITstar"]
PATH_RE = re.compile(r"^path_(?P<planner>.+)_env(?P<env>\d+)_run(?P<run>\d+)_seed(?P<seed>\d+)\.csv$")


def load_boxes_csv(path: str) -> List[Dict[str, float]]:
    with open(path, newline="") as f:
        rows = []
        reader = csv.DictReader(f)
        required = {"xmin", "xmax", "ymin", "ymax", "zmin", "zmax"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} missing columns: {sorted(missing)}")
        for row in reader:
            rows.append({k: float(row[k]) for k in required})
        return rows


def load_path_csv(path: str) -> Tuple[List[float], List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y", "z"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} missing columns: {sorted(missing)}")
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            zs.append(float(row["z"]))
    return xs, ys, zs


def draw_box_wireframe(ax, b: Dict[str, float], lw: float = 0.5) -> None:
    xmin, xmax = b["xmin"], b["xmax"]
    ymin, ymax = b["ymin"], b["ymax"]
    zmin, zmax = b["zmin"], b["zmax"]
    corners = [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    for i, j in edges:
        ax.plot(
            [corners[i][0], corners[j][0]],
            [corners[i][1], corners[j][1]],
            [corners[i][2], corners[j][2]],
            linewidth=lw,
            color="0.3",
            alpha=0.8,
        )


def set_equal_aspect_3d(ax, xlim: Tuple[float, float], ylim: Tuple[float, float], zlim: Tuple[float, float]) -> None:
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)


def index_results(outdir: str, planners_filter: List[str]) -> Tuple[
    Dict[int, str], Dict[int, List[int]], Dict[int, Dict[int, Dict[str, Tuple[str, str]]]], List[str]
]:
    boxes_by_env: Dict[int, str] = {}
    for p in glob.glob(os.path.join(outdir, "four_rooms_boxes_env*.csv")):
        m = re.search(r"four_rooms_boxes_env(\d+)\.csv$", os.path.basename(p))
        if m:
            boxes_by_env[int(m.group(1))] = p

    indexed: Dict[int, Dict[int, Dict[str, Tuple[str, str]]]] = defaultdict(lambda: defaultdict(dict))
    planners_seen = set()

    for p in glob.glob(os.path.join(outdir, "path_*_env*_run*_seed*.csv")):
        m = PATH_RE.match(os.path.basename(p))
        if not m:
            continue
        planner = m.group("planner")
        env = int(m.group("env"))
        run = int(m.group("run"))
        seed = m.group("seed")

        if planners_filter and planner not in planners_filter:
            continue

        planners_seen.add(planner)

        # Keep lexicographically first if multiple files exist for same planner/env/run.
        prev = indexed[env][run].get(planner)
        if prev is None or p < prev[1]:
            indexed[env][run][planner] = (seed, p)

    env_to_runs: Dict[int, List[int]] = {}
    for env, run_map in indexed.items():
        env_to_runs[env] = sorted(run_map.keys())

    planners = [p for p in DEFAULT_PLANNERS if p in planners_seen]
    for p in sorted(planners_seen):
        if p not in planners:
            planners.append(p)

    return boxes_by_env, env_to_runs, indexed, planners


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default="results", help="Folder containing benchmark CSV outputs")
    ap.add_argument("--env", type=int, default=None, help="Initial env id")
    ap.add_argument("--run", type=int, default=None, help="Initial run id")
    ap.add_argument("--planners", nargs="*", default=DEFAULT_PLANNERS, help="Planner subset")
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, CheckButtons
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "This script requires matplotlib. Install it in your active environment "
            "(e.g., pip install matplotlib)."
        ) from exc

    boxes_by_env, env_to_runs, indexed, planners = index_results(args.outdir, args.planners)
    envs = sorted(env_to_runs.keys())

    if not envs:
        raise FileNotFoundError(
            f"No path files found in {args.outdir}. Expected: path_<PLANNER>_env<env>_run<run>_seed<seed>.csv"
        )

    if args.env is not None and args.env in envs:
        env_idx = envs.index(args.env)
    else:
        env_idx = 0

    current_env = envs[env_idx]
    runs = env_to_runs[current_env]
    if args.run is not None and args.run in runs:
        run_idx = runs.index(args.run)
    else:
        run_idx = 0

    fig = plt.figure(figsize=(12.0, 8.0))
    ax = fig.add_axes([0.05, 0.08, 0.68, 0.86], projection="3d")
    check_ax = fig.add_axes([0.76, 0.45, 0.22, 0.45])
    check_ax.set_title("Planners", fontsize=10)
    checks = CheckButtons(check_ax, planners, [True] * len(planners))

    btn_prev_env_ax = fig.add_axes([0.76, 0.34, 0.10, 0.06])
    btn_next_env_ax = fig.add_axes([0.88, 0.34, 0.10, 0.06])
    btn_prev_run_ax = fig.add_axes([0.76, 0.26, 0.10, 0.06])
    btn_next_run_ax = fig.add_axes([0.88, 0.26, 0.10, 0.06])
    btn_prev_env = Button(btn_prev_env_ax, "Prev Env")
    btn_next_env = Button(btn_next_env_ax, "Next Env")
    btn_prev_run = Button(btn_prev_run_ax, "Prev Run")
    btn_next_run = Button(btn_next_run_ax, "Next Run")

    help_ax = fig.add_axes([0.76, 0.06, 0.22, 0.16])
    help_ax.axis("off")
    help_ax.text(
        0.0,
        1.0,
        "Rotate: left-drag\nZoom: scroll\nPan: right-drag\n\nButtons switch env/run.",
        va="top",
        fontsize=9,
    )

    boxes_cache: Dict[int, List[Dict[str, float]]] = {}
    path_cache: Dict[str, Tuple[List[float], List[float], List[float]]] = {}
    state = {"env_idx": env_idx, "run_idx": run_idx}

    def get_current_env_run() -> Tuple[int, int]:
        env = envs[state["env_idx"]]
        runs_local = env_to_runs[env]
        if state["run_idx"] >= len(runs_local):
            state["run_idx"] = len(runs_local) - 1
        run = runs_local[state["run_idx"]]
        return env, run

    def redraw() -> None:
        env, run = get_current_env_run()
        ax.cla()

        # Draw boxes (if available for this env).
        if env in boxes_by_env:
            if env not in boxes_cache:
                boxes_cache[env] = load_boxes_csv(boxes_by_env[env])
            for b in boxes_cache[env]:
                draw_box_wireframe(ax, b, lw=0.5)

        selected = {label.get_text() for label, status in zip(checks.labels, checks.get_status()) if status}
        run_map = indexed.get(env, {}).get(run, {})

        seed_parts = []
        plotted_any = False
        for planner in planners:
            if planner not in selected:
                continue
            if planner not in run_map:
                continue
            seed, path_file = run_map[planner]
            if path_file not in path_cache:
                path_cache[path_file] = load_path_csv(path_file)
            xs, ys, zs = path_cache[path_file]
            ax.plot(xs, ys, zs, linewidth=1.8, label=planner)
            seed_parts.append(f"{planner}:seed{seed}")
            plotted_any = True

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        set_equal_aspect_3d(ax, (-5.0, 5.0), (-5.0, 5.0), (-1.5, 1.5))
        if plotted_any:
            ax.legend(loc="upper left")

        seed_info = ", ".join(seed_parts) if seed_parts else "no selected planner paths for this env/run"
        fig.suptitle(f"OMPL 3D Trajectories - env {env}, run {run}\n{seed_info}", fontsize=10)
        fig.canvas.draw_idle()

    def on_prev_env(_event) -> None:
        state["env_idx"] = (state["env_idx"] - 1) % len(envs)
        state["run_idx"] = 0
        redraw()

    def on_next_env(_event) -> None:
        state["env_idx"] = (state["env_idx"] + 1) % len(envs)
        state["run_idx"] = 0
        redraw()

    def on_prev_run(_event) -> None:
        env, _ = get_current_env_run()
        runs_local = env_to_runs[env]
        state["run_idx"] = (state["run_idx"] - 1) % len(runs_local)
        redraw()

    def on_next_run(_event) -> None:
        env, _ = get_current_env_run()
        runs_local = env_to_runs[env]
        state["run_idx"] = (state["run_idx"] + 1) % len(runs_local)
        redraw()

    btn_prev_env.on_clicked(on_prev_env)
    btn_next_env.on_clicked(on_next_env)
    btn_prev_run.on_clicked(on_prev_run)
    btn_next_run.on_clicked(on_next_run)
    checks.on_clicked(lambda _label: redraw())

    redraw()
    plt.show()


if __name__ == "__main__":
    main()
