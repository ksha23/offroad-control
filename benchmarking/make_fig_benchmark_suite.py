#!/usr/bin/env python3
"""Coverage of the open-source benchmarking suite (Sec. X, fig:benchmark_suite).

Every closed-loop experiment that ``run.py --tier paper`` executes is shown with
its Chrono-trial count, coloured by the paper theme it backs. Counts are read
live from ``run.py``'s own ``--dry-run`` (the ``estimated_runs`` each command
declares), so the figure always matches the suite rather than a hand-kept list;
the annotated total is the number of trials reproduced by the single command.

Output: ``my_paper/paper_figures/benchmark_suite.png``
"""
from __future__ import annotations
import re
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "my_paper" / "paper_figures" / "benchmark_suite.png"

# sweep name -> (readable label, theme). Names not listed fall back to a
# title-cased label under "Other" so the figure never silently drops a sweep.
THEME_ORDER = ["Tire surrogate", "Terrain estimator", "Speed & control",
               "Obstacle safety", "5G latency", "Collision warning", "Integrated"]
THEME_COLOR = {
    "Tire surrogate":   "#4c72b0",
    "Terrain estimator": "#55a868",
    "Speed & control":  "#8172b3",
    "Obstacle safety":  "#c44e52",
    "5G latency":       "#dd8452",
    "Collision warning": "#937860",
    "Integrated":       "#da8bc3",
    "Other":            "#999999",
}
LABELS = {
    "tire_models": ("Static tire benchmark", "Tire surrogate"),
    "tire_model_with_estimator_ablation": ("Live-estimator tire benchmark", "Tire surrogate"),
    "rig_vs_vehicle": ("Rig-vs-vehicle retrain", "Tire surrogate"),
    "rollout_diag": ("Open-loop prediction", "Tire surrogate"),
    "terrain_estimator": ("Estimator (window-MLP)", "Terrain estimator"),
    "terrain_transition": ("Terrain transition", "Terrain estimator"),
    "estimator_lhs_manifold": ("100-soil manifold sweep", "Terrain estimator"),
    "bench_estimators_fair": ("Estimator backends (OL)", "Terrain estimator"),
    "bench_estimators_cl": ("Estimator backends (CL)", "Terrain estimator"),
    "cl_estimator_fused": ("Fused-UKF closed-loop", "Terrain estimator"),
    "cl_estimator_all": ("Estimator head-to-head", "Terrain estimator"),
    "open_loop_terrain_estimator": ("Open-loop excitation", "Terrain estimator"),
    "throttle_dob_ablation": ("Throttle DOB ablation", "Speed & control"),
    "ff_drag": ("Feedforward-drag ablation", "Speed & control"),
    "speed_profile": ("g-g speed profile", "Speed & control"),
    "safety": ("Safety filter (blind)", "Obstacle safety"),
    "safety_planner_aware": ("Two-layer safety stack", "Obstacle safety"),
    "dob_cbf_ablation": ("DOB-CBF surrogate ablation", "Obstacle safety"),
    "autonomous_obstacle_tire": ("Planner tire under filter", "Obstacle safety"),
    "convoy_cf": ("Convoy counterfactual", "Obstacle safety"),
    "latency_compensation": ("5G latency compensation", "5G latency"),
    "latency_awareness": ("Latency-awareness ablation", "5G latency"),
    "collision_warning": ("Warning severity", "Collision warning"),
    "brake_test": ("Brake-stop validation", "Collision warning"),
    "integrated_hero": ("Integrated mission", "Integrated"),
}


def _sweep_counts():
    out = subprocess.run(
        [sys.executable, str(ROOT / "benchmarking" / "run.py"), "--tier", "paper", "--dry-run"],
        capture_output=True, text=True, cwd=str(ROOT),
    ).stdout
    counts = {}
    for name, n in re.findall(r"\[([a-z_]+)\]\s+estimated_runs=(\d+)", out):
        counts[name] = counts.get(name, 0) + int(n)
    return counts


def main():
    counts = _sweep_counts()
    rows = []
    for name, n in counts.items():
        if n <= 0:
            continue
        label, theme = LABELS.get(name, (name.replace("_", " ").title(), "Other"))
        rows.append((label, n, theme))
    # sort: theme order, then descending run count within theme
    rank = {t: i for i, t in enumerate(THEME_ORDER + ["Other"])}
    rows.sort(key=lambda r: (rank.get(r[2], 99), -r[1]))
    total = sum(n for _, n, _ in rows)

    labels = [r[0] for r in rows]
    vals = [r[1] for r in rows]
    colors = [THEME_COLOR.get(r[2], "#999") for r in rows]
    ypos = list(range(len(rows)))[::-1]

    fig, ax = plt.subplots(figsize=(8.6, 0.34 * len(rows) + 1.2))
    ax.barh(ypos, vals, color=colors, edgecolor="white", height=0.72)
    for y, v in zip(ypos, vals):
        ax.text(v + total * 0.006, y, f"{v:,}", va="center", fontsize=8.5, color="#333")
    ax.set_yticks(ypos); ax.set_yticklabels(labels, fontsize=9)
    ax.set_xlabel("Chrono closed-loop trials")
    ax.set_xlim(0, max(vals) * 1.14)
    ax.margins(y=0.01)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)

    seen, handles = set(), []
    from matplotlib.patches import Patch
    for _, _, t in rows:
        if t not in seen:
            seen.add(t)
            handles.append(Patch(fc=THEME_COLOR.get(t, "#999"), label=t))
    ax.legend(handles=handles, fontsize=8, loc="lower right", frameon=False,
              title="paper theme", title_fontsize=8.5)
    ax.set_title(f"Benchmarking suite: {len(rows)} experiments, "
                 f"{total:,} Chrono trials from one command "
                 f"(\\texttt{{run.py --tier paper}})".replace("\\texttt{", "").replace("}", ""),
                 fontsize=10.5)
    fig.tight_layout()
    fig.savefig(OUT, dpi=220, bbox_inches="tight")
    print(f"[ok] wrote {OUT}  ({len(rows)} sweeps, {total:,} total trials)")


if __name__ == "__main__":
    main()
