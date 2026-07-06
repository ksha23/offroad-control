#!/usr/bin/env python3
"""Render the terrain- and latency-aware forward-collision-warning UI as the
operator sees it (Sec. IX, fig:warning_ui).

The three panels are the operator's heads-up warning overlay at three moments
of a straight-in approach to a hazard, escalating GREEN -> ORANGE -> RED. Every
number shown is produced by the *real* ``CollisionWarningSystem`` (same code the
sim node runs) evaluated on a scripted approach, so the display is faithful to
the deployed component rather than a mock-up: severity, time-to-collision,
required stopping distance vs. clearance, and the terrain-conditioned brake
budget a_b(n) all come from ``evaluate()``.

Output: ``my_paper/paper_figures/warning_ui.png``
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
try:
    import flatpath  # noqa: F401  # adds sim subdirs to path (as the sim node does)
except Exception:
    pass
from safety.collision_warning import CollisionWarningSystem, _SEVERITY_NAMES  # noqa: E402

OUT = ROOT / "my_paper" / "paper_figures" / "warning_ui.png"

# Severity palette (matches the {GREEN,YELLOW,ORANGE,RED} HMI banner).
SEV_COLOR = {0: "#2ecc71", 1: "#f1c40f", 2: "#e67e22", 3: "#e74c3c"}
PANEL_BG = "#12161c"
EDGE = "#2a3340"
INK = "#e6e9ee"
DIM = "#9aa6b2"

TERRAIN_N = 0.5           # soft clay -> conservative, terrain-aware budget
SPEED = 6.0               # m/s
LATENCY_S = 0.15          # one-way command/camera delay


def _make_system():
    """Real warning system, preferring the rig surrogate brake budget of the
    paper; falls back to the linear-interp ceiling if the checkpoint or its
    dependencies are unavailable."""
    rig = ROOT / "nn_models" / "rig_rate_64_32"
    for tire_dir in (str(rig) if rig.exists() else None, None):
        try:
            return CollisionWarningSystem(tire_model_dir=tire_dir, update_interval_s=0.0)
        except Exception:
            continue
    return CollisionWarningSystem(update_interval_s=0.0)


def _real_warnings(cws):
    """Drive the system down a straight-in approach; return one representative
    CollisionWarning for each of GREEN / ORANGE / RED."""
    cws.set_teleop_delay(LATENCY_S)
    picked: dict[int, object] = {}
    for d in [x * 0.5 for x in range(90, 8, -1)]:        # 45.0 m -> 4.5 m
        w = cws.evaluate({"x": 0.0, "y": 0.0, "psi": 0.0, "u": SPEED},
                         [(d, 0.0, 1.2)], terrain_n=TERRAIN_N)
        picked.setdefault(w.severity, w)
    order = [s for s in (0, 2, 3, 1) if s in picked]
    return [picked[s] for s in order[:3]]


def _panel(ax, w, a_brake):
    sev = int(w.severity)
    col = SEV_COLOR[sev]
    ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.axis("off")
    ax.add_patch(FancyBboxPatch((0.2, 0.2), 9.6, 9.6, boxstyle="round,pad=0.15",
                                fc=PANEL_BG, ec=EDGE, lw=1.5))

    # --- severity banner ---
    ax.add_patch(FancyBboxPatch((0.7, 7.7), 8.6, 1.5, boxstyle="round,pad=0.1",
                                fc=col, ec="none"))
    ax.text(1.15, 8.45, "●", color="#12161c", fontsize=15, va="center")
    ax.text(1.9, 8.45, _SEVERITY_NAMES[sev], color="#12161c", fontsize=17,
            fontweight="bold", va="center")
    hint = {0: "CLEAR", 1: "CAUTION", 2: "SLOW", 3: "BRAKE"}[sev]
    ax.text(9.0, 8.45, hint, color="#12161c", fontsize=12, fontweight="bold",
            va="center", ha="right")

    # --- 4-step severity ladder (current highlighted) ---
    for i in range(4):
        x0 = 0.7 + i * 2.18
        on = (i == sev)
        ax.add_patch(Rectangle((x0, 6.95), 2.0, 0.42,
                               fc=SEV_COLOR[i] if on else "#20272f",
                               ec=SEV_COLOR[i], lw=1.4 if on else 0.8,
                               alpha=1.0 if on else 0.55))

    # --- clearance vs. required stopping distance ---
    clr = w.clearance if w.clearance != float("inf") else 40.0
    dstop = w.stopping_distance
    scale = max(clr, dstop, 1.0) * 1.15
    bx, bw, by = 0.9, 8.2, 5.6
    ax.text(0.9, 6.35, "clearance vs. required stop", color=DIM, fontsize=9.5, va="center")
    ax.add_patch(Rectangle((bx, by), bw, 0.5, fc="#20272f", ec=EDGE, lw=0.8))
    ax.add_patch(Rectangle((bx, by), bw * min(dstop / scale, 1.0), 0.5,
                           fc="#7f2b25", ec="none"))
    cx = bx + bw * min(clr / scale, 1.0)
    ax.plot([cx, cx], [by - 0.18, by + 0.68], color=col, lw=3)
    ax.text(cx, by + 0.95, f"clr {clr:4.1f} m", color=col, fontsize=9, ha="center")
    dx = bx + bw * min(dstop / scale, 1.0)
    ax.text(dx, by - 0.55, f"stop {dstop:4.1f} m", color="#d98880", fontsize=9, ha="center")

    # --- numeric readouts ---
    rows = [
        ("speed", f"{SPEED:.1f} m/s"),
        ("time-to-collision", "--" if w.ttc == float("inf") else f"{w.ttc:.2f} s"),
        (f"brake budget  a_b(n̂={TERRAIN_N:.1f})", f"{a_brake:.1f} m/s²"),
        ("one-way latency", f"{int(LATENCY_S*1000)} ms"),
        ("margin", "--" if w.margin == float("inf") else f"{w.margin:+.1f} m"),
    ]
    y = 4.6
    for k, v in rows:
        ax.text(0.95, y, k, color=DIM, fontsize=9.5, va="center")
        ax.text(9.05, y, v, color=INK, fontsize=10.5, fontweight="bold",
                va="center", ha="right")
        y -= 0.82


def main():
    cws = _make_system()
    a_brake = cws._brake_decel_for_terrain(TERRAIN_N)
    warns = _real_warnings(cws)
    if len(warns) < 3:
        warns = (warns + warns[-1:] * 3)[:3]

    fig, axes = plt.subplots(1, 3, figsize=(10.2, 4.0))
    fig.patch.set_facecolor("white")
    titles = ["approaching (clear)", "closing", "brake now"]
    for ax, w, t in zip(axes, warns, titles):
        _panel(ax, w, a_brake)
        ax.set_title(t, fontsize=10, color="#333")
    fig.suptitle("Operator forward-collision-warning overlay "
                 "(soft clay, 150 ms link) — states from the deployed warning system",
                 fontsize=11, y=1.02)
    fig.tight_layout()
    fig.savefig(OUT, dpi=220, bbox_inches="tight", facecolor="white")
    print(f"[ok] wrote {OUT}  (brake budget a_b({TERRAIN_N})={a_brake:.2f} m/s^2, "
          f"tire_model={'rig-NN' if cws._brake_table else 'linear-interp fallback'})")


if __name__ == "__main__":
    main()
