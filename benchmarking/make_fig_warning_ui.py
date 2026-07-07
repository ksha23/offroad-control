#!/usr/bin/env python3
"""Render the terrain- and latency-aware forward-collision-warning UI as the
operator actually sees it (Sec. IX, fig:warning_ui): the Chrono::Sensor driver
POV over the deformable terrain and boulder field, with the live HUD (steering
wheel + throttle/brake bar) AND the collision-warning overlay composited on top.

The warning banner + readouts (severity, time-to-collision, clearance vs.\
required stopping distance, terrain-conditioned brake budget a_b(n)) are produced
by the *real* ``CollisionWarningSystem`` --- the same code the sim node runs ---
evaluated at a RED (brake-now) moment of a straight-in approach, so the display
is faithful to the deployed component rather than a mock-up.

Background: ``my_paper/paper_figures/hil_hud_pov.png`` (real POV + HUD capture).
Output:     ``my_paper/paper_figures/warning_ui.png``
"""
from __future__ import annotations
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import FancyBboxPatch, Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
try:
    import flatpath  # noqa: F401  # adds sim subdirs to path (as the sim node does)
except Exception:
    pass
from safety.collision_warning import CollisionWarningSystem, _SEVERITY_NAMES  # noqa: E402

POV = ROOT / "my_paper" / "paper_figures" / "hil_hud_pov.png"
OUT = ROOT / "my_paper" / "paper_figures" / "warning_ui.png"

SEV_COLOR = {0: "#2ecc71", 1: "#f1c40f", 2: "#e67e22", 3: "#e74c3c"}
INK = "#f2f5f8"
DIM = "#c2ccd6"
TERRAIN_N = 0.5      # soft clay -> conservative, terrain-aware brake budget
SPEED = 6.0          # m/s
LATENCY_S = 0.15     # one-way command/camera delay


def _make_system():
    rig = ROOT / "nn_models" / "rig_rate_64_32"
    for tire_dir in (str(rig) if rig.exists() else None, None):
        try:
            return CollisionWarningSystem(tire_model_dir=tire_dir, update_interval_s=0.0)
        except Exception:
            continue
    return CollisionWarningSystem(update_interval_s=0.0)


def _red_warning(cws):
    """Drive the system down a straight-in approach and return the RED
    (brake-now) warning; fall back to the closest-in state if RED never fires."""
    cws.set_teleop_delay(LATENCY_S)
    last = None
    for d in [x * 0.5 for x in range(90, 8, -1)]:      # 45.0 m -> 4.5 m
        w = cws.evaluate({"x": 0.0, "y": 0.0, "psi": 0.0, "u": SPEED},
                         [(d, 0.0, 1.2)], terrain_n=TERRAIN_N)
        last = w
        if w.severity == 3:
            return w
    return last


def main():
    cws = _make_system()
    a_brake = cws._brake_decel_for_terrain(TERRAIN_N)
    w = _red_warning(cws)
    sev = int(w.severity)
    col = SEV_COLOR[sev]

    img = mpimg.imread(POV)
    H, W = img.shape[0], img.shape[1]

    fig, ax = plt.subplots(figsize=(W / 130, H / 130))
    ax.imshow(img, extent=[0, W, H, 0])
    ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")

    # --- top severity banner (spans the frame) ---
    bh = 0.085 * H
    ax.add_patch(Rectangle((0, 0), W, bh, fc=col, ec="none", alpha=0.95, zorder=5))
    ax.text(0.018 * W, bh / 2, "●", color="#12161c", fontsize=17, va="center", zorder=6)
    hint = {0: "CLEAR", 1: "CAUTION", 2: "SLOW DOWN", 3: "BRAKE"}[sev]
    ax.text(0.052 * W, bh / 2, f"FORWARD COLLISION WARNING — {_SEVERITY_NAMES[sev]}",
            color="#12161c", fontsize=15, fontweight="bold", va="center", zorder=6)
    ax.text(0.985 * W, bh / 2, hint, color="#12161c", fontsize=15, fontweight="bold",
            va="center", ha="right", zorder=6)

    # --- info card (top-left, opposite the HUD in the bottom-right) ---
    cw, cx0, cy0 = 0.30 * W, 0.018 * W, bh + 0.02 * H
    ch = 0.30 * H
    ax.add_patch(FancyBboxPatch((cx0, cy0), cw, ch, boxstyle="round,pad=0.4",
                                fc="#12161c", ec=col, lw=1.6, alpha=0.82, zorder=6))
    clr = w.clearance if w.clearance != float("inf") else 40.0
    dstop = w.stopping_distance
    rows = [
        ("time-to-collision", "--" if w.ttc == float("inf") else f"{w.ttc:.2f} s"),
        ("clearance", f"{clr:.1f} m"),
        ("required stop", f"{dstop:.1f} m"),
        (f"brake budget a_b(n̂={TERRAIN_N:.1f})", f"{a_brake:.1f} m/s²"),
        ("one-way latency", f"{int(LATENCY_S*1000)} ms"),
    ]
    y = cy0 + 0.055 * H
    ax.text(cx0 + 0.012 * W, cy0 + 0.028 * H, "terrain- & latency-aware FCW",
            color=col, fontsize=11, fontweight="bold", va="center", zorder=7)
    for k, v in rows:
        ax.text(cx0 + 0.012 * W, y, k, color=DIM, fontsize=10.5, va="center", zorder=7)
        ax.text(cx0 + cw - 0.012 * W, y, v, color=INK, fontsize=11.5, fontweight="bold",
                va="center", ha="right", zorder=7)
        y += 0.045 * H

    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    fig.savefig(OUT, dpi=150, bbox_inches="tight", pad_inches=0)
    print(f"[ok] wrote {OUT}  (severity={_SEVERITY_NAMES[sev]}, a_b({TERRAIN_N})={a_brake:.2f}, "
          f"tire={'rig-NN' if cws._brake_table else 'fallback'})")


if __name__ == "__main__":
    main()
