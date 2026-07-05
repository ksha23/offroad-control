#!/usr/bin/env python3
"""Run any paper benchmark (or all of them) by flag.

This is the single entry point for the benchmarking suite.  Each --only
name corresponds to one paper section.  Without --only, all sub-sweeps
run, then ``publish_paper_figures.py`` refreshes ``my_paper/paper_figures/``.

Usage::

    python benchmarking/run.py --tier paper
    python benchmarking/run.py --tier pilot --only safety dob_cbf_ablation
    python benchmarking/run.py --tier smoke --dry-run

Tiers: smoke (~15 min, syntax check), pilot (~6 h, paper-quality pilot),
paper (full final matrix, ~12+ h), stress (high-speed/bumpy stress tests).
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_ROOT = SCRIPT_DIR / "results"


@dataclass(frozen=True)
class SuiteCommand:
    name: str
    argv: list[str]
    estimated_runs: int
    note: str
    env: dict | None = None  # extra env vars merged in for just this command


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--tier", choices=["smoke", "pilot", "paper", "stress"], default="pilot",
                   help="smoke: quick syntax checks; pilot: manageable high-speed matrix; "
                        "paper: broad final matrix; stress: high-speed/bumpy safety stress tests.")
    p.add_argument("--only", nargs="+", default=[],
                   help="Subset names: tire_models, safety, safety_planner_aware, "
                        "dob_cbf_ablation, throttle_dob_ablation, "
                        "autonomous_obstacle_tire, terrain_estimator, terrain_transition, "
                        "latency_profile, latency_compensation, "
                        "tire_model_with_estimator_ablation, rig_vs_vehicle, ff_drag, "
                        "collision_warning, brake_test, convoy_cf, bench_estimators_fair, "
                        "bench_estimators_cl, cl_estimator_fused, cl_estimator_all, "
                        "estimator_lhs_manifold, open_loop_terrain_estimator, "
                        "integrated_hero, latency_awareness, rollout_diag.")
    p.add_argument("--dry-run", action="store_true", help="Print commands without running them.")
    p.add_argument("--continue-on-error", action="store_true")
    p.add_argument("--workers", type=int, default=None,
                   help="Override the per-sub-sweep ProcessPoolExecutor worker count "
                        "(forwarded to every Chrono sub-script that accepts --workers; "
                        "latency_profile has no Chrono runs and is skipped). Default: "
                        "each sub-script's own default (6). Lower this (e.g. 4) for the "
                        "memory-heaviest sweeps if the box is memory-constrained.")
    p.add_argument("--timeout", type=float, default=None,
                   help="Override the per-run wall-clock timeout (s) forwarded to every "
                        "Chrono sub-script that accepts --timeout (latency_profile skipped). "
                        "Default: each sub-script's own (180-240 s). Raise this (e.g. 400) "
                        "to give the slowest runs head-room under worker contention.")
    p.add_argument("--base-port", type=int, default=20000,
                   help="First port block. Each experiment gets a separated block.")
    p.add_argument("--port-stride", type=int, default=2300,
                   help="Port spacing between experiment blocks. Default 2300 keeps "
                        "all 19 experiment blocks (base_port..base_port+18*stride) inside "
                        "the 65535 ceiling at base_port=20000. Sweeps run serially, so "
                        "blocks only need to exceed the per-sweep worker span (~2*workers).")
    p.add_argument("--latency-profile-json",
                   default=str(ROOT / "latency_profiles" /
                               "5g_nhits_youtube_ul_scm_youtube_ul_smoke.json"))
    p.add_argument("--train-5g", action="store_true",
                   help="Retrain/export the N-HiTS 5G traffic checkpoint before latency sweeps.")
    p.add_argument("--no-publish", action="store_true",
                   help="Skip the final publish_paper_figures step that copies canonical "
                        "figures into my_paper/paper_figures/.")
    return p.parse_args()


def tier_matrix(tier: str) -> dict[str, list[str] | int | float]:
    if tier == "smoke":
        return {
            "terrains": ["clay"], "paths": ["sinusoidal"], "speeds": ["5"],
            "bumps": ["0"], "seeds": 1, "time": 8.0,
        }
    if tier == "pilot":
        return {
            "terrains": ["clay", "dirt", "sand"],
            "paths": ["sinusoidal", "lane_change", "right_left"],
            "speeds": ["5", "7", "9"],
            "bumps": ["0", "4"],
            "seeds": 2,
            "time": 12.0,
        }
    if tier == "stress":
        return {
            "terrains": ["clay", "dirt", "sand"],
            "paths": ["sinusoidal", "lane_change", "right_left"],
            "speeds": ["7", "9"],
            "bumps": ["4", "8"],
            "seeds": 5,
            "time": 15.0,
        }
    return {
        "terrains": ["clay", "dirt", "sand"],
        "paths": ["sinusoidal", "lane_change", "right_left"],
        "speeds": ["5", "7", "9"],
        "bumps": ["0", "4", "8"],
        "seeds": 5,
        "time": 15.0,
    }


def count(*groups: list[str] | int) -> int:
    total = 1
    for group in groups:
        total *= group if isinstance(group, int) else len(group)
    return total


def base_args(m: dict[str, list[str] | int | float]) -> list[str]:
    return [
        "--terrains", *m["terrains"],
        "--paths", *m["paths"],
        "--speeds", *m["speeds"],
        "--bumpiness", *m["bumps"],
        "--seeds", str(m["seeds"]),
        "--time", str(m["time"]),
    ]


def python_cmd(script: str, *args: str) -> list[str]:
    return [sys.executable, "-u", str(SCRIPT_DIR / script), *args]


def build_commands(args: argparse.Namespace) -> list[SuiteCommand]:
    m = tier_matrix(args.tier)
    terrain_count = len(m["terrains"])
    seeds = int(m["seeds"])
    common = base_args(m)
    commands: list[SuiteCommand] = []
    ports = {name: args.base_port + i * args.port_stride for i, name in enumerate([
        "tire_models", "safety", "dob_cbf_ablation",
        "autonomous_obstacle_tire", "terrain_estimator", "latency_compensation",
        "throttle_dob_ablation", "safety_planner_aware",
        "tire_model_with_estimator_ablation", "terrain_transition",
        "rig_vs_vehicle", "ff_drag", "collision_warning", "convoy_cf",
        "open_loop_terrain_estimator",
        "integrated_hero", "latency_awareness", "rollout_diag", "speed_profile",
    ])}
    if max(ports.values()) + args.port_stride - 1 > 65535:
        raise SystemExit(
            f"Port plan exceeds 65535. Lower --base-port or --port-stride. Plan: {ports}"
        )

    if args.train_5g:
        commands.append(SuiteCommand(
            "train_5g",
            python_cmd("train_5g_nhits.py", "--skip-train", "--experiment-id", "scm_youtube_ul_smoke"),
            0,
            "Re-export trained N-HiTS 5G trace/profile; omit --skip-train manually for retraining.",
        ))

    if args.tier == "smoke":
        smoke_scripts = [
            ("tire_models", "mpc_tire_model_sweep.py"),
            ("safety", "safety_filter_sweep.py"),
            ("dob_cbf_ablation", "dob_cbf_nn_ablation.py"),
            ("autonomous_obstacle_tire", "autonomous_obstacle_tire_model_sweep.py"),
            ("terrain_estimator", "terrain_estimator_benchmark.py"),
            ("terrain_transition", "terrain_transition_benchmark.py"),
            ("throttle_dob_ablation", "throttle_dob_ablation.py"),
        ]
        for i, (name, script) in enumerate(smoke_scripts):
            commands.append(SuiteCommand(name, python_cmd(script, "--quick", "--base-port", str(ports.get(name, args.base_port + i * 1000))), 1, "Quick smoke run."))
        commands.append(SuiteCommand(
            "latency_profile",
            python_cmd("latency_profile_figure.py", "--profile-json", args.latency_profile_json),
            0,
            "Latency profile raw data/figures.",
        ))
        commands.append(SuiteCommand(
            "latency_compensation",
            python_cmd("latency_compensation_sweep.py", "--quick", "--base-port", str(ports["latency_compensation"]),
                       "--latency-profile-json", args.latency_profile_json),
            2,
            "Quick latency profile closed-loop smoke run.",
        ))
        # Wiring checks for the folded-in standalone sweeps (results-dir only,
        # so they don't clobber paper_figures/). bench_estimators, brake_test and
        # the cl_estimator comparisons write figures/CSVs directly and have no
        # quick knob, so they are exercised only at pilot/paper tier.
        commands.append(SuiteCommand(
            "rig_vs_vehicle",
            python_cmd("rig_vs_vehicle_tire_sweep.py", "--quick",
                       "--base-port", str(ports["rig_vs_vehicle"])),
            1, "Quick rig-vs-vehicle smoke run."))
        commands.append(SuiteCommand(
            "ff_drag",
            python_cmd("ff_drag_ablation.py", "--terrains", "clay", "--speeds", "5",
                       "--seeds", "1", "--base-port", str(ports["ff_drag"])),
            1, "Quick ff-drag smoke run."))
        commands.append(SuiteCommand(
            "collision_warning",
            python_cmd("collision_warning_test.py", "--terrains", "clay",
                       "--base-port", str(ports["collision_warning"])),
            1, "Quick collision-warning smoke run."))
        commands.append(SuiteCommand(
            "convoy_cf",
            python_cmd("convoy_counterfactual_eval.py", "--reckless-throttle", "0.6",
                       "--convoy", "stalled", "--filters", "none", "dob_cbf",
                       "--delays", "0.0", "--time", "10",
                       "--base-port", str(ports["convoy_cf"])),
            2, "Quick convoy-counterfactual smoke run."))
        return filter_commands(commands, args.only)

    tire_models = ["pacejka", "tmeasy", "vehicle_rate"]
    commands.append(SuiteCommand(
        "tire_models",
        python_cmd("mpc_tire_model_sweep.py", "--models", *tire_models, *common,
                   "--base-port", str(ports["tire_models"])),
        count(tire_models, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "Tracking/speed/runtime by tire model.",
    ))

    # Planner-blind safety table (tab:safety_blind) compares the DEPLOYED DOB-CBF
    # against BOTH no-filter and a textbook min-deviation CBF-QP baseline
    # (vanilla_cbf: no DOB, no NN surrogate, no reactive steer) so the gain is
    # credited to the augmentations, not just "any filter beats none".
    blind_flavors = ["none", "vanilla_cbf", "dob_cbf"]
    commands.append(SuiteCommand(
        "safety",
        python_cmd("safety_filter_sweep.py", "--flavors", *blind_flavors, *common,
                   "--base-port", str(ports["safety"])),
        count(blind_flavors, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "Obstacle safety comparison: DOB-CBF vs vanilla CBF-QP vs no-filter (planner blind).",
    ))

    # The planner-aware sweep keeps the two-endpoint none/dob_cbf comparison.
    safety_flavors = ["none", "dob_cbf"]

    # Planner-aware variant: lets the NMPC's in-horizon softplus barriers do
    # their share. Comparing this against the planner-blind safety sweep above
    # is the abstract's "two-layer obstacle-avoidance stack" evidence.
    commands.append(SuiteCommand(
        "safety_planner_aware",
        python_cmd("safety_filter_sweep.py", "--flavors", *safety_flavors,
                   "--blind-and-aware", "--output-suffix", "planner_aware",
                   *common, "--base-port", str(ports["safety_planner_aware"])),
        count(safety_flavors, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds) * 2,
        "NMPC in-horizon barrier ablation: same shields, planner-aware vs planner-blind.",
    ))

    dob_variants = ["no_filter", "dob_cbf_nn", "dob_cbf_no_nn"]
    commands.append(SuiteCommand(
        "dob_cbf_ablation",
        python_cmd("dob_cbf_nn_ablation.py", "--variants", *dob_variants, *common,
                   "--base-port", str(ports["dob_cbf_ablation"])),
        count(dob_variants, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "DOB-CBF NN usage ablation.",
    ))

    commands.append(SuiteCommand(
        "autonomous_obstacle_tire",
        python_cmd("autonomous_obstacle_tire_model_sweep.py", "--models", *tire_models,
                   "--safety-flavor", "dob_cbf", "--mpc-blind-obstacles", *common,
                   "--base-port", str(ports["autonomous_obstacle_tire"])),
        count(tire_models, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "Autonomous obstacle avoidance by MPC tire model under fixed DOB-CBF shield.",
    ))

    terrain_speeds = ["5", "7"] if args.tier != "stress" else ["7", "9"]
    terrain_paths = ["sinusoidal"]
    terrain_cases = terrain_count + 6
    # The deployed learned (window-MLP) terrain estimator is trained/validated
    # only over bumpiness {0,4} (see its TRAINING_METADATA). Its vertical-
    # dynamics features go out-of-distribution at bumpiness 8 and alias bump-
    # induced vertical motion as firm-soil stiffness, so we evaluate it within
    # its training envelope rather than reporting an OOD-bumpiness failure as if
    # it were in-distribution. (The offline NN-UKF validation harness in
    # benchmarking/lib/ukf_paper_validation.py has no such limit.)
    terrain_bumps = [b for b in m["bumps"] if int(b) <= 4]
    commands.append(SuiteCommand(
        "terrain_estimator",
        python_cmd("terrain_estimator_benchmark.py", "--distributions", "id", "ood",
                   "--terrains", *m["terrains"], "--paths", *terrain_paths,
                   "--speeds", *terrain_speeds, "--bumpiness", *terrain_bumps,
                   "--seeds", str(seeds), "--ood-terrains", "6", "--time", "20",
                   "--metric-start", "8",
                   # tab:estimator_pilot / Fig 6 report the window-MLP proprioceptive
                   # channel specifically (the deployed Fused-UKF is Sec. VI), so pin
                   # the learned n-only backend rather than the script's default.
                   "--estimator-backend", "learned", "--estimator-mode", "n",
                   "--base-port", str(ports["terrain_estimator"])),
        count(terrain_cases, terrain_paths, terrain_speeds, terrain_bumps, seeds),
        "Terrain estimator under excited sinusoidal maneuvers (bumpiness within "
        "the learned estimator's {0,4} training envelope).",
    ))

    # Spatial soil transition: the plant soil changes type partway across the
    # patch (per-location SCM callback); measures how fast the online estimator
    # tracks the new n and how tracking holds while it catches up. Flat soil
    # only (bumpiness 0) so the response is the soil step, not bump aliasing.
    transition_pairs = ["clay_to_sand", "sand_to_clay", "clay_to_dirt",
                        "dirt_to_clay", "dirt_to_sand", "sand_to_dirt"]
    commands.append(SuiteCommand(
        "terrain_transition",
        python_cmd("terrain_transition_benchmark.py", "--transitions", *transition_pairs,
                   "--paths", *terrain_paths, "--speeds", "5", "--bumpiness", "0",
                   "--seeds", str(seeds), "--time", "24", "--transition-x", "45",
                   "--metric-start", "8", "--base-port", str(ports["terrain_transition"])),
        count(transition_pairs, terrain_paths, ["5"], ["0"], seeds),
        "Online terrain estimator tracking a mid-run spatial soil transition.",
    ))

    commands.append(SuiteCommand(
        "latency_profile",
        python_cmd("latency_profile_figure.py", "--profile-json", args.latency_profile_json),
        0,
        "Latency profile raw samples and figures.",
    ))

    latency_filters = ["none", "dob_cbf"]
    commands.append(SuiteCommand(
        "latency_compensation",
        python_cmd("latency_compensation_sweep.py", "--filters", *latency_filters,
                   "--latency-profile-json", args.latency_profile_json, *common,
                   "--base-port", str(ports["latency_compensation"])),
        count(latency_filters, m["terrains"], m["paths"], m["speeds"], m["bumps"], seeds),
        "5G-profile command/camera latency robustness.",
    ))

    # Throttle-DOB ablation: same standard MPC, NN tire model, no obstacles;
    # toggles --dob-ki/--dob-max to zero so we can measure how much of the
    # speed-tracking story the asymmetric DOB actually owns.
    dob_variants_ablation = ["dob_on", "dob_off"]
    commands.append(SuiteCommand(
        "throttle_dob_ablation",
        python_cmd("throttle_dob_ablation.py", "--variants", *dob_variants_ablation,
                   *common, "--base-port", str(ports["throttle_dob_ablation"])),
        count(dob_variants_ablation, m["terrains"], m["paths"], m["speeds"],
              m["bumps"], seeds),
        "Asymmetric throttle DOB on vs off.",
    ))


    # Tire model x live terrain estimator: tests the abstract's
    # "order-of-magnitude over Pacejka and TMeasy" claim, which the static
    # tire-model sweep cannot speak to.  Overrides ``common``'s --time and
    # passes --metric-start=8 so the KPI window starts after the estimator
    # has had time to settle (this is also why the figures differ from the
    # static sweep: same vehicle, different observation window).
    # nn_wrong_prior (controller prior locked to dirt) is folded in here so the
    # single run feeds BOTH tab:tires_estimator (pacejka/tmeasy/nn_static/
    # nn_estimator rows) and tab:wrong_prior (nn_static/nn_wrong_prior/
    # nn_estimator, filtered to clay+sand x sinusoidal x v in {5,7} x flat).
    estimator_variants = ["pacejka_static", "tmeasy_static", "nn_static",
                          "nn_estimator", "nn_wrong_prior"]
    commands.append(SuiteCommand(
        "tire_model_with_estimator_ablation",
        python_cmd("tire_model_with_estimator_ablation.py",
                   "--variants", *estimator_variants, *common,
                   "--time", "20.0", "--metric-start", "8.0",
                   "--base-port", str(ports["tire_model_with_estimator_ablation"])),
        count(estimator_variants, m["terrains"], m["paths"], m["speeds"],
              m["bumps"], seeds),
        "Tire model with live terrain estimator on vs static params.",
    ))

    # ---- Standalone paper sweeps folded in so ``run.py --tier paper``
    #      reproduces EVERY table and figure from one command. Each uses its
    #      own script-appropriate matrix (not the shared grid). ----

    # Controlled rig-vs-whole-vehicle surrogate sweep (script default 6-surrogate
    # x 72-cell matrix; ~432 runs). tab:tires_rig_vs_vehicle.
    commands.append(SuiteCommand(
        "rig_vs_vehicle",
        python_cmd("rig_vs_vehicle_tire_sweep.py",
                   "--base-port", str(ports["rig_vs_vehicle"])),
        432,
        "Controlled rig-vs-whole-vehicle tire-surrogate sweep (tab:tires_rig_vs_vehicle).",
    ))

    # Feedforward sinkage-drag vs reactive DOB (sinusoidal, v in {5,7}). tab:ffdrag.
    commands.append(SuiteCommand(
        "ff_drag",
        python_cmd("ff_drag_ablation.py", "--terrains", *m["terrains"],
                   "--speeds", "5", "7", "--seeds", str(seeds),
                   "--base-port", str(ports["ff_drag"])),
        count(4, m["terrains"], ["5", "7"], seeds),
        "Feedforward sinkage-drag vs reactive DOB (tab:ffdrag).",
    ))

    # Forward collision-warning lead time, terrain x latency, no controller. tab:cw_lead.
    commands.append(SuiteCommand(
        "collision_warning",
        python_cmd("collision_warning_test.py", "--terrains", *m["terrains"],
                   "--base-port", str(ports["collision_warning"])),
        count(m["terrains"], 3),
        "Forward collision-warning lead-time sweep (tab:cw_lead).",
    ))

    # FCW brake-stop distance validation (no CLI matrix; 3 terrains x 3 speeds x 3 seeds).
    commands.append(SuiteCommand(
        "brake_test",
        python_cmd("brake_test.py"),
        27,
        "Forward-collision-warning brake-stop validation (fig:cw_brake_validation).",
    ))

    # Counterfactual convoy replay -- headline safety attribution. 3 reckless
    # throttles x 3 convoy scenarios x 5 command delays = 45 cells/filter. tab:convoy_cf.
    commands.append(SuiteCommand(
        "convoy_cf",
        python_cmd("convoy_counterfactual_eval.py",
                   "--reckless-throttle", "0.4", "0.6", "0.8",
                   "--convoy", "lead_brake", "cut_in", "stalled",
                   "--filters", "none", "dob_cbf",
                   "--delays", "0.0", "0.1", "0.2", "0.3", "0.4",
                   "--base-port", str(ports["convoy_cf"])),
        90,
        "Counterfactual convoy replay: causal collision-prevention (tab:convoy_cf).",
    ))

    # 100-soil terrain-estimator head-to-head (open-loop 'fair' + closed-loop). tab:estimator_lhs100.
    commands.append(SuiteCommand(
        "bench_estimators_fair",
        python_cmd("bench_terrain_estimators_lhs.py", "--n", "100",
                   "--n-min", "0.40", "--n-max", "1.30", "--steer-amp-rad", "0.6",
                   "--open-loop-throttle", "0.75", "--out-name", "lhs100_fair"),
        100,
        "100-soil terrain-estimator open-loop benchmark (tab:estimator_lhs100).",
    ))
    commands.append(SuiteCommand(
        "bench_estimators_cl",
        python_cmd("bench_terrain_estimators_lhs.py", "--n", "100",
                   "--n-min", "0.40", "--n-max", "1.30", "--steer-amp-rad", "0.6",
                   "--open-loop-throttle", "-1", "--target-speed", "5.0",
                   "--log-suffix", "_cl", "--out-name", "lhs100_cl"),
        100,
        "100-soil terrain-estimator closed-loop benchmark (tab:estimator_lhs100).",
    ))

    # Deployed Fused-UKF closed-loop backend (+ all-backend comparison): the
    # tab:estimator_pilot row and the backends / ukf_observability figures.
    commands.append(SuiteCommand(
        "cl_estimator_fused",
        python_cmd("closed_loop_estimator_compare_fused.py"),
        54,
        "Deployed Fused-UKF closed-loop estimator backend (tab:estimator_pilot).",
    ))
    commands.append(SuiteCommand(
        "cl_estimator_all",
        python_cmd("closed_loop_estimator_compare_all.py"),
        54,
        "All-backend closed-loop estimator comparison (backends + observability figs).",
    ))

    # 100-soil MANIFOLD terrain-estimator head-to-head, 4 backends incl the
    # deployed Fused-UKF -- this (not the full-LHS bench_estimators above) is the
    # source for tab:estimator_lhs100. Manifold = n swept along the clay-dirt-sand
    # preset manifold with the other 5 soil params known (+-jitter); full-LHS
    # jitter of all 6 params makes every backend read as a flat band.
    commands.append(SuiteCommand(
        "estimator_lhs_manifold",
        python_cmd("closed_loop_estimator_lhs.py", "--n", "100", "--mode", "manifold"),
        400,
        "100-soil manifold terrain-estimator head-to-head, 4 backends incl Fused-UKF (tab:estimator_lhs100).",
    ))

    # Open-loop window-MLP terrain-estimator scatter (fig:estimator_scatter, the
    # proprioceptive-channel n-tracking on scripted open-loop excitation).
    commands.append(SuiteCommand(
        "open_loop_terrain_estimator",
        python_cmd("open_loop_terrain_estimator_benchmark.py",
                   "--distributions", "id", "ood", "--ood-terrains", "6",
                   "--seeds", str(seeds), "--estimator-backend", "learned",
                   "--base-port", str(ports["open_loop_terrain_estimator"])),
        count(terrain_count + 6, seeds),
        "Open-loop window-MLP terrain-estimator scatter (fig:estimator_scatter).",
    ))

    # Integrated end-to-end "hero" mission: the whole stack in one run
    # (fig:integrated_hero). Single mission -> no --workers/--timeout; writes
    # integrated_hero_run.png into its own result dir (copied to paper_figures
    # by make_paper_figures).
    commands.append(SuiteCommand(
        "integrated_hero",
        python_cmd("integrated_hero_run.py",
                   "--base-port", str(ports["integrated_hero"])),
        1,
        "Integrated end-to-end hero mission (fig:integrated_hero).",
    ))

    # Latency-awareness dose-response: DOB-CBF told vs not told about the command
    # delay (fig:latency_awareness). Self-contained counterfactual sweep; takes
    # --workers/--timeout. Writes latency_awareness_ablation.png into its result
    # dir (copied to paper_figures by make_paper_figures).
    commands.append(SuiteCommand(
        "latency_awareness",
        python_cmd("latency_awareness_ablation.py",
                   "--base-port", str(ports["latency_awareness"])),
        5 * 2 * 3 * 3,  # convoy x delay x reckless-throttle x {none,blind,aware}
        "Latency-awareness blind-vs-aware DOB-CBF dose-response (fig:latency_awareness).",
    ))

    # Rollout-prediction diagnostic: a tiny tire-model sweep run with
    # LOG_MPC_PREDICTIONS=1 so each solve dumps mpc_predictions.npz, which
    # rollout_prediction_validation.py (a make_paper_figures step) overlays
    # against the plant (fig:rollout_prediction_validation). sand+clay at v7 is
    # exactly what that plotter features (--terrain sand --profile v7).
    commands.append(SuiteCommand(
        "rollout_diag",
        python_cmd("mpc_tire_model_sweep.py", "--models", "vehicle_rate",
                   "--terrains", "sand", "clay", "--paths", "sinusoidal",
                   "--speeds", "7", "--bumpiness", "0", "--seeds", "1",
                   "--output-prefix", "mpc_tire_model_sweep_rollout_diag",
                   "--base-port", str(ports["rollout_diag"])),
        2,
        "MPC prediction-logging diagnostic for fig:rollout_prediction_validation.",
        env={"LOG_MPC_PREDICTIONS": "1"},
    ))

    # Static-curvature vs terrain-aware g--g speed-profile ablation
    # (speed_profile_gg figure). Uses the paper's double_lane_change + sinusoidal
    # path set; emits the paired schema make_fig_speed_profile.py reads.
    # publish_paper_figures republishes the paired CSV; make_paper_figures redraws.
    commands.append(SuiteCommand(
        "speed_profile",
        python_cmd("speed_profile_ablation.py",
                   "--paths", "double_lane_change", "sinusoidal",
                   "--base-port", str(ports["speed_profile"])),
        2 * 2 * 3 * 2 * 2,  # {static,terrain} x 2 paths x 3 terrains x 2 speeds x 2 seeds
        "Terrain-aware g--g speed-profile ablation (speed_profile_gg figure).",
    ))

    return filter_commands(commands, args.only)


def filter_commands(commands: list[SuiteCommand], only: list[str]) -> list[SuiteCommand]:
    if not only:
        return commands
    wanted = set(only)
    unknown = wanted - {cmd.name for cmd in commands}
    if unknown:
        raise SystemExit(f"Unknown --only names: {', '.join(sorted(unknown))}")
    return [cmd for cmd in commands if cmd.name in wanted]


def write_suite_manifest(suite_dir: Path, args: argparse.Namespace, commands: list[SuiteCommand]) -> None:
    with (suite_dir / "suite_manifest.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["name", "estimated_runs", "note", "command"])
        for cmd in commands:
            writer.writerow([cmd.name, cmd.estimated_runs, cmd.note, " ".join(cmd.argv)])
    with (suite_dir / "suite_args.txt").open("w") as f:
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v!r}\n")


def main() -> None:
    args = parse_args()
    os.environ.setdefault("ACADOS_SOURCE_DIR", "/home/ksha/Documents/sbel/acados")
    commands = build_commands(args)
    # Folded-in sweeps that do NOT accept --workers / --timeout (brake_test and
    # the closed-loop estimator comparisons take neither; collision_warning and
    # the LHS estimator benches take --workers but not --timeout).
    no_workers = {"latency_profile", "train_5g", "brake_test",
                  "cl_estimator_fused", "cl_estimator_all", "integrated_hero"}
    no_timeout = no_workers | {"collision_warning",
                               "bench_estimators_fair", "bench_estimators_cl",
                               "estimator_lhs_manifold"}
    for cmd in commands:
        if args.workers is not None and cmd.name not in no_workers:
            cmd.argv.extend(["--workers", str(args.workers)])
        if args.timeout is not None and cmd.name not in no_timeout:
            cmd.argv.extend(["--timeout", str(args.timeout)])
    suite_dir = RESULTS_ROOT / f"paper_suite_{args.tier}_{datetime.now():%Y%m%d_%H%M%S}"
    suite_dir.mkdir(parents=True, exist_ok=False)
    write_suite_manifest(suite_dir, args, commands)

    total_runs = sum(cmd.estimated_runs for cmd in commands)
    print(f"Suite: {args.tier}  commands={len(commands)}  estimated Chrono runs={total_runs}")
    print(f"Manifest: {suite_dir / 'suite_manifest.csv'}")
    for cmd in commands:
        print(f"\n[{cmd.name}] estimated_runs={cmd.estimated_runs}")
        print(" ".join(cmd.argv))
        if args.dry_run:
            continue
        cmd_env = os.environ.copy()
        if cmd.env:
            cmd_env.update(cmd.env)
        rc = subprocess.run(cmd.argv, cwd=str(ROOT), env=cmd_env).returncode
        if rc != 0 and not args.continue_on_error:
            raise SystemExit(rc)

    if args.dry_run or args.no_publish:
        return
    # First publish the suite's canonical KPI CSVs (scoped to THIS suite dir),
    # then regenerate every paper figure from the fresh results. make_paper_figures
    # only plots/publishes (no re-simulation), so together they refresh every
    # table CSV and figure in my_paper/paper_figures/ from this one run.
    publish_cmd = [sys.executable, "-u", str(SCRIPT_DIR / "publish_paper_figures.py"),
                   "--suite-dir", str(suite_dir)]
    print("\n[publish_paper_figures]")
    print(" ".join(publish_cmd))
    subprocess.run(publish_cmd, cwd=str(ROOT), env=os.environ.copy())
    figs_cmd = [sys.executable, "-u", str(SCRIPT_DIR / "make_paper_figures.py")]
    print("\n[make_paper_figures] regenerating every table CSV + figure from this suite")
    print(" ".join(figs_cmd))
    subprocess.run(figs_cmd, cwd=str(ROOT), env=os.environ.copy())


if __name__ == "__main__":
    main()
