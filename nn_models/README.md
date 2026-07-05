# Active Neural Checkpoints

`nn_models/` is the canonical runtime model directory. It intentionally
keeps only the checkpoints currently referenced by either the deployed
runtime or a live paper claim.

| Directory | Role |
| --- | --- |
| `terrain_window_mlp/` | **Deployed** online terrain estimator (v7 weights — broad-coverage 3600-trace training). See [`TRAINING_METADATA.md`](terrain_window_mlp/TRAINING_METADATA.md). |
| `vehicle_rate_64_32_lhs/` | **Deployed** whole-vehicle rate tire surrogate used by every paper sweep. See [`TRAINING_METADATA.md`](vehicle_rate_64_32_lhs/TRAINING_METADATA.md). |
| `rig_rate_64_32/` | Tire-rig rate baseline retained for rig-vs-vehicle diagnostics and §III ablations. See [`TRAINING_METADATA.md`](rig_rate_64_32/TRAINING_METADATA.md). |
| `vehicle_static_32_16_lhs/` | Static-features vehicle surrogate; retained because §III and `archive/2026-07-05_deliverables_legacy/cte_vehicle_static_vs_rate.py` actively reference it for the force-RMSE-vs-CTE analysis. |
| `terrain_window_mlp_het/` | Heteroscedastic window MLP (calibrated per-sample n-uncertainty); loaded at runtime by the Dallas UKF proprioceptive measurement (`simulation/estimators/dallas_ukf_terrain_estimator.py`, §VI). |
| `vehicle_fy_64_32/` | Whole-vehicle Fy surrogate for the Dallas-style UKF (`benchmarking/lib/ukf_paper_validation.py`, §VI). |
| `rig_rate_paper118_v2_64_32/` | Paper118-spec (Dallas 2021) rig rate surrogate used by the Dallas-style UKF reproducer (`ukf_paper_validation.py`, §VI). |
| `rig_rate_32_16/`, `rig_static_32_16/`, `vehicle_rate_32_16_lhs/` | 32–16 members of the rig/vehicle × static/rate capacity family exercised by the §III rig-vs-vehicle sweep (`benchmarking/rig_vs_vehicle_tire_sweep.py`) and `make_fig1_4way.py`. |

Each deployed checkpoint ships with a `TRAINING_METADATA.md` describing
its dataset path, generator script, hyperparameters, sign convention,
and known limitations.

## Retired / archived checkpoints

Stale or experimental checkpoints (development snapshots, held-out
ablations, "didn't help" architectural variants) are **not** kept in this
polished repo. Recover them from git history or the original chrono-HIL
working copy (`project/SCM_Final/`), which retains the full development
tree including the earlier `archive/2026-05-*` cleanups.

Archived 2026-07-03 (not referenced by the runtime or any paper result):
`vehicle_twohead_128/` and `vehicle_rate_twohead_64_32/` (unified two-head
surrogate — not a reported §VI backend) and `terrain_window_mlp_speedcond/`
(speed-conditioned window MLP variant). The matching trainers
(`train_vehicle_twohead.py`, `build_peraxle_csv.py`) and the two-head
backend in `ukf_paper_validation.py` were removed in the same pass.
