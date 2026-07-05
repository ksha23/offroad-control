# offroad-control

Framework, benchmarks, and paper for *Terrain-Aware, Latency-Robust Control for
Off-Road Autonomy and Teleoperation on Deformable Terrain*: a differentiable
neural SCM tire surrogate, an online Bekker-`n` terrain estimator, terrain-aware
NMPC speed planning, an asymmetric throttle disturbance observer, an
intent-preserving DOB-CBF safety filter, 5G-latency teleoperation (with a
frame-delayed driver POV), forward collision warning, and a counterfactual-replay
evaluation — all on PyChrono's Soil Contact Model (SCM) deformable-terrain plant.
Every figure and table in the paper regenerates from one command
— `python benchmarking/run.py --tier paper` (runs every paper sweep, then
republishes all figures and table CSVs). The two HIL/human figures
(`hil_hud_pov.png`, `teleop_counterfactual_clearance.png`) need the G29
and a human driver and are the only exceptions. Build the paper PDF
with `tectonic my_paper/paper.tex`.

- **Setup:** [`SETUP.md`](SETUP.md) — conda env (`environment.yml`) + from-source PyChrono/acados.
- **Large data & raw results:** [`DATA.md`](DATA.md) — restore with `data_sync/data_sync.sh pull <tag>`.
- **Contributor / agent guide:** [`AGENTS.md`](AGENTS.md).

## Layout

| Path | Contents |
| --- | --- |
| `simulation/` | The runtime, organized into role subpackages (below). Historical flat imports (`from nn_tire_model import …`) still resolve via the `flatpath.py` compatibility shim; new code should prefer the package form (`from simulation.tire_models.nn_tire_model import …`). |
| `simulation/runtime/` | Process/plant layer: `chrono_sim_node.py` (PyChrono HMMWV plant), `launch_decoupled.py` (spawns sim + controller), `chrono_setup.py`, `hil_messages.py` (ZeroMQ-fallback wire types; ROS 2 / Chrono::ROS is the default transport, see `docs/ROS_INTERFACE.md`). |
| `simulation/control/` | acados NMPC: `acados_mpc_controller_node.py`, `acados_mpc_solver.py`, `mpc_helpers.py`, `speed_profile.py`. |
| `simulation/tire_models/` | Tire surrogates the MPC/CBF query: `nn_tire_model.py`, `analytical_tire_models.py`, `tire_input_features.py`. |
| `simulation/estimators/` | Online terrain estimators: `learned_terrain_estimator.py` (window-MLP) and the `*_ukf_terrain_estimator.py` / `fused_terrain_estimator.py` UKF backends. |
| `simulation/teleop/` | Human-in-the-loop I/O: `g29_controller.py` (wheel/pedals), `delayed_pov.py` (glass-to-glass camera delay), `hil_hud.py`. |
| `simulation/scenarios/` | Worlds and stimuli: `traffic.py`, `spatial_terrain.py`, `terrain_gen.py`, `reference_path.py`, `path_utils.py`, `latency_profile.py`. |
| `simulation/shared/` | Cross-cutting helpers: `collision_detector.py`, `live_debug_plotter.py`, `param_consistency.py`. |
| `simulation/framework/` | The six-extension-point contract: `interfaces.py` declares one `Protocol` per role (CommandSource, SafetyFilter, CollisionWarning, TireModel, TerrainEstimator, LatencyProfile); `registry.py` keeps a per-role `Registry`; `builtins.py` currently registry-wires the safety-filter, collision-warning, and latency-profile roles; `test_conformance.py` checks those registry-instantiated roles against their Protocols. |
| `simulation/safety/` | The active DOB-CBF safety filter for intent-preserving HIL and the terrain- and latency-aware `collision_warning.py` warning module. DOB-CBF is the only safety filter; the earlier MPPI/NMPC shields are not present. A textbook `vanilla_cbf` baseline is retained for comparison. |
| `simulation/sensors/`, `simulation/terrain_classifier/` | Chrono sensor wrappers and the RF terrain classifier node (unchanged by the reorg). |
| `benchmarking/` | The full benchmarking suite. `benchmarking/run.py` is the single orchestrator; `publish_paper_figures.py` writes canonical figures into `my_paper/paper_figures/`. Sub-scripts test one paper claim each |
| `nn_training/` | Canonical trainers: `train_static_v3.sh` (rig static), `train_rate_v2.sh` (rig rate), `train_vehicle_lhs.sh` (whole-vehicle variants), `train_terrain_window_mlp.py` (window terrain estimator), `train_vehicle_fy_surrogate.py` (whole-vehicle Fy surrogate for the Dallas UKF). `train_variant.py` is the shared tire-NN trainer |
| `data_collection/` | Tire-rig (`collect_static_data.cpp`, `collect_rate_data.cpp`), closed-loop tire-surrogate (`collect_closed_loop_data.py`), the broad multi-axis terrain-estimator collector (`collect_broad_terrain.py`), and the Dallas-UKF SCM collectors (`run_dallas_scm.py` single-run, `collect_lhs_training_scms.py` LHS sweep) |
| `utilities/` | Closed-loop trace collection (`collect_diverse_terrains.py`, `collect_rich_excitation.py`) and diagnostic/offline utilities |
| `nn_models/` | Active trained checkpoints only: one rig tire surrogate, one whole-vehicle tire surrogate, and one n-only terrain estimator |
| `data/` | Four categories: `tire_rig/`, `whole_vehicle/`, `terrain_estimator/` (window-MLP traces), and `dallas_scm/` (Dallas-UKF SCM logs: `lhs_train300/` training sweep, `lhs100/` + `lhs100_cl/` benchmarks, canonical `clay/sandy_loam/sand.npz`) |
| `latency_profiles/` | Generated 5G traffic profiles |
| `config/` | `terrain_yamls/` (LHS-sampled terrain configs for `closedloop_sine_lhs_fair_v2` and `rich` excitation sets) |
| `data/paths/` | Reference-path definitions for the tracking/planning experiments |
| `data_sync/` | `data_sync.sh` — snapshot/restore the large off-machine artifacts (see `DATA.md`) |
| `docs/` | Design docs: framework contracts, ROS interface, SCM force model, longitudinal force balance, surrogate/estimator findings, HIL protocol, and `PAPER_TABLE_VALUE_PROVENANCE.md` (paper number → source map) |
| `my_paper/` | `paper.tex` (IEEEtran two-column; build the PDF with `tectonic my_paper/paper.tex`), `paper_figures/` (current figures + CSV backings), original ACMD abstract |
| root | `environment.yml` + `SETUP.md` (setup), `DATA.md` (large-data restore), `AGENTS.md`/`CLAUDE.md` (contributor guide) |

## Framework contracts

The runtime is organised around six extension points. Each has one
`Protocol` in `simulation/framework/interfaces.py` and one `Registry`
in `simulation/framework/registry.py`. The safety, warning, and latency
roles are currently registry-instantiated; command-source, tire-model,
and terrain-estimator variants are still selected through the established
CLI/factory paths used by the benchmarks. Adding a new registry-backed
safety flavor is one decorator with no edits to the consumers:

```python
from simulation.framework import SAFETY_FILTERS, SafetyFilter

@SAFETY_FILTERS.register("my_filter")
class MyFilter:
    def filter(self, s, t, b, state, obs): ...
    def update_command_age(self, t): ...
    def set_teleop_delay(self, d): ...
    def get_diagnostics(self): ...

# anywhere in the runtime / benchmarks:
shield = SAFETY_FILTERS.create("my_filter", vehicle_params=..., ...)
```

| Registry | Protocol method | Current flavors |
| --- | --- | --- |
| `COMMAND_SOURCES`    | `next_command`     | acados NMPC, Logitech G29, WASD |
| `SAFETY_FILTERS`     | `filter`           | DOB-CBF; no-filter bypass |
| `COLLISION_WARNINGS` | `evaluate`         | ttc (terrain + latency) |
| `TIRE_MODELS`        | `predict`          | rate-MLP, axle-rate-MLP, Pacejka, TMeasy |
| `TERRAIN_ESTIMATORS` | `observe`/`estimate` | sliding-window MLP (`n`), Dallas-style UKF (offline only) |
| `LATENCY_PROFILES`   | `delay`            | constant, replay, learned 5G N-HiTS |

`python simulation/framework/test_conformance.py` instantiates the
registry-wired DOB-CBF and TTC warning roles and `isinstance`-checks them
against their declared Protocols; a registry-wired flavor that drifts
from its API fails the check rather than producing silent runtime errors. See
`docs/FRAMEWORK_CONTRACTS.md` for the full contract map and current
boundary limitations.

## Environment

```bash
conda env create -f environment.yml     # creates env "scm-terrain"
conda activate scm-terrain
```

`environment.yml` pins the installable stack (CasADi 3.6, PyTorch,
NumPy/SciPy/Pandas/Matplotlib/scikit-learn, pyzmq, pygame). Two dependencies are
**built from source** and are covered in [`SETUP.md`](SETUP.md):

- **PyChrono** (required) — build Project Chrono with the `PYTHON`, `VEHICLE`,
  `IRRLICHT`, and `SENSOR` modules, then `export PYTHONPATH=<chrono_build>/bin`.
- **acados** (only to run the NMPC controller) — build it, `pip install -e`
  `acados_template`, and `export ACADOS_SOURCE_DIR=<acados>` before any acados
  import.

Restore the large data/results the benchmarks read via
[`DATA.md`](DATA.md) (`data_sync/data_sync.sh pull <tag>`).

## Reproducing the paper

`benchmarking/run.py --tier paper` is **the one command**: it runs all 20
paper sweeps at the full matrix (serially — one Chrono sweep at a time), then
regenerates every table CSV and figure in `my_paper/paper_figures/` (it invokes
`publish_paper_figures.py` + `make_paper_figures.py` automatically at the end).
Nothing else needs to be run by hand.

```bash
# THE command: every table + figure in the paper (large; ~15 h on a 24-core box)
python benchmarking/run.py --tier paper

# Same orchestrator, smaller/faster:
python benchmarking/run.py --tier pilot                          # ~6 h reduced matrix
python benchmarking/run.py --tier smoke                          # ~20 min, every sweep fires once
python benchmarking/run.py --tier paper --only safety convoy_cf  # subset by name
python benchmarking/run.py --tier paper --dry-run                # print the plan, run nothing
python benchmarking/run.py --tier paper --workers 6 --timeout 400 --continue-on-error

# Figures only (no Chrono re-run), re-plotted from the newest results:
python benchmarking/make_paper_figures.py
```

### Terrain-estimator comparison (paper §VI)

Three estimators are compared on the same Chrono SCM ground truth:
the Dallas-style **state-augmented UKF** with two tire backends
(analytical **Bekker** and the whole-vehicle **NN** surrogate
`nn_models/vehicle_fy_64_32/`), and the deployed sliding-window
**MLP** regressor. The full rebuild is four steps (A → D).

The benchmark LHS box restricts `bekker_n` to `[0.40, 1.30]` so all
three estimators are evaluated inside the window-MLP's training range
and the SCM patch's physical regime (the SCM model is unsimulable
below n ≈ 0.37). The NN-UKF surrogate is trained on a *widened* soil
box (so the canonical clay/dirt/sand presets are interior, not
box-corner, points — a standard-box surrogate fails on clay).

**Step A — train the NN-UKF tire surrogate (widened box, ~25 min)**

```bash
# 300 disjoint LHS training scenarios (seed 7), widened soil box,
# half open-loop / half PI-cruise throttle
python data_collection/collect_lhs_training_scms.py --n 300 --workers 8 \
    --seed 7 --widened-box --out-dir data/dallas_scm/lhs_train300

# train the (128,64) MLP, 90/10 split-by-scenario → vehicle_fy_64_32/
python nn_training/train_vehicle_fy_surrogate.py \
    --lhs-dir data/dallas_scm/lhs_train300 \
    --hidden 128 64 --epochs 400 --decim 2 --test-frac 0.10
```

**Step B — broad 100-LHS benchmark, two excitation modes (Fig. 8 + 9)**

```bash
# Open-loop (constant throttle 0.75) — the paper-headline Fig. 8.
# This is the MLP's native training excitation. ~30 min incl. SCM.
python benchmarking/bench_terrain_estimators_lhs.py --n 100 --workers 8 \
    --n-min 0.40 --n-max 1.30 --steer-amp-rad 0.6 \
    --open-loop-throttle 0.75 --out-name lhs100_fair

# Closed-loop (PI cruise to 5 m/s) — separate SCM-log dir via suffix
python benchmarking/bench_terrain_estimators_lhs.py --n 100 --workers 8 \
    --n-min 0.40 --n-max 1.30 --steer-amp-rad 0.6 \
    --open-loop-throttle -1 --target-speed 5.0 \
    --log-suffix _cl --out-name lhs100_cl
```

`lhs100_fair.csv` (OL) and `lhs100_cl.csv` (CL) back the 100-soil
estimator table (`tab:estimator_lhs100`); see
`docs/PAPER_TABLE_VALUE_PROVENANCE.md`.

Writes `lhs100_fair.{png,csv}`, `lhs100_cl.{png,csv}`,
`lhs100_cl_vs_ol.png`. (Add `--skip-collection` to re-run only the
estimator pass on existing SCM logs, ~2 min.)

**Step C — 3-preset single-trace spot check (Fig. 10)**

```bash
# Regenerate the three canonical logs at the benchmark excitation
# (amp 0.6, PI cruise). NOTE: --open-loop-throttle -1 selects PI
# cruise; a value >=0 would be a constant open-loop throttle.
for t in clay dirt sand; do
  out=$([ $t = dirt ] && echo sandy_loam || echo $t)
  python data_collection/run_dallas_scm.py --terrain $t --time 50 --lead-in 3 \
      --steer-amp-rad 0.6 --open-loop-throttle -1 --target-speed 5.0 \
      --output data/dallas_scm/${out}.npz
done
# run all three estimators → terrain_estimator_comparison.{png,csv}
python benchmarking/eval_terrain_estimators.py
```

**Step D (optional) — paper118-faithful Bekker-vs-NN UKF only**

```bash
python benchmarking/lib/ukf_paper_validation.py   # → ukf_dallas_validation_scm.png
```

**Current results** (NN-UKF = best median in every mode):

| Benchmark | Bekker-UKF | NN-UKF | Window MLP |
| --- | --- | --- | --- |
| 100-LHS open-loop (Fig. 8) median | 34.3 % | **12.9 %** | 17.0 % |
| 100-LHS closed-loop (Fig. 9) median | 20.9 % | **9.4 %** | 15.7 % |
| Canonical clay (Fig. 10) | 22.9 % | **18.0 %** | 24.8 % |
| Canonical sandy loam | 4.9 % | **3.2 %** | 8.7 % |
| Canonical dry sand | 36.2 % | 10.8 % | **7.5 %** |

### Human-in-the-loop safety-filter rounds

The HIL rounds (paper §VI-A) are **not** part of `benchmarking/run.py`
because they require a human driver. Run separately:

```bash
# Default G29 protocol, symmetric link (camera delay = command delay)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none dob_cbf \
    --delays 0.0 0.15 0.30 \
    --rounds 3 --manual-mode g29 --vis-mode sensor

# 5G-style asymmetric link (heavier video downlink than command uplink)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none dob_cbf \
    --delays 0.0 0.15 0.30 \
    --camera-delay-scale 1.6 \
    --rounds 3 --manual-mode g29 --vis-mode sensor

# WASD smoke test that runs in 8 s with no driver input — pipeline
# integration check only (vehicle sits still, no collisions, no
# intervention). Useful for verifying the wiring after env changes.
python benchmarking/human_delay_compensation_rounds.py --quick --auto-start
```

Each round delays both the operator command path
(`--manual-input-delay`) and the driver POV camera
(`--camera-input-delay = camera_delay_scale * delay`). The active
safety filter additionally receives `--teleop-delay` so its
predictive horizon is delay-aware. The script prompts before each
round so the driver can get set; pass `--auto-start` to skip.

Per-cell metrics aggregated by the script:
- safety: `collisions` (unique obstacles hit), `near_misses`,
  `min_clearance_m`;
- intrusiveness: `intervention_rate_pct`, `mean_abs_dsteer`,
  `mean_abs_dthrottle` (how often the shield fires and how far it
  pulls the operator's command);
- tracking: `rms_cte_m`, `speed_ratio`;
- each metric is reported per (filter, delay) cell with mean and
  std across the `--rounds` repeats.

Outputs land in
`benchmarking/results/human_delay_compensation_rounds_<ts>/`:
`results.csv` (per-round), `summary_by_filter_delay.csv`
(aggregated), `summary.md`, and `figures/` (6-panel
safety-vs-intrusiveness panel + collision heatmap). The publish step
picks them up if the timestamp suffix matches.

## NN models

| Directory | Role |
| --- | --- |
| `rig_rate_64_32/` | Retained tire-rig rate surrogate for rig-vs-vehicle diagnostics |
| `vehicle_rate_64_32_lhs/` | Default whole-vehicle rate surrogate for standard NMPC and safety-filter sweeps |
| `terrain_window_mlp/` | Retained online terrain estimator; n-only output |
| `rig_rate_paper118_v2_64_32/` | Paper118-spec rig NN (uniform LHS, widened α and Fz) used by the Dallas-style UKF reproduction; see `benchmarking/lib/ukf_paper_validation.py` |
| `vehicle_fy_64_32/` | **Whole-vehicle Fy surrogate** for the Dallas-style state-augmented UKF — trained on a 300-scenario disjoint widened-box LHS sweep (`--widened-box`, half OL / half PI-cruise throttle). Predicts $(F_{y,\mathrm{total}}, M_{\mathrm{yaw,total}})$ directly, replacing the rig NN + rig-to-vehicle calibration scalar (no post-hoc scalar). Held-out Fy R² = 0.90. Drives the 100-LHS benchmark and canonical spot-check in paper §VI; see its `TRAINING_METADATA.md`. |

Older static, axle-rate, and joint-estimator checkpoints and the PIL
tire model are not present in this repo. Recover them from git history
only when replaying historical ablations.

## Datasets

The active data tree has exactly three categories:

| Directory | Purpose |
| --- | --- |
| `data/tire_rig/` | Open-loop single-tire SCM rig CSVs (`scm_static_100k_v4.csv`, `rate_v2_100k.csv`, `rate_paper118_v2_15k.csv`) used by `train_static_v3.sh` / `train_rate_v2.sh` / the Dallas UKF baseline. The v1 sweeps (`rate_v1_100k`, `rate_paper118_30k`) are not retained |
| `data/whole_vehicle/lhs/` | Closed-loop LHS-sampled training data used by `train_vehicle_lhs.sh` to produce `vehicle_rate_64_32_lhs` and its matched-architecture variants |
| `data/terrain_estimator/` | Sliding-window traces used to train `terrain_window_mlp` — `traces_broad_v7/` (active; 3600 scenarios spanning 180 LHS cells × scripted+closed-loop × 3 speeds × 3 paths × 2 bumpiness), `traces_vertical_v5/` (predecessor, 400 scripted traces with vertical IMU channels) |

Legacy `closed_loop_v1/v2/v3_rich/` datasets and the `5g_generated/`
profile cache are not retained; recover them from git history if needed.

## Paper

`my_paper/paper.tex` (IEEEtran two-column) is the self-contained full
paper and the contract for what has been built and claimed; the PDF is a
build artifact, not tracked. Build it with `tectonic my_paper/paper.tex`.
`my_paper/abstract.tex` is the original single-page ACMD 2026 abstract.

## Tire-rig training (preserved baseline)

The tire-surrogate pipeline has two generations (paper
§III-C). The active root keeps one checkpoint from each generation;
retired variants live in git history.

| Generation | Where | What it does |
| --- | --- | --- |
| Tire rig | `data_collection/collect_static_data.cpp`, `collect_rate_data.cpp` | Chrono SCM single-tire rig sweep over $(\kappa, \alpha, F_z, \theta_\mathrm{soil})$; logs the ground-truth tire force per query |
| Tire rig trainer | `nn_training/train_static_v3.sh`, `train_rate_v2.sh` (call `train_variant.py`) | Trains static / rate variants; the retained active baseline is `rig_rate_64_32` |
| Closed-loop | `data_collection/collect_closed_loop_data.py` | Runs the actual MPC stack on randomised scenarios and logs the live operating point with the SCM ground-truth force |
| Closed-loop trainer | `nn_training/train_vehicle_lhs.sh` | Trains the retained LHS whole-vehicle checkpoint `vehicle_rate_64_32_lhs` |

If you need to retrain the rig models from scratch, re-collect via the
Chrono rig binaries in `data_collection/` and rerun
`bash nn_training/train_static_v3.sh` / `train_rate_v2.sh`.

## Collision warning (modular HMI signal)

`simulation/safety/collision_warning.py` is a swappable forward
collision-warning module that runs in parallel with whatever safety
filter is selected (or none). It outputs a discrete severity
{GREEN, YELLOW, ORANGE, RED} signal that downstream code (e.g. an HMI
overlay) can consume. The default flavor is time-to-collision with
two extensions: braking deceleration is computed *analytically at
init time* by querying the deployed rig surrogate
(`rig_rate_64_32`) over a sweep of braking slip ratios at each
$\hat n \in [0.40, 1.30]$ on a 0.05 grid; the resulting table runs
from $a_b\!\approx\!2.3\,\mathrm{m/s^2}$ on soft clay to
$4.7\,\mathrm{m/s^2}$ on firm sand. The live $\hat n$ from the
terrain estimator indexes this table at runtime. The operator
reaction budget inflates with the EMA one-way latency plus the
standard deviation of recent jitter. Soft soil and high jitter both
fire the warning earlier.

Two validators ship with the warning module:

```bash
# 1) Forward lead time at fixed throttle into a single rock — sweeps
#    terrain x latency and verifies lead time grows monotonically with
#    softer soil and longer one-way delay.
python benchmarking/collision_warning_test.py --workers 3

# 2) Brake-decel validation against actual Chrono SCM stops — 27 full
#    brake trials (3 terrains x 3 initial speeds x 3 seeds). The
#    analytical a_b(n) prediction lands within 0.17 m mean absolute
#    error of the recorded stopping distance.
python benchmarking/brake_test.py
```

## Scope notes

The live terrain-estimation story is n-only. The controller maps the
estimated `n` along the retained clay--dirt--sand Bekker--Mohr manifold
to recover the complete soil vector used by NMPC and the safety shield.

The safety layer is presented as swappable rather than as a single
hard-coded filter. DOB-CBF is the active filter because it solves for
the closest safe command and therefore has the cleanest
intent-preserving HIL story; a textbook `vanilla_cbf` baseline is
retained for comparison. DOB-CBF is the only safety filter; the earlier
MPPI/NMPC shields are not present.

Do not present removed exploratory ablations as part of the selected
framework story in the paper or slides.
