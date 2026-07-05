# Environment setup

Reproducing this project needs three things: a Python env (easy, via conda), a
**from-source PyChrono build** (the deformable-terrain plant), and — only to run
the NMPC controller — a **from-source acados build**.

The two source builds are pinned as **git submodules** under `third_party/`
(the exact fork + commit this project was built against), so you get the right
versions with one command instead of hunting for them:

```bash
git clone <this-repo> && cd terrain-aware-offroad-control
git submodule update --init --recursive third_party/acados   # ~acados + its deps
git submodule update --init            third_party/chrono     # ~1 GB Chrono clone
```

- `third_party/chrono` → **upstream** `github.com/projectchrono/chrono` @
  `81d8f2491`, on **`main`** (this is `main`'s merge of `feature/cuda13.3-gcc15`,
  which brings the CUDA 13 / GCC 15 / Thrust 3.x build-compat this modern
  toolchain needs). This replaces the old `zzhou292/chrono` fork — recent
  upstream Chrono (10.0.0-dev) already carries the Sensor/SCM functionality this
  project relied on the fork for.
- `third_party/acados` → `github.com/acados/acados` (with its blasfeo/hpipm
  submodules, hence `--recursive`).

> **Validation status (2026-07-03):** fully validated on the `scm-terrain`
> Python 3.12 env against this `main` commit (clean full Chrono build; CUDA 13.1,
> SWIG 4.2, Vehicle/Sensor+OptiX/Irrlicht/VSG enabled):
> - **Plant:** `chrono_setup` builds HMMWV_Full + SCM deformable terrain and steps under load.
> - **Closed-loop acados NMPC:** headless `launch_decoupled` (sim + controller over
>   ZMQ) completes with **RMS lateral CTE ≈ 0.024 m** on a sand lane-change.
> - **GPU sensor camera:** a ray-traced driver-POV frame renders on the GPU via OptiX.
>
> The only Chrono-API code change required was a data-path helper rename
> (`SetDataPath`→`SetVehicleDataPath`, `GetDataFile`→`GetVehicleDataFile`),
> handled back-compatibly in `simulation/runtime/chrono_setup.py`.

You still have to **build** each (submodules pin *what* to build, not the build
itself). Steps below reference the submodule paths.

## 1. Python environment (conda)

```bash
conda env create -f environment.yml      # env "scm-terrain", pinned deps
conda activate scm-terrain
```

This installs the pip/conda stack (numpy, scipy, pandas, matplotlib,
scikit-learn, casadi, pyzmq, pygame, torch, + optional msgpack/PySDL2). It does
**not** install PyChrono or acados — those are built from source below.

## 2. PyChrono (required) — build from source

The code imports `pychrono.core`, `pychrono.vehicle`, `pychrono.sensor`, and
`pychrono.irrlicht`, so Chrono must be built **with those modules enabled**.
There is no pip/conda package that ships all of them (notably the **Sensor**
module, used for the ray-traced driver-POV camera, is not in the public
conda-forge `pychrono`) — hence the source build.

Source is the pinned submodule `third_party/chrono` (upstream
`projectchrono/chrono` @ `81d8f2491`, on `main`). Build it with (CMake):

| CMake flag | Enables | Extra prerequisites |
|---|---|---|
| `-DCH_ENABLE_MODULE_PYTHON=ON`   | SWIG Python bindings (`pychrono`) | SWIG, a matching Python |
| `-DCH_ENABLE_MODULE_VEHICLE=ON`  | HMMWV + **SCM** deformable terrain | Chrono data dir |
| `-DCH_ENABLE_MODULE_IRRLICHT=ON` | chase-cam visualization | Irrlicht |
| `-DCH_ENABLE_MODULE_SENSOR=ON`   | ray-traced driver-POV camera | **CUDA + NVIDIA OptiX SDK** + GLFW/GLEW |

Build Chrono against the **same Python** as the conda env (this project uses
Python 3.12 — the pinned Chrono commit's SWIG bindings link `libpython3.12`).
Then, in the activated env, expose the bindings and the Chrono data directory:

```bash
# cmake -S third_party/chrono -B third_party/chrono/build -DCH_ENABLE_MODULE_PYTHON=ON \
#   -DCH_ENABLE_MODULE_VEHICLE=ON -DCH_ENABLE_MODULE_IRRLICHT=ON -DCH_ENABLE_MODULE_SENSOR=ON
# cmake --build third_party/chrono/build -j
export PYTHONPATH="$PWD/third_party/chrono/build/bin:$PYTHONPATH"
export CHRONO_DATA_DIR="$PWD/third_party/chrono/data/"     # HMMWV meshes etc.
python -c "import pychrono.vehicle, pychrono.sensor, pychrono.irrlicht; print('pychrono OK')"
```

Persist these by dropping them in
`$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh` so they are set on activation.

> Headless / no-GPU note: `pychrono.sensor` needs CUDA + OptiX at build and a
> GPU at run time. The autonomous sweeps run `--vis-mode none` (no camera) and
> only need Vehicle; the driver-POV / HIL features need Sensor.

## 3. acados (required only to run the NMPC controller)

The tire/estimator benchmarks and paper-figure regeneration do **not** need
acados. Running the acados NMPC controller does.

```bash
# build the pinned submodule third_party/acados (cmake + make; see its docs)
pip install -e third_party/acados/interfaces/acados_template
export ACADOS_SOURCE_DIR="$PWD/third_party/acados"
export LD_LIBRARY_PATH="$PWD/third_party/acados/lib:$LD_LIBRARY_PATH"
python -c "import acados_template; print('acados_template OK')"
```

`export ACADOS_SOURCE_DIR=...` must be set before any acados import (the
controller preloads `libacados.so` from it via `ctypes`).

## 4. ROS 2 transport (optional — the Chrono::ROS-native path)

The sim<->controller link defaults to a self-contained ZeroMQ transport (nothing
below is needed for that). Passing `--transport ros` instead runs the link over
ROS 2 / Chrono::ROS: the rich telemetry rides rclpy topics (`/hil/port_*`) and
the sim additionally exposes the chassis on the ROS graph via Chrono's own
`ChROSPythonManager` (`/clock`, `~/chrono/vehicle/state/{pose,twist,accel}`).

Prereqs (already present on this workstation): **ROS 2 Jazzy** (py3.12, matches
`scm-terrain`) and the prebuilt colcon workspace holding `chrono_ros_interfaces`
(the message package the `chrono_ros_node` IPC subprocess needs). Source both
*before* the PyChrono env vars so the from-source pychrono libs win on
`LD_LIBRARY_PATH`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/packages/chrono_ros_ws/install/setup.bash    # chrono_ros_interfaces
# ... then the PyChrono/acados exports from §2–3 ...
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"
python -c "import rclpy, pychrono.ros; print('ros stack OK')"
```

Parallel sweeps must set a unique `ROS_DOMAIN_ID` per worker to isolate their DDS
graphs. `--transport ros` is validated at closed-loop parity with ZeroMQ.

## 5. Verify

```bash
conda activate scm-terrain
python -c "import numpy, scipy, pandas, matplotlib, sklearn, casadi, zmq, pygame, torch; print('py stack OK')"
python -c "import pychrono.vehicle, pychrono.sensor; print('pychrono OK')"
python -c "import acados_template; print('acados OK')"   # if you built acados
```

## Later / TODO

- Investigate a **public `pychrono` image** (conda-forge, or a prebuilt
  container) to remove the source-build step for users who don't need the
  Sensor camera — tracked as a follow-up; the source build is the current path.
- Restore the large data with `data_sync/data_sync.sh pull <tag>` (see `DATA.md`).
