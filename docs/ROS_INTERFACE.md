# ROS 2 / Chrono::ROS interface

The decoupled HIL stack runs over **ROS 2 (DDS)** by default, integrated the
Chrono-native way through **Chrono::ROS**. ZeroMQ is retained as a self-contained
fallback (`--transport zmq` or `HIL_TRANSPORT=zmq`) for setups without ROS 2.

## Environment

See `SETUP.md` §4. In short, before running with the ROS transport:

```bash
source /opt/ros/jazzy/setup.bash
source ~/packages/chrono_ros_ws/install/setup.bash   # chrono_ros_interfaces
# ... the PyChrono/acados env from SETUP.md §2-3 ...
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"
```

## Node graph

Each closed-loop run is three OS processes (launched by
`simulation/runtime/launch_decoupled.py`) on a per-run `ROS_DOMAIN_ID`
(derived from the sim port, so parallel sweep workers are isolated):

```
 chrono_sim_node ──(scm_hil/vehicle_state @100Hz)──▶ acados_mpc_controller_node
        ▲                                                     │
        └──────────(scm_hil/control_cmd @10Hz)◀───────────────┘
        │            (+ terrain re-conditioning piggybacked)
        │
        └─▶ ChROSPythonManager (Chrono-native):
              /clock
              /chrono_ros_node/chrono/vehicle/state/{pose,twist,accel}

 terrain_classifier.classifier_node (optional, --terrain-classifier):
   subscribes scm_hil/{vehicle_state,control_cmd}
   publishes  scm_hil/terrain_estimate ─▶ controller
```

## Topics

| Topic | Dir | Rate | Payload |
| --- | --- | --- | --- |
| `scm_hil/vehicle_state` | sim → controller/classifier/HUD | 100 Hz | rich `VehicleState` (pose, wheel encoders, tire forces, obstacles, operator+applied inputs). Also carries the lifecycle `SimStatus`. |
| `scm_hil/control_cmd` | controller → sim | 10 Hz | `ControlCommand` (steer/throttle/brake + MPC state) with the live terrain estimate piggybacked (latest-only, DDS *keep-last* depth 1). |
| `scm_hil/terrain_estimate` | classifier → controller | ~4 Hz | `TerrainEstimate` (class + probabilities). |
| `/chrono_ros_node/chrono/vehicle/state/{pose,twist,accel}` | sim → graph | 50 Hz | **Chrono::ROS-native** chassis body state (standard ROS messages via `ChROSBodyHandler`). |
| `/clock` | sim → graph | — | `ChROSClockHandler`. |

The `scm_hil/*` topics carry the framed msgpack payload in a
`std_msgs/UInt8MultiArray` (schema is the dataclasses in
`simulation/runtime/hil_messages.py`); QoS is `KEEP_LAST` depth 1 + `BEST_EFFORT`
(= the ZeroMQ `CONFLATE` latest-only semantic). The `/chrono/...` topics are
standard typed ROS messages published by Chrono's own handlers.

## Running

```bash
# ROS default (needs the ROS env above)
python simulation/runtime/launch_decoupled.py --path sinusoidal --terrain clay --time 15
ros2 topic list          # /scm_hil/*, /chrono_ros_node/..., /clock
ros2 topic echo /chrono_ros_node/chrono/vehicle/state/pose

# Self-contained fallback (no ROS needed)
HIL_TRANSPORT=zmq python simulation/runtime/launch_decoupled.py ...
# or per run:  --transport zmq
```

The benchmark sweeps (`benchmarking/run.py`) inherit the transport default; run
them under the ROS env for ros, or `HIL_TRANSPORT=zmq run.py ...` for the
fallback.

## Not yet done (opt-in future work)

The current design uses ROS 2 topics + Chrono::ROS handlers but keeps the repo as
plain Python scripts. Fully following Chrono::ROS's package conventions would
additionally mean: a colcon `ament_python`/`ament_cmake` package with `package.xml`,
typed `.msg` definitions (replacing the msgpack-in-`UInt8MultiArray` blobs) built
via `rosidl`, `ros2 launch` files replacing `launch_decoupled.py`'s subprocess
spawning, and node entry points (`ros2 run scm_hil sim_node`). That restructure
would make ROS 2 a hard build/run dependency (dropping the self-contained ZeroMQ
fallback), so it is deliberately left as an explicit opt-in.
