# openarm_teleop_cpp

ROS 2 (Jazzy) package with a C++ port of the OpenArm VR teleoperation pipeline that runs as a Dora dataflow
in `dora-openarm-ros2`. It replaces three python nodes with a single ROS 2 node:

| Python (dora-openarm-ros2) | C++ (this repo) |
|---|---|
| `dora-openarm-vr` Quest UDP receiver + One-Euro smoothing | `quest_vr` library |
| `dora-openarm-kinematics` IK (openarm_control 0.1.0 + mink 1.1 + DAQP) | `openarm_ik` library |
| `dora_ros2_bridge/main.py` (dora-to-ros2) | `teleop_bridge` library |
| the dataflow wiring the three together | `openarm_teleop_node` |

The low-pass filters of `main.py` (`--lpf-*`, `--cmd-lpf-hz`) are not ported.

## Layout

```
apps/            executables (ROS 2 node, replay/check harnesses, benchmark)
include/, src/   quest_vr, openarm_ik, teleop_bridge libraries
config/          node parameters (teleop.yaml), MuJoCo model (copy of openarm_mujoco/v1), grasp_presets.yaml
launch/          teleop.launch.py
third_party/     DAQP 0.7.2 QP solver (C, MIT), vendored unmodified
tests/           golden-file tests; tests/reference/ holds the python scripts that recorded them
```

## Dependencies

- CMake ≥ 3.22, a C++20 compiler
- Eigen3, yaml-cpp (`sudo apt install libeigen3-dev libyaml-cpp-dev`, or `rosdep install --from-paths src --ignore-src -y`)
- nlohmann_json ≥ 3.10: the system package when installed (`nlohmann-json3-dev`),
  otherwise CMake downloads release 3.11.3 at configure time.
- MuJoCo 4.0.1: downloaded by CMake at configure time from the official GitHub
  release (x86_64 or aarch64, chosen from the CPU, SHA256-checked). To use a local
  install instead: `-DMUJOCO_ROOT=<dir with include/ and lib/>`.
- ROS 2 Jazzy (rclcpp, ament_index_cpp, sensor/geometry/trajectory/control_msgs)

## Build and test

Clone into the `src/` of a colcon workspace (for example `ws/src/openarm_teleop_cpp`), then:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openarm_teleop_cpp
colcon test --packages-select openarm_teleop_cpp && colcon test-result --verbose
```

The downloaded `libmujoco.so.4.0.1` is installed into `install/openarm_teleop_cpp/lib`
next to the node.

Each test feeds recorded inputs to a C++ harness and compares the result against
the output of the original python code (`tests/data/*.json.gz`). The tests need
only `python3` with its standard library.

| Test | Compares | Max error |
|---|---|---|
| `parity_with_python` | Quest receiver + smoothing | 2e-7 m, 1e-7 rad |
| `ik_step1_kinematics` | TCP pose and body Jacobian vs mink | 5e-16 |
| `ik_se3` | SE(3) log / Jlog vs mink's C code | 2e-14 |
| `ik_boxqp_{daqp,activeset}` | box QP vs DAQP | 1e-13 |
| `ik_replay_{daqp,activeset}`, `ik_replay_daqp_threads` | 600-tick IK replay vs openarm_control 0.1.0 | 2e-10 rad |
| `bridge_parity` | every ROS message of `main.py` (real, real + 200 Hz timer, sim) | 0 |

To re-record a golden file, run the matching `tests/reference/*.py` script with
`--record tests/data` from a `dora-openarm-ros2` checkout's python environment
(each script's docstring gives the command).

## Run

Start the robot first, then the node, then the Quest app. Stop the dora
dataflow before starting the node, because both use UDP port 5006 and publish
to the same controllers.

```bash
# terminal 1: robot (fake hardware first)
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true use_fake_hand:=true

# terminal 2: teleop
source install/setup.bash
ros2 launch openarm_teleop_cpp teleop.launch.py                 # real, bimanual, 200 Hz arm commands
ros2 launch openarm_teleop_cpp teleop.launch.py arms:=right      # right arm only
ros2 launch openarm_teleop_cpp teleop.launch.py mode:=sim cmd_rate_hz:=0
```

On the Quest, send the stream to this PC's IP, port 5006. Holding grip makes
that arm follow. In `bimanual` mode, the IK starts only after both grips are
held. The node waits for `/joint_states` before it starts solving.

Everything runs in one process (`openarm_teleop_node`) on separate threads:
UDP receiver, control loop (receiver + IK at `tick_hz`, one IK worker per arm),
and the ROS executor.

All parameters are in `config/teleop.yaml`. Pass `params_file:=<file>` to use
your own file.

| Parameter | Default | Notes |
|---|---|---|
| `host`, `port` | `0.0.0.0`, `5006` | Quest UDP stream |
| `tick_hz` | 500 | receiver + IK loop rate |
| `mode` | `real` | `real`: per-arm `JointTrajectory` with the safety ramp; `sim`: `/joint_command` |
| `arms` | `bimanual` | `bimanual`, `right` or `left` |
| `model_xml` | `config/openarm_mujoco/v1/scene.xml` | |
| `grasp_presets` | `config/grasp_presets.yaml` | |
| `cmd_rate_hz` | 200 (yaml) | > 0: publish arm commands from a fixed-rate timer; 0: every tick |
| `controller_tilt_deg` | 25 | tilt about the controller's own Y axis |
| `wait_for_joint_states` | true | start the IK from `/joint_states` |
| `invert_thumb_{x,y}_{left,right}` | x_left false, others true | negate a thumbstick axis before it drives the thumb (x → metacarpal, y → proximal); all false = `main.py` in git |
| `initial_thumb_rad` | `[1.57, 0.0]` | thumb target before the first stick event |
| `ik.qp_backend` | `daqp` | `daqp` or `activeset` (self-written) |
| `ik.threads_per_arm` | true | solve each arm on its own thread |
| `ik.max_iterations`, `ik.damping`, `ik.lm_damping`, `ik.posture_cost`, `ik.position_cost`, `ik.orientation_cost` | 10, 0.1, 0.01, 0.01, 1, 1 | same as the dataflow yaml |

`ros2 run openarm_teleop_cpp ik_bench install/openarm_teleop_cpp/share/openarm_teleop_cpp/config/openarm_mujoco/v1/scene.xml`
times the IK. One bimanual
solve takes about 40 µs, compared with about 1.2 ms in python.

## Third-party licenses

- DAQP (`third_party/daqp`): MIT, see `third_party/daqp/LICENSE`.
- OpenArm MuJoCo model (`config/openarm_mujoco`): Apache-2.0, Enactic, Inc. (header of each XML file).
- MuJoCo (downloaded at build time): Apache-2.0.
