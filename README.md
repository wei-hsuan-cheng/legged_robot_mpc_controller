# Legged Robot MPC Controller

ROS 2 controller package for legged robot MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git). The first migration target is the Unitree G1 humanoid whole-body MPC from [`wb_humanoid_mpc`](https://github.com/wei-hsuan-cheng/wb_humanoid_mpc.git).

- Humanoid common, whole-body (WB), and centroidal MPC core are vendored under [`include/`](./include) and [`src/core/`](./src/core).
- Simulation runs through [`mujoco_ros2_control`](https://github.com/wei-hsuan-cheng/mujoco_ros2_control.git) only (no `mujoco_vendor`).
- All legacy `task.info` / `reference.info` / `gait.info` loading is removed: the whole-body MPC is configured entirely from `generate_parameter_library` YAML (see Configuration below).
- The `ros2_control` hardware description is a separate xacro ([`g1.ros2_control.xacro`](./description/g1/urdf/g1.ros2_control.xacro)), same pattern as `dynamics_mpc_controller`.

Migration status and remaining milestones are documented in [`docs/humanoid_migration.md`](./docs/humanoid_migration.md).

## Build and Install

Make sure the workspace `src` contains this package, `ocs2_ros2`, and `mujoco_ros2_control`, then install dependencies:

```bash
cd <workspace_dir>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

First download the `mujoco` pre-built library (details in [`mujoco_ros2_control`](https://github.com/wei-hsuan-cheng/mujoco_ros2_control.git)):

```bash
cd <your_path>
# Check x86_64 or aarch64
wget -O mujoco-3.3.7-linux-x86_64.tar.gz \
  https://github.com/google-deepmind/mujoco/releases/download/3.3.7/mujoco-3.3.7-linux-x86_64.tar.gz && \
tar -xzf mujoco-3.3.7-linux-x86_64.tar.gz
export MUJOCO_DIR=<your_path>/mujoco-3.x.x # e.g. mujoco-3.3.7 (depends on your own version)
```

Then build all pkgs up-to `legged_robot_mpc_controller`:

```bash
cd <workspace_dir>
export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
export MAKEFLAGS=-j2 && \
export NINJAFLAGS=-j2 && \
colcon build --symlink-install \
  --packages-up-to legged_robot_mpc_controller \
  --executor sequential --parallel-workers 2 \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
  . install/setup.bash
```

## Run MuJoCo Example

Unitree `g1` environment. MuJoCo starts **paused** (`mujoco_wait_to_start:=true` by default); [`controller_sequence.py`](./launch/controller_sequence.py) loads and activates `joint_state_broadcaster` (and the MPC controller when `spawnMpcController:=true`), then calls the `/mujoco_ros2_control/start` service so physics begins with controllers already active:

```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_wb_mpc_controller \
  mujoco_headless:=true \
  velocityCommandGui:=true
```

The launch sequence activates the MPC controller before starting MuJoCo physics.
If MPC activation fails, MuJoCo is not started; this avoids beginning simulation with zero torque on the humanoid.

Useful launch args:

```bash
velocityCommandGui:=true | false
spawnMpcController:=true | false         # false is only for environment smoke tests; the robot will not balance
mpcControllerName:=humanoid_centroidal_mpc_controller | humanoid_wb_mpc_controller
use_mujoco_sim:=true | false             # false: plain ros2_control_node (fake hardware)
use_fake_hardware:=false | true          # mock_components/GenericSystem when not using MuJoCo
ros2ControlCommandInterface:=effort | position
initialPoseFile:=<...>/initial_pose.yaml # initial joint state (config/g1/initial_pose.yaml)
rviz:=true | false
mujoco_headless:=true | false
mujoco_wait_to_start:=true | false       # paused start + /mujoco_ros2_control/start service
mujoco_real_time_factor:=1.0             # double
mujoco_publish_rate:=100.0               # double
gt_enabled:=true | false                 # floating-base ground-truth odometry for visualization / ROS consumers
gt_body_frame:=pelvis                    # MuJoCo body published as ground truth
mpcFreq:=50                              # integer
mrtFreq:=1000                            # integer
libFolder:=auto_generated/g1             # CppAD codegen output
mujocoModelFile:=scene.xml               # swap scene: boxes / stairs / slope in description/g1/mujoco
```

Useful topics:

```bash
# Floating-base ground truth from mujoco_ros2_control (nav_msgs/Odometry).
# MPC reads the same pelvis body directly through ros2_control state interfaces.
ros2 topic echo /mujoco/ground_truth/odom

# Actuated joint states
ros2 topic echo /joint_states
```

## Configuration

No `.info` files are used. All whole-body MPC settings live in ROS 2 parameters:

- [`src/humanoid_wb_mpc/humanoid_wb_mpc_controller_parameter.yaml`](./src/humanoid_wb_mpc/humanoid_wb_mpc_controller_parameter.yaml) declares the full parameter surface (`generate_parameter_library`): model settings, foot-constraint gains, swing trajectory, SQP/rollout/MPC solver settings, initial state, `Q`/`R`/`Q_final` cost diagonals, task-space foot cost weights, and the friction-cone / contact-moment / joint-limit / foot-collision constraint parameters.
- [`config/g1/ros2_controllers.yaml`](./config/g1/ros2_controllers.yaml) holds the G1 values under `ocs2.*`. Joint-indexed arrays (costs, initial state, default joint state) are index-aligned with `robot.jointNames`.
- [`config/g1/gait.yaml`](./config/g1/gait.yaml) is the named gait library (mode sequence templates), referenced by `ocs2.gait.gaitFile`.
- [`config/g1/initial_pose.yaml`](./config/g1/initial_pose.yaml) sets the simulation start pose consumed by the `ros2_control` xacro.

The Params-to-config adapter ([`src/humanoid_wb_mpc/wb_mpc_config_builder.cpp`](./src/humanoid_wb_mpc/wb_mpc_config_builder.cpp))
maps these parameters onto the vendored MPC core (`WBMpcInterface::Config`).

## Floating-Base State

`mujoco_ros2_control` exposes the MuJoCo floating-base body (`pelvis`) through read-only `ros2_control` state interfaces under the sensor prefix `pelvis`. 
The MPC controller reads these state interfaces directly for its observation, so floating-base feedback does not depend on a ROS topic subscription.

The ground-truth odometry topic and TF are still published when `gt_enabled:=true`, but they are for RViz and other ROS consumers:

```bash
ros2 topic echo /mujoco/ground_truth/odom
```

The exported pelvis pose is represented in the world frame. The exported pelvis twist is represented in the pelvis/body-local frame and converted in the MPC controller before writing the OCS2 observation.

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
