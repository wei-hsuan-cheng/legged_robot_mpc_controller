# Legged Robot MPC Controller

ROS 2 controller integration for legged robot MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git). The Unitree G1 humanoid centroidal dynamics & whole-body MPC are migrated from [`wb_humanoid_mpc`](https://github.com/wei-hsuan-cheng/wb_humanoid_mpc.git).

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

```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_centroidal_mpc_controller \
  mpcFreq:=80 \
  mrtFreq:=1000 \
  mujoco_headless:=true \
  velocityCommandGui:=true
```

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

All MPC settings live in ROS 2 parameters:

| Controller | Parameter declaration | Config adapter | Interface |
|---|---|---|---|
| `humanoid_wb_mpc_controller` | [`src/humanoid_wb_mpc/humanoid_wb_mpc_controller_parameter.yaml`](./src/humanoid_wb_mpc/humanoid_wb_mpc_controller_parameter.yaml) | [`src/humanoid_wb_mpc/wb_mpc_config_builder.cpp`](./src/humanoid_wb_mpc/wb_mpc_config_builder.cpp) | `WBMpcInterface::Config` |
| `humanoid_centroidal_mpc_controller` | [`src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller_parameter.yaml`](./src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller_parameter.yaml) | [`src/humanoid_centroidal_mpc/centroidal_mpc_config_builder.cpp`](./src/humanoid_centroidal_mpc/centroidal_mpc_config_builder.cpp) | `CentroidalMpcInterface::Config` |

Both declare model settings, foot-constraint gains, swing trajectory, SQP/rollout/MPC solver settings, initial state, `Q`/`R`/`Q_final` cost diagonals, task-space foot cost weights, and the friction-cone / contact-moment / joint-limit / foot-collision constraint parameters. 

The whole-body state is:
- `[base pose, joint positions, base velocity, joint velocities]` with joint accelerations + contact wrenches as inputs.

The centroidal state is:
- `[normalized centroidal momentum, base pose, joint positions]` with joint velocities + contact wrenches as inputs.
- Additional centroidal-only costs (ICP, torso task-space tracking via `costs.taskSpaceCosts`, leg external-torque costs via `costs.legTorqueCost`). 

Loaders shared by both controllers (gait map, reference config, cost-matrix assembly) live in [`common/config/config_builder_utils.hpp`](./include/legged_robot_mpc_controller/common/config/config_builder_utils.hpp).

Robot-specific values:

- [`config/g1/gait.yaml`](./config/g1/gait.yaml) is the named gait library (mode sequence templates), referenced by `ocs2.gait.gaitFile` and shared by both controllers.
- [`config/g1/initial_pose.yaml`](./config/g1/initial_pose.yaml) sets the simulation start pose consumed by the `ros2_control` xacro.


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
