# Legged Robot MPC Controller

ROS 2 controller integration for legged robot MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git). The Unitree G1 humanoid centroidal dynamics & whole-body MPC are migrated from [`wb_humanoid_mpc`](https://github.com/wei-hsuan-cheng/wb_humanoid_mpc.git).

Migration status and remaining milestones are documented in [`docs/humanoid_migration.md`](./docs/humanoid_migration.md).


## Build and Install

- Clone this repo
  ```bash
  git clone \
    https://github.com/wei-hsuan-cheng/legged_robot_mpc_controller.git \
    -b main
  ```

- Clone all sub-repo with vcs
  ```bash
  cd <workspace_dir>/src
  mkdir legged_robot_mpc_controller_dependencies
  vcs import < legged_robot_mpc_controller/legged_robot_mpc_controller.repos
  ```

- Install `pinocchio` library (**3.9.x required**; `packages.ros.org` only serves the newest
  build, which is `4.0.0` now, so install `3.9.0` from the ROS snapshot archive and hold it)
    ```bash
    # Import the ROS snapshot archive key and add the 2026-03-29 humble snapshot
    # (works on x86_64 and arm64; the arch is taken from dpkg)
    curl -s "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xAD19BAB3CBF125EA" | \
      sudo gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-snapshots-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-snapshots-archive-keyring.gpg] http://snapshots.ros.org/humble/2026-03-29/ubuntu jammy main" | \
      sudo tee /etc/apt/sources.list.d/ros2-snapshots.list
    sudo apt update

    # Install pinocchio 3.9.0 from the snapshot and pin it so a later apt upgrade
    # does not pull 4.x. The exact build id differs per architecture
    # (amd64: ...20260304.203533, arm64: ...20260307.163259), so resolve it from apt:
    PINOCCHIO_VERSION=$(apt-cache madison ros-humble-pinocchio | awk '/snapshots.ros.org/ {print $3; exit}')
    sudo apt install ros-humble-pinocchio=${PINOCCHIO_VERSION}
    sudo apt-mark hold ros-humble-pinocchio

    # Drop the snapshot source again afterwards
    sudo rm /etc/apt/sources.list.d/ros2-snapshots.list && sudo apt update

    # Check the installed version and the hold:
    # expect "Version: 3.9.0-..." and flag "hi" (h = held, i = installed)
    dpkg -s ros-humble-pinocchio | grep Version
    dpkg -l ros-humble-pinocchio | tail -1
    ```

- First install by `rosdep`
  ```bash
  # rosdep install
  cd <workspace_dir>
  sudo rosdep init # if you never did this
  rosdep update
  rosdep install --ignore-src --from-paths src -y -r
  ```

- Build pkgs up-to `legged_robot_mpc_controller`
  ```bash
  cd <workspace_dir>
  NUM_JOBS=2 && \
  export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
  export MAKEFLAGS=-j${NUM_JOBS} && \
  export NINJAFLAGS=-j${NUM_JOBS} && \
  colcon build --symlink-install \
    --packages-up-to legged_robot_mpc_controller \
    --executor sequential --parallel-workers ${NUM_JOBS} \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
  ```


## Run MuJoCo Example

Launch humanoid robot:
```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_centroidal_mpc_controller \
  mpcFreq:=100 \
  mrtFreq:=1000 \
  mujoco_headless:=true \
  velocityCommandGui:=true
```


### Base targets

Base twist command:
```bash
# Select twist tracking
ros2 topic pub --once /humanoid/target_mode \
  std_msgs/msg/String "{data: base_pose}"

# Publish twist command
ros2 topic pub -r 50 /humanoid/walking_velocity_command \
  ocs2_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.25, linear_velocity_y: 0.0,
    desired_pelvis_height: 0.7925, angular_velocity_z: 0.0}"
```

Base pose command:

```bash
# Select pose tracking
ros2 topic pub --once /humanoid/target_mode \
  std_msgs/msg/String "{data: base_pose}"

# Publish pose command
ros2 topic pub --once /humanoid/base_pose_command \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.7925},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
```


### Arm joint / frame-relation targets (`/humanoid/mpc_targets`)

| `command_type` | Meaning |
|---|---|
| `joint` | Track arm joint positions. One trajectory; states cover exactly the tracked arm joints (shoulder + elbow), ordered by `joint_names`. Overrides the built-in gait arm swing. |
| `frame_relation` | Track the relative pose of a leaf frame expressed in a reference frame. Convention (matching `mpc_controllers`): `source_frames[i]` is the reference (root) frame — a robot frame such as `pelvis`, or a global frame (`world`) — and `target_frames[i]` is the tracked leaf frame (a hand). One trajectory per pair; states are `[position xyz, quaternion xyzw]` of target expressed in source. Optional `frame_relation_tracking_weights` (6 per pair: position xyz, orientation xyz) override the configured defaults. |
| `joint_frame_relation` | Both at once: the joint trajectory first in `target_trajectories`, then one per frame pair. |
| `default` | Clear all external targets and revert to the built-in posture + gait arm swing. |


Sample target publishers in [`launch/command/mpc_targets/`](./launch/command/mpc_targets/):

```bash
# Sine arm-joint swing (command_type: joint)
ros2 run legged_robot_mpc_controller joint_tracking_target.py

# Hold both hands at pelvis-relative poses (command_type: frame_relation)
ros2 run legged_robot_mpc_controller frame_relation_tracking_target.py

# Fixed arm posture + left-hand pose target together (command_type: joint_frame_relation)
ros2 run legged_robot_mpc_controller joint_frame_relation_tracking_target.py

# Switch back to the built-in arm-swing reference
ros2 topic pub --once /humanoid/mpc_targets ocs2_msgs/msg/MpcTargets "{command_type: 'default'}"
```

Minimal one-shot CLI examples:

```bash
# Hold both arms at a raised posture (command_type: joint)
ros2 topic pub --once /humanoid/mpc_targets ocs2_msgs/msg/MpcTargets "{
  command_type: 'joint',
  joint_names: [left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint, left_elbow_joint,
                right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_joint],
  target_trajectories: [{time_trajectory: [0.0],
                         state_trajectory: [{value: [0.3, 0.0, 0.0, 0.6, 0.3, 0.0, 0.0, 0.6]}],
                         input_trajectory: [{value: []}]}]}"

# Move the left hand to a pelvis-relative pose (command_type: frame_relation;
# source = reference frame, target = tracked leaf frame)
ros2 topic pub --once /humanoid/mpc_targets ocs2_msgs/msg/MpcTargets "{
  command_type: 'frame_relation',
  source_frames: [pelvis],
  target_frames: [left_rubber_hand],
  target_trajectories: [{time_trajectory: [0.0],
                         state_trajectory: [{value: [0.32, 0.15, 0.20, 0.0, 0.0, 0.0, 1.0]}],
                         input_trajectory: [{value: []}]}]}"
```

Both channels are soft costs balanced against the rest of the MPC; raise the command weights to track tighter. `frame_relation` is currently registered for the centroidal controller only.

Useful launch args:

```bash
velocityCommandGui:=true | false
spawnMpcController:=true | false         # false is only for environment smoke tests; the robot will not balance
mpcControllerName:=humanoid_centroidal_mpc_controller | humanoid_wb_mpc_controller
use_mujoco_sim:=true | false             # false: plain ros2_control_node (fake hardware)
use_fake_hardware:=false | true          # mock_components/GenericSystem when not using MuJoCo
ros2ControlCommandInterface:=effort | effort_pd | position
mujocoEffortCommandMode:=actuator | qfrc_applied
initialPoseFile:=<...>/initial_pose.yaml # initial joint state (config/g1/initial_pose.yaml)
rviz:=true | false
mujoco_headless:=true | false
mujoco_wait_to_start:=true | false       # paused start + /mujoco_ros2_control/start service
mujoco_real_time_factor:=1.0             # double
mujoco_publish_rate:=100.0               # double
gt_enabled:=true | false                 # floating-base ground-truth odometry for visualization / ROS consumers
gt_body_frame:=pelvis                    # MuJoCo body published as ground truth
mpcFreq:=100                             # integer
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

- [`config/g1/gait.yaml`](./config/g1/gait.yaml) is the named gait library (mode sequence templates), referenced by `ocs2.gait.gaitLibraryFile` and shared by both controllers.
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
