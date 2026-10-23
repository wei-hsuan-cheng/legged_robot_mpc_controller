# Legged Robot MPC Controller

[ROS 2 controller](https://control.ros.org/rolling/index.html) integration for legged robot MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git). Tested in [MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html) simulation environment. The codes are successfully tested in [`ros2 jazzy`](https://docs.ros.org/en/jazzy/index.html).


### Interactive Velocity and Base Height Control via GUI & Joystick

<p align="center">
  <img src="https://github.com/wei-hsuan-cheng/gifs/blob/main/legged_robot_mpc_controller/g1.gif?raw=true" width="400" />
</p>


## MPC Formulations

This repository contains examples for two MPC formulations: **centroidal dynamics (hierarchical) vs. whole-body inverse dynamics**.

### Centroidal Dynamics MPC
The centroidal MPC optimizes over the **whole-body kinematics** and the center of mass dynamics, with a choice to either use a single rigid body model or the full centroidal dynamics. This specific approach builds up on the centroidal model in ocs2 by generalizing costs and constraints to a 6-DoF contact among others. A conscise explanation of the ocs2 centroidal model can be found in [Sleiman et. al., A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation](https://arxiv.org/abs/2103.00946).

### Whole-Body Inverse Dynamics MPC
The **whole-body inverse dynamics MPC** jointly optimizes floating-base and joint configurations and velocities, contact wrenches, generalized accelerations, and joint torques. Full rigid-body dynamics are imposed through the recursive Newton-Euler algorithm (RNEA): floating-base dynamics are enforced throughout the prediction horizon, while actuated-joint inverse dynamics and torque limits are enforced within a configurable initial torque window. The implementation adapts the formulations in [Khazoom et al., Tailoring Solution Accuracy for Fast Whole-body Model Predictive Control of Legged Robots](https://arxiv.org/abs/2407.10789) and [Molnar et al., Whole-Body Inverse Dynamics MPC for Legged Loco-Manipulation](https://arxiv.org/abs/2511.19709) to OCS2's SQP solver and the G1 humanoid.


## Build and Install

- Clone this repo
  ```bash
  git clone https://github.com/wei-hsuan-cheng/legged_robot_mpc_controller.git
  ```

- Clone all sub-repo with vcs
  ```bash
  cd <workspace_dir>/src
  mkdir legged_robot_mpc_controller_dependencies
  vcs import < legged_robot_mpc_controller/legged_robot_mpc_controller.repos
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

Two MPC controllers are available, selected with `mpcControllerName`. Both walk with the same gait, references, estimator and bring-up; they differ in the model the MPC plans with.

| `mpcControllerName` | MPC | state / input |
|---|---|---|
| `humanoid_centroidal_mpc_controller` (default) | centroidal dynamics + full kinematics | `[h/m, q]` / `[W, qd_j]` |
| `humanoid_whole_body_inverse_dynamics_mpc_controller` | whole-body inverse dynamics (see [below](#whole-body-inverse-dynamics-mpc)) | `[q, v]` / `[W, a, tau_j]` |

**Launch the humanoid robot (centroidal MPC)**:
```bash
cd <workspace_dir>
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_centroidal_mpc_controller \
  mpcFreq:=100 \
  mrtFreq:=1000 \
  rviz:=true \
  mujoco_headless:=false \
  baseCommandGui:=true
```

**Launch the humanoid robot (whole-body inverse-dynamics MPC)**:
```bash
cd <workspace_dir>
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller \
  mpcFreq:=90 \
  mrtFreq:=1000 \
  rviz:=true \
  mujoco_headless:=false \
  baseCommandGui:=true
```

The first launch of each controller generates and compiles its CppAD libraries (several minutes); later launches load them from the cache. Everything else in this README (GUI, scripted sequences, base targets, estimator, bring-up services) works the same with either controller; services and parameters live under the controller's own node name, e.g. `/humanoid_whole_body_inverse_dynamics_mpc_controller/stand_up`.


### Scripted trajectory tests (figure-eight, flat ground and ramp)

`launch/figure_eight_command.py` walks a closed figure-eight (Gerono lemniscate)
at **constant speed** with a **sinusoidal pelvis height**, publishing the same
`WalkingVelocityCommand` topic the GUI uses. It exercises forward speed, both turn
directions and the curvature reversal at the crossing continuously, without ever
leaving a small patch of floor.

Three things will bite you if you skip them:

- **Pick a scene without the stairs.** `scene.xml` puts a 0.10 m riser at
  `x = 0.75 m`; the default figure-eight spans `x` in `[-2, +2]` and would walk
  straight into it. Use `scene_flat.xml` (flat) or `scene_ramp.xml` (3 deg ramp).
- **Turn the command GUI off.** It publishes its slider values to the *same
  topic* at 50 Hz, so leaving it up means its zeros fight the script.
- **Launch from the workspace root.** `libFolder` is a relative path
  (`auto_generated/g1`, with `centroidal_mpc_g1/` and `wb_id_mpc_g1/` inside);
  from anywhere else the cached CppAD libraries are not found and the model is
  regenerated.

**Flat ground:**
```bash
# Terminal 1
cd <workspace_dir>
source install/setup.bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  rviz:=true \
  mujoco_headless:=false \
  mujocoModelFile:=scene_flat.xml \
  baseCommandGui:=false \
  diagnosticsLog:=true diagnosticsLogPrefix:=/tmp/fig8_%t

# Terminal 2
cd <workspace_dir>
source install/setup.bash
ros2 run legged_robot_mpc_controller figure_eight_command.py --dry-run   # check the envelope
ros2 run legged_robot_mpc_controller figure_eight_command.py
```

`--dry-run` prints path length, lap time, peak/mean yaw rate and the tightest turn
radius, and warns when the geometry leaves the measured-safe envelope
(`vx <= 0.30 m/s`, `|omega| <= 0.5 rad/s`). That is how the defaults were chosen:
`--lx 4.0 --ly 1.6 --speed 0.22` gives a peak yaw rate of 0.466 rad/s. Geometry,
speed, laps and all height parameters are arguments:

```bash
ros2 run legged_robot_mpc_controller figure_eight_command.py \
  --lx 4.0 --ly 1.6 --speed 0.22 --laps 2 \
  --zmin 0.77 --zmax 0.80 --zperiod 13.0
```

**Ramp (3 degree slope, terrain test):**
```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mujocoModelFile:=scene_ramp.xml \
  baseCommandGui:=false
```
`scene_ramp.xml` is flat until `x = 1.5 m`, then rises 3 degrees to `z = 0.21 m`
at `x = 5.49 m`. Use a forward walk rather than the figure-eight for this, since
the lemniscate stays within `x <= 2` and would barely touch the slope:

```bash
ros2 run legged_robot_mpc_controller command_sequence.py --sequence walk_far
```

To have the swing planner actually follow the slope instead of assuming a flat
floor, set `ocs2.model.useTerrainHeightEstimate: true` — see
**Configuration** below for the caveat, it is off by default for a reason.

**Analysing a run:**
```bash
ros2 run legged_robot_mpc_controller analyze_diagnostics.py /tmp/fig8_<stamp> --timeline 5
```
Reports estimator-vs-ground-truth error, filter consistency (NIS), contact
bookkeeping, per-joint jitter, solver health and the per-cost-term breakdown, and
locates the loss of balance if there was one.

### Base targets

**Base twist command**:
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

**Base pose command**:

> WORK IN PROGRESS

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


### Fixed-sequence stair climbing (`stair_climb` target mode) (*centroidal MPC only*)

The centroidal MPC provides an example of climbing a staircase with **known ground-truth geometry** using a pre-compiled, fixed sequence: gait (mode schedule), foothold placements, swing lift-off/touch-down heights, and a pelvis reference (zero pitch/roll) are all generated once from [`config/g1/terrain/stair_climbing/*.yaml`](./config/g1/terrain/stair_climbing/) at trigger time. No perception / plane segmentation is involved.

**Run example**:

```bash
# Launch
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_centroidal_mpc_controller \
  mpcFreq:=100 \
  mrtFreq:=1000 \
  rviz:=true \
  mujoco_headless:=false \
  baseCommandGui:=false

# Trigger the target mode
ros2 topic pub --once /humanoid/target_mode std_msgs/msg/String "{data: terrain_walk}"

# Then the stair climbing motion starts automatically
```

**Auto-test**:
[`tests/stair_climbing_test.sh`](./tests/stair_climbing_test.sh) launches the simulation headless, triggers the climb, monitors the pelvis ground-truth odometry and both foot TF frames, and prints one of `VERDICT: SUCCESS | INCOMPLETE | FALL | NO_ODOM` (exit code 0 only on `SUCCESS`, so it can gate CI):

```bash
# Workspace sourced; runs from anywhere (cds to the workspace root internally
# so the CppAD cache in auto_generated/ is found)
ros2 run legged_robot_mpc_controller stair_climbing_test.sh
# or with explicit args / from the source tree:
./tests/stair_climbing_test.sh /tmp/stair_climbing_test.log 45 90   # [log] [monitor_s] [startup_wait_s]
```


### Terrain-aware walking (`terrain_walk` target mode) (*centroidal MPC only*)

Perception-free *online* terrain locomotion:
Instead of a pre-scripted sequence, the robot follows a plain **velocity command** while a `TerrainFootholdPlanner` selects footholds each solver cycle over the same ground-truth staircase geometry (implementation of the [T-RO 2023 perceptive-locomotion](https://arxiv.org/abs/2208.08373) pipeline, without the elevation-map / plane-segmentation integrated).

Each cycle it:
1. Extrapolates a nominal foothold under the hip (Raibert heuristic + capture-point velocity feedback)
2. Projects it onto the terrain surface, prefers stepping **up** onto a reachable tread (step-up bonus)
3. Anchors the stance laterally to the stair centerline
4. Feeds the per-phase support heights to the swing planner, and terrain-adapts the pelvis height (zero pitch/roll) while gating forward momentum so the CoM cannot overrun the feet at a riser.

**Run example**:

```bash
# Launch
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_centroidal_mpc_controller \
  mpcFreq:=100 \
  mrtFreq:=1000 \
  rviz:=true \
  mujoco_headless:=false \
  baseCommandGui:=true

# Trigger the target mode
ros2 topic pub --once /humanoid/target_mode std_msgs/msg/String "{data: terrain_walk}"

# Then send base twist command to play with the robot
```

**Auto-test** (headless, drives the climb and prints a verdict; exit 0 only on `SUCCESS`):

```bash
VX=0.08 ros2 run legged_robot_mpc_controller terrain_walk_test.sh /tmp/terrain_walk.log 90
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
baseCommandGui:=true | false
spawnMpcController:=true | false         # false is only for environment smoke tests; the robot will not balance
mpcControllerName:=humanoid_centroidal_mpc_controller | humanoid_whole_body_inverse_dynamics_mpc_controller
use_mujoco_sim:=true | false             # false: plain ros2_control_node (fake hardware)
use_fake_hardware:=false | true          # mock_components/GenericSystem when not using MuJoCo
ros2ControlCommandInterface:=effort | effort_pd | position
mujocoEffortCommandMode:=actuator | qfrc_applied
initialPoseFile:=<...>/initial_pose.yaml # initial joint state (config/g1/initial_pose.yaml)
rviz:=true | false
mujoco_headless:=false | true
mujoco_wait_to_start:=true | false       # paused start + /mujoco_ros2_control/start service
mujoco_real_time_factor:=1.0             # double
mujoco_publish_rate:=100.0               # double
gt_enabled:=true | false                 # floating-base ground-truth odometry for visualization / ROS consumers
gt_body_frame:=pelvis                    # MuJoCo body published as ground truth
mpcFreq:=100                             # integer
mrtFreq:=1000                            # integer
libFolder:=auto_generated/g1             # CppAD codegen output (centroidal_mpc_g1/, wb_id_mpc_g1/ inside)
mujocoModelFile:=scene.xml               # swap scene: boxes / stairs / slope in description/g1/mujoco
stairClimbingFile:=stair_climbing.yaml
terrainWalkingFile:terrain_walking.yaml
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
| `humanoid_centroidal_mpc_controller` | [`src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller_parameter.yaml`](./src/humanoid_centroidal_mpc/humanoid_centroidal_mpc_controller_parameter.yaml) | [`src/humanoid_centroidal_mpc/centroidal_mpc_config_builder.cpp`](./src/humanoid_centroidal_mpc/centroidal_mpc_config_builder.cpp) | `CentroidalMpcInterface::Config` |
| `humanoid_whole_body_inverse_dynamics_mpc_controller` | standalone [`src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller_parameter.yaml`](./src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller_parameter.yaml) | [`src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp`](./src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp) | `WbIdMpcInterface::Config` |

It declares model settings, foot-constraint gains, swing trajectory, SQP/rollout/MPC solver settings, `Q`/`R`/`Q_final` cost diagonals, task-space foot cost weights, and the friction-cone / contact-moment / joint-limit / foot-collision constraint parameters. 

The centroidal state is:
- `[normalized centroidal momentum, base pose, joint positions]` with joint velocities + contact wrenches as inputs.
- Additional centroidal-only costs (ICP, torso task-space tracking via `costs.taskSpaceCosts`, leg external-torque costs via `costs.legTorqueCost`). 

Shared loaders (gait map, reference config, cost-matrix assembly), kept generic so a future MPC formulation can reuse them, live in [`common/config/config_builder_utils.hpp`](./include/legged_robot_mpc_controller/common/config/config_builder_utils.hpp).

Robot-specific values:

- `ocs2.gait` in each controller's block of [`config/g1/ros2_controllers.yaml`](./config/g1/ros2_controllers.yaml) holds the gait templates (`templates`), the velocity-based gait ladder (`transitions`) and the swing-foot trajectory (`swingTrajectory`), as ROS parameters.
- [`config/g1/initial_pose.yaml`](./config/g1/initial_pose.yaml) sets the simulation start pose consumed by the `ros2_control` xacro.


### Terrain height (`ocs2.model.useTerrainHeightEstimate`)

`adaptToCurrentGroundHeight()` computes a ground-height estimate from the stance
feet and historically discarded it (`terrainHeight = 0.0`). That constant is what
the swing trajectory is built around, so it hard-codes a flat floor at the world
origin — the single thing preventing this stack from walking on sloped or stepped
ground, however well the estimator tracks the terrain.

Setting `useTerrainHeightEstimate: true` uses the estimate instead, rate-limited
by `maxTerrainHeightStep` per solver iteration. Pair it with a
`stateEstimator.height.source` that does not itself assume a plane — `inekf` or
`anchored`. `kinematic` and `blend` pin the stance feet to `groundZ` and would
feed the flat assumption straight back in.

**Off by default, and not yet trustworthy.** On the 3 degree ramp it more than
doubles survival (40.7 s → 94.1 s) and halves peak pitch (0.50 → 0.22 rad), but
the reported height drifts: `|e_z|` reaches 0.39 m on flat ground and 0.13 m on
the ramp. The two are the same effect — tightening `maxTerrainHeightStep` removes
the drift and the benefit together (0.01 → 149 s walked / 0.39 m drift;
0.002 → 68 s / 0.079 m; 0.0005 → 64 s / 0.050 m). The gain is largely the
base-height target being allowed to run away with the drifting terrain reference,
not terrain adaptation.

The loop is: with `anchored`, touchdown anchors are computed from the filter's own
height, so enabling the terrain reference lets filter drift feed the anchors,
which feed the reference. `inekf` (no blend) shows it far less — `|e_z|` 0.053
instead of 0.390 under the same setting. Bounding this is the open problem before
terrain walking can be relied on.

### InEKF height and contact sources

Measured on the figure-eight, flat ground, survival time:

| `height.source` | `contact.source: scheduled` | `contact.source: torque` |
|---|---|---|
| `kinematic` | **127.8 s** | 12.7 s |
| `blend` | 60.3 s | 8.2 s |
| `anchored` | 43.2 s | 6.6 s |
| `inekf` | 26.3 s | 7.4 s |

`torque` contact detection collapses everything to 6–13 s. `ContactEstimator`
assumes contact *i* is served by joints *3i…3i+2* — a 12-DoF quadruped layout that
maps G1's heel/toe onto the wrong joints entirely. This is structural, not a
`beta0`/`beta1` tuning problem; use `scheduled`.

`kinematic` wins on flat ground because it pins the feet to the *same* plane the
controller assumes while `useTerrainHeightEstimate` is off — that is consistency,
not terrain generality, which is exactly why terrain support needs both changes
together.

## Whole-Body Inverse-Dynamics MPC

`humanoid_whole_body_inverse_dynamics_mpc_controller` plans with the full rigid-body dynamics instead of the centroidal model, following

- L. Molnar et al., [*Whole-Body Inverse Dynamics MPC for Legged Loco-Manipulation*](https://arxiv.org/abs/2511.19709) (code: [`wb-mpc-locoman`](https://github.com/lukasmolnar/wb-mpc-locoman))
- C. Khazoom et al., [*Tailoring Solution Accuracy for Fast Whole-body Model Predictive Control of Legged Robots*](https://arxiv.org/abs/2407.10789), RA-L 2024 (MIT Humanoid)

in OCS2 (SQP, multiple shooting, HPIPM). On the G1 (23 MPC joints, `nv = 29`):

```
x = [q, v]                     58   q: base position + ZYX Euler, joints;  v = dq/dt
u = [W_l, W_r, a, tau_j]       64   contact wrenches, generalized acceleration, joint torques
xdot = [v; a]                       (a double integrator: the dynamics are a constraint)
```

| constraint | rows | where | |
|---|---|---|---|
| RNEA base rows `[RNEA(q,v,a) - J^T W]_base = 0` | 6 | every node | Newton-Euler (centroidal) dynamics |
| RNEA joint rows `[RNEA(q,v,a) - J^T W]_joints = tau` | 23 | first `ocs2.model.torqueConstraintNodes` nodes | defines the torque that is sent |
| `-tau_max <= tau <= tau_max` from `control.torqueLimit` (soft) | 46 | same nodes | |
| stance: `a_c + Kd v_c + Kp [z - z_ref; tilt] = 0` | 6 per foot | in contact | `Kd = 1/dt`: `v_c = 0` at the next node |
| swing: vertical spline tracking at acceleration level, `W = 0` | 1 + 6 per foot | in swing | |
| joint velocity limits `\|qd_j\| <= scale * URDF limit` (soft) | 46 | every node | ETH; the joint velocities are states here |
| friction cone, CoP (soft), joint limits, foot collision (soft) | | | as in the centroidal MPC |

Two adaptations of the papers to OCS2:

- **Torque rows only on the first nodes** (ETH `tau_nodes = 3`, MIT 2 steps). Only the first torques reach the robot; the base rows shape the motion over the rest of the horizon. The window must also cover the MPC latency: the controller sends the policy's `tau` while it is inside the window and falls back to the plan's inverse dynamics if a slow solve leaves it outside (logged as a torque-window warning).
- **Contact at acceleration level.** ETH imposes `v_c = 0` on the state directly, but OCS2's SQP hands only state-input equalities to the QP (a pure state equality is transcribed but never enforced). With the exact double-integrator discretization, `a_c + v_c/dt = 0` at node `k` is `v_c = 0` at node `k+1`, so `ocs2.model.footConstraint` holds acceleration-level gains in this controller's block (velocity gains `1/dt`, non-zero acceleration gains).

Model-independent components remain shared: gait FSM, swing planner, base twist/pose commands, arm posture tracking, foot/torso task-space costs, leg external-load cost, estimator interfaces and bring-up. Capture-point foot placement is disabled for WB-ID; flat-ground horizontal footholds are chosen by the optimizer. The inner loop uses `q_des`, `qd_des` from the planned state and `tau_ff = tau` from the planned input with the `effort_pd` joint servo. `control.feedforward: inverse_dynamics` instead evaluates RNEA at the measured state with the plan's accelerations and wrenches.

**Run it:**

```bash
cd <workspace_dir>
# estimator-driven (default floatingBaseSource), flat scene, GUI
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller \
  floatingBaseEstimator:=humanoid_inekf \
  mujocoModelFile:=scene_flat.xml

# ground-truth base, scripted
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller \
  floatingBaseSource:=ground_truth_state floatingBaseEstimator:=none \
  mujocoModelFile:=scene_flat.xml baseCommandGui:=false
ros2 run legged_robot_mpc_controller command_sequence.py --sequence robustness --settle-before 0
```

**CppAD cache:** `auto_generated/g1/wb_id_mpc_g1/` (21 libraries, generated on the first launch). The `wb_id_mpc_` prefix keeps them apart from the centroidal libraries, several of which have the same model names but other dimensions.

**Configuration:** the `humanoid_whole_body_inverse_dynamics_mpc_controller` block of [`config/g1/ros2_controllers.yaml`](./config/g1/ros2_controllers.yaml) has its own generated schema and builder. It does not load the centroidal schema. All state/input weights live in a single `ocs2.costs` section; the former `wholeBody.*` parameter names are removed, not aliases.

The WB-ID block's `ocs2.gait.templates` uses 0.40 s single support and 0.15 s double support in `slow_walk`; the centroidal block keeps the original timing.
The WB-ID nominal joint posture matches the default 0.60 m crouch so estimator bring-up does not hand MPC a near-straight stance with a sudden height change.
See [`Codex.md`](./Codex.md) for the flat-ground tuning results and remaining yaw/speed errors; other heights and terrain are not validated by these tests.
The latest standalone-parameter/capture-disabled checkpoint is **not built or walking-tested**; earlier results used the capture heuristic and must not be attributed to this checkpoint.
GUI full-scale commands now match the selected YAML limits: WB-ID vx 0.25 m/s, vy 0.15 m/s, yaw 0.15 rad/s, with controller-side slew limits.

| key | meaning |
|---|---|
| `ocs2.model.torqueConstraintNodes` | nodes carrying the joint-torque rows and torque limits (default 3; `<= 0`: all) |
| `ocs2.constraints.jointTorqueLimits.{mu,delta}` | relaxed barrier of the torque limits |
| `ocs2.constraints.jointVelocityLimits.{scale,mu,delta}` | joint velocity limits as a fraction of the URDF ones (`scale <= 0`: off) |
| `ocs2.costs.q/qFinal.{basePose,baseVelocity,jointPositions,jointVelocities}` | complete state cost; pose `[x,y,z,yaw,pitch,roll]`, velocity `[vx,vy,vz,yaw_rate,pitch_rate,roll_rate]` |
| `ocs2.costs.r.{contactWrenches,baseAccelerations,jointAccelerations,jointTorques}` | complete input weights (times `ocs2.costs.rScaling`) |
| `ocs2.reference.velocityCommandRateLimits` | physical command slew limits `[ax,ay,height_rate,yaw_acceleration]`; zero disables an axis |
| `control.feedforward` | `policy_torque` (default) or `inverse_dynamics` |

The base uses Translation + SphericalZYX coordinates: its angular velocity state entries are Euler rates, not a spatial angular-velocity vector. This cleanup does not change that representation to a quaternion/Lie-group model. Task-space and leg-load costs are active G1 adaptations beyond the papers' state/input objective. Contact inequalities remain soft barriers, unlike the papers' explicit inequalities; this is not an exact reproduction. The ICP cost, frame-relation tracking, WBC inner loop, and diagnostics CSV are not exposed by the WB-ID schema. Stair climbing and terrain walking are untested with it.

**Code:** core in [`include/humanoid_wb_id_mpc`](./include/humanoid_wb_id_mpc) / [`src/core/humanoid_wb_id_mpc`](./src/core/humanoid_wb_id_mpc) (`WbIdMpcInterface`, `WbIdMpcRobotModel`, `WbIdDynamics`, `WbIdRneaConstraintCppAd`, `WbIdFootAccelerationConstraintCppAd`, `WbIdJointTorqueLimitConstraint`, `WbIdTorqueWindow`, `WbIdInitializer`, `WbIdMpcTargetTrajectoriesCalculator`); controller in [`src/humanoid_whole_body_inverse_dynamics_mpc`](./src/humanoid_whole_body_inverse_dynamics_mpc).

## Robot Description and Frames

There is one robot URDF, [`description/g1/urdf/g1_29dof.urdf`](./description/g1/urdf/g1_29dof.urdf), rooted at the pelvis with **no `world` link and no floating joint**. The MPC and the estimators add the floating base themselves (Pinocchio), MuJoCo owns its free joint in the MJCF, and the base pose reaches the TF tree from the estimator. [`g1.ros2_control.xacro`](./description/g1/urdf/g1.ros2_control.xacro) includes that URDF and adds the `<ros2_control>` block; the result is the single `/robot_description`, used by the controller manager and robot_state_publisher.

The environment is not part of the robot: [`stairs.urdf.xacro`](./description/g1/urdf/stairs.urdf.xacro), rooted at `world` and driven by the same `stairs_params.yaml` as the MuJoCo stairs, is published by a second robot_state_publisher as `/environment/robot_description` (the "Environment" display in rviz). Pass `environmentDescription:=""` with a scene that has no stairs, e.g. `scene_flat.xml`.

```
world ──(static, identity)──▶ odom ──(estimator, odom → pelvis)──▶ pelvis ──(robot_state_publisher)──▶ links
  ├──(environment robot_state_publisher)──▶ stairs_base ──▶ stairs_step_1 … N
  └──(simulator ground truth)──▶ gt/pelvis
```

- `odom → pelvis` is the estimate, published as TF by the estimator controller next to `/humanoid/state_estimate/odom`. It appears once the bring-up initialises the estimator.
- `world → odom` is identity until something localises the robot, so estimator drift (x, y, yaw) shows as the robot moving relative to the stairs. A localisation source - ground truth in sim, lidar or markers on hardware - would replace the static transform.
- Ground truth is `world → gt/pelvis`, for comparison. With `floatingBaseEstimator:=none` nothing estimates, and ground truth publishes `world → pelvis` instead.

## Floating-Base State (ground truth vs. state estimator)

The floating-base feedback is chosen with two launch arguments: **`floatingBaseSource`** (what drives the MPC) and **`floatingBaseEstimator`** (which filter runs).

```bash
# (default) the estimator drives the MPC: IMU + joint encoders + scheduled contacts
ros2 launch legged_robot_mpc_controller g1.launch.py \
  floatingBaseSource:=state_estimator \
  floatingBaseEstimator:=humanoid_inekf         # or humanoid_linear_kf

# simulator/hardware body state drives the MPC; the estimator still runs alongside
ros2 launch legged_robot_mpc_controller g1.launch.py \
  floatingBaseSource:=ground_truth_state
```

| `floatingBaseSource` | Feedback used by the MPC |
|---|---|
| `state_estimator` *(default)* | The estimate of the estimator controller named by `floatingBaseEstimator`. Nothing reads ground truth: the filter initialises itself from stance during the bring-up below, and its estimate drives control from then on. |
| `ground_truth_state` | Body pose/twist from `ros2_control` state interfaces (MuJoCo ground truth in simulation; the robot's own base state on hardware). |

| `floatingBaseEstimator` | Filter | Tuning |
|---|---|---|
| `humanoid_inekf` *(default)* | contact-aided invariant EKF (Hartley et al. 2018) | `humanoid_inekf:` in [`ros2_controllers.yaml`](./config/g1/ros2_controllers.yaml) |
| `humanoid_linear_kf` | two-stage linear KF (Bledt et al. 2018); falls on varied commands, kept for comparison | `humanoid_linear_kf:` in [`ros2_controllers.yaml`](./config/g1/ros2_controllers.yaml) |
| `none` | — (only valid with `ground_truth_state`) | |

Each estimator is its **own ros2_control controller** (`HumanoidInEkfController` / `HumanoidLinearKfController`), loaded and activated before the MPC:

```
                    exported state interfaces (MPC reads)
  humanoid_inekf  ---------------------------------------------------->  humanoid_centroidal_mpc_controller
  joints + IMU in   <estimator>/position.*, orientation.*,                  claims both sets; the estimator
                    linear_velocity_world.*, angular_velocity.*, valid,     claims nothing of the MPC's
                    status.{ready,epoch,running_time}, diagnostics.*

                  <----------------------------------------------------
                    exported reference interfaces (MPC writes, as command interfaces)
                    <estimator>/contact.left, contact.right   1 = stance, 0 = swing
                    <estimator>/initialize_epoch              "initialise from stance now"
```

Both directions are ros2_control interfaces exported by the estimator, and only the MPC claims them, so the chain has no cycle. (A cycle - each controller claiming the other's interfaces - is what the controller manager cannot activate: its activation check recurses without end.) Claiming the reference interfaces puts the estimator in chained mode; the controller manager restarts it once to do that when the MPC activates.

The controller manager runs the two one after the other in each tick, so one direction is a tick (1 ms at 1 kHz) late. The MPC waits until the exported `status.epoch` equals the epoch it wrote before it trusts `status.ready`.

```bash
ros2 control list_controllers
# joint_state_broadcaster            active
# humanoid_inekf                     active
# humanoid_centroidal_mpc_controller active
```

With `ground_truth_state` the estimator still runs (the MPC writes an epoch at activation), which is the way to compare it against ground truth without risking the robot. The diagnostics log carries the estimate and the filter's scalar internals (covariance std-devs, NIS, bias, height). The per-contact innovation columns are not exported, because their names depend on the estimator's contact frames. Either way it publishes its estimate for evaluation:

```bash
ros2 topic echo /mujoco/ground_truth/odom        # ground truth (also used for RViz/TF)
ros2 topic echo /humanoid/state_estimate/odom    # the running estimator
```

### Bring-up (no ground truth)

With `floatingBaseSource:=state_estimator` the controller never uses ground truth, so it can run unchanged on hardware. It follows MIT Cheetah's `Passive → StandUp → Locomotion`, adapted to a humanoid that starts crouched with its feet flat:

| phase | command | estimator | leaves when |
|---|---|---|---|
| Passive | joint damping only | off | `~/stand_up` (or `bringUp.autoStart`) |
| StandUp | joint PD to `reference.defaultJointState`, gravity compensation from the IMU attitude | off | interpolation done (`standUpSeconds`) |
| Settle | hold the nominal pose | off | trunk at rest per the IMU for `settleSeconds`, **and** `~/start_locomotion` (or `autoStart`) |
| Converge | hold | initialised from stance (IMU attitude, joint encoders), then running | `readiness.minSeconds` elapsed and divergence guards pass |
| StartingMpc | hold | running | first policy (solver built off the RT thread, seeded from the estimate) |
| Blend → Locomotion | stand → MPC over `blendSeconds`, then MPC only | running | — |
| Fault | joint damping, latched | — | trunk tilt past `bringUp.tiltLimit` in any phase |

In simulation `bringUp.autoStart: true` runs it end to end (about 10 s). The robot spawns **crouched** by default (`config/g1/initial_pose_crouch.yaml`), as a real robot would start, so every launch exercises the stand-up. For a standing spawn, which `floatingBaseSource:=ground_truth_state` needs because it has no stand-up:

```bash
ros2 launch legged_robot_mpc_controller g1.launch.py mujocoModelFile:=scene_flat.xml \
  initialPoseFile:=$(ros2 pkg prefix legged_robot_mpc_controller)/share/legged_robot_mpc_controller/config/g1/initial_pose.yaml
```

#### Manual bring-up (`bringUp.autoStart: false`)

This is the procedure for the real robot, and the way to rehearse it in simulation. Set in `config/g1/ros2_controllers.yaml`:

```yaml
humanoid_centroidal_mpc_controller:
  ros__parameters:
    floatingBase:
      stateInterfaceName: ""      # hardware only: the robot exports no body pose to claim
    bringUp:
      autoStart: false
```

The config is symlinked into the install, so a relaunch picks it up without rebuilding. In simulation the default crouched spawn gives the stand-up something to do.

**1. Launch, with the robot hanging on a hoist or crouched on the floor.** The controller activates in `passive` and holds the joint pose it measured at activation, on the joint servo alone with no feed-forward, so this is safe whether the feet carry weight or not:

```
[HumanoidCentroidalMpcController] activated in bring-up (passive); ...
[HumanoidCentroidalMpcController] bring-up: passive (holding the activation pose); call ~/stand_up with the feet on the floor.
```

**2. If it is hanging, lower it until both soles are flat on the floor.** The stand-up assumes the weight is on the feet.

**3. Stand up:**

```bash
ros2 service call /humanoid_centroidal_mpc_controller/stand_up std_srvs/srv/Trigger
# response: success=True, message='standing up'
```

The joints interpolate to the nominal pose over `bringUp.standUpSeconds` (3 s), then hold it:

```
bring-up: passive -> standing up
bring-up: standing up -> settling
bring-up: settling | gyro 0.012/0.10 rad/s | |accel|-g +0.031/0.50 m/s^2 | not at rest yet
bring-up: settling | gyro 0.004/0.10 rad/s | |accel|-g +0.008/0.50 m/s^2 | at rest; call ~/start_locomotion
```

**4. Wait for `at rest; call ~/start_locomotion`.** It appears once the IMU has seen the trunk still for `bringUp.settleSeconds`. On a hoist, take the load off now, but leave the robot attached.

**5. Start locomotion:**

```bash
ros2 service call /humanoid_centroidal_mpc_controller/start_locomotion std_srvs/srv/Trigger
# response: success=True, message='initialising the estimator; the MPC starts once it is ready'
```

The estimator initialises from the IMU and the joint encoders, converges for `readiness.minSeconds` (5 s), and then the MPC starts from its estimate on its own:

```
bring-up: settling -> estimator converging
bring-up: estimator converging | epoch 1, running 2.0 s, not ready | roll/pitch std 0.0255 | height std 0.0123 | velocity std 0.1322
bring-up: estimator converging -> starting MPC
bring-up: starting MPC -> blending into MPC
bring-up: blending into MPC -> locomotion
```

**6. Only after `-> locomotion`, send velocity commands** (base command GUI, or `/humanoid/walking_velocity_command`). Before that the MPC is not running, and a command sent early is buffered and acted on the moment it starts.

**Calls at the wrong time are refused, not remembered.** Nothing is queued for later, so call again at the right step:

| call | refused when | response |
|---|---|---|
| `stand_up` | not in `passive` | `success=False`, `refused: stand_up is only valid in 'passive'; bring-up is in '...'` |
| `start_locomotion` | not in `settling` | `success=False`, `refused: start_locomotion is only valid in 'settling'; ...` |
| `start_locomotion` | in `settling`, trunk not at rest yet | `success=False`, `refused: the trunk is not at rest yet ...; retry` |

This is deliberate. A `start_locomotion` remembered from too early, with the robot still hanging, would initialise the estimator in the air, because its stance initialisation assumes the feet are on the ground.

**Fault.** If the trunk tilts past `bringUp.tiltLimit` (0.8 rad) in any phase after `passive`, the controller logs `bring-up FAULT: trunk tilt ...`, switches to joint damping and stays there. To retry, restart the controller, i.e. relaunch. Note that deactivating it writes zero to the joint commands, which the `effort_pd` servo turns into a move toward the zero pose, so support the robot before doing that on hardware.


**Closed-loop status:** verified on flat ground — `tests/state_estimator_closed_loop_test.sh` reports `VERDICT: SUCCESS` (2.1 m of walking, height error 1.8 mm, roll/pitch < 0.6°, body-velocity error 0.026 m/s). Backward, lateral and turning motions all match the ground-truth-driven behaviour. Note the default scene has a staircase in front of the robot, so plain forward walking runs into it and falls **with ground truth as well** — flat-ground tests command motion away from the stairs.

See [`docs/humanoid_state_estimation.md`](./docs/humanoid_state_estimation.md) for the equations, implementation and the terrain limitation of the kinematic height anchor.

## Acknowledgements

- The original code implementation of humanoid centroidal MPC: [`manumerous/wb_humanoid_mpc`](https://github.com/manumerous/wb_humanoid_mpc)
- Reference implementation for the whole-body inverse dynamics MPC formulation: [`lukasmolnar/wb-mpc-locoman`](https://github.com/lukasmolnar/wb-mpc-locoman)


## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
