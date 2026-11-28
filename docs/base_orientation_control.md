# Trunk roll and pitch commands

Upkie and Pineapple lean their trunk left or right on command: D-pad left/right on the DualSense (L2 held), or a
`geometry_msgs/Vector3` on `/humanoid/base_orientation_command` from any node. Both wheels stay on the ground; the
legs take different lengths. D-pad up/down with L2 held commands pitch on both robots as well.

## How other projects do it

- Wheel-legged robots without a roll joint between the legs and the base roll the trunk with the leg-length
  difference: DIABLO uses a PD roll controller that changes the two leg lengths, beside a height controller and a leg
  angle-difference PD [1]; patented wheel-legged controllers turn a roll error into a leg-length change and then, by
  inverse kinematics, into joint angles, for robots with "no degree of freedom in a roll direction between the wheel
  legs and the base" [2, 3]; a 6-DoF wheeled biped governs pose and leg splitting with an MPC and needs a leg height
  difference to adjust roll [4].
- OCS2-based legged MPC stacks (legged_control [5], and this repository) carry body orientation in the base-pose
  reference of the target trajectories; roll and pitch were fixed at 0 here.

Here the whole-body inverse-dynamics MPC already plans the full base pose and every joint, so the roll command is a
reference change, not a separate controller: the base reference takes the commanded roll, and the joint posture
reference takes the matching leg lengths.

## Design

- **Reference** (`WbIdMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories`): the base pose
  reference [x, y, z, yaw, pitch, roll] took roll = pitch = 0; it now takes the commanded values, clamped to
  `ocs2.reference.maxBaseRoll` / `maxBasePitch` and slewed at `baseOrientationRate` (0.5 rad/s). Base-twist mode only;
  the base_pose mode already took a full pose.
- **Per-leg posture** (`TargetTrajectoriesCalculatorBase::getPostureForPose`): with pitch-only legs a wheel stays in
  its leg's plane, so for a trunk roll phi the left wheel sits `rollLegHalfWidth * sin(phi)` higher than at level and
  the right one as much lower. Each leg's joints take the posture-table row of its own height, h +- that offset (the
  second-order 1/cos(phi) term, about 3 mm at 0.2 rad, is left out). Legs are found from the joint names (left_/L_
  +1, right_/R_ -1, arm joints 0). Without this the posture cost holds both legs at the level posture and fights the
  roll.
- **Command frame**: the topic is in the command frame; `target.commandFrameYaw` turns it into the base frame (Upkie:
  pi, so a positive command is a negative base roll and the robot leans right as the operator sees it).
- **Pitch posture**: the height-table joint reference remains nominal under pitch; it is a soft preference, not
  a fixed joint trajectory. The MPC optimizes hip/thigh and knee motion to balance the pitched trunk while satisfying
  rolling contact and inverse dynamics. Flat-ground tests at +-0.2 rad passed without extra pitch compensation or
  changing any cost/constraint weights.
- **Level outside locomotion**: the controller drops the command and the reference to level on every bring-up phase
  other than locomotion (stand-up, sit-down, braking, fall, fault) and ignores the topic outside locomotion; the
  teleop also levels roll and pitch when the robot leaves locomotion.
- **Off by default**: `maxBaseRoll` and `maxBasePitch` default to 0 (no subscription, level trunk), so G1 and H2
  are unchanged.

## Parameters

`ocs2.reference` in `config/<robot>/ros2_controllers.yaml`:

| parameter | Upkie | Pineapple | meaning |
| --- | --- | --- | --- |
| `maxBaseRoll` | 0.25 | 0.3 | roll limit [rad]; 0 disables |
| `maxBasePitch` | 0.2 | 0.2 | pitch limit [rad]; 0 disables; leg adjustment is optimized, not prescribed |
| `baseOrientationRate` | 0.5 | 0.5 | slew of the reference roll/pitch [rad/s] |
| `rollLegHalfWidth` | 0.191 | 0.227 | wheel offset from the base centre [m] (URDF wheel joints at y = +-0.191 / +-0.227) |

`target.baseOrientationTopic` (default `/humanoid/base_orientation_command`): x roll, y pitch [rad], absolute.

The limits keep both legs inside the posture tables at the default height (Upkie 0.245 m +- 0.047 m inside
0.125-0.355 m; Pineapple 0.36 m +- 0.067 m inside 0.14-0.46 m). Near the top of the height range the longer leg
runs off the table and is held at its last row; the MPC then trades a little roll against height (both soft costs).

## Joystick

legged_robot_teleop: with L2 held, D-pad left (button 13) / right (14) ramps the roll at `roll_rate` (0.3 rad/s) up
to the robot's `maxBaseRoll`, which the launch reads from the controller YAML (0: roll buttons off). The roll holds
when L2 is released; L1 (Center) levels it with the height.
D-pad up (11) / down (12) ramps pitch forward/back at `pitch_rate` (0.3 rad/s) with L2 held.
Positive pitch leans forward in the command frame; opposite D-pad buttons cancel.
Right stick forward/back (axis 3) raises/lowers height with L2 held; `height_axis_threshold: 0.3` ignores the first
30% of stick travel and rescales the rest linearly to the maximum `height_rate` (0.2 m/s in the teleop YAML).
Releasing the D-pad or L2 holds the tilt target; L1 with L2 released restores level roll/pitch and nominal height.
Velocity sticks must be centered, the height stick inside its deadband and active tilt buttons released before
enabling L2 or using the Square stand/sit toggle. G1/H2 ignore tilt buttons but retain stick height control.

```bash
# from another node or a terminal (Pineapple: lean right 0.2 rad)
ros2 topic pub -r 20 /humanoid/base_orientation_command geometry_msgs/msg/Vector3 "{x: 0.2, y: 0.0, z: 0.0}"
```

## Results (headless MuJoCo, `jetpack_jazzy`, KF estimate, MPC 200 Hz)

Command on the topic from locomotion + 3 s: +0.2 rad for 6 s, -0.2 rad for 6 s, then level (mean over the last 3 s
of each segment):

| robot | +0.2 | -0.2 | level | height (target) | pitch while rolled |
| --- | --- | --- | --- | --- | --- |
| Upkie (base frame, command frame turned by pi) | -0.195 | +0.195 | 0.000 | 0.248 m (0.245) | <= 0.002 rad |
| Pineapple | +0.201 | -0.202 | 0.000 | 0.363 m (0.36) | <= 0.002 rad |

The real teleop node driven by a synthetic DualSense (L2 armed, D-pad right 0.6 s, D-pad left 1.2 s, then L1 with
L2 released; 0.6 s at 0.3 rad/s is 0.18 rad):

| robot | after D-pad right | after D-pad left | after L1 |
| --- | --- | --- | --- |
| Upkie (base frame) | -0.175 | +0.175 | 0.000 |
| Pineapple | +0.181 | -0.182 | 0.000 |

Regression: `upkie_posture_cycle_test.py --cycles 2 --joystick --fall-test` passes on Upkie (max locomotion tilt
0.040 rad) and Pineapple (0.029 rad); controller Python tests 95 passed; teleop 58 tests (three new roll tests).

Not tested: the physical DualSense, tilt while stepping, or tilt commands on terrain. The base command GUI has no
roll/pitch control.

## Pitch validation (2026-11-28)

The results below used the earlier right-stick pitch binding, before pitch moved to D-pad up/down and height to the stick.
The MPC model and tuning did not change with the remapping.

Headless MuJoCo in `ros_jazzy`, wheel-aware linear KF feedback, requested MPC 200 Hz, the real C++ teleop node driven
by synthetic DualSense messages: stand up, pitch forward/back, hold with L2 released, combine roll and pitch, L1
level, sit down. Upkie's command frame is turned by pi: a positive pitch command becomes negative base-frame pitch.

| Robot | Base pitch for +0.2 rad command | Base pitch for -0.2 rad command | L1 level pitch |
| --- | --- | --- | --- |
| Upkie | -0.1967 | +0.1949 | +0.0009 |
| Pineapple | +0.1943 | -0.1946 | +0.0002 |

Both passed without a fall or changes to optimization weights, posture tables, dynamics or solver code.
The final repeat also commanded 0.1 m/s forward while holding combined roll/pitch: Upkie settled at base-frame
roll -0.2442 and pitch -0.0994 rad (commands +0.25/+0.10); Pineapple at +0.3012/+0.0981 rad (commands +0.30/+0.10).
Both then leveled on L1 and completed sit-down to idle. The 63 teleop checks passed, including pitch-only DDS output,
arming guards, limits, hold-on-release, best-effort QoS and reset outside locomotion.
Only the OCS2 robot YAMLs enable pitch; TurboMPC is unchanged, and G1/H2 retain disabled pitch commands.
The shared teleop defaults to `maximum_pitch: 0` when the selected controller has no pitch limit.
No CppAD library regeneration is needed for this reference/input change.

### Current D-pad Mapping Validation

Both robots passed the updated harness with D-pad pitch, combined tilt and 0.1 m/s driving, L1 level, a height-stick
deadband check and a 2 cm height increase, then sit-down to idle. These use the same wheel-aware KF and requested
200 Hz MPC rate as above; all 68 shared-teleop checks also passed.

| Robot | Base pitch for +0.2 / -0.2 commands [rad] | Settled height / commanded height [m] |
| --- | --- | --- |
| Upkie | -0.1967 / +0.1958 | 0.26485 / 0.26500 |
| Pineapple | +0.1946 / -0.1946 | 0.38068 / 0.38000 |

The height target stayed unchanged at 25% vertical stick deflection (within the 30% deadband).
These are synthetic joystick simulation results, not a physical gamepad or hardware test.

Repeat in an otherwise unused ROS domain, from the sourced workspace with this repository as the current directory:

```bash
ROS_DOMAIN_ID=196 python3 tests/base_orientation_test.py --robot-name upkie --prefix /tmp/codex_pitch_upkie
ROS_DOMAIN_ID=196 python3 tests/base_orientation_test.py --robot-name pineapple --prefix /tmp/codex_pitch_pineapple
```

The test launches and stops its own flat-ground simulation and teleop, saves ground-truth/joint samples and stage
statistics beside the prefix, and checks D-pad pitch limits, combined tilt tracking, L1 leveling, the height-stick
deadband and height tracking, and return to idle.

## Sources

1. DIABLO: A 6-DoF Wheeled Bipedal Robot Composed Entirely of Direct-Drive Joints, https://arxiv.org/pdf/2407.21500
2. Method and apparatus for controlling wheel-legged robot (patent),
   https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12661784
3. Mobile robot motion control method and mobile robot (patent),
   https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12594804
4. Whole-Body Control With Terrain Estimation of A 6-DoF Wheeled Bipedal Robot, https://arxiv.org/html/2511.06397
5. legged_control, https://github.com/qiayuanl/legged_control
