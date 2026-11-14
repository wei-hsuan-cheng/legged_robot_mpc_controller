# Wheeled Biped (Upkie) WB-ID MPC: Handoff

Branch `feat/wheeled_biped`, based on `jazzy` at `ae134c0`.
The original [implementation checklist](design_notes/wheeled_biped/wheeled_biped_first_step.md) and [research survey](design_notes/wheeled_biped/wheeled_biped_wb_id_mpc_survey.md) are preserved as working notes under [design_notes/wheeled_biped](design_notes/wheeled_biped/README.md), separate from the formal implementation documentation.
Update this file at each milestone.
The implemented equations, current weights, rolling-contact equalities and execution limits are collected in [Wheeled-Biped WB-ID MPC](wheeled_biped_whole_body_inverse_dynamics_mpc.md).


## Branch Checkpoint (2026-11-14)

The user requested committing and pushing the complete current Upkie checkpoint with a detailed commit message, retaining the current code and parameter values.
The checkpoint includes model assets and provenance, rolling-contact and inverse-dynamics extensions, wheel-drive configuration, optional stand-up playback, diagnostics, acceptance-test tooling and formulation documentation.
Research and initial implementation notes live under `docs/design_notes/wheeled_biped/`, not beside the formal formulation document.
The low-level PD execution was verified by source inspection: hardware `write()` runs every physics step using live MuJoCo joint state, while MRT updates the held references, feedforward and gains.
For the default Upkie scene this means 2 kHz PD with 1 kHz MRT and a requested 100 Hz MPC rate.
No new build or simulator test was run for the documentation and commit/push steps; the results below remain the prior recorded evidence, not fresh validation of the complete checkpoint.
Outstanding work remains the friction/damping comparison, repeated acceptance suite, full-stack stand-up, hop/single-support development, rolling-contact estimation and G1/H2 regressions.


## Formulation Documentation (2026-11-14)

Added the implementation-specific math document linked above after reading the current controller, configuration builder, contact kinematics, cost evaluators and OCS2 penalty functions.
It uses the existing humanoid document's dollar-delimited inline/display LaTeX and one-sentence-per-line prose, with two blank lines before headings.
It distinguishes the 24-state/30-input Euler-coordinate OCP, full-horizon RNEA, armature and smoothed Coulomb friction, eleven double-support wheel-contact rows, tracking costs, soft inequalities and final actuator clipping.
The document records the shared friction cone's default regularization of 25 N^2, whose zero-tangential-force nonnegative-residual threshold is about 8.33 N at coefficient 0.6.
It also records that the shared lateral equality is a signed average without axle-sign alignment, so it is not a proof of individual-wheel lateral no-slip; cancellation and equality rank need checking in future tests.
No controller code, parameters, model assets, build or simulation were changed or run for this documentation update.


## Status (2026-11-14)

| Milestone | State |
| --- | --- |
| Model import and MJCF/URDF reconciliation | Done |
| Rolling-wheel WB-ID MPC, wheel drive, ground truth (standing spawn) | Works: balance, 0.2 m/s drive and braking in the first run |
| Unmodelled wheel friction (zero-command creep) | Diagnosed; friction model added; comparison against wheel PD started, only the friction-model variant ran (results below) |
| Activation segfault | Root cause found and fixed (hard-coded foot frame names) |
| M1 acceptance suite (3x roll+brake, 3x push recovery) | Not yet run cleanly; the first suite attempt hit the segfault |
| Kneeling idle -> planned stand-up -> MPC | Planned and implemented; never run in the full stack |
| Small symmetric hop | Not started (gait-request topic is ready) |
| Short single support, then trot | Not started |
| Estimator closed loop (linear KF adapted to rolling) | Not started; ground truth only so far |
| G1/H2 regression after the shared-code changes | Not run |


## Terminology

Driving with both wheels in rolling contact is double support with no contact switches.
Ascento and wheeled ANYmal (Bjelonic et al.) call it **driving** (hybrid walking-driving when mixed with steps), Xin & Vijayakumar **rolling**, the user's documents **wheel drive**.
This branch uses **wheel drive**.


## Model

| Item | Choice |
| --- | --- |
| Robot | Upkie: hip, knee and wheel per side, six actuators, 5.4605 kg |
| Simulation reference | [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie) `d789578` MJCF: servos replaced by unit-gear torque motors capped at 16 N m (hip/knee) and 1.7 N m (wheel); [`upkie.xml`](../description/mujoco/robots/upkie/upkie.xml), [`scene.xml`](../description/mujoco/robots/upkie/scene.xml) |
| MPC model | [`upkie.urdf`](../description/urdf/robots/upkie/upkie.urdf), generated from the MJCF by [`generate_upkie_urdf.py`](../description/urdf/robots/upkie/generate_upkie_urdf.py) |
| Agreement | FK 2e-9 m, gravity torques 5e-9 N m, joint mass matrix (with armature) 1e-10, over random poses |
| Passive joint terms | MJCF armature 0.005 kg m^2 (16x the wheel's own spin inertia) and friction loss 0.1 N m on every joint, both now modelled in the MPC |
| Wheels | Scalar revolute joints (nq = nv = 12), excluded from position limits and posture costs; zero wheel position gain (enforced at configure) |
| ros2_control | [`upkie.ros2_control.xacro`](../description/urdf/robots/upkie/upkie.ros2_control.xacro): `effort_pd` on six joints, `base` GT body sensor, `base_imu`, simulation contact sensors on both wheel bodies |
| Nominal posture | Level trunk 0.30 m, knees forward, tire under the CoM: hip -0.6021, knee -0.9655 (right mirrored) |
| Joint signs | [MODEL_SOURCE.md](../description/urdf/robots/upkie/MODEL_SOURCE.md); positive left wheel rolls forward, positive right wheel backward |

The upstream URDF (5.339 kg, 50 mm tire, different inertias and joint ranges) is a different CAD export, so the MPC URDF is generated from the MJCF rather than reconciled by hand.


## Controller changes

### Rolling-wheel contact ([`WbIdWheelContactConstraintCppAd`](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdWheelContactConstraintCppAd.cpp))

The contact frames are the tire centres on the wheel links (`contactParentJointNames` = wheel joints, `frameTranslation` 31.2 mm along the axle); they spin with the wheel but their position does not.
The wrench `[f; m]` stays at the tire centre, so the RNEA rows are unchanged; `m = rho x f` makes `J_h^T [f; m] = J_c^T f`.
Shared kinematics (contact offset `rho`, its rate, material velocity/acceleration) are in [`WheelContactKinematics.h`](../include/humanoid_wb_id_mpc/common/WheelContactKinematics.h).

| Term | Rows | Active |
| --- | --- | --- |
| `<tire>_wheelRolling` | longitudinal tread slip (accel + velocity), gap (accel + velocity + position), `m - rho x f` (3) | wheel in contact |
| `wheelLateralNoSlip` | one lateral slip row, the contact-weighted mean over the wheels in contact | any wheel in contact |
| `<tire>_wheelSwingVertical` | tire bottom (centre z - r) follows the swing height spline | wheel in the air |
| `<tire>_zeroWrench` (existing) | zero wrench | wheel in the air |

The lateral row is shared because, with no hip roll, both wheels' lateral rows are the same function of the base motion in symmetric double support; two such equalities made the projected equality system rank deficient.
`wheelContact` replaces `footMotionEquality` and `contactMomentXY`, which must be off.
Friction cone and torque/velocity limits are the existing terms.

### Other changes

| Change | Where | Default (G1/H2 unaffected) |
| --- | --- | --- |
| Joint armature in the Pinocchio model (RNEA/CRBA include it) | `ocs2.model.jointArmature`, [createPinocchioModel.cpp](../src/core/humanoid_common_mpc/src/pinocchio_model/createPinocchioModel.cpp) | empty |
| Coulomb joint friction `f_c tanh(v / v_s)` in the joint RNEA rows and the ID fallback | `ocs2.model.jointFriction`, `jointFrictionVelocity`, [WbIdRneaConstraintCppAd.cpp](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdRneaConstraintCppAd.cpp) | empty (separate CppAD library name when on) |
| Wheel joints get +/-inf position bounds and must have zero posture weight | config builder, factory | only with `wheelContact` |
| Arm-swing joints optional (robot without arms) | ModelSettings, builder | 4 names still required when arm swing is on |
| `computeJointTorques` takes the contact frame names (was hard-coded `foot_l/r_contact`: out-of-range read and intermittent segfault on Upkie) | DynamicsHelperFunctions | humanoid names |
| Planned stand-up bring-up for any floating-base source | `bringUp.standUpTrajectoryFile`, `idleSeconds`; controller | empty = humanoid bring-up |
| Height reference restarts at the measured height on the stand-up handover | `WalkingVelocityTarget::requestFilterReset(height)` | unchanged otherwise |
| Gait-request topic: hold a named template instead of the velocity ladder (hop/single-support tests) | `target.gaitTopic` (`/humanoid/gait_command`, `std_msgs/String`, empty = ladder) | subscribed, unused unless published |
| Tracking columns: wheel gap/slip at the contact point, measured wheel normal force and contact | `diagnostics.contactSensorNames`, `~/tracking` | none |
| Stand-up playback feed-forward clamped like the MPC's | `clamp_torque` | n/a |
| Upkie in the launch (`robotName:=upkie`, GT body `base`) and test harness (`--robot-name upkie`, `--launch-arg`) | [robot.launch.py](../launch/robot.launch.py), [wb_id_walk_test.py](../tests/wb_id_walk_test.py) | n/a |

The torque clamp is the same as H2's: the MPC feed-forward is clamped to `jointTorqueLimits` (URDF efforts 16/1.7 N m), the OCP penalises the same limits, and the MuJoCo motors' `ctrlrange` caps the summed PD + feed-forward.


## Results so far (ground truth, flat, standing spawn, MPC 100 Hz / MRT 1 kHz)

First run, before the friction model ([`tests/wb_id_walk_test.py`](../tests/wb_id_walk_test.py), analysed with [`analyze_wheeled_biped.py`](../tests/tuning/analyze_wheeled_biped.py)):

| Phase | Pitch max | Forward speed | Wheel normal force | Tread slip rms | Solve |
| --- | --- | --- | --- | --- | --- |
| Zero command, 10 s | 0.026 rad | -0.018 m/s (creep) | 26-27 N each | <= 0.04 m/s | 2.7 ms |
| Hold 0.2 m/s, 5 s | 0.021 rad | 0.223 m/s | 25.9-27.5 N | <= 0.005 m/s | 2.8 ms |
| Rest after braking, 8 s | 0.019 rad | +0.021 m/s (creep) | 26.2-27.4 N | <= 0.004 m/s | 2.9 ms |

Both wheels in contact throughout (measured flags 1.000), gap <= 0.3 mm, no effort saturation; verdict SUCCESS.
The spawn landing (now 0.5 mm instead of 2 mm) caused a 0.1 rad yaw change in the first 0.2 s; yaw angle is not tracked in `base_twist`.

### The zero-command creep and the friction model (answers to the user's questions)

**What the creep was.** At zero command the robot did not come to rest: it rolled at a steady 1.8 cm/s backwards (17 cm in 10 s, trunk leaning back 0.015 rad), and after braking at a steady 2 cm/s forwards.
It neither fell nor accelerated.
The MPC's planned velocity equalled the measured one, so it was not a tracking error: every solve planned a correction that did not happen.

**Why.** The MJCF has 0.1 N m Coulomb friction on every joint; the MPC model had none.
A wheeled pendulum balances near standstill with wheel torques of about 0.1 N m, the size of the friction: the MPC commanded +/-0.09 N m and the friction absorbed it.
The robot settled into lean + constant wheel speed + motor torque = friction torque, and an MPC without integral action replans the same thing every time.
With the wheel friction removed from a test scene the creep was 0.002 m/s and the wheel torques about 0.

**How friction enters the MPC.** The joint rows of the inverse-dynamics equality become `M a + h + f_c tanh(v / v_s) - tau - J^T W = 0` (`jointFriction` 0.1 N m, `jointFrictionVelocity` 0.05 rad/s), so the planned motor torque includes friction compensation and the policy feed-forward carries it.
`tanh(0) = 0`: static friction at standstill remains (a few mm of CoM deadband).

**H2 does not model it.** H2's MJCF has friction loss 0.2 N m, armature 0.01 and damping 0.05, but its MPC models none of them.
There it is negligible (hundreds of N m, planted feet, stiff kp 600 servo); on Upkie the wheels run kp 0 with a 1.7 N m budget and balance on ~0.1 N m.

**Should the low-level PD fix it, and do papers control wheel velocity only?**
The MPC sends the planned torque (feed-forward) and the planned joint positions and velocities, which `effort_pd` tracks at 2 kHz.
In the creep the planned wheel speed itself equalled the creep, because every solve restarts from the measured state and the base position is not tracked in `base_twist`; the PD therefore saw no error, and the root cause was the MPC model.
The wheel speed loop is still the right inner loop for errors between solves (friction, bumps, contact transients), so the plan is to use both: friction in the MPC and wheel speed damping.
In the literature the wheels are tracked by velocity (plus torque): wheeled ANYmal (Bjelonic et al.) tracks wheel speeds, Upkie's own MPC balancer commands wheel ground velocity with zero wheel position gain, Ascento's balancer feeds back wheel speed and body pitch.
The planned wheel angle is re-anchored to the measurement at every solve, so a wheel kp would only act within one 10 ms interval and adds stiffness against the rolling the base motion requires; holding position belongs to the MPC (a base position/velocity cost), not to wheel kp, which is why the user's document asks for zero wheel position gain (enforced at configure).

### Friction-model vs wheel-PD comparison (last test)

`tests/tuning/upkie_acceptance_suite.sh` was run (as `/tmp/claude_upkie_suite.sh`, results under `/tmp/claude_upkie_<variant>_{roll,push}_1.*` in the `ros_jazzy` container) for three variants:

| Variant | MPC friction | Wheel kd (N m s/rad) |
| --- | --- | --- |
| `model_only` | 0.1 N m | 0.05 |
| `pd_only` | 0 | 0.3 |
| `both` | 0.1 N m | 0.3 |

Stopped early on the user's request; only `model_only` ran:

| Run | Result |
| --- | --- |
| `model_only` roll+brake | SUCCESS (`/tmp/claude_upkie_model_only_roll_1*`; drift numbers not yet extracted, use `compare_wheeled_biped_runs.py`) |
| `model_only` push (4 pulses in the zero-command phase) | Survived all four pulses and finished every phase: base height 0.300 m throughout, pitch <= 0.01 rad, position within about 6 cm. Yaw swung by up to 0.24 rad during the pushes (open item). The verdict was lost to a harness bug (odometry written after its CSV was closed when a wrench sequence is used), now fixed in `wb_id_walk_test.py` |
| `pd_only`, `both` | Not run |

Servo structure in every variant: wheels kd only (kp 0, damping towards the planned wheel speed), hips and knees kp and kd around the planned trajectory, and the MPC torque feed-forward always on all six joints.


## Planned stand-up (kneeling idle -> MPC), not yet run in the stack

The user's start-up: an idle pose resting on the floor (kneeling on the knee-motor housings and both wheels, as in the Upkie photo), a planned feasible stand-up, then the MPC; the same phase machine as the H2 bring-up.

Kneeling, the CoM stays 5-16 cm ahead of the wheel contacts for every hip/knee configuration that keeps the knees down (knees carry 29-93% of the weight), so a quasi-static stand-up is impossible, and the wheel torque alone (2 x 1.7 N m) only drives the robot forward on its knees.
As in Ascento (an apply-torque phase until the body has the energy to reach upright, then the balancer) and Upkie (open loop until just before the body leaves the floor, then MPC), the stand-up is a planned dynamic push:

1. fold to hip -0.504 / knee -2.687 in 0.533 s (knees fully folded bring the CoM closest to the wheels);
2. push to hip -0.207 / knee -2.196 in 0.377 s with 1.361 N m forward wheel torque: the hips press the knee housings into the floor and the body pivots back about the axle.

The parameters minimise the pendulum's divergent component at knee lift-off, `xi = (x_com - x_wheel) + v_rel / omega`; with `xi ~ 0` the body coasts towards CoM-over-wheel.
Exported plan: handover (knee lift-off) at 0.696 s, xi +0.012 m, pitch -0.71 rad (nose up), pitch rate -0.67 rad/s, CoM height 0.104 m, largest servo command 0.80 of the limits.
[`plan_stand_up.py`](../config/upkie/plan_stand_up.py) (offline, MuJoCo Python) reproduces it and has a `--search` option; output [`stand_up_trajectory.yaml`](../config/upkie/stand_up_trajectory.yaml), idle spawn [`initial_pose_idle.yaml`](../config/upkie/initial_pose_idle.yaml).

Controller flow with `bringUp.standUpTrajectoryFile` set: Passive (idle hold with the plan's gains, `idleSeconds` 3 s) -> StandUp (playback of measured q, v and applied torque from the planning rollout; the MPC starts solving at its start) -> at `handover_time`, if a policy is valid, Blend (`blendSeconds` 0.05) -> Locomotion, with the height reference restarted at the measured height and ramped at 0.1 m/s to 0.30 m; no valid policy -> Fault.
`tiltLimit` is 1.2 rad for Upkie because the trunk leans back 0.71 rad at the handover.

To run (not yet done; minimum height must allow the low crouch):

```bash
python3 src/legged_robot_mpc_controller/tests/wb_id_walk_test.py --robot-name upkie --source ground_truth_state \
  --minimum-height 0.05 --startup-timeout 900 --record-tracking \
  --axis vx --speed 0.05 --hold 1 --ramp 1 --settle 10 --rest 4 \
  --set 'bringUp.standUpTrajectoryFile="/root/ocs2_ros2_ws/install/legged_robot_mpc_controller/share/legged_robot_mpc_controller/config/upkie/stand_up_trajectory.yaml"' \
  --launch-arg initialPoseFile:=/root/ocs2_ros2_ws/install/legged_robot_mpc_controller/share/legged_robot_mpc_controller/config/upkie/initial_pose_idle.yaml \
  --prefix /tmp/upkie_standup
```

Risks to check first: the MPC catching a tucked, back-leaning, low robot (it is the first state the MPC sees with only wheel contacts); the idle settle in the full stack matching the planning rollout (gains 20/0.5, 3 s); the harness's 0.15 s fall detector during the rise.


## Test it yourself

In the `ros_jazzy` container, from `/root/ocs2_ros2_ws` (launch from there: the CppAD cache path `auto_generated/upkie` is relative), after the build below:

```bash
source install/setup.bash
# 1. Wheel drive with the GUI: standing spawn, ground-truth base, MuJoCo and RViz windows.
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=upkie \
  floatingBaseSource:=ground_truth_state floatingBaseEstimator:=none \
  rviz:=true mujoco_headless:=false baseCommandGui:=true
#    The GUI's forward axis is normalised by maxDisplacementVelocityX (1.0 m/s); keep vy at 0
#    (a wheeled biped cannot move sideways). Yaw rate turns by differential wheel speed.

# 2. The same from a script instead of the GUI (another terminal):
ros2 topic pub -r 20 /humanoid/walking_velocity_command ocs2_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.2, linear_velocity_y: 0.0, desired_pelvis_height: 0.30, angular_velocity_z: 0.0}"

# 3. Push the base (MuJoCo external wrench): launch/push_recovery_test.py with
#    config/tests/upkie_pitch_push.yaml, or run the scripted push test below.

# 4. Scripted, headless, logged (one roll+brake and one push run, verdicts printed):
bash src/legged_robot_mpc_controller/tests/tuning/upkie_acceptance_suite.sh mine 1
python3 src/legged_robot_mpc_controller/tests/tuning/analyze_wheeled_biped.py /tmp/upkie_mine_roll_1 --plot

# 5. Kneeling idle -> planned stand-up -> MPC (not yet validated): see "Planned stand-up" below.
```

Do not run two simulations at once (shared topic names).
On the GUI the height slider is absolute; the Upkie nominal is 0.30 m and `maxDeltaPelvisHeight` is 0.08 m.


## How to build and test

Container `ros_jazzy` on the Parallels VM; sources from the Mac at `/root/src`; workspace `/root/ocs2_ros2_ws`.

```bash
cd /root/ocs2_ros2_ws && source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --packages-select legged_robot_mpc_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
# manual run (standing spawn, ground truth)
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=upkie \
  floatingBaseSource:=ground_truth_state floatingBaseEstimator:=none rviz:=true mujoco_headless:=false
# acceptance suite: TAG N [extra harness args]; results /tmp/upkie_<TAG>_{roll,push}_<i>.*
bash src/legged_robot_mpc_controller/tests/tuning/upkie_acceptance_suite.sh m1 3 \
  --launch-prefix "gdb -batch -ex run -ex bt --args"
```

Analyse on any machine with numpy/matplotlib: `tests/tuning/analyze_wheeled_biped.py PREFIX --plot` (per phase: pitch, speed, per-joint feed-forward vs applied effort and saturation, wheel normal force/contact, slip, gap, solver residuals) and `tests/tuning/compare_wheeled_biped_runs.py PREFIX...` (drift and push table).
The pitch push sequence is [`config/tests/upkie_pitch_push.yaml`](../config/tests/upkie_pitch_push.yaml): +/-1.5 N m pitch torque and +/-6 N base force pulses of 0.2 s.
A full rebuild takes about 11 minutes (ModelSettings/helper headers are included everywhere); the first launch after a CppAD-relevant change generates the libraries in `auto_generated/upkie/` (about a minute).


## Next steps, in order

1. Finish the friction-model vs wheel-kd comparison (`pd_only`: no MPC friction, wheel kd 0.3; `both`: friction + kd 0.3; commands in the comparison section), pick the combination, then run the M1 acceptance suite (3x roll+brake, 3x push recovery) and confirm repeatability, the zero-command drift and the push-induced yaw swing.
2. Run the planned stand-up in the stack; adjust the plan (`plan_stand_up.py --search`) or the handover if the MPC cannot catch it.
3. Hop: add a `hop` template (e.g. STANCE 0.5 s, FLY 0.1-0.15 s) and request it on `/humanoid/gait_command`; check takeoff/landing torques, measured flight (contact sensors) and touchdown slip; consider a wheel-spin cost in flight.
4. Short single support (`left_leg`/`right_leg` templates for ~0.1-0.2 s): no hip roll, so lateral balance comes only from rolling the trunk with leg-length differences; measure roll before attempting alternating support (trot).
5. Estimator: adapt the linear KF to rolling contacts (wheel contact gives a base-velocity measurement through zero tread velocity and a height measurement; no fixed foot landmark), then rerun the milestones closed loop on the estimate.
6. Regression-test G1/H2 after the shared changes (`computeJointTorques` signature, optional arm-swing joints, height-reset argument, gait topic subscription, tracking columns).


## Traps found

- rclcpp rejects empty YAML lists (`fixedJointNames: []`): omit the key.
- `pkill -f <pattern>` inside `bash -c` matches its own command line (the workspace path contains `ros2`); kill by PID.
- Editing the parameter schema while a build runs leaves the generated header stale; rebuild after.
- `jointArmature` is baked into the generated CppAD libraries (set `codegen.recompileLibraries` or clear `auto_generated/upkie` after changing it); `jointFriction` and the wheel gains are parameters.
- Pass the gdb launch prefix as one argument: `--launch-prefix "gdb -batch -ex run -ex bt --args"`.
- `/mujoco/ground_truth/odom` twist is in the body inertial frame (see the H2 caveat); use the tracking `obs_base` columns or pose differences.
