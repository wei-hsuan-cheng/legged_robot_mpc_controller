# Humanoid Whole-Body Inverse Dynamics MPC: Implementation Guide

This guide maps [Formulation and Signal Flow](./humanoid_whole_body_inverse_dynamics_mpc.md) to the current code and parameters.
Links use repository-relative paths with line anchors, which may shift as the source changes.


## 1. Entry Points and Configuration

| Responsibility | Implementation |
| --- | --- |
| Lifecycle, observations and actuation | [controller.cpp](../src/controller/controller.cpp) |
| Standalone parameter schema | [parameters.yaml](../src/controller/parameters.yaml) |
| Current G1 values | [ros2_controllers.yaml:189](../config/g1/ros2_controllers.yaml#L189) |
| Config translation | [config_builder.cpp](../src/controller/config_builder.cpp) |
| OCP assembly | [WbIdMpcInterface.cpp:102](../src/mpc/wb_id/WbIdMpcInterface.cpp#L102) |
| G1 model and mesh paths | [g1_29dof.xml:16](../description/mujoco/robots/g1/g1_29dof.xml#L16), [g1_29dof.urdf:16](../description/urdf/robots/g1/g1_29dof.urdf#L16) |
| Simulation scene composition | [G1 scene.xml:3](../description/mujoco/robots/g1/scene.xml#L3), [H2 scene.xml:3](../description/mujoco/robots/h2/scene.xml#L3), [shared module layout](../description/mujoco/README.md) |
| Hardware interface description | [g1.ros2_control.xacro:17](../description/urdf/robots/g1/g1.ros2_control.xacro#L17) |

WB-ID uses a dedicated generated parameter schema.
The builder maps solver settings at [line 87](../src/controller/config_builder.cpp#L87), gait settings at [line 119](../src/controller/config_builder.cpp#L119) and costs at [line 167](../src/controller/config_builder.cpp#L167).
The interface validates the Translation + SphericalZYX dimensions at [WbIdMpcInterface.cpp:44](../src/mpc/wb_id/WbIdMpcInterface.cpp#L44).
The common launch selects the robot URDF at [line 42](../launch/robot.launch.py#L42), the separate RViz environment at [line 46](../launch/robot.launch.py#L46), and the robot-local MuJoCo scene at [line 155](../launch/robot.launch.py#L155).
The selected controller YAML supplies both MPC and GUI height through [the parameter resolver](../launch/robot.launch.py#L224-L249); `refBaseHeight` remains an optional override of both.
Scene names passed through `mujocoModelFile` are unchanged by the description-directory reorganization.


## 2. Model and Observation

[WbIdMpcRobotModel.h:13](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L13) defines $`x=[q,v]`$ and $`u=[W_L,W_R,a,\tau_j]`$.
Dimensions are constructed at [line 37](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L37), and block indices are defined at [line 48](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L48).
The inherited `getBaseComVelocity` name returns pelvis world linear velocity and Euler rates in WB-ID.
Joint velocities are read from the state at [line 102](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L102), while generalized accelerations and joint torques are read from the input at [line 117](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L117).

[build_observation():1559](../src/controller/controller.cpp#L1559) combines joint interfaces and floating-base feedback.
Estimator feedback is read at [line 1515](../src/controller/controller.cpp#L1515).
For simulation debugging, `floatingBaseSource:=ground_truth_state` can replace estimator feedback with GT, but this bypass is not available for real-world deployment.
Quaternion normalization, yaw unwrapping and angular velocity conversion are at [line 1624](../src/controller/controller.cpp#L1624).
The optional generalized velocity low-pass is at [line 1645](../src/controller/controller.cpp#L1645).
`observation.input` is zeroed at [line 1569](../src/controller/controller.cpp#L1569), not filled with measured torques or wrenches.


## 3. Dynamics, Constraints and Initialization

### 3.1 Double integrator and RNEA

[WbIdDynamics.cpp:18](../src/mpc/wb_id/dynamics/WbIdDynamics.cpp#L18) implements $`\dot x=[v,a]`$.
Its analytical linearization is initialized at [line 9](../src/mpc/wb_id/dynamics/WbIdDynamics.cpp#L9).
[WbIdRneaConstraintCppAd.cpp:58](../src/mpc/wb_id/constraint/WbIdRneaConstraintCppAd.cpp#L58) evaluates RNEA and subtracts contact Jacobian transpose contributions.
Base and joint equations are registered independently at [WbIdMpcInterface.cpp:167](../src/mpc/wb_id/WbIdMpcInterface.cpp#L167).
When `inverseDynamics.activate` is enabled, base rows span the horizon while joint rows check the torque window at [WbIdRneaConstraintCppAd.cpp:54](../src/mpc/wb_id/constraint/WbIdRneaConstraintCppAd.cpp#L54).

### 3.2 Torque window

The builder converts `torqueConstraintNodes` into $`(N_\tau-0.5)\Delta t`$ at [line 154](../src/controller/config_builder.cpp#L154).
[WbIdTorqueWindow.h:32](../include/legged_robot_mpc_controller/mpc/wb_id/constraint/WbIdTorqueWindow.h#L32) implements time-based constraint activation.
The synchronized module updates the horizon start at [line 68](../include/legged_robot_mpc_controller/mpc/wb_id/constraint/WbIdTorqueWindow.h#L68).
The runtime policy check uses the policy's own interpolation timestamps at [line 43](../include/legged_robot_mpc_controller/mpc/wb_id/constraint/WbIdTorqueWindow.h#L43).
This matters when a published policy is older than the solver's current horizon.

### 3.3 Foot equalities

[WbIdFootAccelerationConstraintCppAd.cpp:61](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L61) activates stance or swing rows from scheduled contacts.
Spline position, velocity and acceleration references are obtained at [line 66](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L66).
The swing vertical equality is at [line 109](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L109), and stance equalities are at [line 115](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L115).
The class uses frame classical acceleration in world-aligned coordinates.
The config builder maps `ocs2.constraints.footMotionEquality` to these gains at [line 70](../src/controller/config_builder.cpp#L70).
This WB-ID parameter path replaces both `ocs2.model.footConstraint` and the more recent `ocs2.constraints.footConstraint`; update external parameter files and rebuild the generated parameter library and controller before using the renamed YAML.
Only parameter ownership changed: gain values, generated OCP equations and the internal `ModelSettings::footConstraintConfig` storage are unchanged.
The shared [ZeroWrenchConstraint.cpp](../src/mpc/common/constraint/ZeroWrenchConstraint.cpp) supplies swing zero-wrench rows, registered at [WbIdMpcInterface.cpp:194](../src/mpc/wb_id/WbIdMpcInterface.cpp#L194).

### 3.4 Soft inequalities

| Margin | Evaluation | Penalty registration |
| --- | --- | --- |
| Friction | [FrictionForceConeConstraint.cpp:92](../src/mpc/common/constraint/FrictionForceConeConstraint.cpp#L92) | [HumanoidCostConstraintFactory.cpp:196](../src/mpc/common/HumanoidCostConstraintFactory.cpp#L196) |
| CoP rectangle | [ContactMomentXYConstraintCppAd.cpp:86](../src/mpc/common/constraint/ContactMomentXYConstraintCppAd.cpp#L86) | [Factory:173](../src/mpc/common/HumanoidCostConstraintFactory.cpp#L173) |
| Joint positions | [JointLimitsSoftConstraint.cpp](../src/mpc/common/constraint/JointLimitsSoftConstraint.cpp) | [Factory:158](../src/mpc/common/HumanoidCostConstraintFactory.cpp#L158) |
| Joint velocities | [WbIdJointVelocityLimitConstraint.cpp](../src/mpc/wb_id/constraint/WbIdJointVelocityLimitConstraint.cpp) | [WbIdMpcInterface.cpp:154](../src/mpc/wb_id/WbIdMpcInterface.cpp#L154) |
| Joint torques | [WbIdJointTorqueLimitConstraint.cpp](../src/mpc/wb_id/constraint/WbIdJointTorqueLimitConstraint.cpp) | [Interface:175](../src/mpc/wb_id/WbIdMpcInterface.cpp#L175) |
| Foot/knee distances | [FootCollisionConstraint.cpp:95](../src/mpc/common/constraint/FootCollisionConstraint.cpp#L95) | [Factory:142](../src/mpc/common/HumanoidCostConstraintFactory.cpp#L142) |

The fixed surface-normal setter validates and applies its normal at [FrictionForceConeConstraint.cpp:66](../src/mpc/common/constraint/FrictionForceConeConstraint.cpp#L66), while [worldToTerrain():73](../src/mpc/common/constraint/FrictionForceConeConstraint.cpp#L73) selects planned contact normals only in explicit `terrain_walk` mode; ordinary `base_twist` retains the default world-up normal.
Sphere penalty deactivation during double support is in [FootCollisionConstraint.cpp](../src/mpc/common/constraint/FootCollisionConstraint.cpp#L84).
The optional signed lateral margin is evaluated in the same [constraint function](../src/mpc/common/constraint/FootCollisionConstraint.cpp#L146); its double-support mask and parameter vector are built in [FootCollisionConstraint.h](../include/legged_robot_mpc_controller/mpc/common/constraint/FootCollisionConstraint.h#L100).
With positive `minimumLateralSeparation`, the constraint has 17 rows and uses a separate `_lateral_v1` AD cache; zero retains the original 16-row cache and gating.
The factory selects `PieceWisePolynomialBarrierPenalty` for positions/collision and `RelaxedBarrierPenalty` for friction/CoP.
The WB-ID interface selects relaxed-log penalties for velocity/torque limits.
Both penalty classes come from the linked `ocs2_core` dependency, with their exact scalar equations reproduced in the formulation document.

### 3.5 Initializer

[WbIdInitializer.cpp:18](../src/mpc/wb_id/initialization/WbIdInitializer.cpp#L18) seeds supporting forces and joint inverse-dynamics torques using zero acceleration.
Its `compute()` initially keeps `nextState = state` at [line 43](../src/mpc/wb_id/initialization/WbIdInitializer.cpp#L43), supplying a starting guess rather than an equality-feasibility guarantee.
The running quadratic cost instead gets its nominal input from [DynamicsHelperFunctions.h:178](../include/legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h#L178), where only supporting contact forces are populated.


### 3.6 Arm versus body sphere constraint

The WB-ID-only configuration and analytic distance constraint are declared in [ArmBodyCollisionConstraint.h](../include/legged_robot_mpc_controller/mpc/wb_id/constraint/ArmBodyCollisionConstraint.h#L18).
The [constructor and worker-copy constructor](../src/mpc/wb_id/constraint/ArmBodyCollisionConstraint.cpp#L38) validate frames, unique sphere names, finite centers and positive radii, and give each clone independent Pinocchio Data/mapping.
[Center Jacobians](../src/mpc/wb_id/constraint/ArmBodyCollisionConstraint.cpp#L112) include the rotated local-offset cross product before mapping into the OCS2 state.
[Distance values and linearizations](../src/mpc/wb_id/constraint/ArmBodyCollisionConstraint.cpp#L136) use conservative epsilon regularization; only arm/body pairs are enumerated.
The [WB-ID OCP setup](../src/mpc/wb_id/WbIdMpcInterface.cpp#L146) registers `armSelfCollision` with `ArmBodyCollisionSoftConstraint` and the selected existing OCS2 log or polynomial penalty.
This local wrapper filters exactly inactive polynomial rows before OCS2's dense penalty/Hessian assembly, and returns an exact zero approximation when every row is inactive.
The [active-row selection](../src/mpc/wb_id/constraint/ArmBodyCollisionConstraint.cpp#L202) is local to this term; it does not modify the shared OCS2 stack.
All pair distances are evaluated first, then only [selected sphere-center Jacobians](../src/mpc/wb_id/constraint/ArmBodyCollisionConstraint.cpp#L159) are formed and cached within that call.
It does not weaken the distance margins or filter the noncompact log barrier.
Native tests compare value, gradient and Gauss-Newton Hessian against the generic `StateSoftConstraint` for both penalties and both active/inactive polynomial cases.
The [configuration builder](../src/controller/config_builder.cpp#L200) reads named `arms` and `bodies` maps only when activated.
The generated schema defaults to disabled for old configurations; G1/H2 explicitly enable the term.
Reconfigure/restart after changing activation, geometry or penalty parameters.
No changes to OCS2, leg collision, or CppAD code generation are required.

[arm_body_collision_test.cpp](../tests/arm_body_collision_test.cpp) exercises analytic Jacobians, global-pose invariance, worker cloning, coincident centers, activation, invalid configuration and penalty derivatives.
It is registered with CTest when `BUILD_TESTING=ON`.
Its optional `--benchmark` argument compares generic and selected-row evaluation on synthetic 62-state, 128-pair cases; use `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1` for the same environment as the locomotion tests.
Those component timings are not whole-MPC solve rates.
[wb_id_arm_body_collision_test.py](../tests/wb_id_arm_body_collision_test.py) checks G1/H2 frame/configuration validity, nominal clearance and detection of an inward arm target.
[analyze_arm_body_collision.py](../tests/tuning/analyze_arm_body_collision.py) evaluates nominal or replayed measured joint poses and produces clearance CSV/JSON and plots.
It reads the walking test's YAML snapshot when available; replay is not a full mesh-contact certificate.
The walking harness also accepts `--arm-target` as a JSON array in tracked shoulder/elbow order for repeatable avoidance comparisons.


## 4. Costs

### 4.1 Weight partitioning and base errors

[config_builder.cpp:183](../src/controller/config_builder.cpp#L183) partitions Q/Q-final into generic, base-motion and arm-position evaluators without double counting.
R is assembled in $`[W_L,W_R,a_B,a_j,\tau_j]`$ order at [line 194](../src/controller/config_builder.cpp#L194).
[StateInputQuadraticCost.cpp:67](../src/mpc/common/cost/StateInputQuadraticCost.cpp#L67) defines generic deviations and the supporting-force nominal input.
[BaseMotionTrackingCost.cpp:50](../src/mpc/common/cost/BaseMotionTrackingCost.cpp#L50) suppresses planar position/yaw errors in relative-twist mode.
The heading-frame horizontal velocity residual is at [line 73](../src/mpc/common/cost/BaseMotionTrackingCost.cpp#L73), and its $`J^{\mathsf T}QJ`$ quadratic approximation is at [line 99](../src/mpc/common/cost/BaseMotionTrackingCost.cpp#L99).
Generic terminal cost scaling is applied at [HumanoidCostConstraintFactory.cpp:210](../src/mpc/common/HumanoidCostConstraintFactory.cpp#L210).

### 4.2 Arm position targets

[JointTrackingCost.cpp:41](../src/mpc/common/cost/JointTrackingCost.cpp#L41) selects external arm targets when present.
Otherwise the running evaluator uses the arm-swing reference and the terminal evaluator uses raw posture, selected at [line 28](../src/mpc/common/cost/JointTrackingCost.cpp#L28).
The factory constructs both variants at [HumanoidCostConstraintFactory.cpp:118](../src/mpc/common/HumanoidCostConstraintFactory.cpp#L118).

### 4.3 Task-space and load costs

[EndEffectorFootCost.cpp:92](../src/mpc/common/cost/EndEffectorFootCost.cpp#L92) supplies the foot-tracking residual.
Default references, impact factors and optional foothold overrides are prepared at [line 127](../src/mpc/common/cost/EndEffectorFootCost.cpp#L127).
Named link costs are registered at [WbIdMpcInterface.cpp:127](../src/mpc/wb_id/WbIdMpcInterface.cpp#L127) using [EndEffectorKinematicsQuadraticCost.cpp](../src/mpc/common/cost/EndEffectorKinematicsQuadraticCost.cpp).
Their reference link kinematics are computed at [EndEffectorKinematicsQuadraticCost.cpp:96](../src/mpc/common/cost/EndEffectorKinematicsQuadraticCost.cpp#L96), and their quaternion-distance residual is defined at [EndEffectorKinematicCostHelpers.cpp:81](../src/mpc/common/cost/EndEffectorKinematicCostHelpers.cpp#L81).
[ExternalTorqueQuadraticCostAD.cpp:110](../src/mpc/common/cost/ExternalTorqueQuadraticCostAD.cpp#L110) maps $`J_c^{\mathsf T}W_c`$ to selected joints and gates the residual using the opposite foot's impact factor.
It is registered independently of the input torque regularizer at [WbIdMpcInterface.cpp:217](../src/mpc/wb_id/WbIdMpcInterface.cpp#L217).
The builder maps `taskSpaceFootCost.activate` at [line 270](../src/controller/config_builder.cpp#L270) and `legTorqueCost.activate` at [line 259](../src/controller/config_builder.cpp#L259), and filters disabled `taskSpaceCosts.<name>.activate` entries before converting their weights at [line 304](../src/controller/config_builder.cpp#L304).
The interface checks the named-link flag before constructing its CppAD kinematics and guards both foot-cost and leg-load-cost construction at [line 211](../src/mpc/wb_id/WbIdMpcInterface.cpp#L211), following `mpc_controllers`' configure-time `activate` pattern.
All switches default to `true`; `false` removes only the corresponding extra cost, not the main Q/R objective or any contact/dynamics constraint.
Use `taskSpaceFootCost.weights` instead of the old WB-ID `taskSpaceFootCostWeights` key.

### 4.4 Gravity-relative torso tilt cost

[WbIdTorsoTiltCost.h](../include/legged_robot_mpc_controller/mpc/wb_id/cost/WbIdTorsoTiltCost.h) defines the opt-in configuration and the four-component Gauss--Newton residual.
The builder maps `ocs2.costs.torsoTiltCost` at [line 293](../src/controller/config_builder.cpp#L293), and [WbIdMpcInterface.cpp:121](../src/mpc/wb_id/WbIdMpcInterface.cpp#L121) conditionally registers the running cost as `torsoTiltCost`.
[WbIdTorsoTiltCost.cpp:22](../src/mpc/wb_id/cost/WbIdTorsoTiltCost.cpp#L22) validates the tracked frame, positive free angle below $`\pi/2`$, nonnegative finite weights and nonzero finite gravity.
World up, the free-angle cosine, the free angle and square-root weights are passed as seven AD parameters at [line 38](../src/mpc/wb_id/cost/WbIdTorsoTiltCost.cpp#L38), so reconfiguration does not silently reuse old tuning values from generated libraries.
The model cache uses a new `wb_id_torso_tilt_v1_<link>` name, distinct from the generic link tracker.
Forward kinematics and physical world-aligned angular velocity are evaluated from WB-ID $`q,v`$ at [line 66](../src/mpc/wb_id/cost/WbIdTorsoTiltCost.cpp#L66).
The free corridor is implemented by clamping the cosine before `acos` at [line 74](../src/mpc/wb_id/cost/WbIdTorsoTiltCost.cpp#L74); this keeps the inactive upright branch differentiable without evaluating `acos(1)`.
The transverse angular-velocity projection and weighted residual are at [line 79](../src/mpc/wb_id/cost/WbIdTorsoTiltCost.cpp#L79).
No desired state/input trajectory is read, and no position, yaw or CoM constraint is added.
The shipped G1 YAML enables this cost and disables `taskSpaceCosts.torso`; the generic tracker remains available for deliberate reference-tracking tasks.
For intentional bending, widen `freeTiltAngle` in radians, reduce `tiltWeight`, or disable this soft term, and consider the separate pelvis/waist posture costs.
`angularVelocityWeight` still damps transverse motion inside the free-angle corridor.
[wb_id_torso_tilt_cost_test.py](../tests/wb_id_torso_tilt_cost_test.py) checks the reference math and source wiring, not the compiled C++ cost or simulation performance.


## 5. Commands, Gait and Swing

### 5.1 Base command conversion

[Ros2ProceduralMpcMotionManager.cpp:18](../src/common/ros2_procedural_mpc_motion_manager.cpp#L18) bounds incoming walking commands.
Walking velocity, `PoseStamped` and target-mode subscriptions are created at [line 100](../src/common/ros2_procedural_mpc_motion_manager.cpp#L100).
[WalkingVelocityTarget.cpp:62](../src/mpc/common/target/WalkingVelocityTarget.cpp#L62) scales normalized velocity axes.
Filtering, resets, elapsed-time clamping and slew limits are at [line 72](../src/mpc/common/target/WalkingVelocityTarget.cpp#L72).
The stateful low-pass is [BreakFrequencyAlphaFilter.h:48](../include/legged_robot_mpc_controller/mpc/common/reference_manager/BreakFrequencyAlphaFilter.h#L48), separate from observation velocity filtering.

[WbIdMpcTargetTrajectoriesCalculator.cpp:63](../src/mpc/wb_id/command/WbIdMpcTargetTrajectoriesCalculator.cpp#L63) constructs three body-twist reference knots.
The exact planar constant-twist integral is [TargetTrajectoriesCalculatorBase.cpp:132](../src/mpc/common/command/TargetTrajectoriesCalculatorBase.cpp#L132).
Explicit pose conversion is [WbIdMpcTargetTrajectoriesCalculator.cpp:47](../src/mpc/wb_id/command/WbIdMpcTargetTrajectoriesCalculator.cpp#L47).
Target-mode changes request filter reset and pose latching at [ProceduralMpcMotionManager.cpp:111](../src/mpc/common/reference_manager/ProceduralMpcMotionManager.cpp#L111).
[BasePoseTarget.cpp](../src/mpc/common/target/BasePoseTarget.cpp) implements pose-command evaluation and latching.

### 5.2 Gait and swing generation

[MotionPhaseDefinition.h:45](../include/legged_robot_mpc_controller/mpc/common/gait/MotionPhaseDefinition.h#L45) defines supporting-foot mode flags.
[ProceduralMpcMotionManager.cpp:126](../src/mpc/common/reference_manager/ProceduralMpcMotionManager.cpp#L126) implements faster/slower gait guards.
Per-solver target dispatch, measured-speed checks and template selection start at [line 173](../src/mpc/common/reference_manager/ProceduralMpcMotionManager.cpp#L173).
[GaitScheduleUpdater.cpp:45](../src/mpc/common/gait/GaitScheduleUpdater.cpp#L45) delays template insertion, while [GaitSchedule.cpp](../src/mpc/common/gait/GaitSchedule.cpp) extends the schedule.
The WB-ID gait templates, including supported walking and running, start at [ros2_controllers.yaml:686](../config/g1/ros2_controllers.yaml#L686).
[SwitchedModelReferenceManager.cpp:221](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L221) combines schedule, support heights and swing references.
[config_builder_utils.hpp](../include/legged_robot_mpc_controller/common/config/config_builder_utils.hpp#L130) loads optional per-template `swingHeight`, with -1 inheriting `gait.swingTrajectory.swingHeight`.
[GaitSchedule.cpp](../src/mpc/common/gait/GaitSchedule.cpp#L52) preserves per-phase height metadata when inserting, trimming and extending gait templates.
[SwingTrajectoryPlanner.cpp:144](../src/mpc/common/swing_foot_planner/SwingTrajectoryPlanner.cpp#L144) selects the lift-off phase's height and constructs one complete swing spline with an apex above the higher endpoint, shared across all consecutive airborne modes of that foot.
The [flight-spline regression](../tests/swing_flight_test.cpp) checks internal-mode continuity, overrides, fallback, duration scaling, terrain endpoints, invalid inputs, ordinary walking and event-free stance.
The [gait-height regression](../tests/gait_swing_height_test.cpp) checks metadata alignment through gait insertion, rolling-horizon trimming, future replacement and external schedule reset.
See [running validation](./wb_id_running.md) and the [warm-start transition regression](../tests/running_gait_transition_test.cpp) for the separate OCS2 dependency fix required for safe trajectory truncation.
The WB-ID builder maps `ocs2.costs.captureFootPlacement` at [line 283](../src/controller/config_builder.cpp#L283); it is off by default and enabled only in the H2 configuration (see [H2 to 2 m/s](./wb_id_running.md#h2-to-2-ms)).
Shared stair/terrain setup is connected at [controller line 234](../src/controller/controller.cpp#L234), which should not be interpreted as validated WB-ID terrain performance.


### 5.3 Horizontal swing and foothold selection

The equations and prescribed-versus-optimized distinction are in [formulation Section 6.5](./humanoid_whole_body_inverse_dynamics_mpc.md#65-horizontal-swing-motion-and-foothold-selection).
In ordinary `base_twist`, there is no independent foot-XY or touchdown-position optimization variable: positions are obtained by forward kinematics of the optimized whole-body state.

| Behavior | Current implementation |
| --- | --- |
| Whole-body configuration is an optimized state | [WbIdMpcRobotModel.h:37](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L37) defines the state/input dimensions; [line 65](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L65) extracts generalized coordinates; [WbIdDynamics.cpp:18](../src/mpc/wb_id/dynamics/WbIdDynamics.cpp#L18) integrates velocity/acceleration |
| Optional flat-ground foothold target (off by default; on for H2) | [Builder:283](../src/controller/config_builder.cpp#L283) maps `captureFootPlacement`; when enabled, each swing foot gets an xy target `stanceTimeGain * v * T_stance` ahead of the predicted base until its touchdown, tracked with `trackingWeight` (see [Section 5.4](#54-flat-ground-foothold-target-capturefootplacement)) |
| Ordinary walking generates height references only | [Reference manager:312](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L312) selects the normal ground-height/swing-planner path; [SwingTrajectoryPlanner.cpp:144](../src/mpc/common/swing_foot_planner/SwingTrajectoryPlanner.cpp#L144) builds the vertical spline |
| Swing equality leaves XY motion free of a prescribed path | [WbIdFootAccelerationConstraintCppAd.cpp:107](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L107) returns only the vertical acceleration/velocity/position residual during swing |
| Stance stabilizes XY velocity, not XY position | [Constraint:115](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L115) uses horizontal acceleration plus velocity, without an XY position-reference term |
| Foot task-space quantities come from optimized kinematics | [Foot cost:104](../src/mpc/common/cost/EndEffectorFootCost.cpp#L104) extracts the model's generalized coordinates/velocities and evaluates foot FK and twist; this shared class is [registered by WB-ID](../src/mpc/wb_id/WbIdMpcInterface.cpp#L210) |
| Zero position reference is harmless only with zero position weights | [Foot cost:135](../src/mpc/common/cost/EndEffectorFootCost.cpp#L135) initializes the default position reference to zero; current [G1 weights](../config/g1/ros2_controllers.yaml#L395) and [H2 weights](../config/h2/ros2_controllers.yaml#L416) set all three position weights to zero; when the [foothold target](#54-flat-ground-foothold-target-capturefootplacement) overrides the x/y weights it also supplies the reference |
| Horizontal foot speed is regularized, not prescribed | [Foot residual:115](../src/mpc/common/cost/EndEffectorFootCost.cpp#L115) multiplies the linear-velocity error by the impact-proximity factor; [reference:155](../src/mpc/common/cost/EndEffectorFootCost.cpp#L155) sets desired linear velocity to zero; [weight mapping:58](../src/mpc/common/cost/EndEffectorKinematicCostHelpers.cpp#L58) identifies the position/orientation/linear/angular blocks |
| Other objectives select among possible placements | [OCP cost registration:113](../src/mpc/wb_id/WbIdMpcInterface.cpp#L113) adds base, joint and state/input objectives; [H2 posture weights](../config/h2/ros2_controllers.yaml#L354) include the tuned hip-pitch preference |
| Anti-crossing is a soft preference, not a foothold generator | [FootCollisionConstraint.cpp:146](../src/mpc/common/constraint/FootCollisionConstraint.cpp#L146) adds the heading-relative lateral margin; [H2 settings](../config/h2/ros2_controllers.yaml#L527) enable it |
| Explicit terrain/stair modes can add XY guidance | [Reference selection:499](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L499) selects available foothold references; [foot cost:144](../src/mpc/common/cost/EndEffectorFootCost.cpp#L144) replaces the reference and horizontal position weights when one is supplied |

The scheduled contact flags select stance/swing equalities at [constraint line 61](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L61), rather than allowing the solver to choose touchdown timing or detect terrain contact through complementarity.
The planned touchdown XY location is therefore an optimized kinematic outcome at a prescribed event time, not proof of the actual contact location or safety on unknown terrain.

### 5.4 Flat-ground foothold target (`captureFootPlacement`)

The equations are in [formulation Section 6.6](./humanoid_whole_body_inverse_dynamics_mpc.md#66-optional-flat-ground-foothold-target-capturefootplacement).
The target is computed once per solve on the solver thread and is read by the foot cost's parameter evaluation in the same solve.

| Step | Implementation |
| --- | --- |
| Settings and defaults (all off) | [SwitchedModelReferenceManager.h:84](../include/legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h#L84); the running additions `stanceTimeGain` at [line 145](../include/legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h#L145) and `currentSwingOnly` at [line 149](../include/legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h#L149) |
| WB-ID schema, `enabled` default false | [parameter YAML:195](../src/controller/parameters.yaml#L195) |
| YAML to settings | [Builder:278](../src/controller/config_builder.cpp#L278) maps every field |
| Settings to the reference manager | [WbIdMpcInterface.cpp:84](../src/mpc/wb_id/WbIdMpcInterface.cpp#L84) |
| Per-solve update | `preSolverRun` calls `updateCaptureFootholds` at [line 322](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L322), after the solve's mode schedule is stored |
| Disabled path | [line 331](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L331) returns with no valid target, so the lookup below never supplies one |
| Capture point $\xi$ | [line 348](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L348); $v$ is the state's world-frame base velocity ([WbIdMpcRobotModel.h:86](../include/legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h#L86)) |
| $t_{\mathrm{TD},j}$ and $T_{\mathrm{st},j}$ per foot | [line 400](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L400) searches the solve's mode sequence; airborne feet come from the same schedule |
| $\hat p_{b,j}$ | [line 435](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L435) |
| Forward lead $(k_L T_{\mathrm{step}}+k_s T_{\mathrm{st},j})v$ | [line 440](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L440) |
| $p^{\mathrm{nom}}_j$ with centre offset and half width | [line 450](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L450) |
| $\xi_{\mathrm{TD},j}$ and clipped $\delta_j$ | [line 459](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L459) and [line 467](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L467) |
| Store $r_j$ and $t_{\mathrm{TD},j}$ | [line 473](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L473) and [line 476](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L476) |
| Lookup, precedence and window | [getSwingFootholdReference:499](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L499) returns the target only without a stair plan or `terrain_walk`; [line 508](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L508) ends it at $t_{\mathrm{TD},j}$ when `currentSwingOnly` is set |
| Cost term $\ell^{\mathrm{fh}}_j$ | [Foot cost:144](../src/mpc/common/cost/EndEffectorFootCost.cpp#L144) queries the target per node, replaces the $x/y$ position weights with $\sqrt{w_{\mathrm{tr}}}$ at [line 147](../src/mpc/common/cost/EndEffectorFootCost.cpp#L147) and sets the reference at [line 153](../src/mpc/common/cost/EndEffectorFootCost.cpp#L153); weights and reference are CppAD parameters, so no code regeneration is needed |
| H2 values | [config/h2/ros2_controllers.yaml:449](../config/h2/ros2_controllers.yaml#L449) |
| Static check | `test_h2_running_foothold_reference` in [tests/test_wb_id_running.py](../tests/test_wb_id_running.py) |

To return H2 to optimizer-only foot placement, set `ocs2.costs.captureFootPlacement.enabled: false`; restoring its pre-2 m/s behaviour also needs the other settings listed in [H2 to 2 m/s](./wb_id_running.md#h2-to-2-ms).


## 6. Arm Command Support

The WB-ID controller passes tracked arm names to `subscribeMpcTargets` at [controller line 250](../src/controller/controller.cpp#L250).
The shared subscriber dispatches through the parser at [Ros2ProceduralMpcMotionManager.cpp:150](../src/common/ros2_procedural_mpc_motion_manager.cpp#L150).

| `command_type` | WB-ID behaviour | Code |
| --- | --- | --- |
| `joint` | Parses one arm-joint trajectory for `JointTrackingCost` | [mpc_targets_parser.cpp:72](../src/common/target/mpc_targets_parser.cpp#L72) |
| `default` | Clears the external joint target | [Parser:80](../src/common/target/mpc_targets_parser.cpp#L80) |

The WB-ID OCP registers `JointTrackingCost` for arm-joint commands.
Unsupported command types are rejected before changing the reference.


## 7. Solver, Policy and Actuation

[construct_solver():1726](../src/controller/controller.cpp#L1726) accepts only `sqp`, constructs `SqpMpc`, connects MRT and installs the synchronized modules.
The asynchronous worker starts at [line 1691](../src/controller/controller.cpp#L1691).
The control loop forwards each valid observation through OCS2's existing `setCurrentObservation()` before policy checks or bring-up dispatch at [line 677](../src/controller/controller.cpp#L677).
After construction, the worker waits for a new control-loop observation before its first solve at [line 1719](../src/controller/controller.cpp#L1719).
Policy handover checks finite timestamps and requires the evaluation time to lie inside the plan, with two control periods remaining, at [line 690](../src/controller/controller.cpp#L690).
An unusable policy requests `resetMpcNode()` on the solver worker, following the atomic warm-start reset pattern in `dynamics_mpc_controller`, rather than resetting the solver from the control thread.
Startup keeps holding the standing posture and retries; loss of a valid policy during the startup blend returns to `StartingMpc`, while loss after entering estimator-driven `Locomotion` latches the existing `Fault` state.
The ground-truth feedback path holds the last commanded posture while awaiting a valid replacement policy.
No additional age-limit parameter or alternative observation interface is introduced.
[compute_mpc_joint_action():1917](../src/controller/controller.cpp#L1917) evaluates the policy using `ocs2.solver.rollout.policyTimeOffset`.
The same offset is used by policy-interval validation, so the accepted interval and execution time remain consistent; it is a runtime policy look-ahead, not an ODE timestep.
`MRT_BASE::evaluatePolicy()` interpolates the planned state and evaluates its input controller without calling `rolloutPolicy()`.
WB-ID therefore has no `TimeTriggeredRollout` object or MRT `initRollout()` call; its obsolete ODE settings and unused DDP settings storage were removed.
Planned joint position/velocity extraction is at [line 1933](../src/controller/controller.cpp#L1933).
The torque-window check and planned-state inverse-dynamics fallback are at [line 1966](../src/controller/controller.cpp#L1966).
The alternative measured-state inverse dynamics is at [line 1981](../src/controller/controller.cpp#L1981).
Feedforward clipping is [clamp_torque():1909](../src/controller/controller.cpp#L1909).
Joint position/velocity/effort and optional gain interface writes are at [line 2303](../src/controller/controller.cpp#L2303).

`python3 tests/wb_id_startup_contract_test.py -v` checks the startup observation flow, policy bounds, reset ownership and bring-up guards without a ROS build.
These are static regression checks, not a substitute for repeated startup tests in the simulator.


## 8. Parameter Map

The optional WB-ID `ocs2.solver.sqp.timeGrid` is declared in [the parameter schema](../src/controller/parameters.yaml) and mapped/validated in [the configuration builder](../src/controller/config_builder.cpp).
Its `activate` flag defaults false; only enabled WB-ID configurations populate the new OCS2 `dtMax`, `dtGrowth` and `dtFineHorizon` settings.
The builder does not require `fineHorizon` to cover the torque window; joint inverse dynamics and torque-limit penalties retain their time-based activation on either fine or coarse nodes.
OCS2 implements `timeDiscretizationWithGeometricSteps()` in `core/ocs2_oc/src/oc_data/TimeDiscretization.cpp`; `mpc/ocs2_sqp/ocs2_sqp/src/SqpSolver.cpp` selects it only when `dtMax > 0`.
Other controllers retain uniform spacing unless they explicitly opt in.
The native `testTimeDiscretization.cpp` covers fine-window spacing, capped tail growth, contact-event preservation and legacy behavior.
Diagnostics append `grid_nodes`, `grid_min_dt`, `grid_max_dt` and `grid_fine_max_dt` to the best-effort solver topic when tracking diagnostics are enabled.
Run [nonuniform_grid_benchmark.py](../tests/nonuniform_grid_benchmark.py) in the sourced workspace, then [analyze_time_grid.py](../tests/tuning/analyze_time_grid.py) locally for timing/residual/motion plots.
The walking harness sets the launch working directory to the workspace root so generated libraries remain in `ocs2_ros2_ws/auto_generated`, including with a Mac-shared source checkout.


| Path inside the WB-ID controller block | Role |
| --- | --- |
| `robot.jointNames`, `ocs2.model` | Reduced model, contact frames and array ordering |
| `floatingBase.source`, `floatingBase.estimator` | Body state feeding the MPC |
| `control.observationVelocityFilterCutoffHz` | Observation generalized velocity filter |
| `ocs2.mpc` | MPC backend selection, horizon, rates, warm-start and policy window |
| `ocs2.solver.sqp`, `ocs2.solver.rollout` | SQP numerics and policy execution offset only; no ODE rollout settings |
| `ocs2.codegen` | CppAD generation verbosity and forced regeneration |
| `ocs2.model.contactGeometry` | Physical contact-frame offset and sole rectangle |
| `ocs2.constraints.inverseDynamics.torqueConstraintNodes` | Initial inverse-dynamics/torque-limit window |
| `ocs2.costs.q`, `qFinal`, `r` | State/input weight blocks |
| `ocs2.costs.qScaling`, `rScaling`, `terminalCostScaling` | Global weight scaling |
| `ocs2.costs.taskSpaceFootCost.activate`, `.weights` | Enable both foot costs and set their weights |
| `ocs2.costs.taskSpaceCosts.<name>.activate`, `.linkName`, `.weights` | Enable and configure each named link cost |
| `ocs2.costs.torsoTiltCost` | Soft gravity-relative tilt corridor and transverse angular-motion weight |
| `ocs2.costs.captureFootPlacement` | Optional flat-ground swing-foot xy target, off by default ([Section 5.4](#54-flat-ground-foothold-target-capturefootplacement)) |
| `ocs2.costs.legTorqueCost.activate`, `.scaling`, leg joint names and weights | Enable and configure both external-load costs |
| `ocs2.constraints.footMotionEquality` | Acceleration-level stance/swing gains |
| `ocs2.constraints.swingFootZeroWrench.activate` | Register the hard zero-wrench equality for swing feet |
| `ocs2.constraints.legSelfCollision.activate` | Register selected foot/ankle/knee soft separation margins |
| `ocs2.constraints.jointPositionLimits.activate` | Register soft URDF lower/upper position bounds |
| `ocs2.constraints.jointVelocityLimits.activate`, `.velocityScaling` | Register soft URDF velocity bounds multiplied by a scaling in `(0, 1]` |
| `ocs2.constraints` | Friction, CoP, joint and collision soft penalties |
| `ocs2.gait.phaseTransitionStanceTime`, `templates`, `transitions`, `swingTrajectory` | Contact timing, transition guards and swing profiles |
| `ocs2.reference` | Command scaling/slew, nominal posture, height and arm swing |
| `ocs2.reference.groundHeight` | FK support-height reference option and per-solve rate limit |
| `ocs2.reference.armSwing.jointNames` | Joint identities used by the arm-swing reference and model index setup |
| `target.mpcTargetsTopic` | Arm joint command input |
| `control.feedforward` | Feedforward selection |
| `ocs2.constraints.jointTorqueLimits.activate`, `.limits`, `.torqueScaling`, `.mu`, `.delta` | Omitted limits use URDF efforts; scaled bounds are shared with runtime feedforward clipping, which remains active independently of the OCP activation flag |
| `ocs2.solver.rollout.policyTimeOffset` | Policy look-ahead time used by validation and execution |
| `control.jointEffortPd.kp`, `kd` | Inner servo gains |

Read these from the `humanoid_whole_body_inverse_dynamics_mpc_controller` YAML block.
The unused SQP `inequalityConstraintMu` and `inequalityConstraintDelta` parameters have been removed from WB-ID; the linked SQP implementation does not read them.
Actual inequality soft penalties retain their per-term `constraints.<name>.mu` and `.delta` settings.
`printSolverStatistics` is retained because it prints a summary at solver destruction; the other SQP switches remain functional even when their configured value is false.
The schema now lists only the implemented `sqp` backend rather than accepting an unsupported `ddp` selection.
Run [wb_id_solver_contract_test.py](../tests/wb_id_solver_contract_test.py) to check G1/H2 WB-ID schema coverage and removed dead paths.
The package no longer declares a direct `ocs2_ddp` dependency; the shared OCS2 stack may still depend on DDP transitively.
Run [sqp_only_contract_test.py](../tests/sqp_only_contract_test.py) to check the supported solver, parameter schema, execution path, and direct build dependencies.
The startup regression checks above are static and do not constitute runtime validation.


## 9. Blind Walking Tuning and Validation

The measured results, tuning table and limitations are in [Formulation and Signal Flow, Section 9](./humanoid_whole_body_inverse_dynamics_mpc.md#9-blind-walking-robustness-and-tuning).
The references below map that table to the current implementation; only the paired contact-gain reduction was isolated in the ramp comparison.

| Responsibility | Implementation or current parameters |
| --- | --- |
| Stance height/tilt correction gains | [ros2_controllers.yaml:443](../config/g1/ros2_controllers.yaml#L443), [stance equations:116](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L116) |
| Shared gain in swing vertical correction | [WbIdFootAccelerationConstraintCppAd.cpp:109](../src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L109) |
| Running/terminal base z and vz weights | [Q:335](../config/g1/ros2_controllers.yaml#L335), [Qf:351](../config/g1/ros2_controllers.yaml#L351) |
| Foot task-space cost | [weights:388](../config/g1/ros2_controllers.yaml#L388) |
| Swing clearance | [swingHeight:669](../config/g1/ros2_controllers.yaml#L669) |
| Torso free lean and angular damping | [torsoTiltCost:407](../config/g1/ros2_controllers.yaml#L407) |
| Extra leg-load cost activation | [legTorqueCost:415](../config/g1/ros2_controllers.yaml#L415) |
| Joint PD and feedforward | [control:236](../config/g1/ros2_controllers.yaml#L236) |
| Command slew limits and filtering | [limits:634](../config/g1/ros2_controllers.yaml#L634), [WalkingVelocityTarget.cpp:90](../src/mpc/common/target/WalkingVelocityTarget.cpp#L90) |
| Planar-relative base tracking | [BaseMotionTrackingCost.cpp:51](../src/mpc/common/cost/BaseMotionTrackingCost.cpp#L51) |
| Mapped terrain planner inactive outside its explicit mode | [mode gate:189](../src/mpc/common/reference_manager/ProceduralMpcMotionManager.cpp#L189), [world-up fallback:266](../include/legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h#L266) |
| Current flat support-height assumption | [option:638](../config/g1/ros2_controllers.yaml#L638), [reference fallback:126](../src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp#L126) |
| Linear KF flat contact-height observation | [zero heights:159](https://github.com/wei-hsuan-cheng/legged_robot_state_estimator/blob/main/src/linear_kf_floating_base_estimator.cpp#L159), [measurement:303](https://github.com/wei-hsuan-cheng/legged_robot_state_estimator/blob/main/src/linear_kf_floating_base_estimator.cpp#L303) |
| Scheduled estimator contacts | [contact source:165](../config/g1/ros2_controllers.yaml#L165) |
| Automatic recording and scoring | [run_case.sh:35](../tests/tuning/run_case.sh#L35), [command schedule:76](../tests/tuning/tune_probe.py#L76) |
| Whole-run comparison and recorded-state images | [plot_ramp.py:17](../tests/tuning/plot_ramp.py#L17), [replay:69](../tests/tuning/plot_ramp.py#L69) |

Trial folders such as `tuning_results/ramp_rebuilt_baseline`, `ramp_final_headless`, `ramp_final_slow` and `ramp_final_rviz` contain exact configs, logs, arrays and plots on the development machine.
They are git-ignored and are not bundled with this document; [Codex.md](../Codex.md) preserves the trial identifiers and numerical summary.
Replay images reconstruct recorded physical poses and encoders rather than showing a live viewer or an estimated terrain map.


## 10. WB-ID Parameter Layout Migration

These paths are relative to `ocs2` unless a full path is shown; this migration is WB-ID only.
The original layout migration preserved configured values, schema types/defaults/validation and blind-ramp gains; the subsequent activation and scaling additions are described below.
`ocs2.mpc` deliberately remains outside `ocs2.solver`.

| Previous path | Current path |
| --- | --- |
| `model.torqueConstraintNodes` | `constraints.inverseDynamics.torqueConstraintNodes` |
| `model.phaseTransitionStanceTime` | `gait.phaseTransitionStanceTime` |
| `model.useTerrainHeightEstimate` | `reference.groundHeight.useEstimate` |
| `model.maxTerrainHeightStep` | `reference.groundHeight.maxStepPerSolve` |
| `model.verboseCppAd` | `codegen.verbose` |
| `model.recompileLibrariesCppAd` | `codegen.recompileLibraries` |
| `model.armJointNames` | `reference.armSwing.jointNames` |
| `constraints.contact.frameTranslation`, `.rectangle` | `model.contactGeometry.frameTranslation`, `.rectangle` |
| `sqp` | `solver.sqp` |
| `rollout` | `solver.rollout` |
| Full path `control.policyTimeOffset` | Full path `ocs2.solver.rollout.policyTimeOffset` |
| Full path `control.torqueLimit` | Full path `ocs2.constraints.jointTorqueLimits.limits` |
| `constraints.jointLimits` | `constraints.jointPositionLimits` |
| `constraints.jointVelocityLimits.scale` | `constraints.jointVelocityLimits.velocityScaling` |
| `constraints.footConstraint` | `constraints.footMotionEquality` |
| `constraints.zeroWrench` | `constraints.swingFootZeroWrench` |
| `constraints.footCollision` | `constraints.legSelfCollision` |

Arm reference joint names remain required even when arm swing is disabled because common-model setup resolves their indices unconditionally.
Physical contact geometry still feeds the same contact-frame construction and CoP bounds; barrier settings remain under `constraints`.
Both policy-coverage validation and policy execution read the relocated time offset.
The shared torque-limit behavior and pre-existing acceleration-gain schema defaults are unchanged; neither is silently retuned or split by this layout change.
The limit array supplies both the soft OCP torque margins inside the inverse-dynamics torque window and the hard runtime feedforward clamp; final feedback-plus-feedforward torque is not bounded by this controller clamp.
Omitting the override uses the reduced Pinocchio model's URDF effort limits, following the existing URDF/model-based position-limit pattern in `mpc_controllers` and the WB-ID velocity-limit implementation.
The schema's typed empty-vector default is valid, but an explicit `limits: []` runtime YAML override is not: Jazzy reports `parameter_value_from failed` with `No parameter value set` before controller initialization.
`WbIdMpcInterface` [resolves and validates limits during construction](../src/mpc/wb_id/WbIdMpcInterface.cpp#L57), and `getTorqueLimits()` supplies that same vector to the [runtime controller](../src/controller/controller.cpp#L124); missing, nonpositive or nonfinite resolved bounds fail configuration rather than silently disabling protection.
`mu` scales the relaxed log barrier and `delta` sets the margin where its quadratic continuation begins; neither parameter affects runtime clipping.
Update external WB-ID YAML files and dotted overrides, then rebuild the generated parameter library and controller before using these new paths; no deprecated aliases are provided.
`tests/wb_id_parameter_contract_test.py` checks schema/builder mapping, removal of obsolete parameter paths, and both policy-offset readers.
Run `python3 tests/wb_id_startup_smoke_test.py` from a sourced ROS workspace to test real WB-ID startup on an isolated generated flat scene, with MuJoCo headless, RViz and the command GUI disabled; it terminates its own launch group immediately after the `-> locomotion` transition and saves its scene, log and JSON verdict under `/tmp/wb_id_startup`.


## 11. Constraint Activation and Bound Scaling

All nine WB-ID constraint groups now declare `activate` with a default of true; `swingFootZeroWrench` exposes the existing swing-wrench equality separately from stance/swing acceleration equalities.
`footMotionEquality` names the existing stance/swing acceleration equalities, while `legSelfCollision` names the selected foot/ankle/knee separation checks rather than full-body collision avoidance.
Only WB-ID parameter groups and their private activation fields were renamed; the shared `ZeroWrenchConstraint`, `FootCollisionConstraint` and `ModelSettings::footConstraintConfig` class/field names remain unchanged.
The later optional lateral-separation extension uses a distinct AD cache name when enabled.
The following links point to the WB-ID YAML and construction guards:

| Constraint group | Current parameters | OCP registration |
| --- | --- | --- |
| `inverseDynamics` | [YAML:438](../config/g1/ros2_controllers.yaml#L438) | [RNEA rows:167](../src/mpc/wb_id/WbIdMpcInterface.cpp#L167) |
| `footMotionEquality` | [YAML:443](../config/g1/ros2_controllers.yaml#L443) | [Acceleration equalities:197](../src/mpc/wb_id/WbIdMpcInterface.cpp#L197) |
| `swingFootZeroWrench` | [YAML:457](../config/g1/ros2_controllers.yaml#L457) | [Zero-wrench equality:194](../src/mpc/wb_id/WbIdMpcInterface.cpp#L194) |
| `frictionCone` | [YAML:460](../config/g1/ros2_controllers.yaml#L460) | [Soft cone:187](../src/mpc/wb_id/WbIdMpcInterface.cpp#L187) |
| `contactMomentXY` | [YAML:466](../config/g1/ros2_controllers.yaml#L466) | [Soft CoP:190](../src/mpc/wb_id/WbIdMpcInterface.cpp#L190) |
| `jointPositionLimits` | [YAML:471](../config/g1/ros2_controllers.yaml#L471) | [Soft position bounds:142](../src/mpc/wb_id/WbIdMpcInterface.cpp#L142) |
| `jointVelocityLimits` | [YAML:477](../config/g1/ros2_controllers.yaml#L477) | [Scaled soft velocity bounds:154](../src/mpc/wb_id/WbIdMpcInterface.cpp#L154) |
| `jointTorqueLimits` | [YAML:484](../config/g1/ros2_controllers.yaml#L484) | [Soft torque bounds:175](../src/mpc/wb_id/WbIdMpcInterface.cpp#L175) |
| `legSelfCollision` | [YAML:609](../config/g1/ros2_controllers.yaml#L609) | [Soft leg separation:145](../src/mpc/wb_id/WbIdMpcInterface.cpp#L145) |

The generated schema declares the renamed groups at [footMotionEquality:236](../src/controller/parameters.yaml#L236), [swingFootZeroWrench:328](../src/controller/parameters.yaml#L328) and [legSelfCollision:274](../src/controller/parameters.yaml#L274), and the [builder:230](../src/controller/config_builder.cpp#L230) maps all nine activation flags.
The builder maps each switch to a same-purpose `WbIdMpcInterface::Config` field, and the interface skips construction and registration when disabled through the common factory.
`jointPositionLimits` calls the existing factory, which reads `lowerPositionLimit` and `upperPositionLimit` from the reduced URDF model; velocity bounds use `velocityScaling * model.velocityLimit`, and torque bounds use `torqueScaling * resolvedSourceLimits`.
Both scalings default to 1.0 and are validated as finite values in `(0, 1]` by the interface; generated parameters additionally enforce `gt: [0.0]` and `lt_eq: [1.0]`.
The resolved scaled torque vector is shared with the existing runtime feedforward clamp, which remains independent of the OCP torque activation flag and is not a final feedback-plus-feedforward actuator clamp.
The controller warns if dynamics/contact equalities are disabled and uses measured-state RNEA instead of free policy torque variables when inverse dynamics is disabled; these ablations are not validated locomotion modes.
`tests/wb_id_constraint_activation_contract_test.py` checks all 512 switch combinations against the schema, guarded registrations, URDF bound readers, scaling validation and preservation of feedforward clipping; these static checks do not establish solver feasibility for disabled-constraint combinations.


## 12. Future Robot Mesh Ghosts

The visualization settings are declared in the WB-ID schema and configured in each robot's YAML.
`optimizedStateRobotModelsActivate` and `optimizedStateRobotModelsInterval` retain the legacy names and defaults `true` and `0.2 s`; there is no launch argument or separate ghost horizon.

| Stage | Implementation |
| --- | --- |
| Consistent policy snapshot | [Controller handoff](../src/controller/controller.cpp#L741) passes policy timestamps/states and the matching MRT command targets at the existing maximum 10 Hz handoff rate |
| Subscriber-driven copying | [Worker input](../src/visualization/performance_visualization.cpp#L71) returns before copying when no display is subscribed; the [worker](../src/visualization/performance_visualization.cpp#L81) performs FK/publishing only for subscribed displays and fresh snapshots |
| Robot-independent visuals | [URDF extraction](../src/visualization/future_robot_model_visualization.cpp#L70) retains visual origins, mesh scales, multiple visuals and primitive geometry on BODY frames, including reduced-model fixed links |
| Correct state layout | [State-to-pose conversion](../src/visualization/future_robot_model_visualization.cpp#L148) uses `MpcRobotModelBase::getGeneralizedCoordinates`, not the full WB-ID state as a Pinocchio configuration |
| Temporal resolution | [Sampling](../src/visualization/future_robot_model_visualization.cpp#L186) uses the actual policy start/end, rounds horizon/interval to the number of ghosts, then distributes samples equally over that horizon including its endpoint, as in the legacy controller |
| Displays and QoS | [RViz](../config/rviz/humanoid.rviz#L262) enables the best-effort future ghost MarkerArray display; optional terminal state/target topics can be added as separate MarkerArray displays; no extra TF robot publishers are required |

Future ghosts retain RGB `[0.765,0.665,0.905]` and alpha fading from `0.22` to `0.08` (a single ghost uses `0.22`).
Terminal state ghosts retain RGB `[0.180,0.220,0.300]`, alpha `0.40`; terminal target ghosts use the future color and alpha `0.30`.
Markers include a per-topic `DELETEALL` to replace prior poses, and their header uses `visualization.frameId` and the current ROS timestamp.
Terminal targets show the target trajectories associated with that policy, not a reconstructed pose satisfying every independent task-space cost.
The worker owns separate Pinocchio data and a cloned MPC robot model; it never reads the solver's live policy from another thread.
With no subscribers there is still a lightweight worker polling subscription counts, plus one-time URDF visual setup, but no ghost FK, interpolation, marker generation or periodic trajectory snapshot copying.
This changes neither the OCP nor actuator commands.

`tests/test_future_robot_visualization.py` checks parameter defaults, YAML-only controls, shared worker wiring, RViz topics and best-effort settings without ROS.
`tests/future_robot_ghosts_test.py --robot-name g1|h2 --horizon <seconds>` runs a short headless flat walk, subscribes after a subscription-free startup interval, checks the published ghost count, colors, alpha, mesh resources, poses and QoS, and lets the owned walking test stop its simulator normally.


## 13. Collision Spheres

WB-ID declares `visualization.collisionSpheres.activate`, `markerTopic` and `alpha` in its [parameter schema](../src/controller/parameters.yaml#L464).
The schema defaults activation to false; both shipped G1/H2 WB-ID YAMLs enable it with alpha 0.25 and topic `/performance_visualization/collisionSpheres`.
There are no new launch arguments or collision geometry overrides in the visualization group.

The [controller configuration](../src/controller/controller.cpp#L254) copies the resolved OCP geometry into marker settings once.
Only enabled collision groups are drawn: `legSelfCollision` supplies eight foot/ankle spheres and two knee spheres; `armSelfCollision` supplies all named arm and torso/pelvis/head spheres.
G1 therefore draws 32 spheres and H2 draws 34 with their current enabled groups.
These counts are distinct from the number of collision pairs; showing a sphere does not imply every pair of displayed spheres is constrained.
Leg spheres remain visible in double support for geometry inspection, even when the leg sphere penalty is gait-gated off; the minimum-lateral-separation row is not a sphere and is not drawn.

[CollisionSphereVisualization](../src/visualization/collision_sphere_visualization.cpp#L51) converts the observed MPC state through `getGeneralizedCoordinates()` and computes each center as the frame placement acting on the configured local offset.
The display is the latest observed pose handed to the worker, not a future ghost or the oldest policy sample.
Marker diameter is twice the collision radius; neither `minimumDistance` nor the polynomial/barrier buffer is added to the rendered radius.
Green, orange and blue namespaces distinguish legs, arms and body; RViz can hide them independently.
The markers expire after one second of ROS time without updates, so stopped poses do not persist indefinitely.

The existing [visualization worker](../src/visualization/performance_visualization.cpp#L86) owns separate Pinocchio data and a cloned robot model, receives the observation through its nonblocking snapshot handoff and performs FK/publishing only with a sphere-topic subscriber.
The existing WB-ID handoff remains capped at 10 Hz, while `visualization.update_rate` controls worker polling; setting it to 30 Hz does not turn the source handoff into 30 Hz.
The topic uses best-effort, volatile, depth-one QoS, matched by the saved [RViz display](../config/rviz/humanoid.rviz).
This adds no optimizer terms, solver-thread FK, new collision pairs or actuator behavior.
As with the existing ghosts, enabling a display still consumes CPU and adds snapshot-copy work; disabling RViz avoids subscriber-driven marker work.

`tests/test_collision_sphere_visualization.py` checks schema/configuration, shared geometry wiring, threading and RViz/QoS contracts without ROS.
`tests/collision_spheres_test.py --robot-name g1|h2 --prefix /tmp/collision_spheres_<robot>` runs an owned headless flat walk at an 80 Hz MPC request and checks late-subscriber discovery, counts, exact diameters, namespaces, finite poses, alpha, expiry and relative sphere offsets on each rigid link.
