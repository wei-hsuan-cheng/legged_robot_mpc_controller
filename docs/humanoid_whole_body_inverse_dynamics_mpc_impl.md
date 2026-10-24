# Humanoid Whole-Body Inverse Dynamics MPC: Implementation Guide

This guide maps [Formulation and Signal Flow](./humanoid_whole_body_inverse_dynamics_mpc.md) to the current code and parameters.
Links use repository-relative paths with line anchors, which may shift as the source changes.


## 1. Entry Points and Configuration

| Responsibility | Implementation |
| --- | --- |
| Lifecycle, observations and actuation | [humanoid_whole_body_inverse_dynamics_mpc_controller.cpp](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp) |
| Standalone parameter schema | [humanoid_whole_body_inverse_dynamics_mpc_controller_parameter.yaml](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller_parameter.yaml) |
| Current G1 values | [ros2_controllers.yaml:709](../config/g1/ros2_controllers.yaml#L709) |
| Config translation | [whole_body_inverse_dynamics_mpc_config_builder.cpp](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp) |
| OCP assembly | [WbIdMpcInterface.cpp:85](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L85) |
| G1 model and mesh paths | [g1_29dof.xml:16](../description/mujoco/robots/g1/g1_29dof.xml#L16), [g1_29dof.urdf:16](../description/urdf/robots/g1/g1_29dof.urdf#L16) |
| Simulation scene composition | [scene.xml:3](../description/mujoco/scene.xml#L3), [shared module layout](../description/mujoco/README.md) |
| Hardware interface description | [g1.ros2_control.xacro:17](../description/urdf/robots/g1/g1.ros2_control.xacro#L17) |

WB-ID has its own generated parameter schema rather than loading the centroidal controller schema.
The builder maps solver settings at [line 88](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L88), gait settings at [line 124](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L124) and costs at [line 171](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L171).
The interface validates the Translation + SphericalZYX dimensions at [WbIdMpcInterface.cpp:45](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L45).
The launch file selects the robot URDF at [line 56](../launch/g1.launch.py#L56), the separate RViz environment at [line 60](../launch/g1.launch.py#L60), and the top-level MuJoCo scene at [line 178](../launch/g1.launch.py#L178).
Scene names passed through `mujocoModelFile` are unchanged by the description-directory reorganization.


## 2. Model and Observation

[WbIdMpcRobotModel.h:13](../include/humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h#L13) defines $`x=[q,v]`$ and $`u=[W_L,W_R,a,\tau_j]`$.
Dimensions are constructed at [line 37](../include/humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h#L37), and block indices are defined at [line 48](../include/humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h#L48).
The inherited `getBaseComVelocity` name returns pelvis world linear velocity and Euler rates in WB-ID, not centroidal momentum.
Joint velocities are read from the state at [line 105](../include/humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h#L105), while generalized accelerations and joint torques are read from the input at [line 120](../include/humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h#L120).

[build_observation():991](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L991) combines joint interfaces and floating-base feedback.
Estimator feedback is read at [line 1017](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1017).
For simulation debugging, `floatingBaseSource:=ground_truth_state` can replace estimator feedback with GT, but this bypass is not available for real-world deployment.
Quaternion normalization, yaw unwrapping and angular velocity conversion are at [line 1056](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1056).
The optional generalized velocity low-pass is at [line 1075](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1075).
`observation.input` is zeroed at [line 1001](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1001), not filled with measured torques or wrenches.


## 3. Dynamics, Constraints and Initialization

### 3.1 Double integrator and RNEA

[WbIdDynamics.cpp:18](../src/core/humanoid_wb_id_mpc/src/dynamics/WbIdDynamics.cpp#L18) implements $`\dot x=[v,a]`$.
Its analytical linearization is initialized at [line 9](../src/core/humanoid_wb_id_mpc/src/dynamics/WbIdDynamics.cpp#L9).
[WbIdRneaConstraintCppAd.cpp:58](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdRneaConstraintCppAd.cpp#L58) evaluates RNEA and subtracts contact Jacobian transpose contributions.
Base and joint equations are registered independently at [WbIdMpcInterface.cpp:138](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L138).
The base rows are always active, while joint rows check the torque window at [WbIdRneaConstraintCppAd.cpp:54](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdRneaConstraintCppAd.cpp#L54).

### 3.2 Torque window

The builder converts `torqueConstraintNodes` into $`(N_\tau-0.5)\Delta t`$ at [line 159](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L159).
[WbIdTorqueWindow.h:32](../include/humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h#L32) implements time-based constraint activation.
The synchronized module updates the horizon start at [line 68](../include/humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h#L68).
The runtime policy check uses the policy's own interpolation timestamps at [line 43](../include/humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h#L43).
This matters when a published policy is older than the solver's current horizon.

### 3.3 Foot equalities

[WbIdFootAccelerationConstraintCppAd.cpp:61](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L61) activates stance or swing rows from scheduled contacts.
Spline position, velocity and acceleration references are obtained at [line 66](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L66).
The swing vertical equality is at [line 106](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L106), and stance equalities are at [line 112](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L112).
The class uses frame classical acceleration in world-aligned coordinates.
The config builder maps `ocs2.model.footConstraint` to these gains at [line 148](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L148).
The shared [ZeroWrenchConstraint.cpp](../src/core/humanoid_common_mpc/src/constraint/ZeroWrenchConstraint.cpp) supplies swing zero-wrench rows, registered at [WbIdMpcInterface.cpp:159](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L159).

### 3.4 Soft inequalities

| Margin | Evaluation | Penalty registration |
| --- | --- | --- |
| Friction | [FrictionForceConeConstraint.cpp:172](../src/core/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp#L172) | [HumanoidCostConstraintFactory.cpp:196](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp#L196) |
| CoP rectangle | [ContactMomentXYConstraintCppAd.cpp:86](../src/core/humanoid_common_mpc/src/constraint/ContactMomentXYConstraintCppAd.cpp#L86) | [Factory:173](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp#L173) |
| Joint positions | [JointLimitsSoftConstraint.cpp](../src/core/humanoid_common_mpc/src/constraint/JointLimitsSoftConstraint.cpp) | [Factory:158](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp#L158) |
| Joint velocities | [WbIdJointVelocityLimitConstraint.cpp](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdJointVelocityLimitConstraint.cpp) | [WbIdMpcInterface.cpp:125](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L125) |
| Joint torques | [WbIdJointTorqueLimitConstraint.cpp](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdJointTorqueLimitConstraint.cpp) | [Interface:144](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L144) |
| Foot/knee distances | [FootCollisionConstraint.cpp:89](../src/core/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp#L89) | [Factory:142](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp#L142) |

The friction surface-normal setter is unimplemented at [FrictionForceConeConstraint.cpp:62](../src/core/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp#L62).
Collision penalty deactivation during double support is at [FootCollisionConstraint.cpp:77](../src/core/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp#L77).
The factory selects `PieceWisePolynomialBarrierPenalty` for positions/collision and `RelaxedBarrierPenalty` for friction/CoP.
The WB-ID interface selects relaxed-log penalties for velocity/torque limits.
Both penalty classes come from the linked `ocs2_core` dependency, with their exact scalar equations reproduced in the formulation document.

### 3.5 Initializer

[WbIdInitializer.cpp:18](../src/core/humanoid_wb_id_mpc/src/initialization/WbIdInitializer.cpp#L18) seeds supporting forces and joint inverse-dynamics torques using zero acceleration.
Its `compute()` initially keeps `nextState = state` at [line 43](../src/core/humanoid_wb_id_mpc/src/initialization/WbIdInitializer.cpp#L43), supplying a starting guess rather than an equality-feasibility guarantee.
The running quadratic cost instead gets its nominal input from [DynamicsHelperFunctions.h:178](../include/humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h#L178), where only supporting contact forces are populated.


## 4. Costs

### 4.1 Weight partitioning and base errors

[whole_body_inverse_dynamics_mpc_config_builder.cpp:175](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L175) partitions Q/Q-final into generic, base-motion and arm-position evaluators without double counting.
R is assembled in $`[W_L,W_R,a_B,a_j,\tau_j]`$ order at [line 197](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L197).
[StateInputQuadraticCost.cpp:67](../src/core/humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp#L67) defines generic deviations and the supporting-force nominal input.
[BaseMotionTrackingCost.cpp:50](../src/core/humanoid_common_mpc/src/cost/BaseMotionTrackingCost.cpp#L50) suppresses planar position/yaw errors in relative-twist mode.
The heading-frame horizontal velocity residual is at [line 74](../src/core/humanoid_common_mpc/src/cost/BaseMotionTrackingCost.cpp#L74), and its $`J^{\mathsf T}QJ`$ quadratic approximation is at [line 100](../src/core/humanoid_common_mpc/src/cost/BaseMotionTrackingCost.cpp#L100).
Generic terminal cost scaling is applied at [HumanoidCostConstraintFactory.cpp:210](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp#L210).

### 4.2 Arm position targets

[JointTrackingCost.cpp:41](../src/core/humanoid_common_mpc/src/cost/JointTrackingCost.cpp#L41) selects external arm targets when present.
Otherwise the running evaluator uses the arm-swing reference and the terminal evaluator uses raw posture, selected at [line 28](../src/core/humanoid_common_mpc/src/cost/JointTrackingCost.cpp#L28).
The factory constructs both variants at [HumanoidCostConstraintFactory.cpp:118](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp#L118).

### 4.3 Task-space and load costs

[CentroidalMpcEndEffectorFootCost.cpp:92](../src/core/humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp#L92) supplies the model-independent foot residual despite its historical class name.
Default references, impact factors and optional foothold overrides are prepared at [line 127](../src/core/humanoid_centroidal_mpc/src/cost/CentroidalMpcEndEffectorFootCost.cpp#L127).
Named link costs are registered at [WbIdMpcInterface.cpp:108](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L108) using [EndEffectorKinematicsQuadraticCost.cpp](../src/core/humanoid_common_mpc/src/cost/EndEffectorKinematicsQuadraticCost.cpp).
Their reference link kinematics are computed at [EndEffectorKinematicsQuadraticCost.cpp:96](../src/core/humanoid_common_mpc/src/cost/EndEffectorKinematicsQuadraticCost.cpp#L96), and their quaternion-distance residual is defined at [EndEffectorKinematicCostHelpers.cpp:81](../src/core/humanoid_common_mpc/src/cost/EndEffectorKinematicCostHelpers.cpp#L81).
[ExternalTorqueQuadraticCostAD.cpp:110](../src/core/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp#L110) maps $`J_c^{\mathsf T}W_c`$ to selected joints and gates the residual using the opposite foot's impact factor.
It is registered independently of the input torque regularizer at [WbIdMpcInterface.cpp:178](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L178).
The builder maps `taskSpaceFootCost.activate` and `legTorqueCost.activate` at [lines 224 onward](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L224), and filters disabled `taskSpaceCosts.<name>.activate` entries before converting their weights at [line 269](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L269).
The interface checks the named-link flag before constructing its CppAD kinematics and guards both foot-cost and leg-load-cost construction at [line 172](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L172), following `mpc_controllers`' configure-time `activate` pattern.
All switches default to `true`; `false` removes only the corresponding extra cost, not the main Q/R objective or any contact/dynamics constraint.
Use `taskSpaceFootCost.weights` instead of the old WB-ID `taskSpaceFootCostWeights` key; the centroidal controller's schema is unchanged.

### 4.4 Gravity-relative torso tilt cost

[WbIdTorsoTiltCost.h](../include/humanoid_wb_id_mpc/cost/WbIdTorsoTiltCost.h) defines the opt-in configuration and the four-component Gauss--Newton residual.
The builder maps `ocs2.costs.torsoTiltCost` at [line 258](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L258), and [WbIdMpcInterface.cpp:102](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp#L102) conditionally registers the running cost as `torsoTiltCost`.
[WbIdTorsoTiltCost.cpp:22](../src/core/humanoid_wb_id_mpc/src/cost/WbIdTorsoTiltCost.cpp#L22) validates the tracked frame, positive free angle below $`\pi/2`$, nonnegative finite weights and nonzero finite gravity.
World up, the free-angle cosine, the free angle and square-root weights are passed as seven AD parameters at [line 38](../src/core/humanoid_wb_id_mpc/src/cost/WbIdTorsoTiltCost.cpp#L38), so reconfiguration does not silently reuse old tuning values from generated libraries.
The model cache uses a new `wb_id_torso_tilt_v1_<link>` name, distinct from the generic link tracker.
Forward kinematics and physical world-aligned angular velocity are evaluated from WB-ID $`q,v`$ at [line 66](../src/core/humanoid_wb_id_mpc/src/cost/WbIdTorsoTiltCost.cpp#L66).
The free corridor is implemented by clamping the cosine before `acos` at [line 74](../src/core/humanoid_wb_id_mpc/src/cost/WbIdTorsoTiltCost.cpp#L74); this keeps the inactive upright branch differentiable without evaluating `acos(1)`.
The transverse angular-velocity projection and weighted residual are at [line 79](../src/core/humanoid_wb_id_mpc/src/cost/WbIdTorsoTiltCost.cpp#L79).
No desired state/input trajectory is read, and no position, yaw or CoM constraint is added.
The shipped G1 YAML enables this cost and disables `taskSpaceCosts.torso`; the generic tracker remains available for deliberate reference-tracking tasks.
For intentional bending, widen `freeTiltAngle` in radians, reduce `tiltWeight`, or disable this soft term, and consider the separate pelvis/waist posture costs.
`angularVelocityWeight` still damps transverse motion inside the free-angle corridor.
[wb_id_torso_tilt_cost_test.py](../tests/wb_id_torso_tilt_cost_test.py) checks the reference math and source wiring, not the compiled C++ cost or simulation performance.


## 5. Commands, Gait and Swing

### 5.1 Base command conversion

[Ros2ProceduralMpcMotionManager.cpp:18](../src/common/ros2_procedural_mpc_motion_manager.cpp#L18) bounds incoming walking commands.
Walking velocity, `PoseStamped` and target-mode subscriptions are created at [line 100](../src/common/ros2_procedural_mpc_motion_manager.cpp#L100).
[WalkingVelocityTarget.cpp:62](../src/core/humanoid_common_mpc/src/target/WalkingVelocityTarget.cpp#L62) scales normalized velocity axes.
Filtering, resets, elapsed-time clamping and slew limits are at [line 72](../src/core/humanoid_common_mpc/src/target/WalkingVelocityTarget.cpp#L72).
The stateful low-pass is [BreakFrequencyAlphaFilter.h:48](../include/humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h#L48), separate from observation velocity filtering.

[WbIdMpcTargetTrajectoriesCalculator.cpp:63](../src/core/humanoid_wb_id_mpc/src/command/WbIdMpcTargetTrajectoriesCalculator.cpp#L63) constructs three body-twist reference knots.
The exact planar constant-twist integral is [TargetTrajectoriesCalculatorBase.cpp:132](../src/core/humanoid_common_mpc/src/command/TargetTrajectoriesCalculatorBase.cpp#L132).
Explicit pose conversion is [WbIdMpcTargetTrajectoriesCalculator.cpp:47](../src/core/humanoid_wb_id_mpc/src/command/WbIdMpcTargetTrajectoriesCalculator.cpp#L47).
Target-mode changes request filter reset and pose latching at [ProceduralMpcMotionManager.cpp:111](../src/core/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp#L111).
[BasePoseTarget.cpp](../src/core/humanoid_common_mpc/src/target/BasePoseTarget.cpp) implements pose-command evaluation and latching.

### 5.2 Gait and swing generation

[MotionPhaseDefinition.h:45](../include/humanoid_common_mpc/gait/MotionPhaseDefinition.h#L45) defines supporting-foot mode flags.
[ProceduralMpcMotionManager.cpp:126](../src/core/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp#L126) implements faster/slower gait guards.
Per-solver target dispatch, measured-speed checks and template selection start at [line 173](../src/core/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp#L173).
[GaitScheduleUpdater.cpp:45](../src/core/humanoid_common_mpc/src/gait/GaitScheduleUpdater.cpp#L45) delays template insertion, while [GaitSchedule.cpp](../src/core/humanoid_common_mpc/src/gait/GaitSchedule.cpp) extends the schedule.
The current WB-ID `slow_walk` timing is [ros2_controllers.yaml:1048](../config/g1/ros2_controllers.yaml#L1048).
[SwitchedModelReferenceManager.cpp:224](../src/core/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp#L224) combines schedule, support heights and swing references.
[SwingTrajectoryPlanner.cpp:126](../src/core/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp#L126) constructs the isolated swing spline with an apex above the higher endpoint.
The WB-ID builder explicitly disables capture placement at [line 247](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp#L247).
Shared stair/terrain setup is connected at [controller line 164](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L164), which should not be interpreted as validated WB-ID terrain performance.


## 6. Arm Command Support

The WB-ID controller passes tracked arm names and two empty frame-pair lists to `subscribeMpcTargets` at [controller line 181](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L181).
The shared subscriber dispatches through the parser at [Ros2ProceduralMpcMotionManager.cpp:150](../src/common/ros2_procedural_mpc_motion_manager.cpp#L150).

| `command_type` | WB-ID behaviour | Code |
| --- | --- | --- |
| `joint` | Parses one arm-joint trajectory for `JointTrackingCost` | [mpc_targets_parser.cpp:156](../src/common/target/mpc_targets_parser.cpp#L156) |
| `default` | Clears external channels | [Parser:181](../src/common/target/mpc_targets_parser.cpp#L181) |
| `frame_relation` | A nonempty frame pair fails the declared-pair check | [Parser:97](../src/common/target/mpc_targets_parser.cpp#L97) |
| `joint_frame_relation` | Parsing fails before either target channel is changed if a frame pair is undeclared | [Parser:154](../src/common/target/mpc_targets_parser.cpp#L154) |

The WB-ID OCP registers `JointTrackingCost` but not `FrameRelationTrackingCost`.
Implementing frame relations requires schema/builder frame-pair declarations and corresponding cost registration in addition to the existing shared parser.


## 7. Solver, Policy and Actuation

[construct_solver():1159](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1159) accepts only `sqp`, constructs `SqpMpc`, connects MRT and installs the synchronized modules.
The asynchronous worker starts at [line 1189](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1189).
The control loop forwards each valid observation through OCS2's existing `setCurrentObservation()` before policy checks or bring-up dispatch at [line 518](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L518).
After construction, the worker waits for a new control-loop observation before its first solve at [line 1151](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1151).
Policy handover checks finite timestamps and requires the evaluation time to lie inside the plan, with two control periods remaining, at [line 531](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L531).
An unusable policy requests `resetMpcNode()` on the solver worker, following the atomic warm-start reset pattern in `dynamics_mpc_controller`, rather than resetting the solver from the control thread.
Startup keeps holding the standing posture and retries; loss of a valid policy during the startup blend returns to `StartingMpc`, while loss after entering estimator-driven `Locomotion` latches the existing `Fault` state.
The ground-truth feedback path holds the last commanded posture while awaiting a valid replacement policy.
No additional age-limit parameter or alternative observation interface is introduced.
[compute_mpc_joint_action():1343](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1343) evaluates the policy using `control.policyTimeOffset`.
Planned joint position/velocity extraction is at [line 1359](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1359).
The torque-window check and planned-state inverse-dynamics fallback are at [line 1377](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1377).
The alternative measured-state inverse dynamics is at [line 1393](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1393).
Feedforward clipping is [clamp_torque():1335](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1335).
Joint position/velocity/effort and optional gain interface writes are at [line 1661](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1661).

`python3 tests/wb_id_startup_contract_test.py -v` checks the startup observation flow, policy bounds, reset ownership and bring-up guards without a ROS build.
These are static regression checks, not a substitute for repeated startup tests in the simulator.


## 8. Parameter Map

| Path inside the WB-ID controller block | Role |
| --- | --- |
| `robot.jointNames`, `ocs2.model` | Reduced model, contact frames and array ordering |
| `floatingBase.source`, `floatingBase.estimator` | Body state feeding the MPC |
| `control.observationVelocityFilterCutoffHz` | Observation generalized velocity filter |
| `ocs2.mpc`, `ocs2.sqp`, `ocs2.rollout` | Horizon, rates, grid, iterations and integration |
| `ocs2.model.torqueConstraintNodes` | Initial inverse-dynamics/torque-limit window |
| `ocs2.costs.q`, `qFinal`, `r` | State/input weight blocks |
| `ocs2.costs.qScaling`, `rScaling`, `terminalCostScaling` | Global weight scaling |
| `ocs2.costs.taskSpaceFootCost.activate`, `.weights` | Enable both foot costs and set their weights |
| `ocs2.costs.taskSpaceCosts.<name>.activate`, `.linkName`, `.weights` | Enable and configure each named link cost |
| `ocs2.costs.torsoTiltCost` | Soft gravity-relative tilt corridor and transverse angular-motion weight |
| `ocs2.costs.legTorqueCost.activate`, `.scaling`, leg joint names and weights | Enable and configure both external-load costs |
| `ocs2.model.footConstraint` | Acceleration-level stance/swing gains |
| `ocs2.constraints` | Friction, CoP, joint and collision soft penalties |
| `ocs2.gait.templates`, `transitions`, `swingTrajectory` | Contact timing, transition guards and swing profiles |
| `ocs2.reference` | Command scaling/slew, nominal posture, height and arm swing |
| `target.mpcTargetsTopic` | Arm joint command input |
| `control.feedforward`, `policyTimeOffset`, `torqueLimit` | Policy-to-torque execution |
| `control.jointEffortPd.kp`, `kd` | Inner servo gains |

Read these from the `humanoid_whole_body_inverse_dynamics_mpc_controller` YAML block rather than the similarly named centroidal settings.
The startup regression checks above are static and do not constitute runtime validation.


## 9. Blind Walking Tuning and Validation

The measured results, tuning table and limitations are in [Formulation and Signal Flow, Section 9](./humanoid_whole_body_inverse_dynamics_mpc.md#9-blind-walking-robustness-and-tuning).
The references below map that table to the current implementation; only the paired contact-gain reduction was isolated in the ramp comparison.

| Responsibility | Implementation or current parameters |
| --- | --- |
| Stance height/tilt correction gains | [ros2_controllers.yaml:830](../config/g1/ros2_controllers.yaml#L830), [stance equations:114](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L114) |
| Shared gain in swing vertical correction | [WbIdFootAccelerationConstraintCppAd.cpp:106](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdFootAccelerationConstraintCppAd.cpp#L106) |
| Running/terminal base z and vz weights | [Q:887](../config/g1/ros2_controllers.yaml#L887), [Qf:903](../config/g1/ros2_controllers.yaml#L903) |
| Foot task-space cost | [weights:939](../config/g1/ros2_controllers.yaml#L939) |
| Swing clearance | [swingHeight:1061](../config/g1/ros2_controllers.yaml#L1061) |
| Torso free lean and angular damping | [torsoTiltCost:958](../config/g1/ros2_controllers.yaml#L958) |
| Extra leg-load cost activation | [legTorqueCost:966](../config/g1/ros2_controllers.yaml#L966) |
| Joint PD and feedforward | [control:756](../config/g1/ros2_controllers.yaml#L756) |
| Command slew limits and filtering | [limits:1038](../config/g1/ros2_controllers.yaml#L1038), [WalkingVelocityTarget.cpp:90](../src/core/humanoid_common_mpc/src/target/WalkingVelocityTarget.cpp#L90) |
| Planar-relative base tracking | [BaseMotionTrackingCost.cpp:51](../src/core/humanoid_common_mpc/src/cost/BaseMotionTrackingCost.cpp#L51) |
| Mapped terrain planner inactive outside its explicit mode | [mode gate:189](../src/core/humanoid_common_mpc/src/reference_manager/ProceduralMpcMotionManager.cpp#L189), [world-up fallback:307](../include/humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h#L307) |
| Current flat support-height assumption | [option:827](../config/g1/ros2_controllers.yaml#L827), [reference fallback:126](../src/core/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp#L126) |
| Linear KF flat contact-height observation | [zero heights:159](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L159), [measurement:303](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L303) |
| Scheduled estimator contacts | [contact source:168](../config/g1/ros2_controllers.yaml#L168) |
| Automatic recording and scoring | [run_case.sh:35](../tests/tuning/run_case.sh#L35), [command schedule:76](../tests/tuning/tune_probe.py#L76) |
| Whole-run comparison and recorded-state images | [plot_ramp.py:17](../tests/tuning/plot_ramp.py#L17), [replay:69](../tests/tuning/plot_ramp.py#L69) |

Trial folders such as `tuning_results/ramp_rebuilt_baseline`, `ramp_final_headless`, `ramp_final_slow` and `ramp_final_rviz` contain exact configs, logs, arrays and plots on the development machine.
They are git-ignored and are not bundled with this document; [Codex.md](../Codex.md) preserves the trial identifiers and numerical summary.
Replay images reconstruct recorded physical poses and encoders rather than showing a live viewer or an estimated terrain map.
