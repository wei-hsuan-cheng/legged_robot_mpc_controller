# WB-ID MPC Walking Robustness Handoff

## Gravity-Relative Torso Stabilization (2026-10-24, build and startup verified)

Added `WbIdTorsoTiltCost`, registered as the running OCP cost `torsoTiltCost` when `ocs2.costs.torsoTiltCost.activate` is true.
G1 enables it on `torso_link` with `freeTiltAngle: 0.08726646259971647` rad (5 degrees), `tiltWeight: 10.0` and `angularVelocityWeight: 5.0`; these are initial settings, not tested gains.
The generic `taskSpaceCosts.torso` tracker is disabled to avoid competing objectives; the new schema default is opt-in (`activate: false`) for external parameter files.
The residual penalizes tilt beyond the free corridor and physical angular velocity perpendicular to model gravity, with no pose trajectory, position target or yaw target.
The corridor and weights are AD parameters rather than frozen generated-code constants; the model has a distinct `wb_id_torso_tilt_v1_<link>` cache name.
The cosine is clamped before `acos` so upright AD derivatives stay finite; a lower clamp at `-1+1e-9` bounds the singularity near inversion.
Bending remains possible because this is soft, but picking tasks should widen the corridor or reduce/disable this cost and revisit existing pelvis/waist posture penalties.
This is not a CoM stability guarantee, and does not add upper-body collision avoidance.
No contact/dynamics constraint, Q/R weight, estimator, servo or centroidal-controller behavior was changed; retain the user's concurrent foot-orientation weight tuning.
Reference-math and static wiring tests cover the free corridor, yaw invariance, transverse damping, finite upright residual derivatives, bending and schema/config registration.
Built successfully in Parallels container `ros_jazzy`, workspace `/root/ocs2_ros2_ws`, with two jobs, Release and `BUILD_TESTING=OFF`.
The first build exposed a Pinocchio header compatibility issue; `frameId_` now uses `std::size_t` rather than relying on the visibility of `pinocchio::FrameIndex`.
Launched `scene_flat.xml`, MPC 100 Hz / MRT 1000 Hz, `rviz:=false`, `mujoco_headless:=true`, `baseCommandGui:=false`, with the default `state_estimator` source and `humanoid_linear_kf` backend.
OCS2 generated and loaded `wb_id_torso_tilt_v1_torso_link_lib.so` at runtime with `recompileLibrariesCppAd: false`; the cache symlink resolved to `/root/src/mpc/ocs2_auto_generated/g1/wb_id_mpc_g1/wb_id_torso_tilt_v1_torso_link/cppad_generated/`.
The first MPC advance completed in 20.588 ms, the policy was accepted, and bring-up transitioned through blending into locomotion.
Stopped immediately at locomotion as requested, sent no motion commands, and verified that no simulation or ROS test processes remained.
This is startup validation only, not a walking, robustness, bending or hardware validation; the user will test those next.
Container logs: `/tmp/wbid_torso_tilt_build_20261024.log` and `/tmp/wbid_torso_tilt_startup_20261024.log`.

## Optional Cost Activation (2026-10-23, needs build and runtime test)

WB-ID now follows `mpc_controllers`' configure-time cost-registration pattern with `ocs2.costs.taskSpaceFootCost.activate`, `ocs2.costs.taskSpaceCosts.<name>.activate` and `ocs2.costs.legTorqueCost.activate`.
All defaults and shipped G1 flags are `true`, and all existing weights are unchanged.
The WB-ID foot vector moved from `taskSpaceFootCostWeights` to `taskSpaceFootCost.weights`; update external WB-ID parameter files accordingly.
Disabled costs are not constructed or registered, avoiding their CppAD model creation/loading; Q/R and all contact/dynamics constraints remain active.
The centroidal schema and costs are unchanged.
The task-space objectives are not algebraically identical to Q/R, and the load cost regularizes selected entries of `J^T W`, not the torque decision variable.
Disabling a foot cost does not remove the hard stance orientation or vertical contact constraints, so this is an ablation switch, not a terrain-support fix.
Static activation and parameter-contract tests cover schema defaults, all eight switch combinations, config wiring and constructor guards; no ROS build or simulator test was run.

## WB-ID Startup Horizon Recovery (2026-10-23, needs build and runtime test)

The intermittent takeover failure showed control time around 15.7 s while the last plan ended at 9.5024 s.
Observations were only forwarded from `compute_mpc_joint_action()`, so startup could keep solving the initial snapshot and then accept an expired first policy.
OCS2's `MPC_BASE::run()` refuses a new start time beyond its previous final time until reset, making the failure persistent.

The fix follows the existing arm-controller OCS2 pattern: `setCurrentObservation()` on every valid control tick, `updatePolicy()` on the control thread, and `advanceMpc()` on the worker.
The deferred worker waits for a post-construction observation before its first solve.
Policy evaluation is gated on finite timestamps and interval coverage with two control periods of end margin.
An atomic warm-start reset request is consumed by the worker through `resetMpcNode()`, as in `dynamics_mpc_controller`.
No custom observation mailbox, new tuning parameter, OCS2 library change or weight/gain change remains in this patch.
Startup holds and retries; a blend interruption returns to startup, while policy loss after estimator-driven locomotion latches `Fault` and requires controller reactivation.
The ground-truth path holds the last commanded posture until a usable plan arrives.

The arm controllers normally compute their initial policy synchronously in activation, whereas this legged controller must keep standing and estimating while constructing/starting MPC asynchronously.
The centroidal controller has the same latent observation-delivery gap and is not inherently immune, but is unchanged in this WB-ID checkpoint.
Six static startup/thread-ownership regression checks and the WB-ID parameter contract pass; no ROS build or simulator validation was performed for this patch.
Test repeated launches at the usual rates and inspect the new construction/first-advance logs, rejected-policy timestamps and warm-start reset messages.

## Description Layout (2026-10-23)

The current layout is format-first: `description/meshes/robots/g1/`, `description/mujoco/robots/g1/` and `description/urdf/robots/g1/`.
Shared MuJoCo scene entry points and the generator remain at `description/mujoco/`, with reusable presets in `common/backgrounds/` and `common/materials/`, plus `terrains/`, `objects/` and `textures/`.
The RViz staircase xacro is `description/urdf/terrains/stairs.urdf.xacro`, not a MuJoCo resource.
Both staircase xacros use `config/terrains/stairs_params.yaml`, which is format-neutral and installed alongside the description tree.
Launch defaults, robot mesh URIs, scene includes and documentation links were updated; `mujocoModelFile:=scene.xml` and other scene basenames remain unchanged.
Compiled numerical model arrays and simulation options match the pre-move baselines for all three scenes and all optional terrain/object modules; only resource paths changed.
Eleven layout checks pass, including both staircase xacros expanded from a relocated copy of the installed-style description/config trees.
See [the layout guide](description/README.md), [the MuJoCo guide](description/mujoco/README.md) and `tests/test_mujoco_scene_layout.py` for regression coverage.

## Tuning Benchmark and Current Gains (2026-10-23, uncommitted)

- `docs/wb_id_tuning_benchmark.md` has the current state in its section 0, plus the instrumentation, the tools, how to run and read a case, every result, the measurement biases found, and the open items.
- **Instrumentation:** `diagnostics.trackingRate > 0` publishes `~/tracking` (plan / command / measurement per tick) and `~/solver` (per-solve metrics, constraint violation including over the first 3 nodes, and cost terms).
- **Tools and results:** the tools are in `tests/tuning/` (recording via `ros2 bag`, not Python subscribers); results go to `tuning_results/<tag>/` (git-ignored).
- **Shipped WB-ID values:**
  - `jointEffortPd` kp 200 / kd 20;
  - `q/qFinal.baseVelocity` vz 10 and pitch/roll rate 30;
  - arm `q.jointVelocities` 0.06;
  - `sqp.useFeedbackPolicy: false`;
  - `sqpIteration` stays 1.
- **`mujoco_ros2_control` (jazzy, uncommitted): `controller_time_from_sim` (default true).** The controller manager's clock is set from MuJoCo time every update, with its `/clock` remapped away. Before, controllers saw time in 10 ms steps, the default `mujoco_publish_rate`.
- **Net, same 0.5 m/s walk:** bounce 14.4 → 11.5 mm; tracking error legs 0.39 → 0.13°, arms 0.11 → 0.02°; leg / arm torque jitter 5.9 → 4.2 / 1.7 → 0.9 Nm; MPC 78 → 97 Hz.
- **Knee kd 10** has since halved the touchdown spikes. Robustness passed: lateral, turning, stop cycles, 0.25 and 0.75 m/s, with no falls.
- **Resume:** from `docs/wb_id_tuning_benchmark.md` → "Resume here". Round 7 (`walk` gait timing) was in progress at handoff.

## Servo Gains: Controller-Written Per Joint for WB-ID (2026-10-23, implemented, needs build)

This supersedes the launch-argument proposal that was here. Launch has no gain arguments any more; gains are set in YAML or Xacro only.

- **Hardware (`mujoco_ros2_control`, jazzy branch).** Each effort_pd joint that declares `<command_interface name="kp"/>` / `"kd"` exports its `pd_kp` / `pd_kd` as command interfaces. They start at the joint's `effort_pd_kp` / `effort_pd_kd` parameters. A non-finite or negative written gain counts as 0. This mirrors Unitree `LowCmd`, which carries kp/kd with every motor command.
- **Xacro.** `g1.ros2_control_macro.xacro` declares kp/kd on every effort_pd joint. The hardware defaults stay `effort_pd_kp/kd` args of `g1.ros2_control.xacro` (1200/10; wrists 100/1). They apply to centroidal, to the wrists, and to WB-ID before activation.
- **WB-ID.**
  - `control.jointEffortPd.kp/kd` are arrays index-aligned with `robot.jointNames`, currently 1200 / 20 uniform.
  - The controller claims `<joint>/kp`, `<joint>/kd` after the joint and wrist commands, so the `[3 * i + k]` layout is unchanged. It finds their indices by name at activation and writes them every tick in every bring-up phase.
  - On deactivation it leaves them in the hardware rather than zeroing them, which would leave the joints limp.
  - Empty arrays mean the controller doesn't claim them and the hardware gains stay in effect.
  - The contract test checks the lengths.
- **Centroidal** is unchanged: it doesn't claim kp/kd.
- **Not done yet:** per-phase gains (e.g. bring-up vs walking) and a real G1 hardware interface. That interface should map the same kp/kd command interfaces onto `LowCmd`.

## Tested Servo Gain Applied (2026-10-23)

WB-ID runs Kp 1200 / Kd 20 on the 23 MPC joints. This was first set through launch arguments and is now `control.jointEffortPd`, written by the controller (see above). Centroidal stays 1200/10 from the Xacro; all six fixed wrists remain 100/1. The new MJCF retains damping 0.05, its armature and friction; no Q/R or estimator settings were changed. This promotes the temporary Kd-20 test below, not the artificial passive-damping-10 ablation. Forward stability improved in the short test; lateral speed tracking remains unresolved. Restart the launch to apply hardware gains.

## New-Model Fall Investigation (2026-10-23)

Analysis-only session against HEAD `5fe1d79` plus the user's uncommitted rev_1_0 model/launch changes. No production code, model, or parameter settings were changed by this investigation; only this handoff was updated. No build was run.
Since then the rev_1_0 model has replaced `g1_29dof.xml` / `g1_29dof.urdf` in place, so every scene and the launch default use it (see "G1 model" below).

### Controlled Experiments

Ran sequentially in Parallels `ros_jazzy`, flat scene, MPC 100 Hz / MRT 1000 Hz, MuJoCo headless, RViz and base-command GUI off. All cases use the new URDF/MJCF and the new CppAD cache; numerical controller settings are the current checkout's settings. Estimator cases use the default `humanoid_linear_kf`, not InEKF.
The standard sequence settles for 10 s after the harness detects locomotion, ramps for 3 s, holds for 5 s, ramps out for 3 s and rests for 5 s. The lateral test holds for 15 s instead. Fall times are relative to harness start, not process launch; GT cases start at the first observed odometry, estimator cases after the locomotion log.

| Container artifact prefix under `/tmp/` | Change | Result |
| --- | --- | --- |
| `g1_audit_new_kf0795` | Stock new model, height 0.795, KF | Fell at 8.88 s, before walking command. |
| `g1_audit_repeat_kf0795` | Same, repeat with joint logging | Fell at 6.58 s, before walking command. |
| `g1_audit_new_gt0795` | GT feedback, height 0.795 | Fell at 13.95 s, during walking. |
| `g1_audit_new_gt` | GT feedback, height 0.6 | Fell at 16.54 s, during walking. |
| `g1_audit_d10_kf0795` | Only passive damping restored to 10; retain new mass, armature, friction and integrator | Passed 26 s forward sequence; 1.310 m / 2.0 m expected, mean walking vx 0.179 m/s for 0.25 commanded, peak roll/pitch 0.030/0.026 rad. |
| `g1_audit_kd20_kf0795` | Passive damping stays 0.05; only hardware servo Kd 10 -> 20 on the 23 MPC joints | Passed 26 s forward sequence; 1.636 m / 2.0 m expected, mean walking vx 0.218 m/s, peak roll/pitch 0.014/0.032 rad. |
| `g1_audit_kd20_vy_kf0795` | Same Kd 20, lateral 0.15 m/s | Upright for 36 s, peak roll/pitch 0.011/0.017 rad; mean vy 0.050 m/s, 0.909 m / 2.7 m expected. Verdict NO_TRACKING, not success. |

These are short diagnostic trials, not a robustness certification or hardware validation. No fresh old-model control run was performed; the clean ablation retained the entire new model and changed damping alone.
Actuator limits remained enabled. Kp stayed 1200; fixed wrist gains stayed 100/1. Runtime `/robot_state_publisher` robot_description confirmed Kd values 20 and 1 in the active-gain test.

Joint-state evidence, same KF setup, the 3-to-5-second window after locomotion starts, before any velocity command (RMS joint speed, rad/s):

| Joint | Stock new model / Kd 10 | New model / Kd 20 | New model / passive damping 10 |
| --- | ---: | ---: | ---: |
| Left shoulder yaw | 0.6047 | 0.0009 | 0.0002 |
| Waist pitch | 2.7612 | 0.0188 | 0.0010 |
| Left hip pitch | 2.6129 | 0.0442 | 0.0022 |
| Left knee | 1.4226 | 0.0350 | 0.0047 |

Stock waist-pitch and left-hip-pitch efforts were at >=99% of their actuator limits for 2.63% and 1.62% of samples in that window; both were zero with Kd 20. Large joint oscillations exist before gait switching. GT feedback also fails, so an estimator error alone does not explain the regression.

### Interpretation and Next Fix

The dominant demonstrated change is the loss of passive damping: old MJCF damping 10 -> new per-joint damping 0.05, while the hardware servo stayed Kp 1200 / Kd 10. Servo torque is `Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff`, evaluated against live MuJoCo state every physics step; the passive term additionally contributes `-b*dq`. In a local scalar approximation, velocity dissipation changes from Kd+b=20 to 10.05. This is not a full closed-loop stability analysis: targets move, actuator saturation and contact coupling matter, and armature also changed.
Increasing active Kd to 20 while retaining the low physical damping is the first tested controller-side candidate. Do not silently restore damping 10 and call the new model hardware-accurate. Expose per-joint servo gains in a model-specific hardware configuration, start from the tested gain, then check longer forward/lateral/yaw/reversal tests, saturation and transient tracking. Kd is not currently a WB-ID YAML cost/control gain; it lives in the ros2_control xacro defaults.
Only after the inner-loop oscillation is removed should Q/R and gait tuning address the poor lateral speed. Both successful-upright variants settle near GT height 0.764 for target 0.795, leaving about 31 mm height error. Kd 20 also still produces saturation during some early transition samples; do not claim saturation has disappeared globally.

Other model-consistency findings:

- Pinocchio URDF mass = 33.34114202 kg; MJCF declared mass sum = 33.341142 kg. Old equivalents were 35.11514202 and 35.112142 kg. At identical crouched q, ankle frame positions and rotations in old/new URDFs are identical. New mass/leg kinematics do not show an obvious mismatch in these checks; a full inertial-tensor comparison was not performed.
- MuJoCo has reflected rotor inertias, damping and friction; loaded Pinocchio armature is zero and the WB-ID RNEA constraint adds no explicit actuator-dynamics correction. Model rotor inertia and losses consistently in MPC, fallback inverse dynamics and torque feasibility after validating their physical values. Do not treat the handoff's earlier "small torque mismatch" as measured evidence.
- Hip-roll torque limits were raised to 139 in URDF/MJCF/controller YAML, but the ros2_control macro still declares 88 on both hips. Align them when implementing the fix. This inconsistency was not isolated as a cause of the fall.
- Mid360's new upside-down mounting is real in the URDF, but the torso cost builds its orientation reference with FK of the same named frame at x_ref; it does not directly request identity orientation for the lidar. Therefore a simple "MPC tries to flip the lidar upright" explanation is unsupported. Preserve the real transform; optionally choose torso_link as the torso task frame in a separately validated change.
- Keep URDF, MJCF, estimator model and CppAD cache paired. `scene_ramp.xml` still references the old MJCF. Changing URDF alone while reusing an old generated cache remains unsafe.

Artifacts remain in the container: each prefix has `.json`, `.csv`, `.log`, `.yaml`; wrapper runs also have `_joints.csv`, `_model.xml`, `_scene.xml`. `/tmp/g1_model_audit.py` wraps the repository harness to use temporary model files; `/tmp/g1_audit_bin/xacro` applies gain overrides only when prepended to that test's PATH. `/tmp/g1_static_audit.py` and `/tmp/g1_joint_audit.py` provide model and joint statistics. Nothing in the normal launch uses these wrappers. All launched test processes were stopped.

## Latest Checkpoint (2026-10-23, uncommitted): gait parameters + closest-to-hardware G1 model

**User builds and tests; no agent build/sim was run after the model files were added.**

### Gait settings are ROS parameters (built and smoke-tested)

- `config/g1/gait.yaml` and `gait_wb_id.yaml` are removed, along with the `gaitLibraryFile` parameter and launch arg. Each controller block of `ros2_controllers.yaml` now holds everything under `ocs2.gait`:
  - `templates.names` plus one `{modeSequence, switchingTimes}` per name;
  - `transitions` (`minDwellTime`, `switchDelayHorizonFraction`, `ladder`, plus one `{linearVelocity, yawRate, measuredMargin}` per rung);
  - `swingTrajectory` (moved from `ocs2.swingTrajectory`).
- Values are identical to the old files, checked by script. Builders: `buildGaitMap` / `buildGaitTransitionConfig` in `config_builder_utils.hpp`.
- Smoke runs, flat, the same before and after: WB at 0.5 m/s survived, peak tilt 0.037, 87 Hz; centroidal at 0.25 m/s survived. The WB contract test checks templates and ladder.
- The ladder overlaps are deliberate hysteresis; keep `next.min < prev.max`. Up needs command > max AND measured > max - margin. Down needs command < min AND measured < min + margin, on all axes.
- Known quirks, unchanged:
  - measured speed is world-axis and unfiltered;
  - the centroidal "measured yaw" reads `h_ang,x/m`;
  - the centroidal `skip` template has an out-of-order 0.08.

### G1 model: `g1_29dof.xml` / `g1_29dof.urdf` are now rev_1_0 (replaced in place)

- **Single model for everything:** `description/mujoco/robots/g1/g1_29dof.xml` (all scenes include it) and `description/urdf/robots/g1/g1_29dof.urdf` (the launch default `urdfFile`, read by the MPC, both estimators and robot_state_publisher). The temporary `*_correct` files are gone. `g1_29dof_real.xml` is the superseded old-mass variant and nothing includes it.
- **Previously Unitree's deprecated `g1_29dof`** (`mode_machine` 2, 35.11 kg, hip roll 88 Nm). Now `g1_29dof_rev_1_0` (`mode_machine` 5, the current standard 29-DoF). Changes:
  - torso 6.78 vs 8.56 kg, 33.34 kg total, no waist_support_link;
  - waist joint offsets and shoulder/sensor offsets changed;
  - hip roll 139 Nm / 20 rad/s;
  - Mid-360 mounted upside down (`rpy pi 0.0511 0`).
  - Check the robot's `mode_machine` in the Unitree app (Device -> Data -> Robot -> Machine Type). Newer units are `g1_29dof_mode_11..18`.
- **MJCF joint dynamics:**
  - per-motor armature from unitree_rl_lab (hip pitch/yaw + waist yaw 0.010178, hip roll + knee 0.025102, ankles + waist roll/pitch 0.00722, shoulders/elbow/wrist roll 0.00361, wrist pitch/yaw 0.00425);
  - damping 0.05 and frictionloss 0.2 (0.1 on wrist pitch/yaw) from unitree_mujoco, replacing the old damping 10;
  - `implicitfast`.
  - Kept local: contact solref/solimp, free-joint damping 0, `imu-pelvis-quat` sensor, `mid360` site at the rev_1_0 pose.
- **URDF:** rev_1_0 plus local package:// meshes, ankle/waist roll+pitch limits 50 Nm / 37 rad/s (rev_1_0 MJCF and unitree_rl_lab; Unitree's URDF says 35/30), and the four `*_est` foot points. Pinocchio loads it: 33.341 kg, frames present.
- **Launch:** `g1.ros2_control.xacro` takes `robot_urdf_file`, and `g1.launch.py` passes `urdfFile` to it, so robot_state_publisher matches the MPC.
- **CppAD cache:** the default `libFolder` is again `auto_generated/g1`.
  - The libraries generated from the old 35.1 kg URDF were renamed `wb_id_mpc_g1_old_35kg` / `centroidal_mpc_g1_old_35kg`, not deleted. The rev_1_0 WB libraries moved in as `wb_id_mpc_g1`.
  - With `recompileLibrariesCppAd: false` OCS2 never checks that cached libraries match the URDF, so any future model change needs the same swap or a fresh folder.
  - The centroidal rev_1_0 libraries don't exist yet; the first centroidal launch regenerates them, which takes long.
- `control.torqueLimit` hip roll is 139 in both controller blocks; the ros2_control command limit in the xacro is 139 too.
- **Expected risks:**
  - Losing damping 10 removes a lot of free passive stability, so the controller may need retuning.
  - Armature should fix the ankle-chatter instability (kd*dt/I about 12 -> about 0.7), though that's unmeasured.
  - The MPC model has no armature, so expect a small torque mismatch.
- **References (raw GitHub):**
  - unitree_ros `robots/g1_description/{g1_29dof_rev_1_0.xml, g1_29dof_rev_1_0.urdf, g1_29dof.urdf, README.md}`;
  - unitree_mujoco `unitree_robots/g1/g1_29dof.xml`;
  - unitree_rl_lab `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`;
  - unitree_rl_gym `resources/robots/g1_description/g1_29dof.xml`;
  - mujoco_menagerie `unitree_g1/g1.xml`.

## Earlier Checkpoint: User Will Build and Test

**2026-10-22: stop automated builds/simulations here. The standalone parameter cleanup below has NOT been compiled or walking-tested.**
The earlier rate-limiter build completed, but subsequent schema/controller/builder changes require a fresh build.
The user requested committing and pushing this unbuilt checkpoint for their own build/test.

- WB-ID now has one generated parameter schema and one listener; no centroidal Params type, builder call, or nested `wholeBody.*` parameter tree.
- A single `ocs2.costs.q/qFinal` contains `basePose`, `baseVelocity`, `jointPositions`, `jointVelocities`.
- A single `ocs2.costs.r` contains `contactWrenches`, `baseAccelerations`, `jointAccelerations`, `jointTorques`.
- Values were moved without retuning Q/R. Runtime scales remain qScaling=1, rScaling=0.001, terminalCostScaling=4.
- Torque-window setting is `ocs2.model.torqueConstraintNodes`; barrier settings are `ocs2.constraints.jointTorqueLimits` and `jointVelocityLimits`; feedforward is `control.feedforward`.
- Removed WB-only unused centroidal momentum/model settings, inertia-weighted momentum toggle, centroidal input joint-velocity weights, unused controller PD arrays, unused gradient switches/ICP settings, diagnostics CSV parameters and future frame-relation parameters. The separate centroidal controller was not removed or retuned.
- WB capture-foot placement is disabled in its builder and removed from its schema/YAML. The common terrain planner remains a separate feature. Earlier passing runs below used capture placement and do NOT validate this checkpoint; a previous capture-disabled experiment fell.
- Controller-side rate limits are `ocs2.reference.velocityCommandRateLimits=[0.1,0.08,0.05,0.1]` (moved from `target.*` on 2026-10-23) for `[ax,ay,height_rate,yaw_acceleration]` in physical units. Centroidal defaults disable this new limiter. Solver time governs increments, capped at 50 ms per update; reset starts limited twist axes at zero.
- WB GUI ranges are now vx +/-0.25 m/s, vy +/-0.15 m/s, yaw +/-0.15 rad/s. Launch reads GUI scales from the selected YAML, no longer hardcodes 2.4/1.2/1.0. A full normalized topic command now means these smaller limits.
- `tests/wb_id_parameter_contract_test.py` passes locally: duplicate keys, declared runtime keys, unified Q/Qf/R layout, joint-array lengths, and direct builder schema accesses. This does not substitute for compilation or closed-loop testing.

### New Reproduction Evidence Before This Cleanup

Both tests used the old capture-enabled controller with the default linear KF, flat scene, 100 Hz MPC / 1000 Hz MRT, height 0.6 m, and all GUIs disabled.
`/tmp/wbid_step_baseline_vx`: a 0.4 m/s forward command reached in 0.02 s caused a fall at 6.82 s, about 1.8 s after the command, after five seconds of stable MPC stance.
`/tmp/wbid_step_baseline_yaw`: a 0.15 rad/s yaw step survived 30 s including an abrupt stop; achieved 2.04 rad versus 3.0 commanded, average yaw rate 0.098 rad/s, max translation drift 0.328 m.
Thus abrupt commands and speed/gait selection expose failures hidden by the earlier gentle-ramp tests; neither lowering GUI limits nor removing capture placement has yet been demonstrated to fix this.

### Paper Audit

The local Molnar 2025 PDF, section IV-A/B, and `wb-mpc-locoman/optimization/ocp_whole_body_rnea.py` use state `[q,v]`, quadratic state/input tracking, RNEA equalities, contact/swing constraints, and joint limits; horizontal swing placement is optimized rather than capture-point prescribed.
The local Khazoom 2024 PDF, section II-B, uses pose/velocity/contact-force tracking plus first-step torque-change regularization, contact height/no-slip constraints, joint limits and collision constraints/CBFs.
Current WB-ID has the common whole-body state/input structure but is not numerically identical to either paper: Translation + SphericalZYX Euler-rate state versus spatial twist; acceleration-level stabilized contact constraints; uniform OCS2 SQP grid and soft inequality barriers; 6D flat-foot wrenches versus point contacts; extra foot/torso and external-leg-load costs; no first-step torque-change cost or first-node collision CBF.
The retained extra costs are active G1 adaptations, not dead centroidal parameters. Joint-torque zero regularization is not the MIT torque-change regularizer.
Do not claim an SE(3) state-model conversion or paper-equivalent hard constraints based on this parameter cleanup.

Everything below is historical experiment context, not validation of the latest checkpoint.

## Scope and Status (2026-10-22)

Work is on `feat/wb_id_mpc`, starting from `1fbd2e2`.
Only whole-body inverse-dynamics MPC is being tuned; do not copy these weights into centroidal MPC.
The user requests flat-ground, separate forward and lateral walking tests in Parallels Docker `ros_jazzy`, with MuJoCo, RViz and the command GUI all headless/disabled.
Experiments below are simulation results, not hardware or terrain validation.
At that earlier stage, implementation fixes were built in `ros_jazzy`; the latest checkpoint above supersedes this build status.
No robot/test processes are intentionally left running.
GT-driven forward/lateral tests survived 41 s; with the nominal-posture correction the default linear-KF forward test also survived 41 s, but failed the strict heading check (0.552 rad maximum yaw drift).
This is improved survival, not a claim that tracking, all estimator paths, or stairs are solved.

## Reproduction

SSH to `parallels@10.211.55.17`, enter container `ros_jazzy`, source `/opt/ros/jazzy/setup.bash` and `/root/ocs2_ros2_ws/install/setup.bash`, and work in `/root/ocs2_ros2_ws`.
Sources are shared with the Mac checkout.

```bash
python3 src/legged_robot_mpc_controller/tests/wb_id_walk_test.py \
  --axis vx --speed 0.25 --mpc-rate 100 --prefix /tmp/wbid_vx
```

For lateral tests use `--axis vy --speed 0.15`.
The runner selects `scene_flat.xml`, `rviz:=false`, `mujoco_headless:=true`, `baseCommandGui:=false`, and `mrtFreq:=1000`.
It uses ground truth by default to separate controller faults from estimator faults; use `--source state_estimator --estimator humanoid_inekf` or `humanoid_linear_kf` for closed-loop estimation tests.
Each run saves its exact YAML, launch log, GT odometry CSV and JSON verdict at the prefix inside the container.
Commands are specified in physical m/s and normalized using the tested YAML limits.
Phases use simulation time: settle 5 s, ramp in 3 s, hold 25 s, ramp out 3 s, rest 5 s (41 s total).
Fall detection is height below 0.38 m or absolute roll/pitch above 0.8 rad for 0.15 s.
An upright run is not automatically a pass: projected travel must exceed half of the command integral.
The updated runner also requires heading error below 0.5 rad and cross-axis travel below 35% of the intended travel; the early experiments below predate those extra checks.

## Results Before Implementation Fixes

All tests below used GT feedback, 100 Hz requested MPC, 1000 Hz control, and 0.6 m requested pelvis height.
Artifacts are `/tmp/wbid_<name>.{yaml,log,csv,json}` in `ros_jazzy`.

| Name | Change | Result |
| --- | --- | --- |
| `baseline_vx` | Original YAML, vx = 0.25 m/s | Fell at 14.80 s; peak pitch 1.128 rad. |
| `baseline_vy` | Original YAML, vy = 0.15 m/s | Upright 41 s, but only 0.141 m lateral travel and -1.737 m unwanted x travel; not tracking. |
| `sqp2_vx` | Two SQP iterations | Fell at 9.70 s; actual solver rate about 51 Hz instead of roughly 90-98 Hz. |
| `weights_vx` | Weight set A | Upright 41 s; 3.10 / 7.0 m forward travel, insufficient speed tracking. |
| `weights_vy` | Weight set A | Upright 41 s; 1.59 / 4.2 m lateral travel and -1.78 rad final yaw; not acceptable tracking. |
| `freefoot_vx` | Weight set B, capture-foot-placement cost disabled | Fell at 10.52 s; retain this cost for now. |
| `weights2_vx` | Weight set B, capture cost retained | Upright 41 s; 5.22 / 7.0 m forward travel, peak roll 0.069 rad, pitch 0.056 rad. Final y 0.40 m, yaw 0.152 rad. |

Weight set A changes both running and terminal base-pose weights to `[0, 0, 1000, 0, 200, 200]`, and both base-velocity weights to `[100, 100, 30, 10, 10, 10]`.
Weight set B keeps those pose weights and uses base-velocity weights `[300, 300, 30, 100, 10, 10]`.
Base-pose order is `[x, y, z, yaw, pitch, roll]`; base-velocity coordinates are `[vx, vy, vz, yaw_dot, pitch_dot, roll_dot]`, not body angular velocity components.
At this stage these weights were temporary `--set` overrides, not changes saved to `config/g1/ros2_controllers.yaml`.
Set B still has forward speed error (about 0.19 m/s achieved for 0.25 m/s) and walking height error (about 0.65 m for 0.6 m requested), despite good upright stability.

## Implementation Findings

1. Torque-window interpolation accepted invalid knots.
   The torque equality is active for the first three knots at 0, 0.02, 0.04 s; duration is 0.05 s.
   Evaluating at 0.045 s previously passed the time-window test but blended the valid 0.04 s torque with the unconstrained 0.06 s torque.
   `WbIdTorqueWindow::containsInterpolatedInput()` now checks every contributing knot using OCS2's interpolation indexing and the active policy's own start time.
   Beyond that window the existing RNEA feedforward fallback is used.
   The standalone `tests/wb_id_torque_window_test.cpp` passed, including event-duplicate times, old policy/new solver horizons, singleton trajectories and expired plans.
2. Active policy ownership crossed threads.
   The solver worker called `MRT_BASE::updatePolicy()` while the control loop called `evaluatePolicy()` and `getPolicy()` without a lock.
   OCS2 protects its buffer, not reads of the active trajectory; these operations must belong to the same consumer thread.
   Move the policy swap into `update_and_write_commands()` and latch successful first activation there.
   A missing/newly locked buffer is normal, not a solver failure.
   This is a correctness fix; its quantitative contribution to falling has not been isolated experimentally.

The first torque-window rebuild passed (warnings only).
The policy-ownership rebuild also passed (warnings only).

### Tests With Both Implementation Fixes

| Artifact suffix | Weights | Result |
| --- | --- | --- |
| `fixed_weights2_vy` | B | Fell at 10.28 s during the first lateral steps. Forward-only tuning cannot be promoted. |
| `weights3_vy` | Pose `[0,0,1000,0,1000,1000]`, velocity `[300,100,30,300,30,30]`, running and terminal | Upright 41 s, but 1.29 / 4.2 m travel and -0.76 rad yaw. No tracking pass. |
| `weights4_vy` | Same pose; velocity `[300,300,30,1000,30,30]` | Fell at 10.40 s. |
| `weights5_vy` | Set 3 with footCenterOffset = 0 | Upright, but slow and 0.57 rad yaw drift. |
| `weights6_vy` | Set 3 with vy weight 150, yaw-rate weight 1000, footCenterOffset = 0 | Fell at 10.42 s. |
| `shortgait_vy` | Set 3, slow_walk single support 0.40 s / double support 0.15 s | Upright 41 s; 2.17 m lateral travel, peak yaw error 0.054 rad, peak roll 0.036 rad. Mean vy 0.081 m/s for 0.15 commanded. |
| `shortgait_w7_vy` | Set 3 with vy weight 150, shorter gait | Upright 41 s; 2.72 m lateral travel, peak yaw error 0.184 rad. |
| `shortgait_w7_vx` | Same set 7 | Fell at 19.91 s after yaw divergence. |
| `shortgait_w8_vx` | Set 7, both contact-moment mz input weights 100 instead of 0.1 (before rScaling = 0.001) | Upright 41 s, 5.26 m forward, peak roll/pitch 0.036/0.040 rad, peak yaw error 0.422 rad. |
| `shortgait_w8_vy` | Same set 8 | Upright 41 s, 2.46 m lateral, peak roll/pitch 0.046/0.016 rad, peak yaw error 0.081 rad. |
| `saved_gt50_vx` | Set 8 saved in normal YAML, default WB-ID gait, 50 Hz | Upright 41 s, 4.93 m forward, but peak yaw error 0.770 rad fails the heading check. |
| `yaw1000_gt50_vx` | Set 8 with yaw-rate weight 1000 instead of 300 | Upright 41 s, 5.21 m forward, peak yaw error 0.546 rad still fails heading check. Not promoted. |
| `saved_kf100_vx` | Set 8 with original near-straight nominal joint posture; default linear KF | Fell 1.24 s after entering locomotion, before the walking command. Bring-up stood at 0.78 m while the requested MPC height was 0.60 m. |
| `crouch_kf100_vx` | Set 8 with nominal hip/ankle pitch -0.8 rad, knee 1.6 rad on both legs | Upright 41 s; 5.07 m forward, peak roll/pitch 0.035/0.038 rad, minimum height 0.599 m. Mean vx 0.197 m/s for 0.25 commanded. Peak yaw error 0.552 rad fails heading check. |

Higher tilt weights stabilized this lateral case; yaw-rate and speed tracking still need improvement without restoring absolute xy/yaw pose penalties.
Shortening single support improved lateral walking more than simply escalating the motion weights.
The new `gait_wb_id.yaml` changes only `slow_walk`; the launch selects it for WB-ID, leaving centroidal on `gait.yaml` and respecting an explicit `gaitLibraryFile` override.
(Since 2026-10-23 both libraries are the `ocs2.gait.templates` parameters of their controller blocks; the gait files and `gaitLibraryFile` are removed.)
It also corrects the inherited `skip` template's non-monotonic 0.08 s entry to 0.8 s; that gait is not tested.
Set 8 is now saved in `ros2_controllers.yaml`, only in the WB-ID block.
Increasing contact yaw-moment regularization helped forward survival; this is still a soft cost and does not supply the missing torsional wrench-feasibility constraint.
The crouched nominal joint posture is also saved in the WB-ID block; it is used by both reference generation and bring-up and matches the default 0.60 m target without changing estimator parameters or using GT feedback.
The last parameter-only change needs no C++ rebuild with the current shared symlink install.
The final combined configuration has not yet been rerun on GT lateral or on estimator-driven lateral; the preceding GT lateral pass used the same weights/gait with the old nominal joint posture.

## Model and Tuning Interpretation

The state is `[q, v]`, including floating-base position/Euler-ZYX coordinates and joint positions; input is two six-axis contact wrenches, generalized acceleration, and joint torque.
Base RNEA rows are enforced throughout the horizon; joint torque rows only within the initial torque window.
Stance constraints regulate contact acceleration with position/velocity feedback; this is not exactly the papers' discrete no-slip velocity constraint.
The default base-height weight 10 and velocity weights 3 were weak relative to competing posture, torque and foot costs.
Increasing height/tilt and commanded-velocity weights substantially improved tracking/stability in the tests above without removing local xy/yaw invariance.
Increasing SQP iterations alone was counterproductive due to additional delay.
Nonlinear equality residuals spike during contact transitions even in upright runs; log them alongside motion and solve time, not as a stand-alone success criterion.

The low-level `effort_pd` path uses hardware joint gains (typically Kp 1200, Kd 10), not the generic `control.mpcJointKp` field.
The MuJoCo model has joint damping 10; no corresponding passive damping term was identified in WB RNEA, but its effect has not been isolated.
Do not change shared damping/contact physics to hide a WB-only issue or inadvertently regress centroidal MPC.
The capture-foot-placement implementation approximates CoM velocity using base linear velocity for WB; disabling it outright failed in the tested configuration.
The launch default estimator is `humanoid_linear_kf`; explicitly selecting InEKF tests a different configuration.

## Next Verification

1. User is taking over testing; do not start another simulation concurrently.
2. Recheck final saved configuration on both axes, GT and linear KF, at 50/100 Hz; InEKF has not been tested in this WB-ID tuning session.
3. Address yaw drift and speed error without an absolute xy/yaw reference or hiding the error by loosening test thresholds.
4. Consider a proper torsional contact-wrench bound; the current mz penalty is not a feasibility guarantee.
5. Only slow_walk was retuned; higher-speed gait transitions, terrain, other pelvis heights, and hardware remain unverified.
6. No commits or pushes requested for this task.

## Current User Test Command

After rebuilding the latest checkpoint, run in the sourced Jazzy workspace.
The GUI is enabled here for manual testing; keep it disabled for automated commands.

```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller \
  floatingBaseSource:=state_estimator floatingBaseEstimator:=humanoid_linear_kf \
  mujocoModelFile:=scene_flat.xml refBaseHeight:=0.6 \
  mpcFreq:=100 mrtFreq:=1000 \
  rviz:=true mujoco_headless:=true baseCommandGui:=true
```

Start with vx = 0.25 m/s or vy = 0.15 m/s separately, with gentle ramps and zero yaw command.
Under the new WB-ID limits, normalized vx=1 means 0.25 m/s and normalized vy=1 means 0.15 m/s; GUI readouts use those same scales.
The fixed torque-window unit test was compiled and run manually without `NDEBUG`; it is not yet registered with CTest.
Python syntax and `git diff --check` passed.
