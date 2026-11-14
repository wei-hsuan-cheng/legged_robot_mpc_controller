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


## Scene Style Alignment (2026-11-14)

Upkie's `scene.xml` now follows H2's short composition layout: timestep, background, optional terrain/object includes, then robot.
The new `common/backgrounds/background_night_upkie.xml` reuses the humanoid night appearance, textures, lighting strengths and floor properties, but aims lights at `base` and uses smaller viewer framing.
The existing G1/H2 backgrounds are unchanged, and Upkie's original robot colors and material definitions remain intact.
Ramp and uneven-field includes are present as comments, so the default Upkie scene remains flat and does not imply newly validated terrain locomotion.
`implicitfast` moved into `upkie.xml` beside its existing four no-slip iterations, while the scene retains the 0.0005 s timestep and therefore 2 kHz physics-rate PD.
No controller code, gains, actuator caps, inertias, contact geometry or friction parameters were changed.
Local MuJoCo 4.0.1 compilation compared 38 physical arrays/options against the prior scene, all exactly unchanged, including joint/body parameters, contact settings and actuator limits.
A static render at the configured standing pose was visually inspected; the temporary preview is `/tmp/codex_upkie_night.png` on the Mac.
Updated scene-layout tests cover Upkie's assets, night-style/light targets, common floor physics, and flat/ramp/rough fixtures.
Four focused scene tests passed; the complete layout suite passed 14 of 15 tests, with the remaining pre-existing xacro ramp test failing on last-digit vertex-string formatting in the local environment.
The same ramp failure was reproduced from HEAD without the test changes; no terrain files were regenerated to hide it.
These checks compile/render scenes, not closed-loop MPC validation or a ROS build; no robot-controller simulation was launched.


## URDF Visualization Fix and Rate Headroom (2026-11-14)

The user reported correct MJCF rendering but disconnected leg meshes in RViz.
The generator's quaternion-to-RPY conversion lost the coupled roll/yaw rotation at exactly +/-90-degree pitch, producing rotation-matrix entry errors as large as 1.0 for ten geometries.
Added an explicit singular branch, regenerated the URDF and corrected six visual meshes plus four collision primitives.
All seven joint definitions and seven inertial blocks remain byte-for-byte identical as parsed XML; no MJCF, controller gains, contact equalities or MPC settings changed.
Three local regression tests pass: exact/near-singular rotations, all source quaternions and generated geometry/joint/IMU pose agreement.
An offline MuJoCo import/render of the old and corrected URDF reproduced the user's visual defect and confirmed that the corrected model matches the MJCF assembly.
The temporary three-panel comparison is `/tmp/codex_upkie_urdf_comparison.png` on the Mac; it is a static asset check, not an RViz or closed-loop runtime test.
No ROS nodes, builds or closed-loop tests were started, and the user's separate `description/urdf/env.urdf.xacro` edit was left untouched.
Relaunch the robot-state publisher and visualization to reload the corrected URDF; the generator fix does not require new CppAD libraries because the rigid-body model is unchanged.

The user's runtime logs request 100 Hz and measure about 97.7-98.0 Hz, with average MPC advances around 1.8-2.0 ms and observed interval peaks up to 4.9 ms.
The worker deliberately sleeps until its target period, so these logs do not imply a 100 Hz computation ceiling.
The measured advance excludes some post-solve diagnostics/publication and does not bound future worst-case latency.
Recommend 150 Hz as the next trial (6.67 ms period), or 125 Hz (8 ms) for more scheduling margin; neither rate has been validated here.
Do not claim stable 200 Hz from this evidence: its 5 ms budget leaves only about 0.1 ms beyond the observed 4.9 ms solve peak, before other work and scheduling jitter.
Keep the current controller parameters unchanged until the user requests a rate change or benchmark.
The reported zero solver exceptions and zero torque-window overruns are useful health evidence, but the logged equality residuals are not a physical stability guarantee.
Per-term maximum SSE is an unweighted sum over intermediate nodes in the diagnostic loop, whereas the aggregate solver metric uses its own accumulation; do not interpret their different numerical scales as a logging contradiction or a per-joint torque error.


## Formulation Documentation (2026-11-14)

Expanded the formulation with a source-to-code traceability table, exact paper sections/equations/pages and links next to the relevant formulas.
The notes distinguish world-aligned tire-center wrenches from body-axis or world-origin wrenches, and identify the six-wrench layout and shared lateral row as local adaptations.
Friction references support a differentiable model family, not the current numeric values: 0.1 Nm friction and 0.005 kg m^2 armature come from the MJCF, while 0.05 rad/s is an unoptimized local smoothing choice.
Documented the different tanh scale/sign conventions in Pyrhonen et al. Eq. (10), the actual partial-test evidence, and a proposed future calibration protocol clearly separated from completed tests.
No new tuning, runtime validation or controller changes were performed for the reference update.
Added the implementation-specific math document linked above after reading the current controller, configuration builder, contact kinematics, cost evaluators and OCS2 penalty functions.
It uses the existing humanoid document's dollar-delimited inline/display LaTeX and one-sentence-per-line prose, with two blank lines before headings.
It distinguishes the 24-state/30-input Euler-coordinate OCP, full-horizon RNEA, armature and smoothed Coulomb friction, eleven double-support wheel-contact rows, tracking costs, soft inequalities and final actuator clipping.
The document records the shared friction cone's default regularization of 25 N^2, whose zero-tangential-force nonnegative-residual threshold is about 8.33 N at coefficient 0.6.
It also records that the shared lateral equality is a signed average without axle-sign alignment, so it is not a proof of individual-wheel lateral no-slip; cancellation and equality rank need checking in future tests.
No controller code, parameters, model assets, build or simulation were changed or run for this documentation update.


## Rolling-contact Estimator Scope (2026-11-14)

Documented the need for rolling-contact estimation and its primary reference in [formulation Section 7.5](wheeled_biped_whole_body_inverse_dynamics_mpc.md#75-rolling-contact-state-estimation-required-future-adaptation).
[Hosseini, Rodriguez and Behnke, State Estimation for Hybrid Locomotion of Driving-Stepping Quadrupeds, IRC 2022](https://www.ais.uni-bonn.de/~hosseini/se-dsq/IEEE_IRC_22_StateEstimationForDrivingSteppingQuadrupeds.pdf), Fig. 1 and its wheel-aware state-estimation formulation, is a direct reference for adapting legged Kalman-filter measurements to wheel geometry and rolling motion.
Source inspection confirms that our current linear KF assumes stationary planted-contact landmarks; selecting it unchanged is not a validated wheel-drive estimator.
The required main output remains floating-base pose and velocity, but tire-center motion must be distinguished from the approximately zero material tread velocity under no slip.
First adapt velocity and ground-relative height measurements using IMU, leg/wheel encoders and configured tire geometry, with contact/slip reliability gating.
Slip, effective radius, terrain normal and additional biases are optional extensions rather than automatically required extra states; contact forces can remain outside the estimator state.
Proprioception does not supply drift-free global XY/yaw, and arbitrary terrain elevation is not independently observable without assumptions or additional sensing.
Current Upkie runtime evidence still uses ground truth; no rolling estimator implementation, build or estimated-state closed-loop validation was added in this checkpoint.
The next estimator milestone is an estimated-versus-simulator comparison followed by closed-loop balance, drive, braking and push acceptance on estimated state.


## Yaw/Push Failure Fix and Validation (2026-11-14)

The user reported that forward/backward driving works but yaw-only commands and pushes immediately destabilize Upkie.
A baseline 0.3 rad/s yaw test at 100 Hz reproduced a fall 5.84 s after takeover, about 1.84 s after the yaw ramp began; artifacts are `/tmp/codex_upkie_yaw_baseline*` in `ros_jazzy`.
The original four-pulse push sequence also fell at 7.01 s, during recovery from the second pitch-torque pulse, before either force pulse; artifacts are `/tmp/codex_upkie_push_baseline*`.
Before the yaw fall, hip/knee position tracking errors stayed below roughly 0.010/0.006 rad, without actuator saturation, while planned and measured base behavior diverged.
Two implementation defects were found rather than treating this as missing turning capability or immediately retuning costs.

1. The mean lateral row averaged opposite signed wheel axles, canceling at the symmetric posture: its acceleration Jacobian norm was 1.52e-14 versus 1.04 after expressing both wheels in a common lateral basis.
   The corrected row uses the first wheel's lateral direction for every supporting wheel and includes the rotating-direction derivative; longitudinal slip now includes that derivative as well.
   This still constrains mean lateral slip, not independently both tires, and single-support/large-asymmetric-axle cases remain unvalidated.
   Generated wheel libraries use `_v2` names so existing `_v1` tapes cannot silently retain the old function.
2. MuJoCo ground-truth feedback mixed link-frame pose with inertial-frame/COM twist: `mj_objectVelocity(..., mjOBJ_BODY, ..., 1)` returns the principal-inertia frame, not the regular body frame.
   Upkie's base inertia axes are nearly 90 degrees from its link axes; pure physical yaw of 0.3 rad/s returned approximately `[0.01125, 0.29972, -0.00625]` in that wrong basis, which the controller interpreted as link-frame angular velocity.
   Both ros2_control body-state feedback and published odometry now use `mjOBJ_XBODY`, consistent with `xpos/xquat` and the advertised child frame.
   See [MuJoCo 3.3.2 source, mj_objectVelocity](https://github.com/google-deepmind/mujoco/blob/3.3.2/src/engine/engine_support.c#L1166); this simulator fix is in the separate `mujoco_ros2_control` repository and requires rebuilding it too.
   A synthetic rotated/off-center-inertia regression test matches link-origin Jacobians for yaw, pitch, translation and mixed motion, and verifies both feedback paths select XBODY.

The GUI height slider and published-command clamp now use `defaultBaseHeight +/- maxDeltaPelvisHeight` from the selected YAML instead of hard-coded 0.2-1.0 m.
Upkie therefore spans 0.22-0.38 m with 0.30 m at the slider center, while H2 can command its configured heights above 1 m.
The launch passes the existing height delta to the GUI without adding a launch argument or controller parameter; four headless GUI mapping/clamping tests pass, including a fixed-height zero-delta configuration.
Both packages were rebuilt in Release in `ros_jazzy`; wheel `_v2` CppAD functions were generated on first launch and reused afterward.
No MPC cost, constraint gain, PD gain, actuator cap, friction value or command slew limit was retuned.
The following current-parameter tests used ground-truth feedback, an isolated flat scene, requested MPC 100 Hz and MRT 1 kHz, with tracking and joint logging.

| Test/artifact prefix in container `/tmp/` | Result | Key observation |
| --- | --- | --- |
| `codex_upkie_yaw_fixed_1` | Balanced for 19 s; tracking verdict NO_TRACKING | +0.3 rad/s command averaged +0.145 rad/s; max pitch 0.00551 rad, roll 0.00260 rad; slow yaw still undertracks |
| `codex_upkie_yaw_fixed_reverse` | SUCCESS, 19 s | -0.6 rad/s command averaged -0.359 rad/s; max pitch 0.00374 rad, roll 0.00328 rad |
| `codex_upkie_yaw_fixed_max` | SUCCESS, 17 s | Abrupt +1.5 rad/s command with harness ramp 0, controller slew unchanged; mean +1.121 rad/s, max pitch 0.01036 rad, roll 0.00282 rad |
| `codex_upkie_push_fixed_1` | SUCCESS, all four pulses verified | 25 s; max pitch 0.1853 rad, roll 0.000633 rad; final recovery pitch <=0.0103 rad |
| `upkie_codex_fixed_push_1`, `upkie_codex_fixed_push_2` | SUCCESS, both full sequences verified | Two repeats of the suite's 26 s push/drive test; max pitch 0.1848/0.1846 rad, roll <=0.000220 rad |
| `upkie_codex_fixed_roll_1`, `upkie_codex_fixed_roll_2` | SUCCESS | Two 27 s forward/brake tests; measured mean +0.2017/+0.2017 m/s for +0.2 m/s |
| `codex_upkie_reverse_drive` | SUCCESS, 16 s | Measured mean -0.1996 m/s for -0.2 m/s; max pitch 0.01359 rad, roll 0.000228 rad |
| `codex_h2_xbody_regression` | SUCCESS, 15 s | Shared simulator-frame change: H2 forward/brake smoke test, no fall, max pitch 0.02597 rad and roll 0.01472 rad; this is not a full H2 regression |
| `codex_g1_xbody_regression` | SUCCESS, 15 s | G1 forward/brake smoke test, no fall, mean +0.1778 m/s for +0.2 m/s; max pitch 0.00841 rad and roll 0.00586 rad |

Push disturbances were unchanged: +/-1.5 Nm pitch torque then +/-6 N forward force, each 0.2 s, with 3 s pauses.
In the first corrected push trial both contact flags stayed true, measured normal loads remained positive during recovery, and no joint reached the configured effort limits.
The fixed yaw trial's pre-fall-window rate-versus-pose-derivative RMS mismatch dropped from 0.0436 to 0.00230 rad/s for yaw, and from 0.0778 to 0.000719 rad/s for pitch.
The phase/sampling comparison is diagnostic rather than an exact trajectory-matched estimator benchmark.
Joint command/measurement plots show tight position tracking after the fixes; wheel velocities use the mirrored encoder sign convention, so both encoder speeds having the same sign during an in-place turn is correct.
Typical mean solves were 1.9-2.6 ms and p95 about 4.0-4.4 ms; actual logged rate was mostly 96-98 Hz at the requested 100 Hz.
Occasional wall-time outliers occurred, including a 263 ms sample in the first push run, so the earlier 4.9 ms peak is not a worst-case bound and no higher-rate guarantee is justified.
Retain 100 Hz for this tested checkpoint rather than changing frequency while validating the model fixes.
Slow-turn tracking, yaw deadband/friction effects, arbitrary push directions and larger disturbances remain tuning/validation work; the current result is not a guarantee of general push recovery.

Visual evidence on the Mac: `/tmp/codex_upkie_yaw_replay.png` compares measured-state replays of the original 0.3 rad/s fall with the corrected 1.5 rad/s turn, and `/tmp/codex_upkie_push_fixed_1_wheeled.png` shows attitude, velocity, slip, contact force and effort recovery curves.
The mapped Tk GUI was also captured and inspected at `/tmp/codex_upkie_gui_height.png`, with a centered 0.300 m slider and labeled 0.220/0.380 m endpoints; its preview callback publishes nothing.
The analyzer now writes `_wheeled_tracking.png` with planned/measured yaw rate, per-joint command errors and pose-derivative consistency, and includes these statistics in the JSON summary.
The analytic wheel geometry/basis derivative executable passed finite-difference regression checks; four headless GUI tests and two sourced ROS launch-resolution tests passed, as did the two simulator feedback-frame tests.
No simulator-frame compatibility switch was added: XBODY restores the already-advertised link-frame contract rather than offering a second measurement convention.


## Follow-up Model Audit (2026-11-14)

The frame correction is shared with H2/G1: their ground-truth twist now follows the link axes and origin that their pose and controller already assume.
It changes numerical feedback when the inertia axes or COM differ, but does not change those robots' MPC equations, weights, gains or estimator implementation.
H2 also passed `/tmp/codex_h2_xbody_yaw_audit` at requested 80 Hz: +0.3 rad/s averaged +0.29645 rad/s, max pitch/roll 0.01582/0.02635 rad, with no fall over 18 s.
That in-place turn drifted 0.495 m, so it is a turning smoke test rather than proof of unchanged full H2 performance.

The audit compared the generated RNEA and wheel equations against direct Pinocchio calculations, differentiated the material tread slip numerically along the double-integrator trajectory, checked every state/input Jacobian column, checked wrench virtual work, and checked double-support input-equality rank at 12 nominal/perturbed states.
`tests/wheel_model_consistency_test.cpp` passed: maximum equation error 1.07568e-10, maximum normalized AD Jacobian error 1.18102e-08, and minimum singular value of the 23-by-30 input-equality Jacobian 0.0715591.
Finite differences are used only for testing; production derivatives remain CppAD generated.
No further defect was found in these tested equations; this finite audit is not a proof for arbitrary tilt, changing ground normals, flight or single support.

The scan found and corrected two initialization/playback inconsistencies:

- `WbIdInitializer` now seeds each supporting wheel moment as `rho x force` and adds configured smoothed joint friction to its seed torque.
  The native audit checks its joint RNEA and tire-moment residuals at nonzero velocity and tilt; this is still an initial guess, not a projection onto all base/rolling equalities.
  Wheel-disabled, friction-empty G1/H2 configurations retain the old initializer calculation.
- The optional stand-up exporter used a pre-step lift-off time and omitted the actual lift sample, leaving handover after its last exported sample.
  The regenerated plan ends at its 0.6965 s handover; all earlier position/velocity/effort rows are unchanged and no search/tuning was performed.
  Loading now rejects nonfinite sample values and non-strict timestamps, and GT bring-up uses the base orientation for tilt/gravity hold rather than the rotated IMU-site orientation.
  Estimator-based bring-up retains its alignment assumption; full-stack stand-up remains unvalidated and disabled by default.

Six local Python tests pass (four GUI, two stand-up export/data), along with the two separate simulator frame tests.
The sourced ROS launch suite passes six of seven tests; its unrelated environment test expects at least one visual, but the user's currently edited `description/urdf/env.urdf.xacro` is empty.
That user edit is preserved and excluded from the commit rather than modifying their scene or weakening the test.

To rerun the native consistency audit from the sourced container workspace:

```bash
MAKEFLAGS=-j2 colcon build --symlink-install --packages-select legged_robot_mpc_controller \
  --cmake-target wheel_model_consistency_test \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
./build/legged_robot_mpc_controller/wheel_model_consistency_test \
  src/legged_robot_mpc_controller/description/urdf/robots/upkie/upkie.urdf \
  /root/ocs2_ros2_ws/auto_generated/upkie
```

Use the normal Release build with `BUILD_TESTING=OFF` afterward to install all targets.

The final complete Release build passed in 1 min 45 s, and the native equation/Jacobian audit passed again against the installed libraries.
The following tests were rerun after the initializer and GT bring-up corrections, without parameter changes:

| Container artifact prefix under `/tmp/` | Result | Measurements |
| --- | --- | --- |
| `codex_upkie_final_yaw` | SUCCESS, 17 s | Abrupt +1.5 rad/s request; mean +1.1199 rad/s; max pitch/roll 0.01207/0.00309 rad |
| `codex_upkie_final_push` | SUCCESS, 25 s, actual repository night scene | All four original pulses verified; max pitch 0.18804 rad; final recovery pitch <=0.00401 rad; no actuator saturation |
| `codex_upkie_final_drive` | SUCCESS, 21 s, actual repository night scene | +0.2 m/s request averaged +0.20146 m/s; max pitch/roll 0.01246/0.00076 rad; braking remained balanced |
| `codex_h2_xbody_final_yaw` | SUCCESS, 18 s at requested 80 Hz | +0.3 rad/s request averaged +0.29774 rad/s; max pitch/roll 0.02061/0.04042 rad; translation drift 0.487 m |

Logged actual rate means were 97.46 Hz for Upkie yaw, 97.58 Hz for push, 95.59 Hz for drive, and 77.48 Hz for H2 at its 80 Hz target.
These are VM wall-clock measurements with scheduling outliers, not guaranteed rates; Upkie drive had one 82.11 Hz reporting window.
All final Upkie health windows reported zero solver exceptions and torque-window overruns, but finite equality residuals remain because the controller uses one SQP iteration.
The updated measured-state replay `/tmp/codex_upkie_yaw_replay.png` compares the original fall with the final yaw trial; `/tmp/codex_upkie_final_push_wheeled.png` and `/tmp/codex_upkie_final_yaw_wheeled_tracking.png` were generated and inspected.
The push curves show decaying attitude/velocity responses, positive wheel loads and no saturation; the turn still shows periodic planned-rate ripple and command undertracking, not an exact yaw-tracking solution.
All owned simulations were stopped after these tests; full-stack stand-up and arbitrary-direction pushes remain future validation.
The required simulator fix was pushed as `mujoco_ros2_control` commit `44f395d` on `jazzy`; pull and rebuild it together with this controller checkpoint.


## Status (2026-11-14)

| Milestone | State |
| --- | --- |
| Model import and MJCF/URDF reconciliation | Done |
| Rolling-wheel WB-ID MPC, wheel drive, ground truth (standing spawn) | Works: balance, 0.2 m/s drive and braking in the first run |
| Unmodelled wheel friction (zero-command creep) | Diagnosed; friction model added; comparison against wheel PD started, only the friction-model variant ran (results below) |
| Activation segfault | Root cause found and fixed (hard-coded foot frame names) |
| Flat acceptance after yaw/push fixes | Passed repeated forward/brake, reverse/brake, four complete push sequences and turning tests without falls; final night-scene drive/push repeats also passed; slow +0.3 rad/s turn still undertracks |
| Kneeling idle -> planned stand-up -> MPC | Planned and implemented; never run in the full stack |
| Small symmetric hop | Not started (gait-request topic is ready) |
| Short single support, then trot | Not started |
| Estimator closed loop (linear KF adapted to rolling) | Not started; ground truth only so far |
| G1/H2 regression after the shared-code changes | Passed short flat forward/brake smoke tests and two H2 80 Hz turning tests; turning drift remains and broader regression is open |


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

# 3. Push the base from another terminal after MPC takeover.
SHARE="$(ros2 pkg prefix --share legged_robot_mpc_controller)"
ros2 run legged_robot_mpc_controller push_recovery_test.py \
  --sequence "$SHARE/config/tests/upkie_pitch_push.yaml" --delay 5

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

1. Preserve the current successful flat checkpoint and investigate slow-yaw undertracking before changing gains; repeat any friction-model versus wheel-kd comparison only with corrected XBODY feedback, since the older angular-feedback data are not a valid tuning baseline.
2. Run the planned stand-up in the stack; adjust the plan (`plan_stand_up.py --search`) or the handover if the MPC cannot catch it.
3. Hop: add a `hop` template (e.g. STANCE 0.5 s, FLY 0.1-0.15 s) and request it on `/humanoid/gait_command`; check takeoff/landing torques, measured flight (contact sensors) and touchdown slip; consider a wheel-spin cost in flight.
4. Short single support (`left_leg`/`right_leg` templates for ~0.1-0.2 s): no hip roll, so lateral balance comes only from rolling the trunk with leg-length differences; measure roll before attempting alternating support (trot).
5. Estimator: adapt the linear KF to rolling contacts (wheel contact gives a base-velocity measurement through zero tread velocity and a height measurement; no fixed foot landmark), then rerun the milestones closed loop on the estimate.
6. Extend the passed G1/H2 flat forward/brake smoke tests to broader regression after the shared changes (`computeJointTorques` signature, optional arm-swing joints, height-reset argument, gait topic subscription, tracking columns and simulator twist convention).


## Traps found

- rclcpp rejects empty YAML lists (`fixedJointNames: []`): omit the key.
- `pkill -f <pattern>` inside `bash -c` matches its own command line (the workspace path contains `ros2`); kill by PID.
- Editing the parameter schema while a build runs leaves the generated header stale; rebuild after.
- `jointArmature` is baked into the generated CppAD libraries (set `codegen.recompileLibraries` or clear `auto_generated/upkie` after changing it); `jointFriction` and the wheel gains are parameters.
- Pass the gdb launch prefix as one argument: `--launch-prefix "gdb -batch -ex run -ex bt --args"`.
- Old `mujoco_ros2_control` builds returned principal-inertia/COM twist alongside link pose; rebuild the simulator with the XBODY fix, not only the controller, and sanity-check rates against pose differences.
