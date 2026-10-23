# WB-ID MPC Walking Robustness Handoff

## Latest Checkpoint: User Will Build and Test

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
