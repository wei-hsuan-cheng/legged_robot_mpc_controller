# H2 Whole-Body Inverse Dynamics MPC

The initial H2 integration was on `feat/h2`, branched from `feat/wb_id_mpc` at `f56ae9a`; the faster-walking checkpoint below is on `tune/h2`.
It reuses the existing WB-ID controller, reference generation, costs, constraints, estimators and bring-up sequence without changing the C++ architecture or G1 configuration.
The H2 controller YAML contains no centroidal MPC block.


## More Natural Foot Placement (2026-10-24, Current)

This follow-up supersedes the 0.10 m foot radius and collision penalty values in the historical checkpoint below.
The user accepts modest velocity error and prioritizes balance, recovery, no crossing, and less front/back foot placement during lateral walking and turning.
No new controller C++ structure or foothold planner was added in this follow-up.

| Part | Saved change from `f80044c` | Simple reason |
| --- | --- | --- |
| [Hip-pitch posture cost](../config/h2/ros2_controllers.yaml#L359) | Left/right hip-pitch Q and Qf: `0.05 -> 3.0` | Discourage staying with one leg forward and the other backward; this is a soft posture preference, not higher actuator PD stiffness |
| [Foot sphere radius](../config/h2/ros2_controllers.yaml#L535) | `0.10 -> 0.085 m` | Remove unnecessary clearance that can encourage the feet to pass front/back rather than side-by-side |
| [Collision polynomial penalty](../config/h2/ros2_controllers.yaml#L540) | `mu: 3000 -> 1000`, `delta: 0.03 -> 0.02 m` | Retain a pre-contact penalty without making a tandem stance unnecessarily attractive; a stronger tested penalty gave worse turning posture |
| [Knee radius and left/right ordering](../config/h2/ros2_controllers.yaml#L531) | Keep `0.10 m` knee radius and `0.10 m` minimum lateral separation | Keep the existing knee envelope and anti-crossing preference |
| Commands, gaits, PD and estimator | Unchanged from `f80044c` | Do not hide failures by reducing command maxima or the original `0.5 m/s2` lateral slew |
| [Replay analysis](../tests/tuning/analyze_leg_clearance.py) | Add pelvis-heading fore/aft foot separation; support G1 comparison | Measure whether the gait actually becomes less staggered rather than only checking for a fall |

The three foot-proxy centers are at the sole center and at 60 percent of the rectangle's forward/backward bounds.
For the active H2 MuJoCo foot primitives, sampled small-sphere extents and box corners need about 0.0692 m nearest-proxy radius; 0.085 m retains about 1.6 cm padding at the worst sampled extreme.
G1's corresponding sampled padding is about 1.7 cm with its unchanged 0.065 m radius.
The H2 knee-area mesh audit found about 0.090 m maximum horizontal radius among vertices near the knee, so its 0.10 m knee radius was retained.
These are local proxy/primitive checks, not proof that the complete moving leg meshes are enclosed or that soft collision penalties cannot be violated.
The geometry regression checks sampled primitive extremes, not continuous-time mesh clearance.

Stagger means the absolute difference between left/right sole-center positions projected onto the pelvis-heading forward direction.
It is not distance travelled per step; some fore/aft separation is normal during forward walking.
The mean is taken over the commanded walking/turning hold, including swings and double support.

| Flat-ground step command | Hold / total after takeover | Measured result with saved values |
| --- | --- | --- |
| vx `+1.0 m/s` | 30 / 40 s | Upright and stopped; 29.91 m versus 30.0 m requested; pose-derived world vx `0.951 m/s` |
| vy `+0.5 m/s` | 20 / 30 s | Upright and stopped; pose-derived world vy `0.443 m/s`; mean stagger `10.57 cm`, versus `27.90 cm` in the earlier positive lateral baseline |
| vy `-0.5 m/s` | 20 / 30 s | Upright and stopped; pose-derived world vy `-0.449 m/s`; mean stagger `9.02 cm` |
| yaw `+0.7854 rad/s` | 12 / 22 s | Upright and stopped; rotation `8.75 rad` versus `9.42 rad` requested; mean stagger `3.73 cm`, versus `28.55 cm` in the earlier turning baseline |
| yaw `-0.7854 rad/s` | 12 / 22 s | Upright and stopped; rotation `-8.76 rad`; mean stagger `3.54 cm` |

No crossing or sampled opposite-leg overlap was found in these five trials.
Minimum signed widths were 10.78 cm forward, 14.24/15.82 cm lateral and 16.58/16.52 cm turning; the soft ordering margin is still not a hard bound.
Minimum base heights were 0.916 m forward, 0.959/0.965 m lateral and 0.988/0.976 m turning.
The final positive lateral stance had about 2.52 cm stagger and the final positive turning stance about 1.09 cm.
The lateral curves still show periodic sway and wide stepping; this is an improvement, not identical G1 behavior or perfectly side-by-side feet at every instant.

The G1 comparison at vy `0.2 m/s` had mean stagger 1.32 cm, but achieved only about `0.062 m/s` pose-derived lateral speed and returned `NO_TRACKING`.
It therefore provides a visual reference, not a matched-speed proof that H2 should have identical foot motion.
A 0.075 m H2 foot radius with hip-pitch weights 3.0 also worked, with 7.32 cm lateral and 3.79 cm turning mean stagger, but retains less padding around sampled foot extremes.
A 0.085 m radius with `mu=6000, delta=0.02` stayed upright yet produced 31.06 cm turning stagger; that combination was rejected despite its `SUCCESS` balance/tracking verdict.
The saved lower penalty was selected using walking/turning and geometry evidence, before the push test.

All trials used `scene_flat.xml`, `base_twist`, `state_estimator` with `humanoid_linear_kf`, requested MPC90/MRT1000, `baseCommandGui:=false`, no RViz, and no publisher ramp.
The original internal command slew remains active, including the requested `0.5 m/s2` lateral value.
G1 configuration, physical models, PD gains, estimator tuning, base-state/input costs and existing gait settings were not changed in this follow-up.


## First Push Test (Observation Only)

The user requested the first result only, without push-based tuning.
The saved walking parameters above were frozen before this trial and were not changed in response.

[push_recovery_test.py](../launch/push_recovery_test.py) streams a [YAML list](../config/tests/push_recovery.yaml) of body names, frames, forces, torques, pulse durations and pauses to the existing MuJoCo external-wrench interface.
It uses simulation time, best-effort QoS, and clears previously touched bodies between pulses and on exit.
The walking auto-test uses the same scheduler and records simulator-applied wrench feedback in `PREFIX_wrenches.csv`, rather than accepting publication alone as proof of a push.
Feedback vectors are body-frame even for a world-frame command, so verification compares force/torque magnitudes and sample times.

| First trial | Observation |
| --- | --- |
| Scene and motion | H2 flat ground, vy `+0.5 m/s`, 20 s hold, 30 s total after takeover |
| Push sequence | World-frame pelvis force `[0,+80,0] N` at 10.00 s, then `[0,-80,0] N` at 14.15 s; each requested for 0.15 s, with zero applied torque |
| Simulator confirmation | 80 N magnitude confirmed; 16 and 14 nonzero feedback samples, spanning 0.15 and 0.13 s; zero wrench confirmed at the end |
| Outcome | Completed both pushes, continued walking and stopped upright; full test verdict `SUCCESS` |
| Whole-trial tilt and height | Maximum roll/pitch `1.44/1.01 degrees`; minimum base z `0.942 m` |
| Walking speed | Pose-derived world vy `0.448 m/s` |
| Clearance | Minimum signed width `16.27 cm`; no crossing or sampled inter-leg overlap |
| Final stance | Base z `1.008 m`, roll `0.141 degrees`, pitch `-0.046 degrees`; pose-derived speed settles near zero |

Plots show bounded gait oscillations during and after both pulses, followed by a settled stop; this is one moderate walking-push observation, not a measured recovery envelope.
Larger forces, stance-only pushes, torso pushes, torque pulses, multiple simultaneous forces, mixed-axis commands, current tuning on rough terrain, InEKF feedback and real hardware remain unvalidated.
The controller still follows the commanded contact schedule; this test does not establish autonomous recovery stepping while a zero walking command holds the stance gait.

Build/install passed in Docker `ros_jazzy` with two jobs, and the installed `ros2 run` entry passed its dry run.
The 37 focused WB-ID checks and parameter contract passed; discovery ran 31 tests with four native ROS tests skipped locally.
Five new sequence checks cover pulse boundaries, pauses, repeated simulation time, input validation and zeroing on link changes/stop.
Artifacts are in Docker `/tmp/h2_natural_*` and locally under ignored `tuning_results/h2_crossing/`; use `_push.png`, `_clearance.png`, `_support.png` and measured-pose renders for review.
No additional push trials or push-driven parameter adjustments were performed.


## Changes in Plain Words

This historical controller/tuning checkpoint is commit `f80044c` on `tune/h2`; the current follow-up above changes the foot radius, collision penalty and hip-pitch posture weights.
The starting values below describe the working configuration at the beginning of testing, including the user's uncommitted edits, rather than only the previous Git commit.

| Part | What changed or was retained | Intuitive effect |
| --- | --- | --- |
| [Foot/knee clearance envelopes](../config/h2/ros2_controllers.yaml#L526) | Kept the user's 0.10 m sphere radii | Treat the feet and knees as needing more space around each other; these larger envelopes were already present before my tests |
| Collision penalty | `mu: 10 -> 3000`, `delta: 0.00001 -> 0.03` | Start discouraging close approaches before contact, and make squeezing the legs together more costly; this alone did not prevent every fall |
| [Anti-crossing code](../src/core/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp#L146) | Added optional `minimumLateralSeparation: 0.10` | Prefer the left foot to remain left of the right foot, even when one foot is lifted; lateral crossover disappeared in the final sampled trials |
| [Double-support handling](../include/humanoid_common_mpc/constraint/FootCollisionConstraint.h#L94) | Keep the new ordering penalty active while both feet support the robot; leave the old sphere checks off in that phase | Keep an anti-crossing preference through landing and stopping without turning on every old proximity penalty against planted feet |
| [Faster walking rhythm](../config/h2/ros2_controllers.yaml#L609) | Single support `0.60 -> 0.30 s`; double support stays `0.10 s` | Take quicker steps instead of spending too long balancing on one leg; the shorter rhythm completed 1.0 m/s walking where the longer trial fell |
| [Gait selection](../config/h2/ros2_controllers.yaml#L654) | Use only `[stance, slow_walk, walk]` | Keep both-feet-on-ground intervals instead of escalating into unvalidated no-double-support or flight gaits |
| [Reaction to command changes](../config/h2/ros2_controllers.yaml#L652) | Gait switch delay fraction `0.7 -> 0.3` | Schedule the walking transition sooner when the command changes; abrupt lateral starts completed instead of collapsing in the tested comparison |
| [Generated-library safety](../src/core/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp#L58) | Use a separate `_lateral_v1` AD cache for the extra penalty row | Avoid loading an old generated function with the wrong number of outputs; this is a correctness measure, not a faster-walking gain |
| [Constraint copying](../src/core/humanoid_common_mpc/src/constraint/FootCollisionConstraint.cpp#L70) | Preserve the active flag when cloning the constraint | A disabled constraint stays disabled in its solver copies; this was not established as the cause of the original fall |
| [State recording](../tests/wb_id_walk_test.py) | Added optional joint/base replay and estimator snapshots | Let us inspect what the robot actually did, instead of judging only a final success/failure message; it does not change the control law |
| [Plots, replay images and tests](../tests/tuning/analyze_leg_clearance.py) | Added foot-width, overlap, speed, knee-motion and effort analysis plus regression checks | Distinguish a collision trip from a loss-of-support collapse, and check that the new margin stays independent of absolute odometry position/heading |

The final combination completed step-to-vx1.0 walking for 40 s, step-to-vy+/-0.5 walking for 30 s per trial and step-to-yaw0.7854 turning for 22 s, all including a stop back to stance.
No foot crossing or sampled inter-leg overlap was found in those final trials; positive lateral heading drift remains.
These are measured results of the combination, not proof that each individual setting alone guarantees balance.

The requested command maxima and original lateral slew of 0.5 m/s2 were kept; PD gains, Q/Qf/R weights, estimator settings and robot physical models were not changed in the retained tuning.
The tested 0.15 m/s2 lateral slew and increased yaw-rate cost were rejected and are not in the saved settings.
G1 remains on the original collision formulation because the new margin defaults to zero.
The anti-crossing penalty is soft, not a hard collision shield or a guarantee of push recovery.


## Faster Walking and Anti-Crossing Checkpoint (2026-10-24)

Balance, foot clearance and returning to stance take priority over heading accuracy in this checkpoint.
The user restored command normalization maxima to vx=1.0 m/s, vy=0.5 m/s, height delta=0.4 m and yaw=0.7854 rad/s before these trials; those maxima are retained, not reduced to hide a failure.
These are command scales rather than validated bounds on every combination of commands or terrain.

| Setting | Observed baseline | Saved value | Reason |
| --- | --- | --- | --- |
| Foot/knee sphere radii | User had already increased both to 0.10 m | 0.10 m each | Retain the user's larger collision envelopes |
| Collision polynomial penalty | `mu=10`, `delta=0.00001` | `mu=3000`, `delta=0.03` | The baseline had negligible pre-contact buffer and weak violation cost |
| `walk` template | 0.60 s single support, 0.10 s double support | 0.30 s single support, 0.10 s double support | Avoid long single-support excursions at faster speed |
| Selected gait ladder | Included no-double-support and flight templates | `[stance, slow_walk, walk]` | Keep tested double-support walking over the command range |
| Gait switch delay fraction | 0.7 | 0.3 | Let the contact schedule react earlier to step commands without slowing the upper-level command |
| `minimumLateralSeparation` | No signed ordering check | 0.10 m | Discourage a swing foot passing across the other foot even when the 3D distances remain sufficient |
| PD, state/input weights, estimator and command slew limits | Existing H2 values | Unchanged | Isolate the contact-clearance/gait fix; increased yaw-rate weighting did not improve the lateral trial |

The optional lateral margin is a row of the existing collision soft penalty, not a new foothold planner, fixed body-midline restriction or actuator clamp.
It uses the pelvis-heading lateral projection of left-minus-right foot-center position and is invariant to common horizontal translations and yaw rotations of robot and odometry frame.
Its polynomial penalty begins below `minimumLateralSeparation + delta`, which is 0.13 m for this configuration, and can still be violated because it is soft.
The row remains active in double support while the original 16 distance rows remain gated off there.
Set `minimumLateralSeparation: 0.0` to restore the original collision formulation; G1 defaults to zero and its YAML is unchanged.
Set the parent `activate: false` to remove all leg collision soft penalties, then restart the controller.
The enabled variant uses a separate `_lateral_v1` generated-library cache, avoiding reuse of the incompatible 16-row model.

Trials used `robot.launch.py robotName:=h2 mujocoModelFile:=scene_flat.xml`, `base_twist`, `state_estimator` with `humanoid_linear_kf`, requested MPC90/MRT1000, and no GUI or RViz.
The scene was genuinely flat, with no terrain map or foothold plan driving these steps.
Measured base/joint pose replays were used to inspect renders, signed foot width, 3D foot-point clearance, base height and pose-derived velocity curves.
The first comparison table below used the original 0.7 gait switch delay fraction; the subsequent abrupt-transition tests motivated the final 0.3 value.

| Trial | Duration after takeover | Result | Minimum signed foot width | Sampled opposite-leg overlaps |
| --- | --- | --- | --- | --- |
| Baseline vx=0.40 m/s | 9.45 s | Fell after crossing | -2.16 cm | Present before falling |
| Stronger penalty only, vx=0.40 m/s | 8.92 s | Fell; penalty alone was insufficient | Negative | Present |
| Moderate penalty and 0.40 s single support, vx=0.40 m/s | 36 s | Walked and stopped upright | 16.5 cm at 20 Hz replay sampling | None sampled |
| Same 0.40 s gait, vx=0.60 m/s | 36 s | Walked and stopped upright | 13.6 cm at 20 Hz replay sampling | None sampled |
| Same 0.40 s gait, vx=1.00 m/s | 17.70 s | Fell; rejected at this speed | Negative near failure | Present near failure |
| Short gait and lateral margin, vx=1.00 m/s | 36 s | Walked and stopped upright | 14.28 cm | None sampled |
| Short gait and lateral margin, vy=+0.50 m/s | 36 s | Walked and stopped upright; heading drift remains | 11.52 cm | None sampled |
| Short gait and lateral margin, vy=-0.50 m/s | 36 s | Walked and stopped upright | 10.49 cm | None sampled |
| Short gait and lateral margin, yaw=+0.7854 rad/s | 28 s | Turned and stopped upright | 10.94 cm | None sampled |

The saved vx=1.0 trial travelled 23.34 m against 23.0 m requested, with minimum base z0.901 m and maximum roll/pitch 1.19/4.50 degrees.
The saved lateral trials travelled +10.34/-10.95 m in the initial lateral direction against +/-11.5 m requested, but the positive trial accumulated 0.608 rad heading error and is correctly labeled `NO_TRACKING` by the stricter heading-sensitive test verdict.
That verdict is not a fall: both lateral runs completed the full command and final stance interval.
Before the signed margin, the faster-gait lateral trial also stayed upright but still had crossover steps; this is why the new row was added rather than claiming the sphere distances already prevented crossing.
Yaw-rate Q/Qf weight 100 instead of 30 did not improve lateral heading behavior and was rejected; no heading-accuracy change is retained.

An abrupt publisher step to vy=+0.5 with the original 0.7 gait delay still fell after 6.02 s, despite minimum signed foot width 13.71 cm and no sampled inter-leg overlap.
The failure render shows a deep knee collapse, not a crossover trip: the base drops to 0.579 m while maximum roll remains 0.101 rad, both ankle-roll actuators reach their 19 Nm bounds, and the left knee reaches its 360 Nm bound as knee flexion grows above two radians.
The lateral speed curve overshoots toward 0.8 m/s before collapse; solver logs also show large nonlinear equality residuals rather than a solver exception.
These observations are consistent with a poorly supported transition and actuator saturation, but do not isolate one unique root cause of every lateral failure.

Reducing lateral command slew to 0.15 m/s2 made both directions complete step-command trials, but that change was rejected because the user requested retaining the original 0.5 m/s2 rate.
The saved `velocityCommandRateLimits` remains `[0.5,0.5,0.5,2.0]`.
Instead, `switchDelayHorizonFraction=0.3` completed the same positive and negative lateral step commands without changing Q/Qf/R, PD or command slew.
The positive trial completed 30 s with minimum foot width 11.69 cm, minimum base z0.893 m, no sampled overlap and no sampled knee torque within 98 percent of its actuator bound.
The negative trial completed 30 s, travelled -9.45 m against -10.0 m requested and had minimum base z0.951 m.
Both stopped upright; the positive test still returns `NO_TRACKING` for a 0.513 rad heading error, which is separate from the balance result.
The successful earlier-switch trial still reaches ankle-roll saturation occasionally, so this is improved transition behavior rather than proof of generous actuator margin.
The contact schedule is still supplied externally to the OCP; the solver optimizes motion under that schedule rather than freely choosing touchdown/liftoff times.

Final no-override checks used the saved 0.3 switch delay and original command slew limits, with no publisher ramp:

| Step command | Hold / total duration | Balance and stopping result | Additional measurements |
| --- | --- | --- | --- |
| vx=+1.0 m/s | 30 / 40 s | Upright through command and stop | 30.52 m versus 30.0 m requested; minimum foot width 14.84 cm; no sampled inter-leg overlap |
| vy=+0.5 m/s, first comparison | 20 / 30 s | Upright through command and stop | Minimum foot width 11.69 cm; heading-sensitive verdict `NO_TRACKING` |
| vy=+0.5 m/s, repeat | 20 / 30 s | Upright through command and stop | Minimum foot width 11.85 cm; no sampled inter-leg overlap; heading-sensitive verdict `NO_TRACKING` |
| vy=-0.5 m/s | 20 / 30 s | Upright through command and stop | Minimum foot width 11.59 cm; no sampled inter-leg overlap; full test verdict `SUCCESS` |
| yaw=+0.7854 rad/s | 12 / 22 s | Upright through command and stop | Minimum foot width 10.38 cm; no sampled inter-leg overlap; full test verdict `SUCCESS` |

The positive lateral repeat accumulated 0.726 rad heading error, but did not cross its feet or fall; yaw accuracy was explicitly deprioritized for this tuning pass.
The zero publisher ramp does not bypass the original internal slew limits; it tests abrupt upper-level command changes with the configured controller response intact.
The final yaw step turned 8.94 rad against 9.42 rad requested, with translation drift below 6.8 cm.

Raw artifacts are in Docker `/tmp/h2_cross_*.{log,yaml,csv,json}` and locally under ignored `tuning_results/h2_crossing/`.
`tests/wb_id_walk_test.py --record-joints` adds measured-pose replay JSON and optional estimator orientation/twist snapshots; joint and base messages are latest-sample paired rather than hardware synchronized.
`tests/tuning/analyze_leg_clearance.py PREFIX --render` generates clearance CSV/JSON, joint-effort summaries, foot/velocity/support plots and measured-pose screenshots at the narrowest stance and near the end of the run.
Current analyses use all available odometry samples, normally 100 Hz; the two explicitly marked intermediate comparisons used an earlier 20 Hz analysis stride.
Replay overlaps are geometric intersections in the local MuJoCo model, not logged collision forces, and absence of sampled overlap does not prove continuous-time clearance.
The two-job Jazzy Release rebuild passed in 3 min 12 s, including compilation of the schema and collision change.
The five new reference-math/wiring tests are included in 37 passing focused WB-ID checks, and four native ROS launch tests passed separately; the mathematical reference tests do not themselves execute the compiled AD function.
These trials do not establish push recovery, mixed-axis full-speed robustness, full height-range feasibility, rough-terrain capability, real-hardware performance or absence of every possible self-collision.


## Model and Integration

[MODEL_SOURCE.md](../description/urdf/robots/h2/MODEL_SOURCE.md) pins the official Unitree URDF, MJCF, meshes and licenses and records the local coordinate/inertia corrections.
The robot is H2, not H1-2 or H2 Plus.

| Item | H2 integration |
| --- | --- |
| Physical joints | 31 actuated joints in MuJoCo and ros2_control |
| MPC joints | 25: 12 leg, three waist, two head and eight shoulder/elbow joints |
| Reduced-model joints | Six wrist joints fixed at zero; the simulator keeps them servo-controlled |
| Model mass | 75.58746 kg in both URDF and MJCF after correcting the omitted MJCF pelvis inertia |
| Joint order | Pinocchio traversal order, including head before arms and ankle roll before pitch |
| Foot frames | Reframed ankle-pitch joints, contact translation `[0.035, 0, -0.058]`, rectangle `[0.105, -0.105, 0.03, -0.03]` |
| Nominal posture | Level soles at y=+/-0.14 m; support midpoint aligned with the model's CoM in x |
| Pelvis height | Spawn 0.882 m with 2 mm clearance; standing reference 1.02 m |
| IMU | `imu_in_pelvis` at the official URDF offset, matched in MJCF |
| Feedback | Default `humanoid_linear_kf`, as in current G1 WB-ID |
| Generated libraries | Separate `auto_generated/h2/wb_id_mpc_h2/` cache |

The official open-chain H2 model is used, not the alternative closed-loop actuator description.
This remains an ideal joint-torque simulation, not a validated model of the real robot's transmission coupling, actuator bandwidth or communication delay.


## Existing MPC and Head Cost

The existing WB-ID formulation remains:

$$
x = \begin{bmatrix}q\\v\end{bmatrix},\qquad
q = \begin{bmatrix}p_{WB}^W\\\eta_{ZYX}\\q_j\end{bmatrix},\qquad
u = \begin{bmatrix}W_L\\W_R\\a\\\tau_j\end{bmatrix}.
$$

With 25 active joints, the state dimension is 62 and the input dimension is 68.
See the [WB-ID formulation](humanoid_whole_body_inverse_dynamics_mpc.md) for the inverse-dynamics equalities, contact constraints and objective definitions.
Every inherited joint-dependent array is mapped by joint name rather than copied by index.

Both head joints remain optimization variables.
`ocs2.costs.taskSpaceCosts.head` uses `head_yaw_link` and the existing generic frame cost with weights `[0,0,0,100,100,100,0,0,0,5,5,2]`.
Only orientation and angular velocity are penalized; head position is not fixed in world space.
The reference is generated by the existing nominal-joint FK along the base target trajectory, so the head may follow commanded heading while resisting unwanted neck motion.
This is a soft preference, not a hard neck lock or a separate world-fixed gaze controller.
There is no generic torso Cartesian cost in H2; the inherited gravity-relative `torsoTiltCost` remains active.


## Retained Tuning

All non-robot-dependent settings initially followed G1.
The following measured changes are retained in [H2 controller parameters](../config/h2/ros2_controllers.yaml).

| Parameter or integration choice | G1/initial value | Retained H2 value | Evidence and purpose |
| --- | --- | --- | --- |
| Joint PD stiffness | 200 | 600 | The inherited gains failed during H2 stand-up even with a balanced posture |
| Joint PD damping | 20, knees 10 | 40, knees 20 | Retains the existing lower-knee-damping pattern while increasing H2 servo tracking |
| Standing support midpoint | Initial H2 trial x=0.04 m | x=0.01771 m; crouch x=0.04961 m | Fixed x=0.04 caused a settling failure; balance the posture used by existing gravity compensation |
| Q/Qf base velocity | `[30,15,5,30,30,30]` | `[60,300,5,30,30,30]` | Lateral tracking was too weak; increasing vy weight improved displacement without changing the target structure |
| R contact yaw moment, each foot | 5 | 100 | The 0.20 m/s forward trial fell with 5, while the 100 trial completed walking and stopping |
| Stance linear exit threshold | 0.10 m/s | 0.03 m/s | A 0.08 m/s lateral command otherwise stayed in stance |
| Slow-walk linear stop threshold | 0.05 m/s | 0.02 m/s | Provides hysteresis below the new start threshold |
| Command normalization maxima | 1.0 m/s, 0.5 m/s, 1.5708 rad/s | 0.15 m/s, 0.08 m/s, 0.15 rad/s | Conservative initial GUI/topic range; not a full-speed H2 capability claim |

The contact yaw-moment weight is still multiplied by `rScaling=0.001` and is a soft regularizer, not a torsional-friction inequality.
Raising the Q/Qf yaw-rate weight from 30 to 300 with the stronger moment penalty caused another fall after 9.42 s; that change was rejected.
The retained Q/Qf yaw-rate weight is 30, and there is still heading drift under nominally straight commands.
Posture balancing and higher PD gains were both necessary in the tested bring-up; increasing gains alone did not fix the initial settling failure.


## Flat-Ground Results

Trials used the Parallels VM at `10.211.55.17`, Docker `ros_jazzy`, Release build, H2's flat scene (now `robots/h2/scene_flat.xml`), estimator feedback, `base_twist`, no GUI and no RViz.
Ground-truth pose was recorded only for validation, not used as floating-base feedback in these trials.
The requested rates were 100 Hz MPC and 1000 Hz MRT; measured solver advance times were typically around 12 ms, so the VM did not consistently achieve 100 Hz MPC.
Each trial ramped its command over 3 s, walked, ramped back to zero and included a final stance interval.

| Trial | Command | Duration after MPC takeover | Along-axis displacement | Minimum z | Maximum roll / pitch | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| Before moment-weight change | vx=+0.20 m/s | 12.66 s | Incomplete | 0.481 m | 87.0 / 38.1 deg | Fell |
| Stronger moment penalty | vx=+0.20 m/s | 36 s | +4.807 m, requested +4.600 m | 0.959 m | 3.08 / 1.54 deg | Completed and stopped |
| Saved configuration | vx=+0.15 m/s | 41 s | +4.428 m, requested +4.200 m | 0.955 m | 3.13 / 1.25 deg | Completed and stopped |
| Saved configuration | vx=-0.10 m/s | 36 s | -2.069 m, requested -2.300 m | 0.966 m | 2.86 / 1.24 deg | Completed and stopped |
| Stronger moment penalty | vy=+0.08 m/s | 36 s | +1.435 m, requested +1.840 m | 0.968 m | 1.93 / 0.76 deg | Completed and stopped |
| Saved configuration | vy=-0.08 m/s | 36 s | -1.375 m, requested -1.840 m | 0.965 m | 1.98 / 0.76 deg | Completed and stopped |
| Saved configuration | yaw=+0.15 rad/s | 31 s | +2.255 rad, requested +2.700 rad | 0.974 m | 2.78 / 0.69 deg | Completed and stopped |
| Saved configuration after final rebuild | yaw=-0.15 rad/s | 31 s | -2.153 rad, requested -2.700 rad | 0.969 m | 1.66 / 0.92 deg | Completed and stopped |

The 0.20 m/s forward trial used the retained moment weight before the conservative command maximum was saved; testing that speed again requires explicitly increasing `maxDisplacementVelocityX`.
The positive lateral trial used an override numerically equal to the now-saved contact weights; the saved forward, reverse, negative-lateral and yaw trials used no controller overrides.
The +0.15 m/s run accumulated 0.158 rad maximum heading error and 0.282 m cross-axis displacement, while the +0.20 m/s run accumulated 0.359 rad and 0.717 m respectively.
Lateral displacement remains roughly 75-78% of the requested integral, so this is a working low-speed checkpoint, not precise trajectory tracking or a formal robustness guarantee.
Combined vx/vy/yaw commands, full-speed gaits, height sweeps, stairs, ramps, rough terrain, InEKF feedback and real hardware are not validated here.
The optional H2 variants in shared [`config/terrains/`](../config/terrains/README.md) only provide geometry-dependent starting values and are not used by these `base_twist` trials.

Local raw logs, configuration snapshots, CSVs, JSON verdicts, pose plots and a MuJoCo render replayed from measured joint/base state are saved under ignored `tuning_results/h2/`.
The corresponding container files are `/tmp/h2_*.{log,yaml,csv,json}`.
The plot `walking_comparison.png` shows the retained runs keeping bounded roll/pitch and returning to stance, versus the earlier abrupt fall.
The final two-job Jazzy Release build passed in 3.92 s, and the negative-yaw trial subsequently passed with the saved settings and no overrides.
Seven H2 model/xacro checks, 11 scene-layout checks, 23 WB-ID contract checks, nine torso-cost checks and the standalone parameter contract passed.


## Reproduce

For manual flat-ground trials, keep only the robot and background includes in H2's `scene.xml`, and comment out the terrain includes in `description/urdf/env.urdf.xacro`.
The old `scene_flat.xml` filenames in historical results identify the scene used at the time; that separate file is no longer shipped.

```bash
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=h2 \
  mujocoModelFile:=scene.xml \
  mpcFreq:=100 mrtFreq:=1000 \
  rviz:=true mujoco_headless:=false baseCommandGui:=true
```

For an automated isolated forward test from the sourced workspace:

```bash
python3 src/legged_robot_mpc_controller/tests/wb_id_walk_test.py \
  --robot-name h2 \
  --config src/legged_robot_mpc_controller/config/h2/ros2_controllers.yaml \
  --source state_estimator --estimator humanoid_linear_kf \
  --minimum-height 0.65 --startup-timeout 1800 \
  --axis vx --speed 0.15 --hold 25 --ramp 3 --settle 4 --rest 6 \
  --prefix /tmp/h2_saved_vx015
```

The GUI/topic velocities are normalized by the H2 maxima, as in G1; a topic value of 1.0 now requests 0.15 m/s forward, not 1.0 m/s.
Only run one simulator/controller instance at a time because the robot-independent topic and node names are intentionally unchanged.


## Validation Caveat: Published Simulator Twist

Use pose displacement and unwrapped orientation differences for H2's external tracking metrics rather than trusting the existing `/mujoco/ground_truth/odom` twist axes.
The simulator publisher calls `mj_objectVelocity` with `mjOBJ_BODY` and local coordinates, which uses the body's inertial frame, while the message pose and child frame describe the pelvis link.
H2's nonidentity pelvis inertial rotation makes those axes differ substantially.
A local MuJoCo check with identity pelvis rotation and world translational velocity `[1,2,3]` returned approximately `[1.0203,2.9897,-2.0051]` through that local-body call.
This publisher issue has not been changed in the external simulator repository; the walking harness now also reports position-derived world-frame mean speeds.
Do not interpret the near-zero published vy or yaw-rate mean in these logs as a lack of actual lateral or turning motion.
