# WB-ID Tangent-Space Orientation

This branch tests `ocs2.model.orientationRepresentation: tangent_space` on H2.
`euler_zyx` retains the previous implementation for comparison; centroidal MPC is unchanged.
Quaternion decision-variable optimization is deferred.
G1 and H2 currently select `euler_zyx`; tangent mode remains available for comparison.
The user-selected H2 YAML has a 1.0 s horizon, 0.10 m walking swing height and gait-switch horizon fraction 0.3.
The original unchanged-parameter comparison was not a terrain improvement; the October 30 clearance profile completed six terrain and four flat-motion trials, but an additional long traversal fell after the ramp end.
This is an experimental checkpoint, not a guarantee of feasible plans or reliable locomotion on arbitrary terrain.


## Response-Priority Tuning

The tested 2026-10-31 H2 candidate used tangent orientation, one SQP iteration, `dt: 0.02` and the original cost/penalty weights.
The user subsequently restored the Euler baseline values; the following measurements describe the tested candidate, not the current YAML.
Three candidate parameter changes prioritized response without coarsening contact integration:

| Setting | Current Baseline | Tested Candidate | Effect |
| --- | --- | --- | --- |
| `ocs2.mpc.timeHorizon` | 1.0 s | 0.8 s | Approximately 40 rather than 50 nominal intervals, plus contact-event knots |
| `ocs2.gait.transitions.switchDelayHorizonFraction` | 0.3 | 0.375 | Retains the same 0.3 s physical gait-switch delay |
| `ocs2.gait.templates.walk.swingHeight` | 0.10 m on disk | 0.20 m | Gives a 0.15 m scaled walking apex above the first roughly 8.6 cm tile |

The prior handoff described 0.20 m, but the actual source at the start of this round still contained 0.10 m.
Fallback swing height, trot/run heights, torque window, gains, estimator and G1 are unchanged.
No new C++/OCS2 implementation was added in this tuning round; the hard-bound option remains disabled, and the fixed-initial-state experiment was subsequently removed at the user's request.

All tests requested MPC 80 Hz and MRT 1000 Hz, with InEKF feedback, no terrain-map input, headless simulation and `baseCommandGui:=false`.
Baseline instrumented flat walking averaged about 69 Hz, with approximately 6 ms local approximation and 8 ms QP solve time.
Disabling detailed diagnostics did not consistently improve the measured rate.
The selected terrain trials averaged 76.03 Hz, with about 5.31 ms local approximation, 6.32 ms QP time and 12.10 ms total advance.
These headless measurements do not guarantee the same rate while rendering MuJoCo and RViz.

| Selected-profile test | Completed | Achieved Hz | Pose-derived motion |
| --- | --- | --- | --- |
| Isolated rough field, vx command 0.2 | 3/3 | 75.53-76.51 | Mean vx 0.320-0.340 m/s |
| Bounded default-scene/ramp entry, vx command 0.2 | 3/3 | 75.69-76.66 | Mean vx 0.247-0.261 m/s |
| Saved-YAML flat walk, vx command 0.2 | 1/1 | 76.48 | Mean vx 0.256 m/s |
| Saved-YAML flat lateral, vy command 0.25 | 1/1 | 76.10 | Mean vy 0.228 m/s; forward drift 0.083 m/s |
| Saved-YAML flat yaw, command 0.6 rad/s | 1/1 | 78.17 | Mean yaw rate 0.559 rad/s |
| Saved-YAML flat run, vx command 1.0 | 1/1 | 76.88 | Mean vx 1.020 m/s |

All ten selected-profile trials had zero logged solver exceptions and torque-window overruns.
Tests are stopped for manual use; this parameter-only round did not require a rebuild.
These small sequential batches are not proof of general terrain robustness, and did not validate recovery from the ramp's roughly 35 cm end drop.

### Penalty Comparisons

Existing penalties were tested first, changing torque and friction groups separately rather than enabling a new solver path.
For scalar constraint margin $h\leq\delta$, the relaxed barrier has:

$$
\frac{\partial P}{\partial h}=\frac{\mu(h-2\delta)}{\delta^2},\qquad
\frac{\partial^2P}{\partial h^2}=\frac{\mu}{\delta^2}.
$$

Torque margins use Nm and friction margins use N, so equal numerical penalty settings do not give comparable physical scaling.

| Temporary profile | Result | Saved? |
| --- | --- | --- |
| Torque `mu: 0.75`, `delta: 1` | Flat passed; rough stayed upright but oversped to about 0.50 m/s for a 0.2 command | No |
| Torque `mu: 1`, `delta: 2` | Flat/rough passed one each; no solve-rate improvement | No |
| Friction `mu: 0.4`, `delta: 5` | Flat/rough passed one each; no material solve-rate improvement | No |
| 25 ms grid, nearly matched torque-window duration | Faster, but rough 1/3 and ramp entry 1/3 completed | No |
| 25 ms grid plus friction `mu: 0.4` | Rough one completion and one upright tracking failure; ramp entry two falls | No |
| 30 ms grid | Flat passed, rough fell | No |
| 0.8 s horizon with original low clearance | Flat regressions passed; rough and ramp entry only 2/3 each | No, clearance also needed |

One interrupted capture ran out of Docker disk space and is excluded from completed results.
Older owned captures were archived locally before removal from Docker; raw copies, replay images and plots are under ignored `tuning_results/tangent_penalty_1031`.
Polynomial torque penalties were not added or compared.
The selected profile retains torque `mu: 0.5`, `delta: 1` and friction `mu: 0.2`, `delta: 5`.

Replay and numerical checks support improved entry clearance, not exact actuation feasibility: a selected rough run had zero leg-crossing/overlap samples, but its near-term torque ratio still peaked at 1.90.
A selected ramp-entry run reached ratio 2.13 and a small negative planned world-z force of about -21 N.
Rough-ground overspeed and estimator height/tilt errors remain visible; they were not hidden by changing tracking weights.
See [the latest handoff](../Codex.md#saved-response-priority-candidate-2026-10-31-asiataipei) for reproduction artifacts and the full trial history.


## Previous Manual-Test Checkpoint

The 2026-10-30 controller build passed; automated robot tests are stopped at the user's request.
That handoff described one SQP iteration, a 1 s horizon and the original cost/penalty gains, with H2 tangent orientation and 0.20 m walking clearance; the later on-disk check found 0.10 m instead.
G1 is unchanged.
Later short-ramp repeats completed Euler 3/3 versus tangent 2/3 plus a severe-collapse tracking failure, in addition to the failed long traversal described above.
The six successful terrain trials must not be interpreted as reliable terrain recovery.
Shorter-horizon/two-iteration and stronger forward-tracking experiments also had failures and were not saved.

Next work starts with isolated tuning of existing torque/friction penalty parameters and checking their gradient scales, not new implementations.
The previous combined stronger torque/friction RBF experiment failed early in stance; a systematic isolated sweep and polynomial torque-penalty comparison have not been performed.

At that checkpoint, both the initial-state pinning experiment and `ocs2.constraints.jointTorqueLimits.hardBounds` were disabled.
Initial-state pinning has since been removed; only the hard-bound option remains available, still disabled.
It retains the torque RBF and additionally forwards torque-window inequalities through equality projection into HPIPM, including inequality-aware filtering.
This experimental hard path rejects unsupported unprojected mixed constraints and state-only equalities; it does not guarantee nonlinear feasibility after a partial SQP update.
Three native inequality/filter tests, one initial-boundary regression and eight existing SQP regressions passed, but hard bounds have not been robot-tested.
The remaining uncommitted changes add diagnostics, tests, the disabled hard-bound option and the H2 selection/clearance settings; the committed orientation math itself is unchanged.
See [the stopped handoff](../Codex.md#stopped-checkpoint-2026-10-30-1552-utc) for repository revisions and the complete limitations.


## Representation

The orientation decision is a three-component local increment $\phi$, anchored to the measured orientation $\hat R$ at the beginning of each MPC horizon:

$$
R=\hat R\operatorname{Exp}(\phi),\qquad
q_P=\begin{bmatrix}p^W\\\operatorname{quat}(R)\\q_j\end{bmatrix},\qquad
v_P=\begin{bmatrix}v^W\\\omega^B\\\dot q_j\end{bmatrix}.
$$

Pinocchio uses a Translation + Spherical composite joint, with quaternion configuration storage and a three-dimensional angular tangent space.
The quaternion is reconstructed from the anchor and increment, not independently optimized with a unit-norm constraint.
The joint and translational coordinates remain ordinary Euclidean coordinates; subtracting their measured values would only change their origins.

OCS2's state storage includes the four anchor coefficients after the usual two $n_v$ blocks:

$$
x=\begin{bmatrix}p^W\\\phi\\q_j\\v^W\\\omega^B\\\dot q_j\\\hat q_R\end{bmatrix},\qquad
\dot{\hat q}_R=0.
$$

These four coefficients are fixed by the initial condition and zero dynamics, have no tracking cost, and are identical throughout one horizon.
They carry the chart data through OCS2's cloned costs, constraints, policy buffer and visualization without mutable cross-thread chart state.
Consequently, the stored dimension is $2n_v+4$, although the free orientation increment still has only three components.


## Kinematics and Physics

Body angular velocity is not an Euler-angle derivative and is not generally the derivative of a finite rotation vector.
The chart kinematics used here are:

$$
\dot p^W=v^W,\qquad
\dot\phi=J_r(\phi)^{-1}\omega^B,\qquad
\dot q_j=\dot q_j,\qquad
\dot v_P=a_P.
$$

Here $J_r$ is the right Jacobian of the SO(3) exponential.
The configured SQP integration method is unchanged, so RK4 integrates this nonlinear flow when selected.
RNEA and the foot acceleration equalities use the same reconstructed Pinocchio configuration, velocity and acceleration conventions.
Contact wrenches remain world-aligned and joint torque bounds are unchanged.

The reference [wb-mpc-locoman implementation](https://github.com/lukasmolnar/wb-mpc-locoman/blob/main/optimization/ocp_whole_body_rnea.py) instead advances the fixed-anchor increment with $\delta q_{k+1}=\delta q_k+v_k\Delta t_k$.
Our experiment follows its minimal measured-anchor representation but uses the exact angular chart differential rather than that approximate fixed-chart update.
It is not an exact reproduction of the paper's complete discretization, solver or objective.


## Frames and Performance

Neither representation removes the world-referenced attitude from the controller.
Euler stores coordinates of $R_{WB}$; tangent reconstructs that same rotation as $R_{WB}=\hat R_{WB}\operatorname{Exp}(\phi)$.
The local increment is right-multiplied, and its axes are those of the measured anchor, not the world axes.

| Property | `euler_zyx` | `tangent_space` |
| --- | --- | --- |
| Orientation storage | World-referenced yaw, pitch and roll | Three local increments around a measured world-referenced attitude |
| Angular velocity state | ZYX Euler-angle derivatives; neither world nor body angular velocity | Physical XYZ body angular velocity from the estimator |
| Gyro conversion | Configuration-dependent Euler-rate map | Body angular velocity enters directly |
| Large absolute pitch | Rate map becomes singular near +/-90 degrees | No Euler singularity in the chart flow or physical RNEA mapping |
| Coordinate branches | Heading unwrap handles repeated turns; pitch singularity remains | Rebase each horizon; principal log has a relative-pi branch |
| Angular acceleration input | Time derivative of Euler rates | Body angular acceleration |
| Current OCS2 storage | $2n_v$ state entries | $2n_v+4$, including fixed anchor data |

If physical models, references and cost metrics are transformed consistently, changing coordinates alone should not change the robot's physical optimum.
Identical numeric weights are not an identical physical metric here: Euler-rate penalties are configuration dependent, whereas the tangent cost uses physical angular velocity and a rotation-log attitude residual.
Near upright, they behave similarly; at large tilt their penalties can differ significantly.
Tangent coordinates also avoid amplification of gyro noise by the Euler-rate conversion near vertical pitch, but do not reduce the estimator's underlying physical uncertainty or eliminate odometry drift.

Condition-number checks of the Euler gyro-to-rate map at pitch 0/60/80/89/89.9 degrees give approximately 1/3.73/11.43/114.59/1145.92.
The tangent chart's rate map is identity at each new anchor ($\phi=0$); this is a coordinate-math result, not a claim of successful locomotion at those attitudes.
Heading-based walking tasks and lateral collision margins still have a geometric ambiguity when the pelvis forward axis is vertical.

Better conditioning does not guarantee faster solves.
The present OCS2 implementation carries four fixed anchor entries through multiple shooting, increasing QP dimensions, and the chart flow is nonlinear.
At requested 80 Hz, three forward-walking trials averaged 77.06 Hz for Euler and 73.34 Hz for tangent; most of the measured extra time was in the QP solve, not exp/log evaluation.
Use the complete benchmark results and repeats in the handoff before drawing a general speed or robustness conclusion.

Selecting `euler_zyx` restores the retained Euler model, observation conversion, dynamics and base objective, with the same saved parameter values.
It does not undo unrelated tuning changes or shared dependency fixes, and simulated trajectories are not bitwise reproducible under changing solver timing.
The original comparison below used the previous H2 0.10 m walking setting.
The current H2 YAML already selects `euler_zyx`, walk height 0.10 m, horizon 1.0 s and gait-switch fraction 0.3; the tested tangent candidate used 0.20 m, 0.8 s and 0.375 respectively.
Body angular velocity is a physical rate, but is not generally the derivative of the three tangent coordinates: $\dot\phi=J_r(\phi)^{-1}\omega^B$.
Only near zero increment, or for suitable collinear motion, can those vectors be treated as the same.


## Receding Horizon and References

Each new observation supplies its measured anchor.
Before reusing a previous solution, every old planned orientation is reconstructed and expressed in the new chart:

$$
\phi_k^{\mathrm{new}}=\operatorname{Log}\!\left((\hat R^{\mathrm{new}})^\mathsf{T}\hat R^{\mathrm{old}}\operatorname{Exp}(\phi_k^{\mathrm{old}})\right).
$$

Position, physical velocity, joint states and physical inputs do not change under this rebasing.
OCS2 SQP's optional warm-start transform runs before time interpolation and leaves existing users unchanged when unset.
The control thread also re-expresses its observation in the active policy's chart before evaluating that policy.
All knots of one generated command trajectory share an anchor, avoiding interpolation of unrelated quaternion signs.

Base pose commands still accept the existing pose interface, and GUI controls are unchanged.
Euler angles are available for heading selection and human-readable diagnostics; RNEA does not convert body angular velocity into Euler rates.
Heading-based planar locomotion references remain inherently ambiguous when the pelvis forward axis is vertical, independently of the orientation state representation.


## Objective

The dedicated base objective uses a three-dimensional rotation-log residual instead of subtracting yaw/pitch/roll independently.
Its rotation-vector axes are reordered to match the existing weight order `[yaw, pitch, roll]` near upright.
In relative `base_twist` mode, each attitude's heading is removed before comparing tilt, and planar position remains untracked.
Linear and angular motion are compared in the corresponding heading frames, preserving a common planar-origin/heading gauge change.
Full orientation tracking is retained for explicit `base_pose` targets.
That mode compares linear and angular velocities in world axes without extracting a pelvis heading, so the base cost remains defined at vertical pitch.
Running and terminal terms use the same residual and Gauss-Newton derivatives.
Foot, torso, joint and collision terms still operate on the reconstructed physical configuration.


## Automatic Differentiation

The implementation uses Pinocchio's existing CppADCodeGen backend and quaternion `exp3`/`log3`.
The right-Jacobian inverse uses the standard inverse-dexp formula and near-zero series documented by [GTSAM](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SO3.cpp), in a small scalar-templated OCS2 helper.
Its inactive norm is guarded before sqrt/division, and CppADCodeGen differentiates the complete flow.
No finite-difference derivative is used in the MPC.
The installed Pinocchio matrix `Jlog3(Exp(phi))` path produced NaN generated derivatives at zero.
The matrix-only `Jexp3(phi).inverse()` probe passed, but its composition with angular velocity failed generated flow and RK4 Jacobian checks at zero increment; the codegen-safe helper fixes that separately reproduced case.
Pinocchio's backend header must be visible before instantiating its manifold algorithms.
See upstream [CppAD spatial tests](https://github.com/stack-of-tasks/pinocchio/blob/devel/unittest/cppad/spatial.cpp), [joint configuration tests](https://github.com/stack-of-tasks/pinocchio/blob/devel/unittest/cppad/joint-configurations.cpp) and [code-generation backend](https://github.com/stack-of-tasks/pinocchio/blob/devel/include/pinocchio/codegen/cppadcg.hpp).

The bundled CppAD `atan2` also required an axis-derivative fix, based on the [current upstream implementation](https://github.com/coin-or/CppAD/blob/master/include/cppad/core/atan2.hpp).
Inactive divisions are guarded before code generation, preserving the selected expression while avoiding NaN derivatives on the coordinate axes.
The generated base cost passes identity/nonzero gradient, quaternion-sign and common planar-gauge checks.
Generated libraries use a separate `wb_id_tangent_v3_<robot>` folder so an Euler library cannot be mistaken for a tangent model.
The chart still has a finite domain; large absolute rotations are allowed through repeated rebasing, but one horizon must not approach a relative $2\pi$ rotation.
The principal logarithm has an unavoidable branch at a relative $\pi$ rotation.
This representation does not by itself improve estimator accuracy, actuator feasibility or terrain knowledge.


## Original Unchanged-Parameter Comparison

Native generated-math and model tests pass, including zero increments, large absolute tilt, physical integration, mapping Jacobians and warm-start rebasing.
Euler and tangent RNEA results also agree after transforming the same physical velocities, accelerations and generalized forces consistently.
Zero-increment flow and RK4 Jacobians are checked from zero and tiny angular velocities through normal rotation.
The OCS2 dependency has three standalone GTests for inverse-right-Jacobian values, complete generated flow derivatives and generated `atan2` derivatives on axes/quadrants.

The following H2 tests used requested MPC 80 Hz, MRT 1000 Hz, InEKF feedback, the same saved gains and command profiles, headless MuJoCo, no RViz and no command GUI.
Repeated trials reverse representation order on alternate repeats.
The table reports completed commanded-motion trials, not merely survival.

| Scenario | Euler result | Tangent result | Mean achieved Hz, Euler / tangent |
| --- | --- | --- | --- |
| Flat vx 0.5 m/s, 41 s | 3/3 completed | 3/3 completed | 77.06 / 73.34 |
| Flat vy 0.25 m/s, 41 s | 1/1 completed | 1/1 completed | 75.91 / 70.26 |
| Flat yaw 0.6 rad/s, 46 s | 1/1 completed | 1/1 completed | 75.37 / 72.36 |
| Flat run 1.0 m/s, 46 s | 1/1 completed | 1/1 completed | 77.26 / 72.31 |
| Isolated uneven field, vx 0.2 m/s, 36 s | 3/3 completed | 1/3 completed; 2 falls | 77.91 / 72.75 |
| Default uneven-field/ramp scene, vx 0.2 m/s, 36 s | 1 completed, 1 tracking failure, 1 fall | 3 falls | 77.82 / 72.33 |
| Flat vx 0.3 m/s with 50 N lateral pelvis push for 0.5 s | 1/1 completed | 1/1 completed | 76.33 / 70.93 |

No solver exception or torque-window overrun was reported in these 26 trials.
Forward-walking reported mean solve time averaged 12.18 ms for Euler and 13.62 ms for tangent.
Achieved rates and timing are VM-dependent wall-clock measurements; the requested 80 Hz is not a guarantee.
The flat yaw pair made about three complete turns without an orientation-wrap failure.
The push was a mild single pulse, not a measurement of the maximum recovery envelope.

The repeated uneven-field result demonstrates that tangent can complete this terrain, but not that it is as reliable as Euler with the unchanged tuning.
Both representations failed at least one default-scene trial; no unmodeled-terrain success guarantee is established.
Plots show the failing tangent trials developing height loss, increasing tilt and excess forward speed near terrain entry, without a solver exception.
The replay of the first failure found no opposite-leg mesh overlap before the fall, so a collision cause is not established.
Trying a temporary 0.15 m walking swing height did not reliably help: some trials stayed upright but made little progress, and an Euler default-scene trial fell.
That override was rejected; the saved gait heights and tuning values were not changed.
A second temporary tangent-only profile reduced foot height/tilt equality gains from 50/100 to 25/50.
Both uneven-field trials and both default-scene trials fell (8.75/13.11 s and 22.40/9.22 s respectively), so that profile was rejected too.

The simulator's existing GT twist publisher uses MuJoCo `mjOBJ_BODY` local axes, which differ from the H2 pelvis pose axes because of its rotated inertial frame.
Therefore speed and yaw-rate comparisons use derivatives of the recorded GT pose, not raw published twist fields.
This measurement issue does not change these InEKF-feedback control trials; the simulator repository was not modified.
Raw artifacts are in Docker `/tmp/tangent_orientation_benchmark_80hz` and the ignored local `tuning_results/tangent_orientation/benchmark_80hz` directory.
See [Codex.md](../Codex.md) for chronological findings and rejected experiments.
After the final Release rebuild, a fresh flat vx=0.5 m/s smoke pair completed 41 s in each mode, with achieved rates 78.24/75.64 Hz and reported mean solve times 11.74/13.21 ms (Euler/tangent).
That separate post-build check is not included in the 26-trial table above.


## Uneven-Terrain Numerical Investigation

After reports of violent falls, the investigation checked optimized values and derivatives rather than assuming a gain problem.
The new native `wb_id_tangent_constraint_test` checks generated RNEA values against native Pinocchio and every state/input derivative column of the RNEA and stance/swing foot equalities against independent numerical perturbations.
All six cases passed at zero, tiny and nonzero increments and absolute pitch up to 1.2 rad, with worst scaled derivative error about $1.4\times10^{-7}$.
Numerical perturbations are validation only; control still uses generated automatic derivatives.

In the sampled failing plans, all states and inputs remained finite, the anchor stayed unit/constant to about $2.3\times10^{-16}$, and chart magnitude stayed below 1.1 rad.
Those observations do not support a current SO(3) singularity or NaN as the cause of these particular falls.
They do not prove every operating condition is correct.
The plans did request near-term torque around nine times a URDF limit and negative world-z contact force around -2019 N.
Torque limits and friction cones are relaxed-barrier soft penalties, not hard bounds, and clipping feedforward torque makes the executed input differ from the optimizer's assumed input.
Tiny contact-event intervals also make pointwise residuals worth logging separately from the time-integrated SSE.
H2 currently enforces joint RNEA over a 0.19 s initial window (`torqueConstraintNodes: 10`), not the entire 1.0 s horizon.

One upstream SQP behavior also matters: with an interpolated warm start and a partial line-search step, the returned first state need not equal the new measurement exactly.
An experimental first-state pin reset the warm-start first state to the measurement before linearization.
Its native regression demonstrated exact first-state matching under backtracking, but terrain trials did not demonstrate a robustness benefit.
The option, controller parameter and dedicated test were removed at the user's request; the existing OCS2 measured initial-state constraint and its normal SQP correction remain unchanged.
This was an iterate-initialization experiment, not a required tangent-space feature or a missing initial-state constraint.

| Isolated change | Observation | Decision |
| --- | --- | --- |
| Two SQP iterations | Three default-scene falls; achieved about 32-34 Hz | Reject |
| Fixed initial state | Three default-scene falls; rough runs upright but often overspeed | Experiment removed |
| Remove stance height/tilt restoration | One default-scene completion, one rough-ground fall | Reject |
| Much stronger torque/friction barriers | Six early stance falls before walking | Reject |
| Walk swing setting 0.20 m | First six tangent terrain trials completed, including one deep crouch/recovery | Promising clearance change, not a feasibility guarantee |
| Full-horizon joint RNEA with 0.20 m walking swing | One completion and one tracking failure in each terrain scene | Retain existing torque window |

With the same clearance, Euler completed all three isolated rough trials and one of three default-scene trials, with two late falls.
These small sequential batches are not evidence that tangent is inherently more robust than Euler.
The recovering tangent rough run still reached a near-term torque/limit ratio of 20.27 later in the trial; surviving a run is not proof of physical feasibility.

The clearance-only profile kept all other saved gains, one SQP iteration, the 0.19 s torque window and the original initial-state handling.
The completed trials below used requested MPC 80/MRT 1000 Hz, InEKF feedback and the same enabled diagnostics in both representations:

| Scenario with 0.20 m walk setting | Euler | Tangent | Mean achieved Hz, Euler / tangent |
| --- | --- | --- | --- |
| Isolated uneven field, vx=0.2, 36 s | 3/3 completed | 3/3 completed | 73.57 / 67.29 |
| Default uneven-field/ramp scene, vx=0.2, 36 s | 1/3 completed; 2 late falls | 3/3 completed | 73.34 / 70.14 |
| Flat walk vx=0.2, 36 s | Not repeated in this batch | Completed | - / 69.36 |
| Flat vy=0.25, 41 s | Not repeated in this batch | Completed; measured vy=0.239 m/s | - / 69.46 |
| Flat yaw=0.6, 46 s | Not repeated in this batch | Completed; measured yaw rate=0.566 rad/s | - / 69.74 |
| Flat run vx=1.0, 46 s | Not repeated in this batch | Completed; measured vx=1.024 m/s | - / 69.89 |

The two late Euler default-scene falls occurred during stopping after traversing the ramp, not at the first terrain step.
The ramp ends at x=5.5 m with an approximately 0.35 m vertical drop; overshoot can carry the robot off this edge.
That is a different disturbance from the initial 8.6 cm clear-patch boundary, and neither scene was modified to improve the score.
Tangent's worst rough-field run recovered from GT base height 0.589 m; the other two rough minima were 0.954 and 0.991 m.
Default-scene tangent minima were 0.998, 0.998 and 0.952 m.
No solver exception or torque-window overrun was logged in these ten tangent and six Euler trials.
Tangent is still slower in these VM tests, not faster; enabled detailed diagnostics and different batches mean these rates should not be compared directly with uninstrumented measurements.
An additional no-override saved-YAML tangent trial fell at 30.41 s, x=6.956 m, after the ramp's end during ramp-out.
The gait log still showed walking, not an early transition to stance.
The six earlier completions therefore do not establish reliable long-traversal or drop recovery; this failure remains part of the checkpoint record.

The starting clear patch is not bounded by the terrain's adjacent-tile height-difference constraint.
At its forward edge, the right and left tile tops are about 8.60 and 5.37 cm above the starting floor.
H2's original 0.10 m walking setting is duration-scaled to a 7.5 cm swing reference apex; it can therefore hit the first right-side step before scheduled touchdown.
A 0.20 m setting gives a 15 cm reference apex without supplying a terrain map to the controller.
Replay is latest-sample pose reconstruction, not measured contact-force truth; do not infer exact impact timing from images alone.

With `--record-tracking`, the walking harness saves named solver columns to `<prefix>_solver.csv` and copies the SQP iteration ring log to `<prefix>_sqp.csv` after shutdown.
New solver columns include full first-node `x0/*` and `u0/*`, trajectory finiteness, chart/anchor checks, minimum positive mesh interval, near-horizon torque/limit ratio, acceleration extrema and minimum world-z force.
The normal-force diagnostic is world-z, not a projection onto an unknown terrain normal.
Same-time interpolated observation comparisons are approximate, particularly during fast joint motion; they are not an exact boundary-condition test.
These diagnostics run only when tracking diagnostics are enabled and add work to the solver thread, so timing comparisons must use the same logging settings.

For copied artifacts, using Python with NumPy, Matplotlib, PyYAML and MuJoCo:

```bash
python3 tests/tuning/analyze_tangent_solver.py /path/to/test_prefix --start 5 --end 12
```

This writes `_numerics.json` and `_numerics.png`; select a later window to inspect a recovery instead of only the first terrain contact.
Raw investigation directories and the complete chronological results are recorded in [Codex.md](../Codex.md).

In a sourced built workspace:

```bash
python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py
python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py --test wb_id_tangent_model_test
python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py --test wb_id_tangent_constraint_test
```

The first build of this branch must also rebuild its modified OCS2 dependencies, not only the controller package:
Use the matching OCS2 `jazzy` checkpoint containing the warm-start hook and generated-math fixes; do not mix pre-removal binaries with rebuilt libraries.

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install \
  --packages-up-to legged_robot_mpc_controller --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For paired H2 runs with the current saved profile, from the sourced workspace:

```bash
python3 src/legged_robot_mpc_controller/tests/tangent_orientation_benchmark.py \
  --robot-name h2 --mpc-rate 80 --cases vx vy yaw run rough default_scene push \
  --output /tmp/tangent_orientation_benchmark
```

The runner uses the existing headless harness, disables the command GUI and runs one simulator at a time.
The `rough` case isolates the current uneven field; `default_scene` uses the checked-in robot scene, including its ramp.
`ramp_entry` uses the same default scene but a 10 s command hold, separating ramp ascent/stopping from the long traversal's end drop; it is a different scenario and does not replace failed `default_scene` trials.
`push` reuses only the first existing 50 N, 0.5 s pelvis pulse, rather than the complete all-links sequence.
Each pair changes only `orientationRepresentation`; `--repeats` adds repeated trials and `--representations` can select just one implementation during debugging.
The explicit `walk` case uses vx=0.2 m/s on flat ground; the existing `vx` case uses 0.5 m/s, which selects H2's trot at the current transition settings.
For the three-repeat forward/terrain comparison, use `--cases vx rough default_scene --repeats 3`.
`--resume` skips existing completed trials, while `--summarize-only` refreshes pose-derived metrics without launching a simulator.
Keep the raw JSON/CSV/log/replay files alongside `comparison.json` and `comparison.csv`.
Plot copied results with `tests/tuning/plot_tangent_orientation.py <directory>` in an environment with NumPy and Matplotlib.
