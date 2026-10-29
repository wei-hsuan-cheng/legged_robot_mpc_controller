# WB-ID Tangent-Space Orientation

This branch tests `ocs2.model.orientationRepresentation: tangent_space` on H2.
`euler_zyx` retains the previous implementation for comparison; centroidal MPC is unchanged.
Quaternion decision-variable optimization is deferred.
Both robot YAML files retain `euler_zyx` as the default; select `tangent_space` explicitly for this experiment.
Flat-ground tests pass, but the repeated uneven-terrain results below do not justify replacing the Euler default.


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
Current terrain-clearance trials use temporary parameter overrides; the saved gait settings remain unchanged.
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


## Validation Checkpoint

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

In a sourced built workspace:

```bash
python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py
python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py --test wb_id_tangent_model_test
```

The first build of this branch must also rebuild its modified OCS2 dependencies, not only the controller package:
Use OCS2 `jazzy` commit `ffdde1d7e` or a descendant containing its warm-start hook and generated-math fixes.

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install \
  --packages-up-to legged_robot_mpc_controller --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For paired H2 runs with the current unmodified gains, from the sourced workspace:

```bash
python3 src/legged_robot_mpc_controller/tests/tangent_orientation_benchmark.py \
  --robot-name h2 --mpc-rate 80 --cases vx vy yaw run rough default_scene push \
  --output /tmp/tangent_orientation_benchmark
```

The runner uses the existing headless harness, disables the command GUI and runs one simulator at a time.
The `rough` case isolates the current uneven field; `default_scene` uses the checked-in robot scene, including its ramp.
`push` reuses only the first existing 50 N, 0.5 s pelvis pulse, rather than the complete all-links sequence.
Each pair changes only `orientationRepresentation`; `--repeats` adds repeated trials and `--representations` can select just one implementation during debugging.
For the three-repeat forward/terrain comparison, use `--cases vx rough default_scene --repeats 3`.
`--resume` skips existing completed trials, while `--summarize-only` refreshes pose-derived metrics without launching a simulator.
Keep the raw JSON/CSV/log/replay files alongside `comparison.json` and `comparison.csv`.
Plot copied results with `tests/tuning/plot_tangent_orientation.py <directory>` in an environment with NumPy and Matplotlib.
