# Upkie Contact-Aware Stand-Up

The 80 mm-wheel example uses an offline whole-body inverse-dynamics trajectory and a stand-up-only finite-horizon feedback tracker.
It models both knee motor housings and both wheels, rather than applying the wheel-only locomotion MPC while the knees support the robot.
The locomotion MPC model, costs, constraints, servo parameters and 200 Hz recommendation are unchanged.
Use the normal [GUI or joystick commands](upkie_posture_control.md); no optimizer runs during a stand-up request.

## Contact Sequence

| Phase | Supported contacts | Purpose |
| --- | --- | --- |
| 1 | Both wheels and both knees | Shift load off the left knee |
| 2 | Both wheels and the right knee | Raise/reconfigure the left leg and unload the right knee |
| 3 | Both wheels | Raise and align the body before handing over to MPC |

The sequence is prescribed, but the configurations, velocities, accelerations, contact forces, actuator torques and phase durations are optimized.
This is a full 3D optimization, not a prescribed CoM path or a sagittal-only model.
The selected solution has three 1.5 s phases; execution transfers to MPC at 3.9 s, before the optimizer's terminal 4.5 s pose.
The stand-up tracker is not intended to replace the MPC as a permanent balancing controller.

Contact-aware phases are motivated by Klemm et al., [Nonsmooth Trajectory Optimization for Wheeled Balancing Robots With Contact Switches and Impacts](https://marco-tognon-robotics.com/content/non-smooth-trajectory-optimization-wheeled-balancing-robots-contact-switches-and-impacts).
This implementation does not reproduce that paper's nonsmooth optimizer or solve for an arbitrary contact sequence.
Ascento's [Section IV-C](https://arxiv.org/html/2005.11435v1) also separates getting up from balancing; its dynamic stand-up is not the gentle trajectory implemented here.

## Optimization Model

Let $\eta=[p_B^W;\psi;\theta;\phi;q_j]\in\mathbb R^{12}$ contain base position, Euler-ZYX angles and six joint angles.
Pinocchio internally uses a free-flyer quaternion configuration $q_P(\eta)$ and body-coordinate base velocity:

$$
v_P=T(\eta)\dot\eta,
\qquad
a_P=T(\eta)\ddot\eta+\dot T(\eta,\dot\eta)\dot\eta.
$$

The decision variables at each node are $\eta,\dot\eta,\ddot\eta$, six actuator torques $\tau_j$, and four world-frame point forces $f_i\in\mathbb R^3$.
The complete inverse-dynamics equality is enforced at every node:

$$
T^\top\left[
f_{\mathrm{RNEA}}(q_P,v_P,a_P)
+S_j^\top\tau_f
-\sum_i J_{c,i}^{\top}f_i
-S_j^\top\tau_j
\right]=0.
$$

The armature is $0.005\ \mathrm{kg\,m^2}$ per joint, and $\tau_f=0.1\tanh(\dot q_j/0.05)\ \mathrm{Nm}$ matches the current simulator/MPC approximation.
These numbers come from the local robot model, not from a stand-up paper or hardware identification.
Unlike the locomotion MPC's optional near-term torque window, the offline problem enforces all joint inverse-dynamics rows throughout its horizon.
This inverse-dynamics formulation follows the same principle as [Molnar et al., Section IV-B1, Eq. (5)](https://arxiv.org/html/2511.19709v1#S4.SS2.SSS1), but uses offline IPOPT rather than their MPC solver.

Each interval uses trapezoidal integration:

$$
\eta_{k+1}=\eta_k+\frac{\Delta t_k}{2}(\dot\eta_k+\dot\eta_{k+1}),
\qquad
\dot\eta_{k+1}=\dot\eta_k+\frac{\Delta t_k}{2}(\ddot\eta_k+\ddot\eta_{k+1}).
$$

There are 16, 16 and 28 intervals in the three phases.
Each phase duration is bounded by `plan.py --phase-seconds MIN MAX` (default 0.5 and 1.5 s).
The running cost is integrated over time, so the same path traversed more slowly always costs less; the optimum therefore sits on MAX, which sets the stand-up speed.
With the earlier 1-5 s bounds every phase took 5 s (13 s to handover) while using at most 26% of the velocity bounds and 35% of the actuator bounds.
With 1.5 s phases the TO's acceleration bounds become active at the second knee release; actuator use stays at or below 34% (legs) and 57% (wheels).
Joint-position, velocity, acceleration and actuator bounds are hard constraints in this offline problem.
Leg torque bounds are 12 Nm and wheel bounds 1.2 Nm, leaving feedback reserve below the simulator's 16/1.7 Nm limits.
The knee-angle magnitude is limited to 2.38 rad to leave margin before tire/femur contact.
The objective penalizes angular motion, joint motion, acceleration, actuator effort, force changes and unnecessary translation; it does not require a running CoM reference trajectory.

### Contact Geometry

Each wheel uses radius $r=0.08\ \mathrm m$ and axial half-width $h=0.022\ \mathrm m$.
The two housings at each knee are approximated by their combined cylinder: radius $0.05\ \mathrm m$, half-width $0.02685\ \mathrm m$, center $[0,-0.17015,0.0700987]^\top$ in the corresponding femur frame.
For world axle $e_i$, upward normal $n=[0,0,1]^\top$, and center $p_i$, the optimizer uses a smooth finite-width support point:

$$
w_i=n-(e_i^\top n)e_i,
\qquad
\rho_i=-r_i\frac{w_i}{\|w_i\|}
-h_i\frac{e_i^\top n}{\sqrt{(e_i^\top n)^2+\epsilon^2}}e_i,
\qquad
\epsilon=0.02.
$$

The second term accounts approximately for the lower axial edge under roll; a thin disk misses this effect.
This remains a smooth approximation to the collision shapes, not an exact contact-patch model.
The material-point velocity is $v_{c,i}=v_i+\omega_i\times\rho_i$, not the derivative of the moving geometric lowest point.
Active contacts have zero normal gap and zero tangential material velocity at interior nodes.
Released knees have zero force and increasing clearance; their clearance lower bound reaches 15 mm.
Support forces obey $f_z\geq0$, $|f_x|\leq0.7f_z$ and $|f_y|\leq0.7f_z$.
These componentwise bounds are a friction pyramid, not a circular cone of radius $0.7f_z$.
Knee normal-force bounds decrease before each release, and force-change costs discourage abrupt unloading.

Pinocchio evaluates RNEA/FK and CasADi differentiates the expressions, including the contact geometry and constrained tracking model.
The C++ scalar conversion follows [Pinocchio's official CasADi RNEA example](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/casadi/casadi-rnea.cpp); NLP construction uses [CasADi Opti/IPOPT](https://web.casadi.org/docs/#opti-stack).
No finite-difference RNEA gradients or new handwritten rigid-body engine are used.
The generator checks inverse dynamics and CoM against MuJoCo at five perturbed 3D states before optimization.

## Tracking and Handover

The offline tool linearizes contact-constrained dynamics and computes finite-horizon Riccati gains, following the [finite-horizon/trajectory-tracking LQR formulation](https://underactuated.mit.edu/lqr.html).
The contact solve uses a $10^{-5}$ regularization for redundant contact rows.
Existing joint PD feedback is included when computing the additional tracking gains.
At runtime the controller applies

$$
\tau_{\mathrm{ff}}(t)=\tau^\star(t)-K(t)\bigl(\hat x(t)-x^\star(t)\bigr),
\qquad
x=[\eta;\dot\eta].
$$

The normal low-level joint PD servo still runs at the simulator physics rate; the extra state-feedback correction runs in the controller update.
It uses the configured floating-base estimate and joint feedback, not simulator ground truth.
The trajectory is anchored to the current XY position, heading and unwrapped free-wheel angles on every cycle.
The stand-up gains are separate from OCS2's `useFeedbackPolicy` option.

The selected plan starts wheel-only MPC warm-up at 3.3 s and transfers at 3.9 s ($D_1+D_2+0.2D_3$ and $\sum D-0.4D_3$).
Before starting or applying MPC, a forward-kinematics guard requires both knees at least 15 mm above the inferred wheel support plane for 50 ms; the two wheel support heights must agree within 10 mm.
The guard uses exact finite-cylinder lower-height formulas rather than the optimizer's smoothing.
This is a flat-ground geometric check: it does not measure normal loads or prove that a tire is carrying force.
During warm-up the reference height is held near the measured height; at handover the normal height ramp restarts there.
Position, velocity, feedforward and servo gains blend into MPC over the unchanged 50 ms interval.
Missing state, a failed clearance check at handover, an unavailable MPC policy or excessive tilt rejects the transition and latches the existing damping fault/fall state.
Sit-down returns to the original activation idle servo targets, avoiding another sag from holding already-deflected measured angles.

## Recorded Acceptance

The first estimated-state ROS comparison, made with the earlier 5 s phases, uses playback plus the first 3 s of MPC; the durations therefore include the blend.
Torque RMS is averaged over different-duration motions, so it is not an equal-time energy comparison.

| Measurement | Previous push-off | Contact-aware, 5 s phases |
| --- | ---: | ---: |
| Playback including blend | 0.67 s | 13.05 s |
| Peak wheel speed | 64.5 rad/s | 3.8 rad/s |
| Wheel torque RMS | 0.914 Nm | 0.098 Nm |
| Wheel-torque saturation samples | 15.0% | 0% |
| Wheel-center X range | 0.70 m | 0.092 m |
| Maximum wheel clearance | 148 mm | 0.083 mm |
| Knee clearance at locomotion entry | approximately 0 mm | 135 mm |
| Reconstructed tire/femur penetration | 6.3 mm | none detected |

Three further estimated-state synthetic-joystick stand/drive/sit cycles passed, with maximum locomotion tilt 0.0091 rad.
Regenerating from the saved source and seed, then repeating three cycles with turning between stands, also passed; maximum locomotion tilt was 0.0139 rad.
The installed default then passed a normal 0.2 m/s drive/stop trial, tracking approximately 0.189 m/s without retuning locomotion.
A direct-service cycle followed by another stand and an injected fall also passed: Fallen remained latched and all three posture services refused further motion requests.
Visual inspection confirms sequential knee unloading and a slow rise without the original wheel lift or catching lunge.
These results validate this simulation checkpoint, not all disturbances, slopes, contact friction values or real hardware.

![Stand-up comparison: measured height, pitch, reconstructed contact clearance, wheel speed and wheel effort](results/assets/upkie_r80_stand_up.png)

Solid CoM-offset and dashed minimum-knee-clearance curves share each run's color in the middle panel.
The [recorded metrics](results/assets/upkie_r80_stand_up.json) and `tests/tuning/analyze_upkie_stand_up.py` preserve the measurement definitions.

### Shorter Phases

The shipped plan uses 1.5 s phases (`--phase-seconds 0.5 1.5`); the model, contact sequence, bounds and cost are otherwise unchanged.
The same measurements over playback plus the first 3 s of MPC, estimated state, one run each (two runs per plan overlap in the plot):

| Measurement | 5 s phases | 1.5 s phases (shipped) | 1.0 s phases |
| --- | ---: | ---: | ---: |
| Playback including blend | 13.05 s | 3.95 s | 2.65 s |
| Peak wheel speed | 3.9 rad/s | 4.2 rad/s | 4.7 rad/s |
| Peak wheel effort | 0.64 Nm | 0.63 Nm | 0.72 Nm |
| Wheel torque RMS | 0.098 Nm | 0.160 Nm | 0.190 Nm |
| Wheel-torque saturation samples | 0% | 0% | 0% |
| Wheel-center X range | 0.092 m | 0.090 m | 0.087 m |
| Maximum wheel clearance | 0.1 mm | 0.4 mm | 0.8 mm |
| Knee clearance at locomotion entry | 135 mm | 137 mm | 138 mm |
| Reconstructed tire/femur penetration | none | none | none |
| Most forward pitch (second knee release) | -0.20 rad | -0.29 rad | -0.31 rad |
| Tilt in the first 1 s of MPC, regular cycles | 0.009-0.011 rad | 0.013-0.024 rad | 0.016-0.025 rad |
| Mean MPC rate | 190 Hz | 191 Hz | 189 Hz |

The shipped 1.5 s plan then passed five synthetic-joystick stand/drive/sit cycles plus the forced-fall latch, three cycles with turning between stands (maximum locomotion tilt 0.017 rad), and two full course runs.
Locomotion tilt after the first second of MPC is unchanged (0.003-0.005 rad in every plan).
The 1.0 s plan also passed three cycles plus the fall latch and two stand-ups, but with less margin: the TO's acceleration bounds are active on most joints, wheel effort is higher, and measured pitch varied between runs (0.085-0.134 rad), so it is not the default.
With a 2.0 s cap IPOPT did not reach the $10^{-5}$ export threshold within 1000 iterations, also when warm-started from the 1.5 s solution, so no 2 s plan was exported.

![Stand-up speed comparison: 5 s, 1.5 s and 1.0 s phases](results/assets/upkie_r80_stand_up_speed.png)

[Recorded metrics](results/assets/upkie_r80_stand_up_speed.json).

## Regeneration

The shipped trajectory needs no Python optimizer dependencies to run.
Regeneration is optional and specific to the current 80 mm wheel geometry; a radius/width mismatch is rejected.
Run the following from the repository root in an environment with a C++ compiler and CMake:

```bash
python3 -m venv /tmp/upkie_stand_up_env
source /tmp/upkie_stand_up_env/bin/activate
pip install mujoco==3.13.0 casadi pin cmeel-eigen numpy scipy pyyaml
PREFIX=$(python -c 'import site; print(site.getsitepackages()[0])')
WORK=/tmp/upkie_stand_up
mkdir -p "$WORK"
cmake -S config/upkie/stand_up_to -B "$WORK/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX/cmeel.prefix;$PREFIX/casadi"
cmake --build "$WORK/build" -j2
"$WORK/build/upkie_stand_up_model" \
  description/urdf/robots/upkie/upkie.urdf "$WORK/upkie"
python config/upkie/stand_up_to/plan.py --models "$WORK/upkie" \
  --output "$PWD/demo_videos/upkie_stand_up/candidate.yaml"
```

`seed.json` is a warm start, not a fixed motion constraint; it holds the shipped 1.5 s solution, and its node rates are rescaled when `--phase-seconds` changes the duration.
A faster or slower plan only needs a different `--phase-seconds MIN MAX`; the optimum uses MAX for every phase.
The solution JSON records the NLP termination status and measured maximum constraint violation.
An iteration-limited but constraint-feasible iterate may be retained; the export rejects violation above $10^{-5}$ and does not claim an optimal solution merely because it is feasible.
Always validate a regenerated candidate in MuJoCo before replacing the shipped YAML:

```bash
ROS_DOMAIN_ID=169 python3 tests/upkie_posture_cycle_test.py \
  --cycles 3 --joystick --trajectory "$PWD/demo_videos/upkie_stand_up/candidate.yaml" \
  --prefix /tmp/upkie_candidate
```

Run that acceptance command from the repository root in the sourced ROS environment.
The candidate is stored under the shared repository, so it is also available inside the container; temporary host model libraries are not needed for runtime playback.
The geometry/model generation and NLP are offline only; no CasADi, IPOPT or Python Pinocchio dependency was added to the MPC controller.
The wheel-only locomotion OCP still cannot plan knee-supported motion.
Unknown terrain stand-up, unexpected contact switches, online replanning and physical hardware validation remain future work.
