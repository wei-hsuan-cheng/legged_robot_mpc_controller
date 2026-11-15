# Wheeled-Biped State Estimation (Wheel-Aware Linear KF)

This note describes the floating-base estimator that drives the Upkie WB-ID MPC in `feat/wheeled_biped`, inspected on 2026-11-15.
It expands [formulation Section 7.5](wheeled_biped_whole_body_inverse_dynamics_mpc.md#75-rolling-contact-state-estimation) with the complete filter equations, the code line that implements each one, and how the filter differs from the papers it follows.
The configuration is the `humanoid_linear_kf` block of [config/upkie/ros2_controllers.yaml](../config/upkie/ros2_controllers.yaml#L15-L53); closed-loop results are in the [handoff](wheeled_biped_handoff.md#wheel-aware-linear-kf).
Line references are to commit `bbb6f25`.


## 1. One Filter Class, Two Contact Models

The wheel model is a mode of the existing linear KF, not a separate estimator.
[`LinearKfFloatingBaseEstimator`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp), the Cheetah 3 two-stage filter G1/H2 can already select, gained two options in commit `33c3406`:

| Parameter | Default (G1/H2) | Upkie | Effect |
| --- | --- | --- | --- |
| `contactModel` | `point` | `rolling_wheel` | planted contact points as landmarks, or rolling wheels |
| `imuExtrinsics` | `false` | `true` | remove the IMU mounting from attitude, rate and acceleration |

The constructor selects the model ([L80-L90](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L80-L90)) and builds its matrices ([wheels L92-L110](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L92-L110), [points L111-L143](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L111-L143)).
`updateImpl` dispatches to `updateRollingWheels` before any point-model code runs ([L291-L294](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L291-L294)).
The point path changed only in reading the rate and acceleration through the mounting correction ([L283-L290](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L283-L290)), which is the identity with `imuExtrinsics: false`, so the G1/H2 filters compute exactly what they did before.

Shared by both models: the `humanoid_linear_kf` ros2_control controller and its interfaces to the MPC, stage 1, the constant-acceleration process model and its noise scaling, the airborne-contact noise inflation, initialisation from stance, readiness and odometry.
New for wheels: [`imuToBaseOrientation`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L410-L416), [`baseRelativeKinematics`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L418-L442), [`wheelContact`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L444-L460) and [`updateRollingWheels`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L462-L535).


## 2. Per-Tick Pipeline

The estimator is its own chainable controller at the MRT rate (1 kHz), updated in the same controller-manager tick as the MPC; one of the two directions is therefore one tick late.
Each tick, [`update_and_write_commands`](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L220-L280):

1. Reads gyro, accelerometer, the IMU quaternion and the joint encoders ([`read_sensors`](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L178-L218)).
2. On a new initialisation epoch from the MPC, initialises from stance ([L229-L250](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L229-L250); Section 9).
3. Stage 1: sets the attitude from the IMU quaternion ([L251-L253](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L251-L253), [`setBaseOrientation`](../include/legged_robot_mpc_controller/humanoid_state_estimation/linear_kf_floating_base_estimator.hpp#L103)).
4. Reads the two support flags the MPC writes from its mode schedule ([L255-L263](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L255-L263)).
5. Stage 2: predicts with the IMU and corrects with both wheels ([`updateRollingWheels`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L462-L535)).
6. Evaluates readiness ([L275](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L275)) and writes the estimate to the interfaces the MPC claims, and to `/humanoid/state_estimate/odom`.


## 3. Notation

$W$ is the world frame (z up, flat ground plane $z=z_g=0$), $B$ the base frame, $I$ the IMU frame and $i\in\{L,R\}$ a wheel.
$R_{WB}$ is the base attitude, $p$ and $v$ the base position and velocity in $W$, and $\omega_B$ the base angular rate in $B$.
A superscript $\mathrm{rel}$ marks forward kinematics evaluated with the base at the origin and no translational velocity, in world-aligned axes (Section 6).
$\Delta t=1/\texttt{update\_rate}=1\ \mathrm{ms}$ ([controller L81](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L81)).


## 4. Stage 1: Attitude and the IMU Mounting

The attitude is the IMU's own fused quaternion $R_{WI}$; it is not a filter state.
Upkie's IMU is rotated and offset from the base ([URDF L626-L631](../description/urdf/robots/upkie/upkie.urdf#L626-L631): $r_{BI}=[-0.041,\ 0.040,\ 0.130]^{\mathsf T}\ \mathrm m$, rpy $(-\pi/2,\ 0,\ -\pi/2)$), read once from the URDF ([L65-L78](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L65-L78)).
With $R_{BI}$ and $r_{BI}$ that mounting:

$$
R_{WB}=R_{WI}R_{BI}^{\mathsf T},
\qquad
\omega_B=R_{BI}\,\omega_I,
\qquad
a_B=R_{BI}\,a_I-\omega_B\times(\omega_B\times r_{BI}).
$$

The attitude is [`imuToBaseOrientation`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L410-L416); rate and acceleration are [L287-L290](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L287-L290).
The rigid-body transfer also has an angular-acceleration term $-\dot\omega_B\times r_{BI}$, which is left out because no angular acceleration is measured.
At $\dot\omega=10\ \mathrm{rad/s^2}$, a hard catch, it is about $1.4\ \mathrm{m/s^2}$ of unmodelled acceleration, which the wheel velocity rows remove at the time constant of Section 8.
Yaw is never corrected, and roll/pitch are only as good as the sensor's own fusion; in simulation the quaternion is exact.


## 5. Stage 2: State and Process Model

With attitude fixed by stage 1, the remaining process is linear:

$$
x=\begin{bmatrix}p\\ v\end{bmatrix}\in\mathbb R^6,
\qquad
u=R_{WB}\,a_B+g,
\qquad
g=\begin{bmatrix}0&0&-9.81\end{bmatrix}^{\mathsf T},
$$

$$
x_{k+1}^-=A\,x_k+B\,u_k,
\qquad
A=\begin{bmatrix}I_3&\Delta t\,I_3\\0&I_3\end{bmatrix},
\qquad
B=\begin{bmatrix}\tfrac12\Delta t^2 I_3\\ \Delta t\,I_3\end{bmatrix},
\qquad
P_{k+1}^-=A P_k A^{\mathsf T}+Q,
$$

$$
Q=\operatorname{diag}\!\left(\frac{\Delta t}{20}\,\sigma_p I_3,\ \frac{9.81\,\Delta t}{20}\,\sigma_v I_3\right),
\qquad
\sigma_p=\sigma_v=0.02 .
$$

$A,B$ are [L470-L474](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L470-L474), $Q$ is [L477-L479](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L477-L479) and the prediction is [L502-L505](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L502-L505).
The $1/20$ and $9.81$ factors are Cheetah 3's, carried by legged_control and kept identical to the point model ([L305-L312](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L305-L312)), so `imuProcessNoisePosition/Velocity` mean the same in both.

The point model also carries every contact position and holds it constant ($\dot p_{c,i}=0$).
A wheel's geometric contact point moves with the robot whenever it drives, so holding it constant is wrong; the tread particle at the contact is at rest, but a different particle touches the ground every instant, so it is not a lasting landmark either.
What stays true is an instantaneous velocity constraint, which is what the measurement rows use, so the state has no contact block.


## 6. Wheel Contact Geometry

[`wheelContact`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L444-L460) evaluates the lowest point of the tire circle, the disk model of [formulation Section 4.1](wheeled_biped_whole_body_inverse_dynamics_mpc.md#41-instantaneous-contact-point) with $n=e_z$.
Let $o_i$ and $R_i$ be the wheel-link origin (the wheel joint) and rotation, $c_i$ the tire centre in that link (`wheel.centerOffset`, $[0,0,-0.0312]^{\mathsf T}\ \mathrm m$), $e_{\mathrm{local}}$ the axle (`wheel.axleAxis`, local $z$) and $r=0.055\ \mathrm m$:

$$
e_i=R_i\,e_{\mathrm{local}},
\qquad
d_i=\frac{-e_z+(e_z^{\mathsf T}e_i)\,e_i}{\lVert -e_z+(e_z^{\mathsf T}e_i)\,e_i\rVert},
\qquad
s_i=R_i\,c_i+r\,d_i,
\qquad
p_{c,i}^{\mathrm{rel}}=o_i^{\mathrm{rel}}+s_i .
$$

$d_i$ is straight down with its component along the axle removed ([L451-L455](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L451-L455)), so it stays in the disk plane when the axle tilts; for a near-vertical axle it falls back to $-e_z$.
$s_i$ is the offset from the link origin to the contact ([L456-L458](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L456-L458)).
The velocity of the tread material at the contact is the rigid-body velocity of the wheel link at that point ([L459](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L459)):

$$
v_{c,i}^{\mathrm{rel}}=v_{o,i}^{\mathrm{rel}}+\omega_i\times s_i ,
$$

where $v_{o,i}^{\mathrm{rel}}$ and $\omega_i$ are the link's `LOCAL_WORLD_ALIGNED` velocity ([L449-L450](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L449-L450)), and $\omega_i$ includes the base rate, hip, knee and wheel spin.
Wheel odometry needs no separate code: $c_i$ lies on the axle, so the spin $\dot q_{w,i}e_i$ acts only on $r\,d_i$, and $\dot q_{w,i}\,e_i\times r\,d_i$ has magnitude $r|\dot q_{w,i}|$ along the rolling direction, the familiar $r\omega$ term.
The tire is a thin disk here; its 44 mm width is ignored.

The kinematics come from [`baseRelativeKinematics`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L418-L442): Pinocchio with the free flyer at the origin, attitude $R_{WB}$, angular rate $\omega_B$, zero linear velocity, and the encoders.
A rigid displacement of the whole robot adds $v$ to every point's velocity and $p$ to every position, so the world-frame contact quantities are $v+v_{c,i}^{\mathrm{rel}}$ and $p+p_{c,i}^{\mathrm{rel}}$.
The estimated base state therefore never enters the kinematics it is corrected by, which is what keeps the measurement linear in $x$.


## 7. Measurement Model

Rolling without slip on the plane gives, per supporting wheel,

$$
0=v+v_{c,i}^{\mathrm{rel}}
\qquad\text{(tread material at rest)},
\qquad
z_g=p_z+\big(p_{c,i}^{\mathrm{rel}}\big)_z
\qquad\text{(contact on the plane)}.
$$

Stacking both wheels, three velocity rows each and then one height row each ([C at L101-L105](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L101-L105), [y at L488-L489](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L488-L489)):

$$
y=\begin{bmatrix}
-v_{c,L}^{\mathrm{rel}}\\
-v_{c,R}^{\mathrm{rel}}\\
z_g-\big(p_{c,L}^{\mathrm{rel}}\big)_z\\
z_g-\big(p_{c,R}^{\mathrm{rel}}\big)_z
\end{bmatrix}
=C\,x+\nu,
\qquad
C=\begin{bmatrix}
0&I_3\\
0&I_3\\
e_z^{\mathsf T}&0\\
e_z^{\mathsf T}&0
\end{bmatrix}\in\mathbb R^{8\times6},
$$

$$
\nu\sim\mathcal N(0,R_\nu),
\qquad
R_\nu=\operatorname{diag}\!\big(\sigma_{\dot c}\,\mu_L I_3,\ \sigma_{\dot c}\,\mu_R I_3,\ \sigma_h\,\mu_L,\ \sigma_h\,\mu_R\big),
\qquad
\mu_i=\begin{cases}1&\text{wheel }i\text{ scheduled on the ground}\\100&\text{otherwise}\end{cases}
$$

with $\sigma_{\dot c}=0.1$ (`footSensorNoiseVelocity`) and $\sigma_h=0.01$ (`footHeightSensorNoise`) ([L490-L495](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L490-L495)).
The height is relative to the plane under the wheels, the same proprioceptive convention as the MPC's flat ground model, so the estimator and the controller agree on what "height" means.

$C$ has no column for $p_x,p_y$: horizontal position is dead reckoning from $v$ and its covariance grows without bound, as yaw does in stage 1.
Height and all three velocity components are observed directly, which is what the MPC consumes.


## 8. Correction and What the Gains Amount To

The correction is the textbook one ([L507-L513](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L507-L513)):

$$
\nu=y-C\,x^-,
\qquad
S=C P^- C^{\mathsf T}+R_\nu,
\qquad
K=P^- C^{\mathsf T}S^{-1},
\qquad
x=x^-+K\nu,
\qquad
P=(I-KC)\,P^-,
$$

followed by $P\leftarrow\tfrac12(P+P^{\mathsf T})$ against round-off.
The NIS $\nu^{\mathsf T}S^{-1}\nu$, the covariance std-devs and the kinematic height go to the diagnostics ([L515-L526](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L515-L526)).

All matrices above are constant for a given support pattern, so $K$ converges to a constant.
Iterating the Riccati recursion offline with the shipped values gives:

| support | velocity correction per tick | velocity time constant | height time constant | steady std, height / velocity |
| --- | --- | --- | --- | --- |
| both wheels | 1.4 % | 72 ms (crossover 2.2 Hz) | 70 ms | 8.5 mm / 0.026 m/s |
| one wheel | 1.0 % | 101 ms (crossover 1.6 Hz) | 97 ms | 10 mm / 0.031 m/s |

Stage 2 is therefore a complementary filter: above about 2 Hz the velocity is integrated IMU acceleration, below it the wheels' rolling kinematics; the height is integrated IMU motion pulled toward the tires' contact plane.
The reported std is well above the measured error (Section 11) because the noise values are the parameter defaults (written out in the Upkie YAML), carried over from legged_control's quadruped values and not identified for Upkie, and the simulated sensors are noise-free.
Treat the covariance as a conservative bound, not a calibrated one.


## 9. Contacts, Initialisation and Readiness

**Support flags.** With `contact.source: scheduled` the MPC writes one flag per wheel from its mode schedule in [`write_estimator_references`](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp#L1213-L1228), both on the ground until it drives (idle and stand-up playback).
A wheel scheduled off the ground keeps its four rows with $\mu_i=100$ (`swingNoiseMultiplier`), so its gain is nearly zero while the matrix sizes never change.
In the `jump` flight phase both wheels are inflated and the filter runs on the IMU alone for 0.15-0.18 s.

**Initialisation.** [`initializeFromStance`](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L221-L235) takes the attitude from the IMU (mounting removed), evaluates the contacts at zero velocity, and seeds

$$
p=\Big[0,\ 0,\ \tfrac12\textstyle\sum_i\big(z_g-(p_{c,i}^{\mathrm{rel}})_z\big)\Big]^{\mathsf T},
\qquad
v=0,
\qquad
P_0=100\,I_6
$$

([L185-L189](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L185-L189)).
It runs at the kneeling idle, where the knees and both tires rest on the floor ([formulation Section 7.4](wheeled_biped_whole_body_inverse_dynamics_mpc.md#74-optional-planned-stand-up)), so the tire-height rows are valid from the first tick.
A standing spawn cannot start the estimator: an unbalanced wheeled robot does not stand still while the filter initialises.

**Readiness.** [`evaluateReadiness`](../src/humanoid_state_estimation/humanoid_state_estimator.cpp#L46-L68) waits `readiness.minSeconds` (1 s for Upkie), then requires height std below 0.05 m and velocity std below 0.20 m/s ([config L36-L41](../config/upkie/ros2_controllers.yaml#L46-L50)); the steady values of Section 8 pass both.
The roll/pitch guard is inert for this filter: stage 1 has no covariance and `attitude_std` stays zero.


## 10. Parameters

| Parameter | Upkie value | Used at |
| --- | --- | --- |
| `contactModel` | `rolling_wheel` | [L80-L90](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L80-L90) |
| `wheel.radius`, `wheel.centerOffset`, `wheel.axleAxis` | 0.055 m, $[0,0,-0.0312]$ m, $[0,0,1]$ | [L451-L457](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L451-L457) |
| `imuExtrinsics`, `imuFrame` | `true`, `imu` | [L65-L78](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L65-L78) |
| `imuProcessNoisePosition`, `imuProcessNoiseVelocity` | 0.02, 0.02 | [L477-L479](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L477-L479) |
| `footSensorNoiseVelocity`, `footHeightSensorNoise` | 0.1, 0.01 | [L494-L495](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L494-L495) |
| `swingNoiseMultiplier` | 100 | [L493](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp#L493) |
| `contact.source` | `scheduled` | [controller L255-L263](../src/humanoid_state_estimation/humanoid_state_estimator_controller.cpp#L255-L263) |
| `readiness.minSeconds` | 1.0 | [readiness L56](../src/humanoid_state_estimation/humanoid_state_estimator.cpp#L56) |

The ground height $z_g$ is fixed at 0: `height_ground_z` keeps its default because the linear KF exposes no parameter for it.
`footProcessNoisePosition`, `footSensorNoisePosition` and `footRadius` belong to the point model and are unused here.
The parameter definitions and their wiring are [humanoid_linear_kf_parameter.yaml](../src/humanoid_state_estimation/humanoid_linear_kf_parameter.yaml#L109-L186) and [humanoid_linear_kf.cpp](../src/humanoid_state_estimation/humanoid_linear_kf.cpp#L56-L71).


## 11. Measured Accuracy

Against simulator ground truth, with noise-free simulated sensors (handoff and [formulation Section 7.5](wheeled_biped_whole_body_inverse_dynamics_mpc.md#75-rolling-contact-state-estimation)):

| Run | Height | Roll / pitch | Forward velocity |
| --- | --- | --- | --- |
| rolling at 0.3 m/s and braking | RMS 0.2 mm | below 0.0001 rad | RMS 1 mm/s, max 8 mm/s |
| `walk`, `trot` | RMS 0.7-1.1 mm | RMS below 0.0015 rad | RMS 0.01-0.03 m/s |
| `jump` | RMS 1-1.5 mm, max 12 mm | RMS below 0.003 rad | RMS 0.1 m/s, up to 0.6-0.7 m/s at touchdown |
| 2-degree ramp, uneven field (estimate vs world z) | RMS 18 mm, 13 mm, by design (Section 7) | - | - |

Driving the MPC with no GT in the loop, it passed rolling and braking, turning, pitch pushes, the 20 mm strip, the ramp, the uneven field, all four gaits and the one-minute terrain course.

**Raised ground.** On a 3-degree ramp to a plateau 0.50 m above $z=0$ (estimator in control, 0.3 m/s, then 24 s standing on the plateau), GT world z rose from 0.30 to 0.80 m while the estimate stayed at 300 mm (299.9 ± 0.1 on the ramp, 300.1 on the plateau).
The MPC held 0.30 m above the local ground the whole way, with pitch below 0.012 rad.
The velocity rows do not see the climb either: on a slope the lowest tire point is not the contact, and its material velocity is about the climb rate, so the rows read $v_z\approx0$ (estimate 0.0006 against GT 0.0147 m/s), while the horizontal speed matches (0.282 against 0.281 m/s).
The height is therefore always the height above the ground under the wheels, never world height; a remembered elevation decays with a time constant of about 0.1 s (99 % gone in 0.3 s).
The robot does not crouch on high ground or stretch on low ground, and the odometry gives no world z.
The same holds for G1/H2's point model: its stance velocity rows do see the climb, but the foot-height rows pin every stance contact to $z=0$, and a remembered elevation decays in about 0.1 s as well (99 % in 0.4-0.5 s, from the filter equations).

**Ledge drop.** Driving off a 5 cm and a 10 cm edge at 0.3 m/s ([results](results/upkie_ledge_drop.md)), the estimate's error against the true height above the ground stays within -16/+7 mm around the landing and is back within 5 mm in at most 0.17 s.
It has the same shape and decay whether the KF drives the MPC or is only logged while GT drives, so the estimate-to-plan loop showed no amplification.
With GT driving, the flat-ground MPC crouches by the platform height on top and reaches down at the edge.


## 12. Relation to the References

The filter descends from the two-stage estimator of [Bledt et al., Cheetah 3, IROS 2018, Section III-H](https://doi.org/10.1109/IROS.2018.8593885), through its OCS2 port `LinearKalmanFilter` in [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control), which is this class's point model.
The idea of adapting such a filter to wheels comes from [Hosseini, Rodriguez and Behnke, State Estimation for Hybrid Locomotion of Driving-Stepping Quadrupeds, IRC 2022](https://arxiv.org/abs/2211.11390), itself an extension of the Mini Cheetah filter of Katz et al.
The rolling-wheel model takes that idea, not its equations:

| Aspect | Cheetah 3 / legged_control (point model) | Hosseini et al. | This rolling-wheel model |
| --- | --- | --- | --- |
| Attitude | Cheetah 3: Mahony complementary filter; legged_control: IMU quaternion | IMU quaternion (IV-B) | IMU quaternion, mounting rotation and lever arm removed |
| State | $p$, $v$, every contact position | $p$, $\dot p$, driving share $p_w$, $\dot p_w$, four contact positions; 24 (Eq. 10) | $p$, $v$; 6 |
| Velocity rows | leg kinematics vs $-v$ | split: leg kinematics $J\dot q$ vs $\dot p-\dot p_w$, wheel term (Eq. 5) vs $\dot p_w$; their sum is Eq. 6 | one rigid-body tread velocity per wheel, legs and spin together, vs $-v$ |
| Position rows | contact relative to base vs $p-p_c$ | the same, with $p_w$ | none |
| Height | one row per foot against the ground | z of the contact positions, weighted by a height trust (Eq. 9) | one row per wheel against the plane $z_g=0$ |
| Wheel geometry | - | effective radius $a-b\sin(q_1+\phi)$ of a wheel with width (Eq. 4) | lowest point of a thin disk, tilted axle included |
| Contact trust | phase-based (Cheetah 3), binary multiplier (legged_control) | erf phase window times height trust, inflating the noise (Eqs. 8-12) | binary scheduled flag, noise times 100 |
| Validation | hardware | Mini Cheetah with wheels, simulation and hardware, qualitative plots | Upkie simulation, noise-free sensors, RMS against GT |

Each difference is a deliberate simplification for this robot and controller:

1. **No driving-share states.** Hosseini et al. split the base velocity into a stepping part and a driving part because their footstep planner needs the velocity without driving (Eq. 26). The WB-ID MPC consumes only the total base velocity and plans no footholds, so the split would add six states with no consumer. The sum of their two velocity measurements (Eq. 6) carries the same information as the single tread-velocity row here.
2. **No contact positions.** Their legs step and plant, so landmarks pay off during stance; Upkie's support is rolling nearly all the time, and a rolling contact is no landmark (Section 5).
3. **An explicit height row instead of a height trust.** The MPC models a plane at zero under the wheels; reading the height against that same plane keeps the two consistent, for the same reason G1 ships `height.source: kinematic`. The cost is that terrain reads as height: on the ramp the estimate is the height above the wheels' plane, not world z. Their height trust addresses a different case, one contact on a bump: it stops believing that contact's height, while here every supporting contact is pinned to the plane with the same constant $\sigma_h$, so one wheel on a step moves the estimate by about half the step height.
4. **Binary contact trust.** The MPC's schedule is the only contact source, and the gaits lift a wheel for 0.12-0.18 s. A phase window like their Eq. 8 is the published remedy for the touchdown spikes in Section 11, and is not implemented.

Not in either reference: the IMU mounting removal with its centripetal lever-arm term, and the tilted-axle lowest point (from the MPC's own wheel geometry).


## 13. Limitations

- Simulated sensors are noise-free and the IMU quaternion is exact, so every accuracy number above is a best case.
- There are no bias states, as in Cheetah 3. A constant accelerometer bias $b$ would leave a steady velocity error of about $b\,\tau$ with $\tau\approx72\ \mathrm{ms}$, i.e. 7 mm/s for $0.1\ \mathrm{m/s^2}$.
- No slip detection: a wheel that spun in the air lands slipping, and its rows read the slip as base velocity.
- The angular-acceleration lever-arm term of Section 4 is left out.
- Thin-disk geometry, one flat plane at $z_g=0$, yaw and horizontal position from proprioception only.
- The noise values are not identified for Upkie, so the reported covariance is not calibrated (Section 8).
