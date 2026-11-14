# Wheeled-Biped Whole-Body Inverse Dynamics MPC

This note describes the Upkie implementation in `feat/wheeled_biped`, inspected on 2026-11-14.
It complements the [humanoid formulation](humanoid_whole_body_inverse_dynamics_mpc.md) with the wheel-contact model and the actual Upkie costs, constraints and execution path.
The current configuration is [config/upkie/ros2_controllers.yaml](../config/upkie/ros2_controllers.yaml).
Build commands, manual tests, measured results and outstanding work are in the [wheeled-biped handoff](wheeled_biped_handoff.md).
The [initial checklist](design_notes/wheeled_biped/wheeled_biped_first_step.md) and [survey](design_notes/wheeled_biped/wheeled_biped_wb_id_mpc_survey.md) are preserved under `docs/design_notes/wheeled_biped/` and describe the motivation, not the current validation status.


## 1. Architecture and Model

Upkie has a floating base and six actuators, ordered as follows:

$$
q_j=\begin{bmatrix}
q_{L,h}&q_{L,k}&q_{L,w}&q_{R,h}&q_{R,k}&q_{R,w}
\end{bmatrix}^{\mathsf T}.
$$

Each side has a hip, knee and driven wheel, with no hip-roll actuator.
The simulation-reference mass is $m=5.4605\ \mathrm{kg}$ and tire radius is $r=0.055\ \mathrm{m}$.
The MPC URDF is generated from the MuJoCo model to match link frames, joint axes, inertias and tire geometry; see [model provenance](../description/urdf/robots/upkie/MODEL_SOURCE.md).
The optimizer uses Pinocchio kinematics and inverse dynamics, CppAD-generated constraint derivatives, and OCS2 SQP with HPIPM.
The asynchronous MPC worker publishes a policy that the ros2_control thread evaluates at the MRT frequency, followed by an `effort_pd` joint servo.
There is no centroidal-momentum state, reduced cart-pendulum prediction model, or separate whole-body-control QP in this path.
For simulation debugging, the current wheel-drive checkpoint uses `ground_truth_state` with no estimator; GT feedback is not available for real-world deployment.
Rolling-contact estimator feedback has not yet been validated.


## 2. State, Input and Initial Observation


### 2.1 Generalized coordinates

The base is Pinocchio's Translation + SphericalZYX composite joint:

$$
q=\begin{bmatrix}p_{WB}^{W}\\\eta\\q_j\end{bmatrix}\in\mathbb R^{12},
\qquad
\eta=\begin{bmatrix}\psi&\theta&\phi\end{bmatrix}^{\mathsf T},
\qquad
R_{WB}=R_z(\psi)R_y(\theta)R_x(\phi).
$$

Yaw, pitch and roll are ordered ZYX, and the generalized velocity is:

$$
v=\dot q=\begin{bmatrix}v_{WB}^{W}\\\dot\eta\\\dot q_j\end{bmatrix}\in\mathbb R^{12},
\qquad
x=\begin{bmatrix}q\\v\end{bmatrix}\in\mathbb R^{24}.
$$

The angular entries are Euler rates, not body or world angular velocity.
The wheel angles are scalar, unwrapped revolute coordinates, not periodic posture targets.
They remain in the dynamics but are excluded from joint-position limits and position tracking.
The current branch does not use tangent-space orientation integration or quaternion decision variables, and the Euler pitch singularity remains.


### 2.2 Input

$$
u=\begin{bmatrix}W_L\\W_R\\a\\\tau_j\end{bmatrix}\in\mathbb R^{30},
\qquad
W_i=\begin{bmatrix}f_i\\m_i\end{bmatrix}\in\mathbb R^6,
\qquad
a=\dot v\in\mathbb R^{12},
\qquad
\tau_j\in\mathbb R^6.
$$

Each wrench is expressed in world-aligned axes about the tire-center frame, not about a fixed ground point.
The six wrench components per wheel are retained for compatibility, but three moment equalities restrict the wrench to a point force at the instantaneous tread contact.
The tire-center frames are `left_tire_center` and `right_tire_center`, attached to the wheel joints with local translation $[0,0,-0.0312]^{\mathsf T}\ \mathrm{m}$.
Their orientations spin with the wheels, while their centers do not move merely because of wheel spin.


### 2.3 Observation

At each control update, the initial state combines the floating-base observation with measured joint positions and velocities:

$$
q_0=\begin{bmatrix}\hat p_{WB}^{W}\\\operatorname{EulerZYX}(\hat R_{WB})\\q_{j,\mathrm{meas}}\end{bmatrix},
\qquad
v_0=\begin{bmatrix}\hat v_{WB}^{W}\\E_{\mathrm{ZYX}}(\eta_0)\hat\omega_{WB}^{B}\\\dot q_{j,\mathrm{meas}}\end{bmatrix}.
$$

$E_{\mathrm{ZYX}}$ converts body angular velocity to Euler rates, and the observation builder unwraps yaw.
The Upkie YAML disables observation-velocity filtering with `observationVelocityFilterCutoffHz: 0.0`.
The observation input is zero-initialized; measured joint velocities already belong to the state and are not inserted into the acceleration/torque input slots.


## 3. Dynamics and Optimal Control Problem


### 3.1 Kinematic propagation

The shooting dynamics are the same double integrator as the humanoid WB-ID controller:

$$
\dot x=\begin{bmatrix}v\\a\end{bmatrix}.
$$

For constant input on an interval of duration $\Delta t_k$, the exact solution is:

$$
q_{k+1}=q_k+\Delta t_kv_k+\frac12\Delta t_k^2a_k,
\qquad
v_{k+1}=v_k+\Delta t_ka_k.
$$

The configured RK4 discretization reproduces this constant-input flow.
Rigid-body dynamics are imposed as path equalities, rather than used as forward-dynamics state propagation.


### 3.2 Full-body inverse dynamics

Let $S_j=[0_{6\times6}\ I_6]$ select the actuated coordinates, and let $J_{h,i}$ be the world-aligned six-dimensional Jacobian at tire center $i$.
Define the armature-augmented mass matrix and smoothed joint friction:

$$
M_A(q)=M_{\mathrm{rigid}}(q)+S_j^{\mathsf T}A_jS_j,
\qquad
A_j=\operatorname{diag}(0.005,0.005,0.005,0.005,0.005,0.005),
$$

$$
\tau_{f,j}(v)=f_{c,j}\tanh\left(\frac{\dot q_j}{v_s}\right),
\qquad
f_{c,j}=0.1\ \mathrm{Nm},
\qquad
v_s=0.05\ \mathrm{rad/s}.
$$

The armature entries have units $\mathrm{kg\,m^2}$ and are set in the Pinocchio model before generating the inverse-dynamics libraries.
The complete equality is:

$$
M_A(q)a+b(q,v)+S_j^{\mathsf T}\tau_f(v)
-\sum_{i\in\{L,R\}}J_{h,i}(q)^{\mathsf T}W_i
-S_j^{\mathsf T}\tau_j=0.
$$

The six floating-base rows and six joint rows are registered separately, but both remain active throughout Upkie's horizon because `torqueConstraintNodes: 0`.
This differs from H2 configurations that keep joint RNEA and torque penalties only inside a near-term torque window.
The joint friction is an additional torque in the actuated rows, not a base friction force or wheel-ground friction coefficient.
The smooth model satisfies $\tau_f(0)=0$, so it does not reproduce MuJoCo's set-valued static friction exactly.


### 3.3 Optimization problem

$$
\begin{aligned}
\min_{x(\cdot),u(\cdot)}\quad&
\Phi(x(t_f))+\int_{t_0}^{t_f}\big[\ell_{\mathrm{track}}(x,u,t)+\ell_{\mathrm{soft}}(x,u,t)\big]\,dt\\
\text{subject to}\quad&x(t_0)=\hat x_0,\\
&\dot q=v,\qquad\dot v=a,\\
&r_{\mathrm{ID}}(q,v,a,W,\tau_j)=0,\\
&g_{\mathrm{wheel}}(q,v,a,W,t)=0,\\
&W_i=0\quad\text{when wheel }i\text{ is scheduled in swing}.
\end{aligned}
$$

Friction and joint limits enter $\ell_{\mathrm{soft}}$, not additional hard inequalities in this Upkie configuration.
Current settings are a $1.0\ \mathrm{s}$ horizon, nominal $\Delta t=0.02\ \mathrm{s}$, one SQP iteration, four solver threads, equality projection enabled, and no feedback-gain policy.
`timeGrid.activate: false` selects the uniform grid with the solver's normal event handling; the configured coarse-tail parameters are inactive.
Requested MPC and MRT frequencies are $100\ \mathrm{Hz}$ and $1000\ \mathrm{Hz}$, not guaranteed measured rates.


## 4. Rolling-Wheel Geometry and Equalities


### 4.1 Instantaneous contact point

For wheel $i$, let $p_{h,i}$ be the tire center, $e_i$ its unit axle in world coordinates, and $n$ the unit ground normal.
The configured axle is local $[0,0,1]^{\mathsf T}$, transformed through the spinning contact frame:

$$
e_i=R_{Wh,i}e_{\mathrm{local}},
\qquad
w_i=n-(e_i^{\mathsf T}n)e_i,
\qquad
\rho_i=-r\frac{w_i}{\|w_i\|},
\qquad
p_{c,i}=p_{h,i}+\rho_i.
$$

This is the lowest point of a rigid thin disk, not a finite-width tire contact-patch model.
The flat-ground checkpoint uses $n=[0,0,1]^{\mathsf T}$ and ground height zero.
The implementation divides by $\|w_i\|$, so an axle parallel to the ground normal is outside its valid geometry.
No special fallback is implemented for a wheel lying flat on its side.


### 4.2 Tread slip and acceleration

Let $\omega_i$ and $\alpha_i$ be the wheel link's full world-frame angular velocity and acceleration, including wheel spin.
The material velocity of the tread point currently touching the plane is:

$$
v_{c,i}=v_{h,i}+\omega_i\times\rho_i.
$$

This is not the velocity of the moving geometric contact location:

$$
\frac{d p_{c,i}}{dt}=v_{h,i}+\dot\rho_i.
$$

Under ideal rolling, the tread material velocity vanishes while the geometric contact location advances along the floor.
The offset derivative and the acceleration used in the constraint are:

$$
\dot e_i=\omega_i\times e_i,
\qquad
\dot w_i=-(\dot e_i^{\mathsf T}n)e_i-(e_i^{\mathsf T}n)\dot e_i,
$$

$$
\dot\rho_i=-r\left(\frac{\dot w_i}{\|w_i\|}-\frac{w_i(w_i^{\mathsf T}\dot w_i)}{\|w_i\|^3}\right),
\qquad
a_{c,i}=a_{h,i}+\alpha_i\times\rho_i+\omega_i\times\dot\rho_i.
$$

Here $a_{c,i}=d v_{c,i}(q,v)/dt$ follows the instantaneous contact evaluation, rather than the acceleration of one permanently marked tread particle.
This derivative assumes a fixed ground normal; there is no $\dot n$ term.
Pinocchio supplies the center's classical acceleration in world-aligned axes.


### 4.3 Five rolling rows per supporting wheel

Define the in-plane lateral and longitudinal directions:

$$
l_i=\frac{e_i-(e_i^{\mathsf T}n)n}{\|e_i-(e_i^{\mathsf T}n)n\|},
\qquad
t_i=n\times l_i,
\qquad
g_i=n^{\mathsf T}p_{c,i}-z_i^d.
$$

For each scheduled supporting wheel, the implementation enforces:

$$
\begin{aligned}
0&=k_{a,t}\,t_i^{\mathsf T}a_{c,i}+k_{v,t}\,t_i^{\mathsf T}v_{c,i},\\
0&=k_{a,n}\,n^{\mathsf T}a_{c,i}+k_{v,n}\,n^{\mathsf T}v_{c,i}+k_{p,n}\,g_i,\\
0_3&=m_i-\rho_i\times f_i.
\end{aligned}
$$

The current values are $k_{a,t}=k_{a,n}=1$, $k_{v,t}=k_{v,n}=50$ and $k_{p,n}=50$.
The first row damps longitudinal slip, the second corrects contact gap and normal slip, and the final three restrict the wheel-center moment.
These gains belong to equality constraints, not soft objective weights.
Finite gains correct nonzero slip/gap over time; they do not impose the separate algebraic conditions $v_c=0$ and $g=0$ at every state.
The normal row reads the swing planner's position reference, but does not subtract desired normal velocity or acceleration in its rolling branch.
Those desired derivatives are zero for the present flat, fixed-support checkpoint.

The moment relation makes the retained six-dimensional wrench equivalent to a point force:

$$
J_{c,i}=J_{v,h,i}-[\rho_i]_\times J_{\omega,i},
\qquad
J_{h,i}^{\mathsf T}\begin{bmatrix}f_i\\\rho_i\times f_i\end{bmatrix}=J_{c,i}^{\mathsf T}f_i.
$$

Unlike a planted humanoid sole, this model cannot choose an independent wheel-center couple or a center of pressure inside a rectangle.


### 4.4 One shared lateral row

Let $\gamma_i\in\{0,1\}$ be the scheduled contact flag and $N_s=\gamma_L+\gamma_R$.
When at least one wheel supports the robot, a single additional equality is:

$$
0=\sum_i\frac{\gamma_i}{N_s}
\left(k_{a,t}\,l_i^{\mathsf T}a_{c,i}+k_{v,t}\,l_i^{\mathsf T}v_{c,i}\right).
$$

The code uses the ground normal of the first scheduled supporting wheel for this shared calculation.
The shared row was introduced to avoid redundant lateral rows in the symmetric double-support projection.
It constrains the signed mean, not each wheel's lateral slip individually, so opposite residuals can cancel.
Each lateral direction comes directly from that wheel's axle without an additional common-sign alignment.
Individual wheel lateral slip and the projected equality rank therefore still require inspection, especially with mirrored axle signs, turning, asymmetry or different contact normals.
There are eleven wheel-contact equality rows in double support: five per wheel and one shared lateral row, in addition to the twelve RNEA rows.


### 4.5 Swing and flight support

For a scheduled swing wheel, the rolling rows are disabled and all six wrench entries are set to zero.
A vertical equality follows the swing planner:

$$
0=k_{a,n}(a_{h,i,z}-\ddot z_i^d)
+k_{v,n}(v_{h,i,z}-\dot z_i^d)
+k_{p,n}(p_{h,i,z}-r-z_i^d).
$$

This branch approximates tire-bottom height as center height minus radius, rather than using the tilted disk's full $\rho_i$ geometry.
It does not prescribe swing-wheel XY position or touchdown wheel spin.
During flight both wrenches are zero and the shared lateral row is inactive, but full-body inverse dynamics remains active.
These code paths exist, but the shipped Upkie gait library contains only double-support `stance`; hopping and alternating support are not established capabilities.


## 5. Tracking Costs and References


### 5.1 Base-motion objective

In `base_twist` mode, the horizontal position and yaw-angle errors are deliberately zero:

$$
e_{B,p}=\begin{bmatrix}0&0&z-z_d&0&\operatorname{wrap}(\theta-\theta_d)&\operatorname{wrap}(\phi-\phi_d)\end{bmatrix}^{\mathsf T}.
$$

Let $R_2(\psi)$ be the planar yaw rotation.
The base-velocity residual is:

$$
e_{B,v}=\begin{bmatrix}
R_2(\psi)^{\mathsf T}v_{B,xy}^{W}-R_2(\psi_d)^{\mathsf T}v_{B,d,xy}^{W}\\
v_{B,z}^{W}-v_{B,d,z}^{W}\\
\dot\eta-\dot\eta_d
\end{bmatrix},
\qquad
\ell_B=\frac12\|e_{B,p}\|_{Q_p}^2+\frac12\|e_{B,v}\|_{Q_v}^2.
$$

The current diagonal entries are:

$$
\operatorname{diag}Q_p=[0,0,100,0,20,50],
\qquad
\operatorname{diag}Q_v=[20,20,1,10,1,1].
$$

The tracked height is the base origin's height, not the whole-body CoM height, and the linear velocity is base velocity despite legacy accessor names containing `Com`.
Pitch and roll are softly stabilized while yaw rate is tracked without restoring a fixed yaw angle.
Returning the forward command to zero asks the robot to stop moving, not to return to a particular XY position.
There is no separate CoM-position, capture-point, centroidal-momentum or integral drift-rejection objective in this configuration.


### 5.2 Joint posture and motion

$$
\ell_j=\frac12\|q_j-q_j^d\|_{Q_j}^2+\frac12\|\dot q_j-\dot q_j^d\|_{Q_{\dot j}}^2,
$$

$$
q_j^d=[-0.6021397,-0.9655142,0,0.6021397,0.9655142,0]^{\mathsf T},
\qquad
\dot q_j^d=0,
$$

$$
\operatorname{diag}Q_j=[1,1,0,1,1,0],
\qquad
\operatorname{diag}Q_{\dot j}=[0.01,0.01,0,0.01,0.01,0].
$$

The wheel angles and spin speeds have zero tracking weights, allowing wheel rotation to emerge from base-motion tracking and rolling physics.
Wheel spin speeds still have soft velocity limits.
Upkie has no tracked arm joints, so the shared dedicated arm-position evaluator contributes no arm objective.


### 5.3 Input regularization

$$
\ell_u=\frac12\|u-u_d\|_R^2,
\qquad
R=10^{-3}\operatorname{diag}(R_W,R_{a_B},R_{a_j},R_\tau).
$$

The listed vectors are the unscaled diagonal entries:

| Block | Current entries |
| --- | --- |
| Each wheel wrench $[f_x,f_y,f_z,m_x,m_y,m_z]$ | $[0.01,0.01,0.001,0.001,0.001,0.001]$ |
| Base accelerations | Six entries of $0.005$ |
| Joint accelerations | Six entries of $0.005$ |
| Joint torques | $[0.01,0.01,0.1,0.01,0.01,0.1]$ |

The running input-cost evaluator constructs a weight-supporting reference independently of the zero input arrays stored in the target trajectory:

$$
f_i^d=\frac{\gamma_i}{N_s}\begin{bmatrix}0\\0\\mg\end{bmatrix}\quad(N_s>0),
\qquad
m_i^d=0,\qquad a_d=0,\qquad\tau_j^d=0.
$$

In flight, all nominal wrenches are zero.
The nominal wheel normal load in double support is approximately $26.8\ \mathrm{N}$ per wheel.
These are soft references, so the optimizer may shift loads and generate nonzero contact moments as required by $m_i=\rho_i\times f_i$.
The torque regularizer is on motor torque magnitude, not on torque changes between solves.
No first-node torque-rate or previous-torque matching cost is registered here.


### 5.4 Terminal cost and inactive additions

The terminal objective repeats the base, joint-position and joint-velocity residuals with Qf and no terminal input cost.
Qf has the same base and joint-velocity weights as Q, but joint-position weights $[2,2,0,2,2,0]$.
`qScaling` and `terminalCostScaling` are both $1$, while `rScaling` is $0.001$.
The builder partitions Q/Qf among base and generic evaluators without double counting.
Soft path-limit penalties are running terms; no separate terminal joint-limit barrier is registered by this setup.
Foot task-space tracking, named torso tracking, gravity-relative torso tilt, capture-foot placement, external leg-load regularization and arm swing are disabled or absent in the current Upkie YAML.
They are not hidden additions to the equations above.


### 5.5 Command conditioning

Normalized topic/GUI commands $c_x,c_y,c_\psi\in[-1,1]$ are scaled to:

$$
v_x^{\mathrm{cmd}}=1.0c_x,
\qquad
v_y^{\mathrm{cmd}}=0.1c_y,
\qquad
\dot\psi^{\mathrm{cmd}}=1.5c_\psi.
$$

Units are $\mathrm{m/s}$ for translation and $\mathrm{rad/s}$ for yaw rate; the height command is absolute, with nominal $z_d=0.30\ \mathrm{m}$.
The shared command filter and slew limiter condition the references, with rate limits $[0.5,0.5,0.1,2.0]$ for $[v_x,v_y,z,\dot\psi]$.
At each solve, the target trajectory re-anchors to the current base pose and integrates the conditioned command, with knots at $t_0$, $t_0+0.7T$ and $t_0+T$.
The velocity references have zero world vertical velocity and zero pitch/roll rates, even while a changing height command moves the height reference between solves.
Upkie supports forward driving, differential turning and leg-length height adjustment in principle, not independent sideways translation.
The GUI still accepts a nonzero lateral command without a wheel-specific rejection, so keep $c_y=0$.
Configured command maxima are not validated stability limits.


## 6. Soft Inequalities and Barrier Penalties


### 6.1 Ground-force cone

For a supporting wheel on the flat plane, the actual implemented residual is:

$$
h_{\mu,i}=\mu_f f_{i,z}-\sqrt{f_{i,x}^2+f_{i,y}^2+\varepsilon_f}\geq0,
\qquad
\mu_f=0.6,\qquad\varepsilon_f=25\ \mathrm{N^2}.
$$

The regularization is the shared class default, not an extra Upkie YAML setting, and the gripper-force offset is zero.
It implies $f_z\geq\sqrt{25}/0.6\approx8.33\ \mathrm{N}$ for a nonnegative residual at zero tangential force, rather than allowing exactly zero normal load in an active contact.
There is no separately registered hard unilateral-force row; nonnegative normal load is encouraged through this soft cone residual.
The penalty uses `frictionCone.mu: 0.1` and `delta: 1.0`, distinct from the physical cone coefficient $\mu_f$.
This optimizer coefficient is also distinct from MuJoCo floor friction, currently $[0.8,0.1,0.001]$ for sliding, torsional and rolling friction.
The thin-disk MPC does not explicitly reproduce all finite-patch and frictional-moment effects of the MuJoCo tire model.


### 6.2 Joint bounds

The residual convention is $h\geq0$:

$$
h_{q,j}^{-}=q_j-q_j^{\min},\qquad h_{q,j}^{+}=q_j^{\max}-q_j,
$$

$$
h_{v,j}^{\pm}=v_j^{\max}\pm\dot q_j,
\qquad
h_{\tau,j}^{\pm}=\tau_j^{\max}\pm\tau_j.
$$

Position, velocity and effort bounds are read from the generated URDF, with optional torque-array overrides and configured velocity/torque scaling.
Both scalings are $1.0$ in the current Upkie YAML.
The effective bounds are:

| Joint type | Position | Speed | Motor torque |
| --- | --- | --- | --- |
| Hip | $\pm3.05433\ \mathrm{rad}$ | $28.8\ \mathrm{rad/s}$ | $16\ \mathrm{Nm}$ |
| Knee | $\pm2.70526034\ \mathrm{rad}$ | $28.8\ \mathrm{rad/s}$ | $16\ \mathrm{Nm}$ |
| Wheel | Unbounded in the MPC | $111\ \mathrm{rad/s}$ | $1.7\ \mathrm{Nm}$ |

The URDF's placeholder wheel-position bounds $\pm10^4$ are replaced by infinities before constructing the position penalty.
The builder requires both running and terminal wheel-position weights to be zero, and controller configuration rejects a nonzero wheel position-servo gain.


### 6.3 Actual penalty functions

Friction, velocity and torque residuals use the relaxed logarithmic barrier:

$$
B_{\mu,\delta}(h)=\begin{cases}
-\mu\log h,&h>\delta,\\
\mu\left[-\log\delta+\dfrac12\left(\dfrac{h-2\delta}{\delta}\right)^2-\dfrac12\right],&h\leq\delta.
\end{cases}
$$

Joint-position residuals use the piecewise polynomial penalty:

$$
P_{\mu,\delta}(h)=\mu\begin{cases}
\dfrac12h^2-\dfrac\delta2h+\dfrac{\delta^2}{6},&h\leq0,\\
\dfrac{(\delta-h)^3}{6\delta},&0<h<\delta,\\
0,&h\geq\delta.
\end{cases}
$$

The running soft objective sums these penalties over the active residuals:

$$
\ell_{\mathrm{soft}}=
\sum_{i:\gamma_i=1}B_{0.1,1.0}(h_{\mu,i})
+\sum_{j\in\mathrm{hip,knee}}\left[P_{1200,0.1}(h_{q,j}^{-})+P_{1200,0.1}(h_{q,j}^{+})\right]
+\sum_{j=1}^{6}\sum_{\sigma\in\{-,+\}}\left[B_{0.1,1.0}(h_{v,j}^{\sigma})+B_{0.05,0.2}(h_{\tau,j}^{\sigma})\right].
$$

These per-term penalty scales are not multiplied by `rScaling` or `qScaling`.
Both functions remain finite outside the feasible region, so they steer the optimizer but do not guarantee bounded torques, bounded speeds or collision-free execution.
There are no active arm/leg self-collision costs, environment-avoidance terms, humanoid sole CoP bounds, or hard torque inequalities in the current Upkie setup.


## 7. Gait, Initialization and Execution


### 7.1 Current support schedule

The default and only configured gait is `stance`:

$$
\gamma_L(t)=\gamma_R(t)=1.
$$

Here stance means continuous rolling support, not a planted or motionless wheel.
No gait switch is required to accelerate, brake or turn.
The `/humanoid/gait_command` string topic can request an existing template, but it does not itself create a hop or single-support template.
Scheduled contact flags activate optimizer terms; measured contact sensors currently provide diagnostics rather than automatically rewriting the support schedule.


### 7.2 Initial policy seed

The initializer seeds weight-supporting wrenches and estimates joint torques from inverse dynamics at zero generalized acceleration:

$$
\tau_{j,\mathrm{seed}}=S_j\left[\operatorname{RNEA}(q,v,0)-\sum_iJ_{h,i}^{\mathsf T}W_i^d\right].
$$

It holds the seed state unchanged for its next-state output.
This is an initialization guess, not a separately solved feasible equilibrium: the initializer does not explicitly add the smoothed friction correction or project every wheel-contact equality.
The default spawn is the balanced crouch at base height $0.3005\ \mathrm{m}$, approximately $0.5\ \mathrm{mm}$ above the flat contact pose.
With GT feedback and an empty `standUpTrajectoryFile`, startup uses this standing spawn instead of the planned kneeling stand-up.


### 7.3 Low-level joint servo and torque clipping

With `feedforward: policy_torque`, the controller evaluates planned joint positions, velocities and torques and clips the feedforward to the configured effort bounds.
The simulator executes:

$$
\tau_{\mathrm{ff}}=\operatorname{clip}(\tau_j^*,-\tau^{\max},\tau^{\max}),
$$

$$
\tau_{\mathrm{motor}}=\operatorname{clip}\left[
\tau_{\mathrm{ff}}+K_p(q_j^*-q_{j,\mathrm{meas}})+K_d(\dot q_j^*-\dot q_{j,\mathrm{meas}}),
-\tau_{\mathrm{motor}}^{\max},\tau_{\mathrm{motor}}^{\max}\right].
$$

The final clipping in this expression is supplied by the current unit-gear MuJoCo motors with the actuator effort mode, not by the soft OCP barrier.
The configured gains are:

$$
\operatorname{diag}K_p=[15,15,0,15,15,0],
\qquad
\operatorname{diag}K_d=[0.5,0.5,0.05,0.5,0.5,0.05].
$$

Wheel feedback tracks planned spin speed with zero angle stiffness; hips and knees track both position and velocity.
This inner servo is distinct from the optional SQP feedback policy, which is disabled here.
The scene uses a $0.0005\ \mathrm{s}$ physics step and `implicitfast`, so the hardware PD is evaluated at $2\ \mathrm{kHz}$ while controller targets update at $1\ \mathrm{kHz}$.
The simulator calls hardware `write()` every physics step, outside the rate-limited controller-manager `read()`/`update()` block, and the PD reads live MuJoCo positions and velocities on each call.
Between MRT ticks, the targets, feedforward and gains are held, but the feedback torque is recomputed; this is the same `effort_pd` path used by H2.
MuJoCo joint friction acts in the simulation after the motor command, while the MPC's smooth friction model anticipates its torque demand.
A model-based feedforward fallback also adds the configured smoothed joint friction.
Changing to direct `qfrc_applied` bypass would invalidate the actuator-clipping interpretation above.


### 7.4 Optional planned stand-up

An optional trajectory-file path implements idle hold, planned stand-up playback, policy handover, blend and locomotion.
The height command filter resets to the measured base height at handover rather than stepping immediately to the nominal $0.30\ \mathrm{m}$ height.
This playback is not a stand-up trajectory optimized online by the MPC.
It is implemented but has not yet been validated in the complete controller/simulator stack; the default configuration leaves it disabled.


## 8. Implementation Map and Validation Boundary

| Part | Code |
| --- | --- |
| State/input layout | [WbIdMpcRobotModel.h](../include/humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h) |
| Configuration, weights and wheel-angle masking | [whole_body_inverse_dynamics_mpc_config_builder.cpp](../src/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.cpp) |
| OCP registration and activation | [WbIdMpcInterface.cpp](../src/core/humanoid_wb_id_mpc/src/WbIdMpcInterface.cpp) |
| Double-integrator propagation | [WbIdDynamics.cpp](../src/core/humanoid_wb_id_mpc/src/dynamics/WbIdDynamics.cpp) |
| Armature import | [createPinocchioModel.cpp](../src/core/humanoid_common_mpc/src/pinocchio_model/createPinocchioModel.cpp) |
| RNEA and Coulomb friction | [WbIdRneaConstraintCppAd.cpp](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdRneaConstraintCppAd.cpp) |
| Shared disk/contact geometry | [WheelContactKinematics.h](../include/humanoid_wb_id_mpc/common/WheelContactKinematics.h) |
| Rolling, shared lateral and swing equalities | [WbIdWheelContactConstraintCppAd.cpp](../src/core/humanoid_wb_id_mpc/src/constraint/WbIdWheelContactConstraintCppAd.cpp) |
| Relative base-motion residual | [BaseMotionTrackingCost.cpp](../src/core/humanoid_common_mpc/src/cost/BaseMotionTrackingCost.cpp) |
| Input nominal and generic cost | [StateInputQuadraticCost.cpp](../src/core/humanoid_common_mpc/src/cost/StateInputQuadraticCost.cpp) |
| Friction cone and its regularization | [FrictionForceConeConstraint.h](../include/humanoid_common_mpc/constraint/FrictionForceConeConstraint.h), [implementation](../src/core/humanoid_common_mpc/src/constraint/FrictionForceConeConstraint.cpp) |
| Position, friction and terminal cost construction | [HumanoidCostConstraintFactory.cpp](../src/core/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp) |
| Command integration and state reference | [WbIdMpcTargetTrajectoriesCalculator.cpp](../src/core/humanoid_wb_id_mpc/src/command/WbIdMpcTargetTrajectoriesCalculator.cpp) |
| Initial state/input seed | [WbIdInitializer.cpp](../src/core/humanoid_wb_id_mpc/src/initialization/WbIdInitializer.cpp) |
| Observation, execution, stand-up and tracking diagnostics | [humanoid_whole_body_inverse_dynamics_mpc_controller.cpp](../src/humanoid_whole_body_inverse_dynamics_mpc/humanoid_whole_body_inverse_dynamics_mpc_controller.cpp) |

The handoff records preliminary flat-ground balance, forward drive, braking and small push recovery with simulator base feedback.
Repeated acceptance trials, turning and height sweeps, full-stack kneeling stand-up, hopping, sustained single support, terrain adaptation and estimator-based control remain unvalidated.
Logging includes base motion, planned wrenches, feedforward/applied joint effort, measured wheel contacts/normal forces, contact gaps, tread slip and solver residuals.
The slip/gap diagnostics currently assume a horizontal zero-height plane; they are not a generic arbitrary-terrain contact estimator.
Documented equations describe the implementation, not a guarantee that a one-iteration SQP policy satisfies every equality or that soft limits remain unviolated.
