# Wheeled Biped

This research survey and design proposal is not the current implementation specification.
See the [formulation](../../wheeled_biped_whole_body_inverse_dynamics_mpc.md) and [handoff](../../wheeled_biped_handoff.md) for the current code and results.


## MuJoCo model and whole-body inverse dynamics MPC survey

**Best fit.** Use Upkie’s six-actuator model, your Jazzy/H2 WB-ID MPC, and wheel-contact constraints adapted from the ETH and Xin–Vijayakumar papers. Use SeBaJu and TITA as references for the jump sequence. This gives you a full rigid-body MPC that computes all six joint torques and retains your OCS2 SQP, Pinocchio/CppAD, ROS 2 control, and MuJoCo pipeline. I found no public example that combines this exact hardware layout, full-order WB-ID MPC, and all three requested gaits in MuJoCo.

## Quick survey

| Project or paper | What it provides | What I would use |
|---|---|---|
| **Upkie** | Exactly two leg joints and one driven wheel per side. Apache-2.0 model assets. The separate ProxQP MPC balances the robot with a reduced model. The public MuJoCo model ships with position servos on the legs and velocity servos on the wheels. | **Primary model.** Start from the [URDF](https://github.com/upkie/upkie_description/blob/94735fbe6137276a41de0ff4cc04d2e533fa9e33/urdf/upkie.urdf) and [MJCF scene](https://github.com/MarcDcls/mjlab_upkie/blob/d7895789a601943f78b3b9bcdc4def7b444f9913/src/mjlab_upkie/robot/upkie/scene.xml). The [MPC balancer](https://github.com/stephane-caron/mpc_balancer) is a possible balance baseline. |
| **SeBaJu** | Six actuators and a public MuJoCo balance, jump, and landing demo using position/velocity feedback with a phase supervisor. | Reference for preload, thrust, liftoff, touchdown detection, and recovery. [Repository and MJCF](https://github.com/k3nkreate/SeBaJu_BOT_5845) |
| **TITA — Paradiso, Sapienza master’s thesis** | Eight actuators, including one hip-abduction joint per leg. Public C++ MuJoCo stack with online reduced-order NMPC using Crocoddyl FDDP, followed by inverse-dynamics WBC. Demonstrates rolling and obstacle-clearing jumps. I found no explicit repository license. | Reference for a rolling-to-jump trajectory and phase-dependent objectives. [Repository](https://github.com/Emilianogith/TITA-dynamic-obstacle-avoidance) · [Thesis](https://github.com/Emilianogith/TITA-dynamic-obstacle-avoidance/blob/main/assets/paradiso_1940454.pdf) · [MJCF](https://github.com/Emilianogith/TITA-dynamic-obstacle-avoidance/blob/main/TITA_MJ/tita_mj_description/tita_world.xml) · [URDF](https://github.com/Emilianogith/TITA-dynamic-obstacle-avoidance/blob/main/TITA_MJ/tita_description/tita.urdf) |
| **Bjelonic et al. — *Offline motion libraries and online MPC for advanced mobility skills*, IJRR 2022** | Offline trajectories composed into maneuvers and tracked by online MPC re-optimizing from measured state on wheeled ANYmal. | Use a small library of feasible takeoff and landing references once the basic jump works. The online controller can track these references with your full-body model. [Paper](https://journals.sagepub.com/doi/10.1177/02783649221102473) |
| **Bjelonic et al. — *Whole-Body MPC and Online Gait Sequence Generation for Wheeled-Legged Robots*, IROS 2021** | Wheeled ANYmal uses SLQ with single-rigid-body dynamics and whole-body kinematics. Wheel contacts move along the ground and the MPC optimizes joint velocities and ground reaction forces. | Use the rolling-contact formulation as a reference. Keep explicit wheel spin and torque in your WB-ID model. [Paper](https://arxiv.org/abs/2010.06322) |
| **Xin & Vijayakumar — *Online Dynamic Motion Planning and Control for Wheeled Biped Robots*, IROS 2020** | Cart-LIPM for rolling and an underactuated LIPM for lateral stepping, tracked by inverse-dynamics WBC. The simulated robot has an additional actuated ankle-roll axis. I could not verify a controller or model release. | Closest reference for rolling with one wheel in support and for acceleration-level rolling constraints. Use Upkie’s actual joint reachability when adapting the gait. [Paper](https://arxiv.org/abs/2003.03678) |
| **Wang et al. — *Design and Control of SKATER*, IEEE/ASME T-Mech** | Six-DoF wheeled biped with hierarchical MPC/WBC for high-speed turns and terrain adaptation. I could not verify public MuJoCo assets or control code. | Closest peer-reviewed example by actuator count and rolling task. [Paper](https://doi.org/10.1109/TMECH.2024.3420390) |
| **Qin et al. — *Mobile Pedipulation for Object Sliding via Hierarchical Control on a Wheeled Bipedal Robot*, IEEE RA-L 2026** | Trajectory optimization, reduced-order three-rigid-body NMPC, and WBC for rolling and contact transitions. TRON1 wheel-foot has eight actuators and public models. I could not verify a controller release. | Reference for contact-mode planning and force tracking. [Paper and demos](https://yueqin20.github.io/mopedipulator/) · [URDF/MJCF](https://github.com/limxdynamics/tron1-robot-description) |
| **Khazoom et al. and Molnar et al.** | Full-order inverse-dynamics MPC for legged robots, including the formulations your controller builds on. | Direct references for the OCP and torque-window strategy. [Khazoom et al.](https://arxiv.org/abs/2407.10789) · [Molnar et al.](https://arxiv.org/abs/2511.19709) |

## Starting model

I loaded the upstream Upkie scene in MuJoCo 3.17 and stepped it headlessly. It compiled with **13 position coordinates, 12 velocity DoFs, and six actuators**. The extra position coordinate comes from the floating-base quaternion. The two wheels have opposite joint-axis signs at the nominal pose. Both MJCF collision cylinders have a **55 mm radius** and **44 mm width**. The upstream actuator force limits are **±16 N·m** for each hip/knee and **±1.7 N·m** for each wheel.

I also loaded the upstream URDF in Pinocchio 4.1 with the Jazzy branch’s translation + SphericalZYX root. It gives **$n_q=n_v=12$** with the same six joint names. Three integration details matter before tuning.

- **Match the two model descriptions.** I measured 5.4605 kg in the MJCF versus 5.33922 kg in the URDF. The URDF tire collision radius is 50 mm and the MJCF uses 55 mm. Take the MJCF as the simulation reference and reconcile the MPC URDF’s masses, centers of mass, inertias, wheel center/offset and radius against it. Compare forward kinematics, joint signs, gravity torques and the mass matrix at several mapped poses. The URDF’s existing contact frames rotate with the wheels. Use the wheel-center frames and compute the contact offset online. Account for MJCF joint armature, passive friction and damping explicitly when comparing dynamics.
- **Keep scalar, unwrapped wheel angles in the MPC.** The upstream URDF encodes the wheels as revolute without position bounds. My Pinocchio load returned zero lower and upper bounds for both wheels. Explicitly exclude the wheel angles from position barriers, nominal posture costs and position feedback. Keep spin-speed and torque limits. A URDF change to `continuous` produced `nq=14, nv=12` in my check and would break the current `nq == nv` mapping. Retain the scalar representation in the MPC model unless you also change the state integration and mappings.
- **Expose six effort commands.** Change the project’s MJCF variant to unit-gear torque motors with matched limits. Start from the upstream effort limits and a ground-truth floating-base feed. Map actuator channels by name, then confirm signs with a small unloaded wheel-speed and torque test. Drive wheels with effort plus optional speed damping, with zero wheel position gain. Verify that position/velocity feedback plus feedforward respects the final effort limit.

To preview the upstream assets, clone the [MuJoCo model repo](https://github.com/MarcDcls/mjlab_upkie) and run this command from its root with MuJoCo installed.

```bash
python -m mujoco.viewer --mjcf=src/mjlab_upkie/robot/upkie/scene.xml
```

## Full-order MPC formulation

Reuse the H2 structure with the local Euler-coordinate representation, so $x=[q,v]$ has 24 entries. With point forces, the input is $u=[f_L,f_R,a,\tau]$ with 24 entries: six contact-force components, 12 generalized accelerations, and six torques. For the first implementation, keep the branch’s wrench-based input layout, which becomes 30 entries on Upkie with two six-dimensional wrenches at the wheel centers. Constrain their moments to the force-generated moments described below. This reduces code changes and gives the same physical contact model.

Use the branch’s double-integrator state dynamics in its local coordinates. At each node impose

```math
M(q)a+h(q,v)-S^\top\tau
-\sum_{i\in\{L,R\}} J_{c,i}(q)^\top f_i = 0.
```

Here $S$ selects the six actuated coordinates and $J_{c,i}$ maps generalized velocity to the **material velocity at the instantaneous contact point**. Keep all 12 RNEA rows across the horizon for the initial six-actuator implementation. The current H2 setup keeps base rows across its 1 s horizon and actuated rows within a ten-node torque window. For jumping, the full horizon makes the planned thrust and landing respect the actuator model even when those phases are far ahead. The existing branch uses soft effort-limit penalties. For feasibility checks, inspect actual violations and consider hard effort inequalities for takeoff/landing, with matching simulator caps.

### Replace the planted-foot model with rolling wheels

Use a rigid thin-disk approximation on flat ground initially. Let $p_h$ be the tire-center position, $e$ the unit axle direction, $n$ the unit ground normal, and $r$ the tire radius. With $J_{v,h}$ the world-aligned linear Jacobian at the tire center and $J_\omega$ the wheel angular Jacobian, including wheel spin,

```math
\begin{aligned}
\rho &= -r\frac{n-(e^\top n)e}{\|n-(e^\top n)e\|}, &
p_c &= p_h+\rho, \\
J_c &= J_{v,h}-[\rho]_\times J_\omega, &
v_{\mathrm{material}} &= J_c v.
\end{aligned}
```

For a supporting wheel, keep the gap $g=n^\top(p_c-p_{\mathrm{ground}})$ close to zero, enforce low lateral and longitudinal slip, and constrain $f_n\ge 0$ and $\|f_t\|\le \mu f_n$. An acceleration-level form can use $J_c a+\dot J_c v$, differentiating the full configuration-dependent $J_c$, with velocity stabilization and normal gap feedback. $\dot p_c$ tracks the moving geometric contact location. $J_cv$ evaluates the tread material velocity at that location. In straight no-slip rolling the first is generally nonzero and the second is zero.

To keep the existing RNEA wrench interface, use a wheel-center frame and enforce $m_h=\rho\times f$ in the same world-aligned basis as $J_h$. Replace the sole CoP and free-moment constraints. In swing or flight, zero both force and moment for the affected wheel. Keep wheel height/clearance and landing objectives on the tire geometry, and regulate touchdown spin so tangential tread speed is small. For the Upkie cylinder, contact can move toward the tread edge when the base rolls. Check the MuJoCo contact point, slip, and force traces before increasing lean. Also check the rank of the stacked rolling Jacobian in symmetric double support, where lateral rows can be redundant. Use independent equality rows or soften the redundant slip row if equality projection becomes singular.

### Map to the Jazzy branch

| Existing entry point | Wheeled Biped change |
|---|---|
| `WbIdMpcRobotModel.h`, `WbIdDynamics.cpp`, `WbIdRneaConstraintCppAd.cpp` | Keep the scalar state layout and full rigid-body residual. Use wheel-center contact frames and the moment–force relation, or map point forces directly through $J_c^\top$. Retain all actuated rows for the first tuning pass. |
| `WbIdMpcInterface.cpp`, `WbIdFootAccelerationConstraintCppAd.cpp` | Register wheel rolling and swing-clearance rows in place of the planted-foot motion rows. Reuse phase-gated zero wrench and friction. Remove sole CoP and sole-flat orientation assumptions for this robot. |
| `GaitSchedule`, `SwitchedModelReferenceManager`, `SwingTrajectoryPlanner` | Reuse the two-bit left/right support masks and swing timing. Add wheel-height, touchdown spin and jump references. A planted-foot horizontal reference or lateral capture-foot offset can be infeasible with pitch-only legs. |
| `WbIdInitializer.cpp`, H2 controller effort output, and H2 simulation launch | Seed feasible contact forces and torques from the intended phase. Reuse policy torque output and ground-truth debugging. Log wheel slip, normal forces, contact gaps, actual flight time, torque saturation and solver residuals. |

[Jazzy source checked at ae134c0](https://github.com/wei-hsuan-cheng/legged_robot_mpc_controller/tree/ae134c06940bf114dbd57417b7ba1ec9a1407348) · [WB-ID formulation](https://github.com/wei-hsuan-cheng/legged_robot_mpc_controller/blob/jazzy/docs/humanoid_whole_body_inverse_dynamics_mpc.md) · [Implementation guide](https://github.com/wei-hsuan-cheng/legged_robot_mpc_controller/blob/jazzy/docs/humanoid_whole_body_inverse_dynamics_mpc_impl.md) · [H2 running checkpoint](https://github.com/wei-hsuan-cheng/legged_robot_mpc_controller/blob/jazzy/docs/wb_id_running.md)

## Gait bring-up

Here **trot** means alternating left/right support on a biped. The first version should include a double-support transfer between single-support phases. Add brief aerial intervals only after validating single support and landing separately.

| Order | Gait and support schedule | First experiment and pass condition |
|---|---|---|
| **1** | **Wheel drive** · both wheels in rolling contact | Start with equilibrium and pitch-disturbance recovery. Then track straight velocity, brake, reverse and turn. Check bounded pitch, little tread slip, and effort limits. |
| **2** | **Jump** · both support → flight → both support | Start with a small vertical hop from rest using symmetric preload and extension. In flight set both contact forces to zero. Plan takeoff and landing speed with the real effort caps. Check that both wheel contacts are physically absent during flight and that landing recovers without saturation or tip-over. Add a forward rolling jump afterward. |
| **3** | **Trot** · both → left → both → right → repeat | Start with one short swing or wheel lift while the other wheel rolls. In single support the body must manage lateral balance using its available motion and contact forces. The model has no hip roll and cannot independently choose lateral foot placement. Check roll, stance friction, attainable swing clearance and actual contact timing before attempting a sustained alternating gait. |

**Touchdown treatment.** For the first small hop, use compliant MuJoCo contact, a low touchdown normal velocity, phase detection from contact measurements, and replan from the measured state at landing. If hard landing consistently breaks the prediction, add a touchdown velocity reset with a contact impulse, $M(q)(v^+-v^-)=J_c^\top\Lambda$, with a friction and impact law appropriate to the selected contact mode. The branch’s scheduled H2 flight can still produce grounded running in MuJoCo, so verify flight from simulated contact measurements.

**First implementation milestone.** Run one repeatable episode that balances, rolls forward, brakes, performs a small hop, lands and resumes rolling. Keep the same full-order OCP through the episode and switch only the contact mask and references. Run a separate alternating-support feasibility test before relying on trot. With two pitch joints per leg and no hip abduction, that gait is the least established part of the proposal.

**Verification scope.** I loaded the unmodified upstream MJCF in MuJoCo, loaded the upstream URDF in Pinocchio, and inspected the Jazzy source. The torque-mode model changes and wheeled WB-ID controller are a proposed port. Closed-loop wheel drive, jump and alternating support with that port still need validation.
