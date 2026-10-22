# Centroidal MPC + InEKF tuning — working log

Companion to `Codex.md`. One section per phase, appended as work lands, so the
state of the effort is traceable without re-reading diffs.

Plan phases (agreed order):

| Phase | Content | Status |
|---|---|---|
| 0 | Instrumentation. No behaviour change. | **done, validated on real runs** |
| 1 | Estimator correctness: A1–A4, A7 | **done, validated** |
| 2 | Contact model (B1, A8) | A8 answered (torque detection is structurally broken for G1); B1 open |
| 2.5 | Height pseudo-measurement inside the filter | **now the top priority** — it is what blocks terrain walking |
| 3 | Reference / cost consistency | partly done: foot placement added, walking much improved |
| 4 | Parameter sweeps | done for foot placement; `trackingWeight` was the dominant knob |
| 5 | Hardware readiness (A5, GT-free init) | not started |

## Current status

**No falls in 16 ladder runs, 4 stop_cycles runs and 2 figure-eight runs.** This is
the first configuration in this effort that does not fall, and the first whose
per-batch results do not swing between 0/6 and 2/4.

| test | result |
|---|---|
| vx ladder, 200 s, N=8 | **0/8** |
| vx ladder, 102 s, N=8 | **0/8** |
| stop_cycles (8 walk->stop cycles/run), N=4 | **0/4** |
| figure-eight 0.22 m/s, 2 laps, N=2 | **2/2 survived** |

Measured on the 200 s ladder, N=8, 32 walk windows, 40 stop holds:

| quantity | value |
|---|---|
| trunk pitch sd | 0.0110 +- 0.0047 rad |
| trunk roll sd | 0.0667 +- 0.0096 rad |
| pelvis z sd | 0.0056 +- 0.0002 m |
| pitch drift | 0.0008 +- 0.0010 rad/s (bounded) |
| z drift | 0.0003 +- 0.0002 m/s (bounded) |
| settled tilt at stop holds | 0.034 +- 0.033 rad (2.0 deg) |
| estimator \|e_pitch\| | 0.0074 +- 0.0051 rad |
| estimator \|e_z\| | 0.00027 +- 0.00012 m |
| estimator roughness / GT | 1.00 +- 0.13 |
| command / joint 8-60 Hz amplitude | 0.00324 / 0.00304 rad |

Against the previously shipped configuration this is: falls 6/29 -> 0/16, pitch sd
0.0138 -> 0.0110, settled tilt 0.216 -> 0.034, estimator \|e_pitch\| 0.0113 ->
0.0074, and on `stop_cycles` 1/3 -> 0/4 with settled tilt 0.566 -> 0.028.

Every number above is with the **closed-form inverse-dynamics inner loop**
(`control.jointCommandMode: inverse_dynamics`). The WBC alternative on
`feat/wbc` has no comparable walking measurement yet - see the WBC section
below.

**Shipped configuration** (`config/g1/ros2_controllers.yaml`):

```yaml
control:
  jointCommandMode: inverse_dynamics
stateEstimator:
  height:  { source: kinematic }
  contact: { source: scheduled }
ocs2:
  reference:
    defaultBaseHeight: 0.7925
    defaultJointState arms: [-0.30, +-0.20, 0.0, 1.20]
  costs:
    icpErrorWeight: 0.0
    captureFootPlacement:
      enabled: true
      gain: 1.0
      stepWidth: 0.18
      maxAdjustment: 0.20
      trackingWeight: 100.0
      stepLengthGain: 0.0
      footCenterOffset: 0.075
```

## WBC inner loop: working, 6/6 on the vx ladder

The centroidal MPC optimises foot wrenches and swing trajectories; those were
handed to a closed-form RNEA inverse dynamics (`computeJointTorques`), which
evaluates `tau = C(q,qd)qd + g(q) - J^T F` at the *planned* wrench and applies it
open loop. The original OCS2 paper uses a WBC there instead. This adds one, on
branch `feat/wbc`, following the method in `qiayuanl/legged_control` adapted from
4 point contacts to 2 flat 6-DoF contacts.

**It is selectable, and the ID path remains the default** in
`ros2_controllers.yaml`:

```yaml
control:
  jointCommandMode: inverse_dynamics   # or: wbc
```

The ID path is also the per-tick fallback: if the QP fails to converge on a tick,
that tick falls through to inverse dynamics and a throttled warning counts it. A
WBC must never throw in the control loop.

### The QP

One dense QP per control tick over

```
x = [ qddot (nv=29) | w_l w_r (6 each) | tau (nj=23) ]   -> 64 variables
```

| | rows, double support | rows, single support |
|---|---|---|
| floating-base EoM `M qddot + nle = J^T w + S^T tau` | 29 eq | 29 eq |
| stance no-motion `J qddot + Jdot v = 0` (all 6 rows) | 12 eq | 6 eq |
| zero wrench on the swing foot | 0 eq | 6 eq |
| **equalities** | **41** | **41** |
| contact cone, 11 per stance foot | 22 ineq | 11 ineq |
| torque box bounds | 23 | 23 |
| weighted objective rows | 93 | 93 |

The equality and objective row counts are the same in both phases by
construction - a foot either carries a wrench and cannot move, or moves and
carries none - which is why only three distinct problem shapes ever occur and the
HPIPM workspaces are rebuilt essentially never after startup.

The 11 cone rows per stance foot, with `f = w.head(3)`, `m = w.tail(3)` in
LOCAL_WORLD_ALIGNED:

| rows | constraint |
|---|---|
| 1 | unilateral, `-f_z <= -minContactForceZ` |
| 4 | friction pyramid, `+-f_x - mu f_z <= 0`, `+-f_y - mu f_z <= 0` |
| 4 | centre of pressure inside the sole rectangle |
| 2 | torsional friction, `+-m_z - mu_t f_z <= 0` |

CoP is `p_x = -m_y/f_z`, `p_y = m_x/f_z`, bounded by the foot rectangle from
`ContactPolygon::getBounds()` shrunk by `copMarginScale`. Multiplying through by
`f_z > 0` keeps the rows linear.

Weighted objectives, stacked with sqrt-weights so a plain row stack reproduces the
weighted sum of squares: base acceleration PD on the policy pose/twist, joint
acceleration PD, swing-foot 6-D PD (orientation error via `pinocchio::log3`),
stance-wrench tracking toward the MPC's optimised wrench, and a torque/
acceleration regulariser that also keeps `H` positive definite.

### Four things that differ from the quadruped reference

1. **6-DoF flat contacts, not 3-DoF points.** A point contact needs only the
   unilateral and friction rows. A flat foot carries moment, so it gains the CoP
   and torsional-friction rows, and the contact Jacobians are the full 6 rows -
   including in the no-motion constraint, because a stance foot must not rotate
   either.
2. **Two contacts, not four.** The biped has no stance redundancy to spare: in
   single support one foot carries everything, which is exactly why the CoP bound
   matters here and is optional on a quadruped.
3. **Base/joint acceleration PD instead of a centroidal-momentum-rate task.** The
   reference differentiates the desired input numerically to get `hdot_ref`. The
   MPC policy here already supplies a full state and input trajectory, so an
   acceleration-level PD on it avoids that derivative entirely.
4. **HPIPM, not qpOASES.** qpOASES is not in this workspace; HPIPM already comes
   in through `ocs2_sqp`, so `CMakeLists.txt` only needed `ocs2_sqp` added to
   `legged_robot_humanoid_common_core`.

### HPIPM does not want infinite box bounds

The first working build produced **19 QP convergence failures in plain stance**.
The cause was giving all 64 variables a two-sided box bound with `+-inf` on the
ones that are genuinely unbounded. HPIPM's interior-point method then chases
infinite slacks and stalls.

The fix is to express "no bound" by *omission* rather than by infinity:
`boundedIndices` lists only the 23 torque variables, and the one-sided
inequality rows use `d_dense_qp_set_lg_mask` / `ug_mask` with a zero lower mask
instead of `lg = -inf`. **19 failures -> 0.** Worth remembering for any future
HPIPM use in this workspace.

### Rate

The WBC runs at **1 kHz, the same rate as the MRT policy evaluation**, not the
100 Hz MPC rate. `controller_manager.update_rate` is `$(var mrtFreq)` = 1000, and
`update_and_write_commands` -> `compute_mpc_joint_action` -> `wbc_->update` is
unconditional, with no decimation. Closing on the measured state is the whole
point of the inner loop, so running it slower than the control tick would defeat
it.

A headless stance run reached 228.3 s of sim time inside a 230 s wall timeout at
`mujoco_real_time_factor: 1.0`, so the 64-variable QP plus two Pinocchio passes
fit the 1 ms budget on average. Two caveats: double support is the *largest* QP,
so walking is not heavier; and in simulation a slow controller does not miss a
deadline, it slips behind real time - there is no RT-overrun warning to look for.
Hardware needs an actual per-tick solve-time histogram before this is trusted. If
timing ever binds, the `updateDesired` Pinocchio pass is the cheaper thing to cut
(it feeds only the swing and wrench references, which change at 100 Hz anyway).

### Measured on the vx ladder, flat scene, GT base

`6/6 runs survived, 0 QP failures`, and the spread is tight enough to be a
property of the configuration rather than luck - which matters on a ladder this
file elsewhere describes as near-bimodal.

| | ID (shipped) | WBC |
|---|---|---|
| ladder outcome | 2/2 survived | **6/6 survived** |
| QP failures | - | **0** |
| peak pitch | 0.059-0.062 | 0.063-0.071 |
| peak roll | 0.063-0.066 | 0.067-0.072 |
| rest-phase pitch sd | 0.0010-0.0020 | **0.0004** |

Commanded vs achieved forward speed, body frame, mean over the last 60% of each
phase:

| cmd vx | ID | WBC |
|---|---|---|
| +0.05 | +0.01 | +0.02 |
| +0.10 | +0.05 | **+0.06** |
| +0.20 | +0.10 | **+0.11** |
| +0.30 | +0.14 | **+0.16** |
| -0.10 | -0.18 | -0.19 |
| every rest phase | -0.00 | **-0.00** |

So the WBC tracks velocity slightly *better* than the closed-form inverse
dynamics, is quieter at rest, and is marginally worse in peak pitch.

### The weight ratio is the whole ballgame

`weightContactWrench` **must dominate** `weightBaseAccel`. The first guess had it
the other way round (200 : 1) and that configuration **cannot come to rest**:

| | base 200 / wrench 1 | base 20 / wrench 100 |
|---|---|---|
| rest-phase speed | **+0.07 to +0.08 m/s** | -0.00 m/s |
| ladder outcome | **fell 4/4**, 42.6-43.1 s | survived 6/6 |

The mechanism is a feedback loop, not a tuning preference. The MPC has already
optimised the balancing wrench over its whole horizon; at 200 : 1 the QP throws
that away and substitutes a crude base PD, which cannot bring the robot to a
stop. It then creeps forward through every commanded stop - and
`transitionToSlowerGait` gates entry to stance on the *measured* speed, so a
creeping robot never stops stepping, keeps creeping, and goes over at the second
stop. Every fall landed in `rest2`.

The WBC's job is to **realise** the MPC's wrench against the measured state
subject to the physical limits, not to re-derive the balance. The weights now say
so, and the reasoning is recorded in the config next to the numbers because this
is exactly the sort of ratio that gets "tidied" back later.

### The QP settings were unreachable, which hid everything else

`WeightedWbc`'s constructor called `make_unique<HpipmDenseQp>()` with no argument,
so `settings_.qp` was silently discarded and the solver always ran at the
hardcoded `iterMax: 30` with 1e-6/1e-7 tolerances. That is far too tight for a
control QP: once the friction cone and the torque bounds are both active the
interior-point method needs more steps, and a QP that stops at the iteration
limit is reported as a failure.

| | before | after |
|---|---|---|
| `iterMax` | 30 | 100 |
| `tolStat` | 1e-6 | 1e-4 |
| `tolEq` | 1e-7 | 1e-6 |
| HPIPM mode | `speed` | `balance` |
| failures on the ladder | 73304 (~37% of ticks) | 1931 -> 0 |

`tolEq` stays tightest because those rows are the equation of motion and the
stance constraint - the physics, not a preference. `SPEED` drops safeguards that
matter once several inequalities are active at once, which here is every
single-support tick. All seven settings are now `wbc.qp.*`, so tuning needs no
rebuild.

**The warning now names the HPIPM status** (`MAX_ITER after 100 iterations`) via
`HpipmDenseQp::statusString`. Without it a throttled "did not converge" with a
silent 37% fallback to inverse dynamics is indistinguishable from a healthy WBC -
which is precisely how the stance-only "0 failures" result below misled this
effort for an afternoon.

### Torque limits: needed, and not binding

With `control.torqueLimit` set, no joint ever exceeds its bound and 0 of 23 are
over at any sample. The tightest sustained user is the ankle at 78% of 50 Nm. The
shoulders touch 25.0 Nm but at a **0.018% duty cycle** against a 0.45 Nm median,
i.e. a one-or-two-tick transient at activation, and the arms move only 1.5 deg
peak-to-peak. So the bounds are doing their job without shaping the gait.

### What was NOT the WBC's fault

**The ~50% forward velocity deficit is a property of the MPC, not the inner
loop.** The ID baseline on the identical harness achieves `0.01 / 0.05 / 0.10 /
0.14` against commands of `0.05 / 0.10 / 0.20 / 0.30`, and -0.18 against a -0.10
command - i.e. forward tracks at about half and backward overshoots by ~80%. The
WBC reproduces the same pattern. Nothing in this file had measured commanded vs
achieved speed before, so this was invisible; it is now open item 2.

The per-axis `baseKp` change (zero on x, y, yaw) measured **neutral** - it did not
move the fall time at all. It is kept because zero gain on the gauge states is
correct in principle and mirrors the MPC's own `q.basePose` of
`[0,0,100,0,100,100]`, but it should not be cited as the fix.

### Three harness traps found while measuring this

1. **The first "successful" walking batch was a harness bug.** It reported
   `SURVIVED t=228.3s dist=0.09m` twice, which reads like a pass until the
   distance is noticed: the script passed `--sequence there_and_back`, which
   `command_sequence.py` does not accept, so it died on the argparse error and
   **no velocity command was ever published**. Check `<tag>_cmd.log` for argparse
   errors before reading any result; valid names come from `--list`.
2. **`controllersFile` needs an absolute path.** A bare filename throws
   `[Errno 2] No such file or directory` and the harness then waits out its whole
   activation timeout against a sim that does not exist.
3. **`torqueLimit: []` is rejected by rclcpp** - `parameter_value_from failed:
   No parameter value set`, because an empty YAML sequence has no inferable type.
   "No limits" means omitting the key, not emptying it.

Also worth knowing: **`command_sequence.py` has `NOMINAL_HEIGHT = 0.6`**, so every
sequence in this repo commands a 0.600 m pelvis against the MPC's 0.7925 m
nominal. The knee torques (max 22.6 Nm of 139) prove no deep squat actually
happens, which independently confirms open item 7 - the height command is not
tracked. It applies identically to both inner loops, so it does not bias the
comparison, but no ladder result in this file is at the height it appears to
command.

### Config

`config/g1/ros2_controllers_wbc.yaml` is byte-identical to the shipped config
except for the inner loop. The gain change is not cosmetic:

| key | `ros2_controllers.yaml` | `ros2_controllers_wbc.yaml` |
|---|---|---|
| `jointCommandMode` | `inverse_dynamics` | `wbc` |
| `mpcJointKp` (x23) | 1200.0 | **0.0** |
| `mpcJointKd` (x23) | 10.0 | 2.0 |

With the ID inner loop, `effort_pd` runs `kp(q_des-q) + kd(qd_des-qd) + tau_ff`
and at kp = 1200 that stiff joint-space PD does most of the work, with the RNEA
feedforward as a correction. The WBC computes the *complete* joint torque, so any
kp on top of it is a second controller fighting the first. Hence kp = 0, with
kd = 2.0 left as light actuator damping rather than a tracking term.

All 16 `wbc:` settings are now written out in that file at their defaults
(friction 0.7, torsional 0.1, `minContactForceZ` 10, `copMarginScale` 0.9, base
100/20, joint 100/20, swing 350/37, weights 200/1/100/1/1e-3/1e-4), so tuning
needs no rebuild. The ratio to move first is `weightBaseAccel : weightJointAccel`,
currently 200:1 - that is what decides whether the solver spends the redundancy
holding the base or holding the nominal posture.

**`control.torqueLimit` was empty, and for the WBC that is not a missing clamp.**
It means the `tau` block of the decision vector had **no box constraints at all**,
so the QP could return a torque no G1 actuator can produce. It is now set from the
`effort` limits in `g1_29dof.urdf`: 88 hip / 139 knee / 50 ankle, 88/50/50 waist,
25 arms. Deliberately left out of `ros2_controllers.yaml` so the ID configuration
that measures 0/16 falls is untouched.

### Files

| file | what |
|---|---|
| `include/humanoid_common_mpc/whole_body_control/Task.h` | task algebra, `A x = b` / `D x <= f`, `operator+` and sqrt-weight `operator*` |
| `.../HpipmDenseQp.h/.cpp` | HPIPM dense-QP wrapper, workspaces rebuilt only on shape change |
| `.../WeightedWbc.h/.cpp` | `WbcSettings`, the nine task formulations, `update()` |

Note the branch currently has `g1.launch.py` defaulting `controllersFile` to
`ros2_controllers_wbc.yaml`, so a plain launch brings up the WBC and the ID
baseline needs `controllersFile:=ros2_controllers.yaml`. **Flip that back before
this merges to main**, since walking under the WBC is unvalidated.

## Jazzy: the estimators are ros2_control controllers (branch `jazzy`)

The `jazzy` branches of all four repos build on ROS 2 Jazzy (ros2_control 4.48,
Pinocchio 4.1, coal, GCC 13) in the `ros_jazzy` container. On this branch
`HumanoidInEkfController` and `HumanoidLinearKfController` are separate chainable
controllers, and the section below describes `main` (Humble), where the filter
runs inside the MPC.

**Data flow.** Both directions are chained interfaces the estimator exports, and
only the MPC claims them (layout in `estimator_state_interfaces.hpp` - one visitor
generates the state names, `pack` and `unpack`; `reference_names()` lists the
inputs). Estimator -> MPC: the estimate, status and scalar diagnostics as state
interfaces. MPC -> estimator: `contact.left`, `contact.right` and
`initialize_epoch` as reference interfaces the MPC writes as command interfaces
(the MPC writes exported epoch + 1 and waits until the exported epoch matches
before trusting `status.ready`). The controller manager runs the two one after the
other per tick, so one direction is a tick late. Earlier versions carried the
return leg over two latched topics, then over an in-process shared buffer.

**What cannot be chained is a cycle, not two-way data.** Jazzy's
`check_following_controllers_for_activate` recurses into every controller whose
exported interfaces a controller claims, with no visited set, so two controllers
claiming each other's interfaces (state or reference - both are edges) recurse
without end at activation. One controller exporting both kinds and the other
claiming both is fine. Claiming an active controller's reference interfaces makes
the controller manager restart it to switch it into chained mode.

**FLY again.** Before the MPC drives (Blend/Locomotion), the MPC writes both
contact flags as stance rather than reading the reference manager's schedule,
which can still be OCS2's default single-mode FLY schedule.

**Not carried across:** the per-contact diagnostics columns (innovation,
landmarks), whose names depend on the estimator's contact frames, which the MPC
cannot know when it declares its claims. Everything else in the state CSV is
unchanged.

**Jazzy port traps.**
- Controllers no longer inherit the controller manager's `--params-file`; each
  reads only `<name>.params_file`. The launch evaluates the substituted yaml once
  and passes it, keeping the `ParameterFile` alive (it deletes the temp file
  when collected).
- `hpp-fcl` is `coal`; Pinocchio 4's pre-4 header paths are only on the
  `pinocchio::pinocchio` target (`include/pinocchio/deprecated`); `Frame::parent`
  is `parentJoint`.
- `mujoco_ros2_control` must mark its resource manager initialized, or the 4.x
  controller manager loads the URDF a second time itself.

**Measured** (flat scene, InEKF, crouched GT-free bring-up, ID inner loop):

| | combined | robustness |
|---|---|---|
| Humble, embedded estimator | 8.07-8.21 m, pitch 0.040-0.049 | 12.74-12.96 m, pitch 0.079-0.081 |
| Jazzy, embedded estimator | 2/2, 8.15-8.16 m, pitch 0.047-0.052 | 2/2, 12.72-12.94 m, pitch 0.088-0.096 |
| Jazzy, estimator controller | 2/2, 8.07-8.11 m, pitch 0.046-0.050 | 2/2, 12.79-12.92 m, pitch 0.080-0.089 |

While walking under the estimator controller: |e_z| 0.5 mm, pitch 0.004 rad,
roll 0.007 rad, velocity ~0.016 m/s RMS against GT. `ground_truth_state` with the
estimator logged alongside: survived. Linear KF controller: loads and completes the
bring-up; fell at 97 s on `combined` (N=1, its documented weakness).

## State estimator: in the controller, two separate filters, InEKF by default

**Structure.** `HumanoidInEkf` and `HumanoidLinearKf` are two separate classes
(`src/humanoid_state_estimation/humanoid_{inekf,linear_kf}.cpp`), each declaring
only its own parameters under its own prefix on the MPC node, from its own file
(its own block of `config/g1/ros2_controllers.yaml`). The MPC controller builds the one
named by `floatingBase.estimator` and runs it as the **first step of every
control tick**, so the observation is always built from that tick's estimate.
`floatingBase.source` has exactly two values, `ground_truth_state` and
`state_estimator`. The URDF and joint list are handed over by the controller, so
there is no second copy of the joint order.

**Why not separate ros2_control controllers.** That was built and then dropped.
On Humble (ros2_control 2.53.1) a controller can export only reference
interfaces, never state interfaces, and the dependency is a cycle (the filter
needs the gait's contact flags, the MPC needs the filter's pose) that chaining
cannot express. A hardware component carrying the estimate worked, but needed
`mujoco_ros2_control` patched to load it, which says it was the wrong
abstraction. Both reference stacks do it this way instead: legged_control and
legubiao/quadruped_ros2_control keep the estimator as a library inside the one
controller, picked by a parameter and updated first in `update()`. On Jazzy,
`on_export_state_interfaces()` is what would make a separate estimator
controller standard.

**Behaviour is unchanged from before the refactor.** A/B against `HEAD`
(`9f62fd1`) built as an overlay, each with its own shipped config, ID inner loop,
`state_estimator`, linear KF, flat scene. All 182 shared config keys compared
equal; the 43 moved estimator values match exactly.

| sequence | `HEAD` fell at | new code fell at |
|---|---|---|
| robustness | 36.6, 68.1 s | 58.8, 49.8 s |
| combined | 63.0, 66.9 s | 65.5, 46.0 s |

**The falls are the linear KF.** On `robustness` and `combined`, linear KF
driving control: 0/8 (both code versions). InEKF: 6/6 (`robustness` 4/4, peak
pitch 0.08; `combined` 2/2, peak pitch 0.045). So the default is now
`humanoid_inekf`; `HEAD` shipped the linear KF.

**`control.mpcJointKp` / `mpcJointKd` / `fixedJointKp` / `fixedJointKd` are
dead.** They are read into `mpc_joint_kp_` etc. at configure and never read
again; the controller writes only `q_des`, `qd_des`, `tau_ff`. The joint PD that
actually runs is `mujoco_ros2_control`'s `effort_pd` servo, at the physics rate,
with gains from the URDF xacro (`effort_pd_kp:=1200.0`, `effort_pd_kd:=10.0` in
`g1.ros2_control_macro.xacro`). Consequences:

- The "kp 0 vs kp 1200" runs in this section changed nothing - every run had
  1200/10. They count as repeats, not as a gain test.
- **Every WBC result above ran with the 1200/10 joint PD on top of the QP
  torque.** "The WBC needs kp = 0" was never in effect; the 6/6 result is
  WBC torque plus a stiff joint servo to the policy, not a torque-only WBC.
- The jitter section's "PD damping: no effect ... identical to three
  significant figures across a 6x change" is what changing a dead parameter
  looks like. Re-check whether that sweep edited `mpcJointKd` or the xacro.

**Readiness is a timer, not a covariance test.** The hand-off to the estimate
waits `readiness.minSeconds` (5 s, the validated warm-up) and then latches. A
covariance gate cannot certify convergence for this filter: it is seeded from
ground truth with a small initial covariance, so it starts confident, and the
biases barely move while standing (gyro-bias std 0.0100 -> 0.0089 rad/s over 5 s,
accelerometer bias flat). The std checks are kept only as divergence guards, on
observable states only (roll/pitch, height, velocity). Yaw and x/y are gauge
states; yaw std reached 0.53 rad late in a run, and gating on it blocked the
hand-off entirely the first time this was tried.

**Open: WBC plus estimator creeps at rest.** The WBC was only ever validated on
ground truth. With the InEKF driving it, the robot survives the vx ladder but
creeps at up to 0.16 m/s through commanded stops, while the estimate itself is
accurate to 0.003 m/s per phase. The likely path is the estimate's transient
velocity spikes (up to 0.16 m/s at contact switches) through `baseKd = 20`. The
ID inner loop with the same estimator stops dead.

**Traps found on the way.**
- The diagnostic CSVs filled the VM disk to 100% again (3.4 GB in `/tmp`); the
  harness now deletes each run's CSV after scoring it.
- A *clean* build of this package can OOM at `NUM_JOBS=2` (two heavy CppAD core
  sources at once on 7.5 GB). Incremental builds are fine; use `NUM_JOBS=1` for
  a clean one.
- `docker cp` drops the executable bit on the replaced file.
- `pkill -f "<pattern>"` inside `bash -c` matches its own command string.
- With `--symlink-install`, deleted source files leave dangling install links and
  stale pluginlib index entries.

## Bring-up without ground truth: crouch -> stand-up -> estimator -> MPC

With `floatingBase.source: state_estimator` nothing in the control path reads
ground truth any more, so the same controller runs on hardware. The previous
arrangement (GT drives the observation until the estimator is ready, then a
switch) was not deployable: a real robot has no GT to start from.

**The sequence follows MIT Cheetah** (`FSM_State_Passive` -> `FSM_State_StandUp`
-> `FSM_State_Locomotion`, where StandUp is a pure PD interpolation that does
not use the state estimate) and legubiao's `StatePassive` -> `BaseFixedStand` ->
`StateOCS2`. Adapted to a humanoid: it starts **crouched with the feet flat**,
not lying flat. Cheetah "lying" is belly down with the legs folded under it and
the feet already on the ground; a humanoid lying on its back needs a whole-body
get-up (rolling, hands, knees), which joint interpolation cannot do. A squat
with hip = ankle = -0.7, knee = 1.4 keeps the soles flat and puts the CoM at
x = +0.061 m inside the sole span [-0.011, +0.129] (FK), so it is statically
stable.

| phase | command | estimator |
|---|---|---|
| Passive | damping (q_des = measured) | off |
| StandUp | joint PD to `defaultJointState`, gravity comp. from the **IMU attitude** | off |
| Settle | hold | off - waits for the trunk at rest **and** `~/start_locomotion` / autoStart |
| Converge | hold | `initializeFromStance()`, then 5 s |
| StartingMpc | hold | solver built off the RT thread, seeded from the estimate |
| Blend / Locomotion | stand -> MPC over 0.5 s / MPC | running, contacts from the gait |
| Fault | damping, latched | tilt > 0.8 rad |

**Measured, standing spawn:** passive 0.0 -> settling 3.0 -> converging 3.8 ->
MPC 8.8 -> blend 9.6 -> locomotion 10.1 s, then `combined` survived. The
self-initialised filter matches the old GT seed: height std 0.014 vs 0.011 m,
velocity std 0.132 vs 0.132 m/s at hand-off. Crouched spawn: same sequence from
`Initial base height: 0.6440 m`.

**Three bugs this flushed out:**

1. **OCS2's default `ModeSchedule()` is the single mode 0, `FLY`.** Read before
   the solver exists, it tells the filter both feet are in the air: every
   contact correction is skipped and the filter integrates the IMU open-loop
   (height std 1.3 -> 101 m in 23 s; the guards correctly refused the hand-off).
   The GT path hid it because building the solver immediately overwrote the
   schedule. Before the MPC runs the bring-up now asserts both feet in stance.
2. **"At rest" must be tested on the trunk, not the joints.** Holding the stand,
   the ankles chatter at ~80 Hz (0.14-0.17 rad/s RMS) while the base moves at
   3-6 mm/s - a joint-velocity threshold never passes. The test is IMU
   zero-velocity detection: low-passed gyro < 0.10 rad/s and |accel| within
   0.5 m/s^2 of g.
3. **Low-pass the vectors, then take the norm.** The norm of g plus a zero-mean
   vibration is biased up (~sqrt(g^2 + sigma^2)); filtering |accel| read
   11.6 m/s^2 at rest.

**The ankle chatter itself is pre-existing and open.** The `effort_pd` servo
runs kp 1200 / kd 10 explicitly at 2 kHz on joints with effective inertia down
to ~0.0004 kg m^2, so kd*dt/I ~ 12 - well past where an explicit damping term
is stable. It also makes a held stand creep (~2.7 mm/s). Worth a look before
reading anything into high-frequency ankle data.

**Sim spawn height.** `mujoco_ros2_control` gained an optional
`initial_base_height` hardware parameter (set in `set_initial_pose()`, next to
the joint values), fed by `g1_base.z` in the initial pose file through the
xacro. `initial_pose_crouch.yaml` carries 0.644; the standing file carries none
and spawns at the MJCF's 0.793 as before.

**Validated in sim (InEKF, ID inner loop, flat scene, autoStart unless noted):**

| case | result |
|---|---|
| crouched spawn, `combined` x2 + `robustness` x2 | 4/4 survived |
| standing spawn, `combined` x2 + `robustness` x1 | 3/3 survived |
| `ground_truth_state` regression after the bring-up changes | survived |
| linear KF through the new bring-up, `combined` | survived (N = 1; it fell 8/8 under the old GT-seeded start) |
| forced fault (`tiltLimit` 0.001) | tripped on entering StandUp, damping latched |
| manual flow, deliberately out of order | early `start_locomotion` refused and not remembered; `stand_up` accepted; second `stand_up` refused; `start_locomotion` during stand-up refused; at rest accepted; then `combined` survived |
| no GT claimed at all (`stateInterfaceName: ""`) | full bring-up with 0 GT interfaces claimed; 1 run, **fell** 75 s in, during `all_low` (lateral + yaw), scored by the simulator's own odom topic |

The no-GT fall is not yet attributed. The control path reads no GT in either
configuration, and the InEKF on `combined` had already fallen once in 10 runs
with GT claimed (`strafe_turn`, also lateral + yaw - the documented weak
spot), so the base rate explains it. Three repeats were started and stopped
before finishing; rerun them before calling the no-GT case validated.

**Two bugs the manual test found, both fixed:** `passive` was pure damping, so a
crouched robot folded up in it (below 0.55 m within 0.3 s) - it now holds the
activation pose on the joint servo with no feed-forward, safe on a hoist or on
the floor; and the services latched calls made in the wrong phase, so an early
`start_locomotion` would have initialised the estimator in the air - each call
is now valid only in its own phase and refused otherwise.

The sim spawns crouched by default (`initial_pose_crouch.yaml`).

**Hardware procedure:** `bringUp.autoStart: false`,
`floatingBase.stateInterfaceName: ""`; `~/stand_up`, lower onto the feet,
`~/start_locomotion`. The estimator initialises only on the second call because
its stance initialisation assumes the feet are on the ground.

## Angular momentum reference: now correct, and it costs something

**The formulation was wrong.** The centroidal state carries `h_ang / m`, and
`h_ang = I_G(q) * omega` - not `omega`. `targetMomentum` was built as

```cpp
targetMomentum << vx, vy, 0.0, 0.0, 0.0, yawRate / mass_;   // wrong
```

which is dimensionally wrong, pins the x/y angular components to zero, and for
the G1 is numerically wrong by `1 / I_zz`. Measured on the model,
`I_zz = 0.539 kg m^2`, so:

| | commanded h_ang,z / m |
|---|---|
| old | 0.02848 * omega_z |
| correct | 0.01535 * omega_z |

Every commanded turn asked for **1.86x the angular momentum the motion actually
carries**, on a terminal component with weight 75. It is now
`I_G(q) * omega / m`, evaluated at the current configuration (I_G is
configuration dependent) and using the full product, so the inertia-product terms
that a yaw rotation of an asymmetric body genuinely produces are included
(`I_xz = 0.043 kg m^2` here - small, but the old code implicitly set it to zero).

**The other axes were checked, not assumed.** The linear x/y target is already
correct in form: normalized linear momentum IS the CoM velocity, so commanding
`(vx, vy)` is right. The `z` component is 0, which is *consistent* with the pose
reference holding a constant commanded height - its derivative is zero. The
height-tracking weakness comes from that pose reference being a step, not from the
momentum term, and is a separate item. The residual linear defect is the
pelvis-vs-CoM conflation (`v_CoM != v_pelvis`), also separate.

### It measures neutral-to-worse, and that is not a contradiction

| sequence | before | with the correct formula |
|---|---|---|
| vx_ladder | 0/16 | 0/4 |
| yaw_ladder | 0/4 | 0/4 |
| combined | 0/4 | 0/4 |
| figure-eight | 2/2 (dist 1.9-2.2 m) | 2/2 (**dist 7.0-7.8 m**) |
| vy_ladder | 2/4 | 3/4 |
| **stop_cycles** | **0/4** | **3/10** |

Turning and combined motion - what the yaw reference most directly drives - are
unaffected, and the figure-eight covers **3-4x the distance per lap**, i.e. it is
tracking the path rather than crabbing. But `stop_cycles` is 3/10 against 0/4, and
settled tilt is 0.068 +- 0.30 against 0.028 +- 0.017.

**Fisher exact on 3/10 vs 0/4 gives p ~ 0.51 - not significant.** Both arms are
too small to separate 0% from 30%, which needs 10-15 samples per arm. What can be
said is that no measured indicator improved except the figure-eight distance, and
several point estimates and spreads moved the wrong way.

The likely reason is not that the physics is wrong now - it is that **everything
downstream was tuned against the inflated reference.** The `h_ang` terminal weight
of 75 was fitted while the target was 1.86x too large; correcting the target
changes the effective authority of that term without changing its weight.

**This is Phase 2 step 1.** The plan already names the `h_ang = 0` conflict as the
gate on the arm work; this is the same knot from the other side. Retuning the
momentum weights against the corrected reference has to happen before the arms can
be made to regulate angular momentum, and it should recover the `stop_cycles`
result.

## Leg jitter: four hypotheses tested, all negative

The reported symptom was leg jitter while walking, suspected to come from the
velocity term added to the RNEA feedforward. Measured with an 8-60 Hz band
amplitude on commanded leg TORQUE (jitter is a torque phenomenon; joint angle
only shows what survives the leg's inertia), plus a 4-8 Hz band added later.

**1. Inverse-dynamics velocity: NOT the cause.** With qdd = 0, RNEA gives
`tau = C(q,qd)*qd + g(q) - J^T F`, so the velocity enters through the Coriolis
term. Now selectable via `control.inverseDynamicsVelocity`:

| mode | falls | torque jitter 8-60 Hz |
|---|---|---|
| measured | 4/8 | 1.467 +- 0.38 |
| policy | 1/8 | 1.463 +- 0.31 |
| zero | 2/8 | 1.430 +- 0.21 |

Across 24 runs spanning no velocity term, the noisy measured one and the smooth
planned one, torque jitter is flat. **Removing the term entirely changes
nothing**, so it cannot be the jitter source. (The fall-rate spread hinted that
`measured` is worse for stability, p ~ 0.28, but that comparison ran at height
0.75 / offset 0.005 and did not survive the config change - `measured` measures
0/8 under the shipped settings. Treat it as confounded.)

**2. Angular momentum formulation: no effect.** 3/18 falls either way over 18 runs
per arm, torque jitter 1.431 vs 1.447. The correct `I_G(q)*omega/m` form is
therefore free, which also retrospectively explains the `stop_cycles` scare
(3/10 vs 0/4, p ~ 0.51) as noise.

**3. PD damping: no effect.** The `effort_pd` loop runs
`kp*(q_des-q) + kd*(qd_des-qd) + tau_ff` with kp = 1200 and kd = 10 uniform across
23 joints whose effective inertia spans 0.0004 to 0.914 kg m^2 - a 2000x range.
That looked like an obvious defect. Replacing kd with per-joint values matched to
`M_ii` did nothing:

| kd | falls | 4-8 Hz tau | 8-60 Hz tau | tau_rms | pitch sd |
|---|---|---|---|---|---|
| uniform 10 | 0/8 | 1.599+-0.20 | 1.322 | 7.089 | 0.01039 |
| zeta = 0.7 | 0/8 | 1.577+-0.17 | 1.324 | 7.036 | 0.01035 |
| zeta = 1.0 | 0/8 | 1.592+-0.20 | 1.347 | 7.068 | 0.01038 |

Identical to three significant figures across a 6x change in damping.

**Two corrections to that analysis, both mine.** First, the MuJoCo model already
applies `damping = 10.0` on every joint, which the zeta calculation ignored: the
true damping is `kd + 10`, so the hips sit at **zeta = 0.30, not the 0.15** first
claimed, and the knee at 0.85 is nearly critical. The "severely underdamped" read
was overstated by 2x. Second, the 4-8 Hz band was added specifically to catch a
hip resonance at 5.8 Hz that the original 8-60 Hz window sat above - but since
damping does not move that band either, its content is **gait harmonics** (0.67 Hz
fundamental, harmonics 6-12 land in 4-8 Hz), i.e. real walking motion rather than
ringing.

**4. Policy time offset: the one knob that mattered.** `policyTimeOffset` evaluates
the MPC policy ahead of the current observation to compensate control latency:

| offset | falls | 8-60 Hz tau | joint jitter | cmd jitter |
|---|---|---|---|---|
| 0.005 | 0/8 | 1.471 | 0.00317 | 0.00352 |
| **0.000** | **0/8** | **1.358** | **0.00296** | **0.00309** |
| 0.010 | **6/8** | 31.0 +- 64 | 0.0227 | 0.0438 |

Zero is 8% quieter in torque and 12% in command than 0.005, at the same fall rate,
and **0.010 is a cliff** - 6/8 falls with 20x the torque jitter. The shipped 0.005
was one step away from it. Now 0.000.

**Where this leaves the jitter.** Nothing on the MPC side and nothing in the PD
loop moves it. Combined with the earlier finding that command and joint feedback
agree within 3.6% in every band from 0 to 100 Hz, the measured torque "jitter" of
~1.4 Nm against a 7.0 Nm RMS is most likely the normal broadband content of
walking on this model, not a defect. If the jitter is visible on real hardware but
not reproducible here, the difference is in what the simulation does not model:
actuator dynamics, transmission compliance, encoder quantisation and the real
500 Hz control loop's latency.

## Read this before quoting any single run

Two independent findings say single-run numbers from this stack are not
measurements:

1. **The vx ladder is close to bimodal.** Two identical runs of the same
   configuration gave "survived 197.6 s" and "fell at 54.8 s". The previously
   shipped configuration measures 2/3 falls at N=3.
2. **Every "SURVIVED t=..." figure earlier in this file is one sample of that
   distribution.** They are kept for history but should not be compared against
   each other.

Everything in the current-status table is a mean with its spread over N=6 runs,
and outcomes are reported as rates. `launch/analyze_diagnostics.py` and the
per-phase quality metrics (drift / sd / hf / band-limited roughness) exist for
this reason.

## The fix that mattered: foot support polygon vs contact frame

**The contact frame is the ankle; the sole is not centred on it.** Measured from
the G1 model, the foot spans **-0.055 m at the heel to +0.125 m at the toe** about
the ankle - the toe reach is **2.27x** the heel reach.

`updateCaptureFootholds()` placed the *ankle* under the predicted base position,
which gives a CoM travelling forward 0.125 m of support margin and a CoM
travelling backward only 0.055 m.

The data matched that exactly before the fix: with foot placement enabled,
forward walking at 0.05-0.30 m/s never fell across six runs, and **4 of 4 falls
were 1.6-3.8 s into the -0.10 m/s phase**, clustered at 84.6-86.8 s.

Subtracting `footCenterOffset` along the heading moves the foot behind the base and
buys rear margin. The static-geometry value that centres the polygon is
`(0.125 - 0.055) / 2 = 0.035 m` - and that turned out to be **measurably too
small**, because the requirement is dynamic, not static. Swept on the ladder,
N=8 per cell:

| offset [m] | ladder falls | pitch sd | settled tilt | vx_030 front margin |
|---|---|---|---|---|
| 0.035 | 2/8 | 0.0138 | 0.185 | -0.114 |
| 0.055 | 1/8 | 0.0125 | 0.105 | - |
| **0.075** | **0/8** | 0.0110 | 0.0488 | -0.093 |
| 0.095 | 0/8 | 0.0099 | 0.0475 | -0.086 |
| 0.115 | 0/8 | 0.0097 | 0.0417 | -0.075 |

**The offset is not a margin trade.** Raising it improved the FORWARD margin too
(-0.114 -> -0.075 at 0.30 m/s) while adding rear margin, so it is changing the
gait dynamics rather than translating the support polygon. That is also why the
statically-derived 0.035 was wrong.

Gains saturate past 0.075 while roll sd slowly degrades (0.0662 -> 0.0698), so the
choice between 0.075 / 0.095 / 0.115 was made on the scenario that was weakest
rather than on the ladder, where all three are 0/8:

| | stop_cycles falls | settled tilt | roll sd | figure-eight |
|---|---|---|---|---|
| **0.075** | **0/4** | **0.0284 +- 0.017** | **0.0721** | 2/2 survived |
| 0.095 | 1/4 | 0.0445 +- 0.14 | 0.0750 | 2/2 survived |

**0.075 is shipped.** 0.115 was rejected despite equal ladder performance: it puts
the foot ~0.14 m behind the CoM, a large behavioural change that a straight-line
0.05-0.30 m/s ladder cannot probe, and there is no model explaining why it would
be safe outside that range.

The original N=3 comparison that first established the effect, at offset 0.035
against none:

| | falls | pitch sd | roll sd | z sd | settled tilt |
|---|---|---|---|---|---|
| offset 0.000 | 2/3 | 0.0559+-0.14 | 0.202+-0.50 | 0.0272+-0.075 | 0.552+-1.0 |
| offset 0.035 | **0/3** | **0.0138+-0.005** | **0.0607+-0.012** | **0.0056+-0.0002** | **0.054+-0.046** |

It removed the FORWARD falls entirely and tightened every spread by roughly an
order of magnitude - the configuration became *repeatable*, which is the part luck
does not produce. It reduced but did NOT eliminate the backward falls: the pooled
rate over 29 runs is 6/29, all in `vx_neg10`. The 0/3 cell above is one batch and
should be read with the per-batch spread in "Current status".

## Two things measured and found NOT to be problems

**Pelvis height was not too high.** A first measurement suggested the commanded
0.7925 m exceeded full leg extension; that was wrong, from a sole-height estimate
that mishandled the foot geometry. Correctly: straight legs put the pelvis at
**0.8096 m**, so 0.7925 m corresponds to **0.47 rad (27 deg) of knee flexion** -
a normal walking posture, not a locked leg. Sweeping the height confirmed it
empirically: with the foothold fix in place, 0.7925 measured 0/6 falls against
1/6 at 0.7500 in that batch, with statistically indistinguishable walk quality.
(Both numbers are single batches - see "Current status" for why that matters - but
the point stands that lowering the height bought nothing.) Height was never the
problem; the foot placement geometry was.

**The MPC command does not chatter.** An RMS-second-difference metric reported the
command as 4-6x "rougher" than the joint feedback. That metric weights a component
at frequency f by f^2, so it is dominated by whatever sits nearest Nyquist -
and logging a 1 kHz policy evaluation at 200 Hz puts exactly such a component in
the command and not in the physical joint. A band-resolved comparison shows command
and feedback agreeing **within 3.6% in every band from 0 to 100 Hz**:

| signal | 0-3 Hz | 3-10 | 10-30 | 30-60 | 60-100 |
|---|---|---|---|---|---|
| knee cmd | 2.31e-1 | 2.26e-2 | 9.73e-3 | 5.30e-3 | 4.04e-3 |
| knee fb | 2.30e-1 | 2.23e-2 | 9.80e-3 | 5.12e-3 | 3.90e-3 |

The metric was measuring the logger. It is now band-limited to 8-60 Hz.

## The walk->stance fix: correct, but its behavioural effect is UNMEASURED

`transitionToSlowerGait()` tested the *command's* yaw rate where it must test the
*measured* yaw rate:

```cpp
bool baseSpeedSlowEnough = (std::abs(baseVelocity(0))  < ... &&
                            std::abs(baseVelocity(1))  < ... &&
                            std::abs(velCommandVec(3)) < ...);   // <- the command
```

`slowerGaitRequested`, evaluated immediately above, has already established the
command is near zero, so the yaw term was trivially true whenever the operator
stopped commanding and the guard degenerated to "vx and vy are small". Stance
cannot take a recovery step, so entering it with residual momentum leaves a lean
nothing can correct. The sibling `transitionToFasterGait()` uses `baseVelocity`
for all three terms, and the variable is called `baseSpeedSlowEnough`. It is a
copy-paste slip and the fix is not in doubt.

**What IS in doubt is whether fixing it changes the outcome.** An A/B on the new
`stop_cycles` sequence (8 walk->stop cycles per run, varied direction and speed),
N=3 per arm, same build otherwise:

| | falls | pitch sd | roll sd | settled tilt |
|---|---|---|---|---|
| fixed | 1/3 | 0.101+-0.20 | 0.460+-0.80 | 0.566+-1.0 |
| reverted | 2/3 | 0.020+-0.023 | 0.072+-0.017 | 0.126+-0.45 |

Fewer falls with the fix, *worse* motion statistics - and every spread is as large
as or larger than its mean. **This is an underpowered experiment and it supports
no conclusion in either direction.** N=3 with 1-2 falls per arm cannot separate a
real effect from which runs happened to fall early. The fix is kept because the
code defect is unambiguous, not because this measured it.

Two things to do properly later: rerun at N>=10 per arm, and hard-link the CSVs so
fall times are recoverable (they were deleted here, so the contradiction between
fall count and motion statistics could not be traced to when each fall occurred).

## stop_cycles is much harder than the ladder, and is the next target

The shipped configuration falls ~21% of the time on the vx ladder (all backward)
but the same build is 1/3 on `stop_cycles`, whose stops follow lateral (vy = 0.15), turning (yaw = 0.30) and
combined commands that the ladder never issues. Settled tilt is 0.566 rad against
0.046 on the ladder.

So the ladder numbers describe forward/backward walking only, not the robot in
general. Lateral and turning stops are the weakest measured behaviour and are
where the next round of work belongs.

## captureFootPlacement must stay enabled

Measured head-to-head at N=3, arms P4, ICP off:

| | falls | where | pitch RMS | roll noise |
|---|---|---|---|---|
| enabled | 2/3 | backward phase only | 0.043-0.045 | 0.055-0.057 |
| disabled | 2/3 | **forward 0.20 / 0.30 m/s** | 0.115 | 0.34-0.38 |

With it enabled, forward walking never failed in six runs. With it disabled,
forward walking itself breaks. It supplies the only foothold xy reference on flat
ground, so without it nothing arrests a lateral fall.

## Arm posture (phase 1 of the arm work)

**Problem.** The nominal arm posture was all-zero, and on the G1 that is not
"arms down". Measured on the model, the all-zero pose puts the hands **0.20 m in
front of the pelvis and 0.095 m above it**, because the forearm link extends
+0.1 m in x from the elbow: the arms are folded out horizontally, not hanging.
That holds **7.04 kg of arm mass at +0.063 m forward** of the pelvis.

**Fix.** The arm block of `reference.defaultJointState` becomes a hanging posture:

| joint | old | new |
|---|---|---|
| shoulder pitch | 0.0 | 0.0 |
| shoulder roll | 0.0 | +-0.20 |
| shoulder yaw | 0.0 | 0.0 |
| elbow | 0.0 | 1.20 |

which puts the wrist at (0.073, 0.220, -0.061) relative to the pelvis - beside and
just below it - and drops the arm-CoM forward offset to **+0.025 m, a 61%
reduction**. `config/g1/initial_pose.yaml` was changed to match, so the robot
spawns in the posture the reference asks it to hold.

**The shoulder roll is not cosmetic.** At zero roll the hands interpenetrate the
hip links by 12.6 mm at this elbow angle. The MPC carries no self-collision
constraint - by design, per the current scope - so the *reference* has to be
collision-free on its own. Verified against the model with all geoms made
collidable under a 20 mm margin: at the shipped nominal there are **0
penetrations and 0 near-misses**, and the envelope stays clear across a +-0.6 rad
shoulder-pitch excursion. That measured +-0.6 rad is what `armSwing.maxOffset`
clamps to.

**What this did NOT do - measured, not assumed.** The posture change does not buy
stability, and it costs a little.

*Standing:* trunk pitch is unchanged at noise level - 0.0057 rad arms-zero against
0.0101 rad arms-down over the same 45 s stance test, both under 0.6 deg. In static
stance the MPC compensates the arm mass either way, so the lean torque never shows
up as pitch.

*Walking:* base-motion noise over the forward phases of the vx ladder, two runs per
configuration, averaged:

| phase | pitch RMS down / zero | roll RMS down / zero | z std down / zero |
|---|---|---|---|
| vx 0.05 | 0.0232 / 0.0179 | 0.0698 / 0.0705 | 0.0056 / 0.0056 |
| vx 0.10 | 0.0421 / 0.0379 | 0.0646 / 0.0648 | 0.0056 / 0.0055 |
| vx 0.20 | 0.0595 / 0.0526 | 0.0575 / 0.0542 | 0.0060 / 0.0056 |
| vx 0.30 | 0.0889 / 0.0739 | 0.0454 / 0.0452 | 0.0059 / 0.0060 |

Pitch noise is **consistently 10-20% worse** with the arms down - the same
direction at every speed in both runs, so it is an effect and not scatter. Roll and
height noise are unchanged. The likely cause is simply that every gain in this
configuration was tuned with the arms held forward: moving 7 kg changes the pitch
inertia and the CoM that the foot-placement nominal was fitted around. Retuning
against the new posture is a phase-2 item.

So this change is justified by **the posture itself** - arms hanging at the sides,
collision-free, which is what was asked for - and by the CoM offset it removes. It
is NOT justified by a stability gain, and it should not be cited as one.

### The vx ladder is bimodal at the reversal - do not read single runs

An arms-down ladder run fell at 85.0 s where an arms-zero run had survived 197.8 s,
which looked like a clear regression from the arm posture. It was not. Repeating
both:

| config | run 1 | run 2 | run 3 |
|---|---|---|---|
| arms-down | fell 85.0 s | fell 84.4 s | fell 85.4 s |
| arms-zero | **survived 197.8 s** | **fell 84.4 s** | - |

The arms-zero configuration falls at 84.4 s too, with near-identical numbers
(peak pitch 0.485, roll 0.355 against 0.487/0.384 for arms-down). The outcome at
this transition is close to a coin flip and the 197.8 s survival was the lucky
draw. **The arm posture does not decide it.** Any single ladder run quoted in this
file that hinges on surviving past ~84 s should be treated as one sample of a
bimodal distribution, including the 197.8 s figure in "Current status".

### Backward walking is the weak direction

Every fall above lands 1.3-2.3 s after `vx_neg10` begins at 83.10 s. Forward
phases at 0.05, 0.10, 0.20 and 0.30 m/s complete every time, in both
configurations; the robot only ever goes over on the reversal to -0.10 m/s.

That asymmetry is not explained yet and is worth its own investigation. The
obvious suspects are all in the foot-placement path, which was derived and tuned
for forward travel: `baseAtTouchdown` advances the nominal by `v * timeToTouchdown`
and `stepLengthGain` biases the step ahead of the base, both of which assume a
sign convention that reversal inverts. The capture-point correction itself is
sign-symmetric, so the feedforward terms are the place to look first.

## Arm swing: shipped off, with a seam for phase 2

The hardcoded swing in `getDesiredState` (amplitudes and phase offset as literal
`0.15`s) moved into a configurable `ArmSwingSettings` and a single private method,
`SwitchedModelReferenceManager::addArmSwingOffsets`.

**It ships disabled**, so the arm reference is a fixed posture. Two reasons:

1. At the shipped amplitude the motion is **2.6 deg at 0.3 m/s** - too small to
   help or hurt.
2. It contradicts the momentum reference. That reference asks for
   `h_ang,x = h_ang,y = 0` with terminal weight 75, while the swing commands a
   motion that necessarily produces non-zero angular momentum. The two fight, and
   that conflict is the reason the arms are worth nothing to balance today.

`addArmSwingOffsets` is the phase-2 seam. It already receives the measured state
(whose centroidal momentum block carries the arms' own contribution through
`A_G(q)`), the reference being tracked, and the time for the gait phase - so a
momentum-regulating law replaces the sinusoid without touching any call site. The
arms are 7.04 kg on ~0.25 m lever arms, so they are a real angular-momentum
actuator; the reason to do this properly is that the centroidal model already
accounts for them. **Resolve the `h_ang = 0` conflict first**, or the cost will
simply cancel whatever the new law commands.

## Initial state

The MPC observation is built entirely from live feedback; nothing in it comes from
`ocs2.initialState`:

| element | source |
|---|---|
| centroidal momentum (6) | `A_G(q) v / m`, so it follows the floating-base twist |
| base pose (6) | the floating-base source - the InEKF once the warm-up window passes, the simulator/hardware body before that |
| joint positions (23) | the ros2_control state interfaces |

`on_activate` now polls those interfaces until they are readable and seeds the
first observation from them, logging the pose it read; if they do not become
readable within 5 s it refuses to activate rather than starting the MPC from a
configured pose the robot may not be in. In simulation that measured pose is
`initial_pose.yaml` by way of the ros2_control xacro.

A related defect went with it: both interface-dropout paths in `build_observation`
logged "holding last observation" but actually returned the configured seed, which
silently asserts a pose rather than admitting the state is unknown. They now hold
the last observation genuinely read from hardware.

### Landed, in order

| commit | what |
|---|---|
| `af9956e` | yaw-as-roll fix; `scene_flat.xml` |
| `834e145` | foot placement aimed at the base position *at touchdown* |
| `de51716` | `trackingWeight` sweep — 3x sustainable speed |
| `3d62b73` | figure-eight command script |
| `d57dba2` | scripts moved to `launch/` and installed |
| `25c46c8` | optional terrain-height reference; `static` fix in the ground estimate |
| `30051ad` | shipped flat-ground config; README + terrain caveat |

Plus, in the other two repos: `3fd9f1e` (`legged_state_estimator`, InEKF bias /
reset / diagnostics) and `3ea029703` (`ocs2_ros2`, out-of-bounds read in
`getGaussNewtonApproximation`).

---

## Phase 0 — instrumentation

**Goal.** Make the two questions that drive every later phase answerable from
data instead of inference:

1. *Is the filter consistent*, not merely close to ground truth? A filter can
   track well while believing its own error is far smaller than it is. Only the
   covariance it reports for itself, and the innovation measured against the
   covariance it predicted for that innovation (NIS), separate those cases.
2. *Which cost term decides the posture the solver picks?* A summed cost cannot
   answer this; only per-term values can.

### What was added

**`legged_state_estimator`** (separate repo, same working tree)

- `CorrectionInfo` (`inekf.hpp`): per-update record of whether a correction
  fired, the stacked innovation `r`, its predicted std `sqrt(diag(S))`, the NIS
  `r' S^-1 r`, and which contacts were corrected / augmented / marginalized.
  Populated in `CorrectRightInvariant`, `CorrectLeftInvariant` and
  `CorrectKinematics`; cleared at the top of every `CorrectKinematics` so a call
  that corrects nothing cannot leave stale diagnostics looking current.
- `InEKF::getLastCorrection()`, `getEstimateBias()`, `setEstimateBias()`,
  `resetState()`.
- `LeggedStateEstimator`: `getInEKF()`, `getLastCorrection()`, and per-block
  covariance std-devs (`getAttitudeStdDev()` … `getAccelerometerBiasStdDev()`),
  `getNumAugmentedContacts()`.

**`legged_robot_mpc_controller`**

- `InekfFloatingBaseEstimator::Diagnostics` — per-tick snapshot: covariance
  std-devs, bias + whether bias is being estimated, NIS, per-contact innovation
  and predicted std, landmark world positions, contact added/removed/corrected/
  in-stance flags, and the three height quantities (`inekf_height` **before** the
  blend, `kinematic_height`, `reported_height`) plus the touchdown anchors.
  All vectors sized once in `allocateDiagnostics()`, so `update()` allocates
  nothing.
- `DiagnosticsCsvLogger` (`common/diagnostics_csv_logger.hpp`) — fixed-schema CSV
  writer fed from the RT loop. Single-producer/single-consumer ring buffer of
  preallocated `double` rows; the control thread only memcpys, a writer thread
  formats and writes. Full ring ⇒ the sample is dropped and counted rather than
  blocking the control loop; the count is reported on deactivation.
- `CentroidalMpcDiagnostics` — owns the schema and both logs.

### Why two files, not one

They come from two threads at two rates, and forcing one schema would have meant
evaluating the cost terms from the control thread. The cost terms read the
reference manager (mode schedule, swing trajectories) which the **solver** thread
writes in `preSolverRun()` — that is the same A7 hazard, and it would have been
introduced deliberately. Evaluating them on the solver thread between iterations
does not race. The observation is handed across with a `try_lock`: a diagnostic
is never worth stalling the control loop for, and a one-tick-stale observation
changes nothing about which term dominates.

`<prefix>_state.csv` — control thread, `stateRate` Hz (default 200):
run phase / mode / contact flags, GT and estimate pose+twist and their errors,
covariance std-devs, bias, NIS, per-contact innovation and landmark, height
triple + anchors, and **the centroidal momentum recomputed from both feedback
sources through the same `A_G(q)` with the same joint state** — which isolates
the estimator's own contribution to the dominant MPC state from any model
difference.

`<prefix>_cost.csv` — solver thread, ≤`costRate` Hz: per-running-term and
per-terminal-term values at the current observation, the state and the reference
those terms track, and the MPC advance time.

### Design notes worth keeping

- **Schema/fill mismatch is checked, not assumed.** The column list and the fill
  code are two lists kept in step by hand; a mismatch would silently shift every
  column after it. `CentroidalMpcDiagnostics::schemaMismatch()` compares the
  values written per row against the declared width, and `on_deactivate` logs an
  error if it ever tripped.
- **Cost-term names are sorted.** `getCostNames()` enumerates an `unordered_map`,
  so without sorting the CSV column order would change between runs and two logs
  could not be compared column-wise.
- **Unset columns are written empty**, which numpy/pandas read back as NaN —
  distinguishable from a real zero.
- **`%t` in the path prefix** expands to the run start time, so a rerun does not
  overwrite the log it is meant to be compared against.

### A1 landed here (ahead of Phase 1)

`CorrectRightInvariant` used to unconditionally zero `Theta` and the bias
cross-covariance. `H` has no bias columns, so that cross-covariance is the *only*
path by which a kinematic measurement can observe the bias; zeroing it makes the
bias rows of the Kalman gain identically zero, `dTheta ≡ 0` every update, and the
bias can never converge — which is what made `estimate_bias_` and the configured
bias random-walk noise inert.

The block was **removed entirely** rather than guarded. `Propagate()` already
decouples the same blocks when `estimate_bias_` is false, so freezing the bias is
handled; doing it again in the correction would additionally discard a
deliberately seeded non-zero initial bias.

**Not yet verified on a run.** The convergence test (inject a known IMU bias,
confirm it converges) is a Phase-1 gate.

### Status

Validated on real runs. No dropped log rows, no schema mismatch, 396/396
estimator validation lines PASS on the 197 s stance run.

**Running it.** `libFolder` defaults to the *relative* path `auto_generated/g1`,
so `ros2 launch` must be run from `~/ocs2_ros2_ws`. From anywhere else the cached
CppAD libraries are not found, the model is regenerated, and generation segfaults
inside `CppAdInterface::createModels`. That cost a lot of time to find; it is not
a code defect but it looks exactly like one.

```bash
cd ~/ocs2_ros2_ws
ros2 launch legged_robot_mpc_controller g1.launch.py \
  floatingBaseSource:=state_estimator diagnosticsLog:=true \
  diagnosticsLogPrefix:=/tmp/diag_%t
python3 launch/command_sequence.py --sequence speed_ladder
python3 launch/analyze_diagnostics.py /tmp/diag_<stamp> --timeline 5
```

Build with `NUM_JOBS=2` per the README — a full-parallelism clean build OOMs
(`cc1plus` killed) on this container.

---

## Phase 1 — estimator correctness (done)

- **A1** bias wipe removed. **Confirmed working**: gyro bias now moves over a
  range of 3.4e-2 rad/s and accelerometer bias 2.5e-1 m/s² during a run. Before
  the fix these were identically zero forever, by construction.
- **A2** configurable initial covariance, exposed as
  `stateEstimator.initialCovariance.*`. Defaults: 1e-2 on attitude/velocity/
  position, 1e-2 gyro bias, 1e-1 accel bias.
- **A3** `init()` now calls `resetState()`, which drops the augmented contact and
  landmark maps together with the state matrix they index into.
- **A4** `base_ang_vel_world_estimate_` derived from the bias-corrected local rate
  and the corrected rotation, instead of a variable only written under
  `dynamic_contact_estimation`.
- **A7** fixed, but **not by the mechanism originally described**. `getContactFlags`
  reads the *base* class's `BufferedValue`, not the shadowing `modeSchedule_`
  member (which only feeds a `getPhaseVariable` that has no callers). The race is
  real all the same: `BufferedValue::get()` is documented as not thread-safe
  against `updateFromBuffer()`, and the solver thread calls the latter inside
  `preSolverRun()`. The control loop now reads a `RealtimeBuffer` snapshot the
  solver publishes once its iteration is complete.

### Measured estimator performance (197 s stance + squat, estimator in the loop)

| Quantity | Result |
|---|---|
| linear velocity error | RMS 0.005–0.008 m/s, peak 0.039 |
| angular velocity error | RMS ~1e-4 rad/s |
| height error vs GT | RMS 0.0000 m (anchored blend pinning it) |
| x / yaw drift | 0.107 m and 0.080 rad RMS over 197 s — unobservable gauge states, covariance grows accordingly |
| filter-vs-reported height drift | −0.00000 m/s |

The height architecture concern (Phase 2.5) **does not bite on flat ground**: the
measured drift between the filter's internal height and the reported blended
height is zero to five decimals over 197 s. Revisit only for terrain.

### Filter consistency (NIS) — the one number worth acting on

From the walking run, before the fall:

| Support | NIS/dof | above 95% band |
|---|---|---|
| single (2 contacts, dim 6) | 1.02 | 10.6% |
| double (4 contacts, dim 12) | 2.18 | 31.8% |

Single support is consistent; double support is over-confident by ~2×. That is
the B1 signature — rigidly linked contacts fed in as independent landmarks — and
it is the evidence for doing Phase 2 (contact model) rather than sweeping
`contactPosition` further. Note the dim-12 median is 9.07 against a mean of
26.14, so the excess is concentrated in transition spikes rather than spread
evenly; touchdown/liftoff handling is the place to look first.

---

## Phase 3 — OCP candidates (raised early; still largely open)

Written when walking failed at 0.10 m/s. That symptom is gone - foot placement
fixed it - but none of the items below were actually addressed, so they remain
open and are the likely ceiling on further speed:

1. **Arm-swing vs zero angular-momentum reference.** `setArmSwingReferenceActive(true)`
   commands a swing scaled by commanded vₓ, while the momentum reference asks for
   h_ang,x = h_ang,y = 0 with terminal weight 75. During walking the logged
   `x_hbar_ang` is non-zero against a reference of exactly zero.
2. **`targetMomentum(5) = yawRate / mass`** — normalized angular momentum is
   (I_G ω)_z / m, not ω_z / m. Off by roughly the z inertia, on a heavily
   weighted terminal component.
3. **v_CoM ≠ v_pelvis**, which the command and base-pose target conflate.
4. During walking the cost is dominated by the foot `TaskSpaceKinematicsCost`
   terms (32.7% + 31.8%), well above `stateInputQuadraticCost` (19.4%) — worth
   checking whether the swing-foot tracking weights are fighting balance.

The B4 height question is now partly answered: the pelvis tracks a 0.75 m command
to 0.763 m, and `ExternalTorqueQuadraticCost` reads 0.0000 in stance, so the leg
torque cost is *not* outbidding base-z tracking at this operating point. The 13 mm
offset is real and reproducible but small; it is not the headline problem.


---

## Walking (current state)

### Two analysis bugs that invalidated the first round

1. **Yaw was being read as roll.** `quaternionToEulerZYX` returns
   *(yaw, pitch, roll)*, but the diagnostics wrote them into `_x/_y/_z` columns and
   the analyser read `_x` as roll. The "lateral roll divergence" reported earlier
   was the robot **veering off heading**. Worse, the fall detector treated that
   column > 0.5 rad as a loss of balance, so a commanded in-place turn was flagged
   as a fall after two seconds while the pelvis sat at its nominal height.
   Columns are now `gt/est/err_euler_{yaw,pitch,roll}` and the fall check uses
   height + pitch + roll only.

2. **Every forward walking test drove into the staircase.** `scene.xml` includes
   `stairs.xml`, first riser at **x = 0.75 m**. The logs show the pelvis reaching
   0.66–0.67 m and stalling for ten seconds before toppling — a swing foot
   stubbing a 0.10 m step. `scene_flat.xml` is the same scene without obstacles;
   use it for gait/balance/estimator work.

Any conclusion in this file dated before those fixes should be re-derived.

### The estimator is not the cause

Measured through a run that ends in a fall at 55.8 s:

| t | \|e_v\| | \|e_ω\| | e_yaw | e_pitch | \|Δh_lin\| |
|---|---|---|---|---|---|
| 10 | 0.011 | 0.0001 | 0.0005 | 0.0003 | 0.011 |
| 30 | 0.011 | 0.0004 | 0.0094 | 0.0075 | 0.011 |
| 50 | 0.048 | 0.0005 | 0.0157 | 0.0114 | 0.048 |
| 55 | 1.35 | 0.015 | 0.75 | 0.055 | 1.29 |

Velocity, angular-velocity and **momentum** error are bounded and flat — no
accumulation. The blow-up at t=55 is after the pelvis is already at 0.40 m, i.e. a
consequence. The only growing quantities are `|e_p|` and `e_yaw`, which are the
unobservable gauge states a contact-aided InEKF must drift in.

The controller is already insulated from them: `q.basePose` and `qFinal.basePose`
put **zero weight on x, y and yaw**, and `BaseMotionTrackingCost` in
`RelativeTwist` mode *skips* those pose deviations entirely and compares
velocities in the respective pelvis frames, so a yaw-gauge change cancels on both
sides. `RelativeTwist` is selected whenever the target mode is not an explicit
base-pose command. The capture-point foot placement and the ICP cost both express
the foothold and the CoM in the same estimated frame, so the drift cancels there
too.

### What was missing: a foothold reference

On flat ground `getSwingFootholdReference()` returned false (no stair plan, no
terrain planner) and the foot task-space cost has zero position weight — so the
swing foot's landing xy was whatever the optimizer produced, with nothing closing
a balance loop through foot placement. Added a capture-point foothold: aim the
swing foot at `com + v/ω`, offset laterally by half the step width, clamped, with
the nominal taken at the **predicted touchdown base position** (using the current
base biases every step backwards by `v·Δt` and pitches the robot forward — worse
the faster it walks).

### Ablation and sweep, on the vx ladder, flat scene

| config | fell at |
|---|---|
| ICP off, foot placement off (original) | 46.4 s (inside vx **0.10**) |
| ICP 15, foot placement off | 48.1 s |
| ICP off, foot placement on | 55.8 s |
| ICP 15, foot placement on | 56.2 s |

`trackingWeight` then turned out to be the dominant knob, and it wants to be
large — a weak weight simply lets the optimizer ignore the foothold:

| trackingWeight | 30 | 60 | 100 | 200 | 400 | 800 | 1600 | 3200 |
|---|---|---|---|---|---|---|---|---|
| fell at [s] | 48.5 | 56.0 | 57.5 | 62.3 | 65.0 | 72.0 | 78.6 | 79.2 |

Other knobs: `gain` optimum is **0.6** (1.0 → 55.8, 1.5 → 55.0, 2.0 → 48.3);
`maxAdjustment` 0.20 slightly better than 0.12; `stepLengthGain` measured
neutral-to-harmful and defaults to 0, because `baseAtTouchdown` already supplies
most of that travel and the two double-count.

### Best configuration (in `ros2_controllers_legacy.yaml`)

```yaml
icpErrorWeight: 0.0
captureFootPlacement:
  enabled: true
  gain: 0.6
  stepWidth: 0.18
  maxAdjustment: 0.20
  trackingWeight: 1600.0
  stepLengthGain: 0.0
```

**80.3 s on the vx ladder** — completes 0.05, 0.10, 0.20 **and 0.30 m/s**, falling
only during the deceleration at the end. The original configuration failed inside
the 0.10 m/s phase. Roughly a 3x increase in sustainable forward speed.

ICP is off in this configuration because that measured best at
`trackingWeight: 1600`. It *did* help at lower weights (400: 72.9 s with ICP vs
65.0 s without) and was not re-tested at 1600 — worth one run.

### Measured limits before this tuning (trackingWeight 100)

| axis | survives | fails at |
|---|---|---|
| vx | 0.05, 0.10 | 0.20 |
| vy | 0.05, 0.10 | 0.20 |
| yaw | 0.1, 0.3, 0.5 | robust |
| fwd+strafe combined | — | fails |
| walk + squat to 0.75 m | 0.7925, 0.77 | 0.75 |

These have **not** been re-measured at `trackingWeight: 1600`. Only the vx ladder
has. That is the first thing to redo.

### Other fixes landed while getting here

- **ocs2_core `getGaussNewtonApproximation` out-of-bounds read** (`3ea029703` in
  `ocs2_ros2`). The inner loop over a Jacobian row had no bound on `j`, so the
  last non-zero of the last row read past the end of `rows[]` and could index
  `dfdxx` with garbage. Latent for any Gauss-Newton AD cost; surfaced when two new
  Jacobian columns changed the sparsity. Note ocs2_core is a **static** library,
  so dependents must be relinked for the fix to take effect.
- **ICP cost was never finished**: the capture point was commented out (leaving a
  CoM-position cost), the reference averaged both feet including the airborne one,
  and `isActive` required *both* feet — switching it off through every
  single-support phase. All three fixed.
- **`getPhaseVariable` out-of-bounds read** — dereferenced `end()` past the last
  event; it feeds the arm-swing reference every solver iteration.
- **Standing oscillation**: knees were swinging ~20° peak-to-peak while commanded
  to stand, because leg posture weights were near zero (knee 0.02) while the
  terminal weight was 8.0 — free mid-horizon, pinned at the end. Raised leg
  weights; knee 19.6° → 5.6° p-p.

(Open items for this section are consolidated at the end of the file.)

---

## InEKF sources, terrain height, and the shipped flat-ground config

### Cross-test: height source x contact source (figure-eight, flat)

| height.source | scheduled | torque |
|---|---|---|
| kinematic | **127.8 s** | 12.7 s |
| blend | 60.3 s | 8.2 s |
| anchored | 43.2 s | 6.6 s |
| inekf | 26.3 s | 7.4 s |

`torque` contact detection collapses everything to 6-13 s, confirming the A8
analysis: `ContactEstimator` assumes contact i is served by joints 3i..3i+2, a
12-DoF quadruped layout that maps G1's heel/toe onto the wrong joints. Structural,
not tunable. Use `scheduled`.

`kinematic` wins on flat because it pins the feet to the same plane the controller
assumes (`terrainHeight = 0.0`). Consistency, not generality.

### The terrain-height blocker, and why it is not shipped on

`adaptToCurrentGroundHeight()` computed a ground-height estimate and then threw it
away with a hardcoded `terrainHeight = 0.0`. That constant is the swing planner's
ground reference, so it hard-codes a flat floor - the single thing preventing
terrain walking regardless of estimator quality. Now selectable via
`ocs2.model.useTerrainHeightEstimate` (off by default), rate-limited per solve.
Also fixed a function-local `static` in `getGroundHeightEstimate()` that shared one
mutable value across every solver thread and across activations.

Measured with `anchored` + terrain on:

| | survived | dist | peak pitch | \|e_z\| |
|---|---|---|---|---|
| ramp, terrain off | 40.7 s | 1.51 m | 0.496 | 0.0016 |
| ramp, terrain on | **94.1 s** | 2.28 m | **0.221** | 0.134 |
| flat, terrain off | 44.0 s | 1.91 m | 0.493 | 0.0019 |
| flat, terrain on | 149.2 s | 23.14 m | 0.493 | **0.390** |

**The gain and the drift are the same effect.** Tightening `maxTerrainHeightStep`
removes both together: 0.01 -> 149 s / 0.390 m; 0.002 -> 68 s / 0.079 m;
0.0005 -> 64 s / 0.050 m. So the flat-ground result is largely the base-height
target running away with the drifting terrain reference, which quietly removes the
height constraint that was destabilising the walk - not terrain adaptation.

The loop is the Phase 2.5 concern: with `anchored`, touchdown anchors are computed
from the filter's own height, so enabling the terrain reference lets filter drift
feed the anchors which feed the reference. `inekf` (no blend) shows it far less -
|e_z| 0.053 vs 0.390 under the same setting, at 52.6 s.

**Open problem.** Bounding this drift is the prerequisite for terrain walking. The
principled fix is a height pseudo-measurement INSIDE the filter rather than a blend
applied outside it, so the filter's own z is corrected instead of being papered
over downstream. On the ramp the tighter limit costs almost nothing (94.1 -> 94.9 s
at 0.002), so if terrain is enabled, use 0.002 rather than 0.01.

### Shipped configuration (flat ground)

```yaml
stateEstimator:
  height:   { source: kinematic }     # best on flat, |e_z| = 0.0005 m, no drift
  contact:  { source: scheduled }     # torque is structurally broken for G1
ocs2:
  model:
    useTerrainHeightEstimate: false   # see above; not trustworthy yet
    maxTerrainHeightStep: 0.002       # value to use if enabling it
  costs:
    icpErrorWeight: 0.0
    captureFootPlacement:
      enabled: true
      gain: 0.6
      stepWidth: 0.18
      maxAdjustment: 0.20
      trackingWeight: 1600.0
      stepLengthGain: 0.0
```

Deliberately excludes the 149 s flat result: it is not reproducible as *walking*
skill, only as a slackened height constraint, and it comes with a 0.39 m height
error that would be dangerous on hardware.


---

## Phase 2 plan — make the arms actually help balance

Phase 1 gave the arms a fixed, collision-free hanging posture and turned the
open-loop swing off. Phase 2 turns them into a working angular-momentum actuator.
The order below is deliberate: steps 1 and 2 are prerequisites, and doing step 3
first would produce a law the cost function silently cancels.

**1. Resolve the `h_ang = 0` conflict. This is the gate.**
The momentum reference asks for `h_ang,x = h_ang,y = 0` with terminal weight 75.
Any arm motion that generates angular momentum is therefore penalised by
construction - which is why the phase-1 swing was worth nothing and why it ships
disabled. Options, cheapest first:
  - Drop the weight on the `h_ang,x/y` terminal components, or zero them, and let
    the momentum be shaped by the arm cost instead of pinned to zero.
  - Better: track a *reference* angular momentum rather than zero, so the swing
    is asked for rather than merely tolerated.
Measure the per-term cost breakdown before and after with the existing
`<prefix>_cost.csv` - this is exactly what that log was built for.

**2. Fix `targetMomentum(5)`.** It sets `yawRate / mass`, but the normalized
angular momentum is `(I_G omega)_z / m`, not `omega_z / m` - off by roughly the z
inertia, on a terminal component with weight 75. The arms are a yaw-momentum
actuator, so a wrong yaw-momentum reference corrupts precisely the axis phase 2
is trying to control. Cheap, local, and it should land before any arm law is
tuned against that reference.

**3. Replace the sinusoid in `addArmSwingOffsets`.**
The seam already receives everything needed: the measured state (whose centroidal
momentum block includes the arms through `A_G(q)`), the reference, and the gait
phase. Two candidate laws, in increasing order of ambition:
  - *Momentum feedback.* Command a shoulder-pitch/roll offset proportional to the
    measured angular-momentum error, `-K (h_ang - h_ang_ref)`. Simple, and it
    reuses the state the MPC already estimates.
  - *Let the optimizer decide.* Stop prescribing an arm trajectory at all: drop
    the arm joint-tracking weights to something small, keep only a posture
    regulariser about the hanging nominal, and let the momentum and ICP costs
    move the arms. This is the principled version - the centroidal model already
    knows what the arms do to `h_ang`, so the solver can exploit them if it is not
    being told exactly where to put them. It is also the bigger change, because
    the arm weights are currently high (30/30/10/20) precisely to stop the arms
    wandering.
Whichever is chosen, it must respect `armSwing.maxOffset`: there is no
self-collision constraint in the MPC, so the clamp is the only thing keeping the
hands out of the hips.

**4. Re-run the standard tests.** Figure-eight and vx ladder, both scenes. The
metric that should move is peak roll - it currently oscillates +-8 deg per step
and is the precursor to most falls, and roll is exactly what a counter-swinging
arm pair has authority over.

## Open work, in priority order

1. **Finish validating the WBC beyond the vx ladder.** It is 6/6 with 0 QP
   failures on `vx_ladder` with a GT base, but that is one sequence on flat
   ground. Still to run: `stop_cycles` (the hardest case for this stack, and the
   WBC's old failure mode was precisely an inability to stop), `vy_ladder`,
   `yaw_ladder`, `combined`, the figure-eight, and the whole set again with
   `floatingBaseSource:=state_estimator` instead of ground truth. Also flip
   `g1.launch.py`'s `controllersFile` default back to `ros2_controllers.yaml`
   before merging to main.
2. **Commanded velocity is not tracked, in EITHER inner loop.** Forward comes out
   at about half (0.30 -> 0.14 with ID, 0.16 with the WBC) and backward overshoots
   by ~80% (-0.10 -> -0.18). Both inner loops agree closely, so this lives in the
   MPC or the command path, not the inner loop. It had never been measured because
   no analysis in this repo compared commanded against achieved speed - the
   per-phase table in `metric2.py` is what surfaced it. The backward overshoot is
   plausibly the same forward-bias asymmetry as open item 3.
3. **Backward walking remains the weak direction, even after the foothold fix.**
   The single fall in the N=6 height comparison was again `vx_neg10`, and the one
   remaining fall mode anywhere is that phase. `footCenterOffset` equalised the
   static margins at 0.09 m each, which is what removed most of it, but the gait
   is still forward-biased in ways this did not address - the swing trajectory and
   the step timing are both tuned around forward travel. Next step would be to
   measure the actual CoM excursion relative to the support polygon edge during
   the reversal rather than reasoning from the static geometry.
4. **Bound the estimator's absolute-z drift (Phase 2.5).** This is the blocker for
   terrain walking and the reason the terrain reference ships off. The right fix
   is a height pseudo-measurement *inside* the InEKF, so the filter's own z is
   corrected, instead of the blend currently applied outside it which leaves the
   internal z drifting uncorrected and lets `anchored` feed that drift back
   through the touchdown anchors.
5. **[SUPERSEDED - see item 3] Backward walking.** Every vx-ladder fall observed lands
   1.3-2.3 s after the reversal to -0.10 m/s; all forward speeds up to 0.30 m/s
   complete every time, with either arm posture. Look at the feedforward terms in
   the foot-placement path first (`baseAtTouchdown`, `stepLengthGain`) - they were
   derived for forward travel and reversal inverts their sign convention, while
   the capture-point correction itself is sign-symmetric.
6. **Retune pitch against the arms-down posture.** Moving 7 kg of arm mass down and
   back cost 10-20% in forward pitch RMS at every speed (table under "Arm
   posture"). The gains were fitted with the arms held forward.
7. **Re-measure the vy / yaw / combined ladders on the shipped configuration.**
   Only the vx ladder and the figure-eight were re-run after the weight was moved
   back to 100 with ICP on; the lateral and yaw limits quoted further down this
   file predate that and are stale. (The `trackingWeight: 1600` follow-ups that
   used to be items 2 and 3 here are withdrawn - that weight overfit the vx
   ladder and is no longer shipped.)
8. **The height command is not tracked during walking.** The figure-eight plot
   shows a commanded 0.77↔0.80 m sinusoid at 13 s producing only step-frequency
   bounce with a slow upward creep. Same family as the 13 mm squat offset, but a
   whole command being ignored rather than a small bias.
9. **Roll oscillates ±8° per step** at 0.22 m/s, much larger than expected, and is
   the precursor to most falls.
10. **[FIXED, needs retuning] `targetMomentum` angular part.** Now `I_G(q)*omega/m`.
   Note the earlier entry here said "off by roughly the z inertia" implying too
   small; it was 1.86x too LARGE, since I_zz = 0.539 < 1. The correction measures
   neutral-to-worse (stop_cycles 0/4 -> 3/10, p ~ 0.51) because the weight-75
   terminal term was tuned against the inflated target. Retune it in Phase 2.
11. **Arm-swing vs zero angular-momentum reference conflict** (Phase 3 item 1). Now
   the blocker for phase 2 of the arm work: the swing ships disabled precisely
   because of this, and it must be resolved before a momentum-regulating arm law
   can do anything. See "Arm swing: shipped off, with a seam for phase 2".
12. **Phase 2 contact model (B1):** double-support NIS is ~2x over-confident,
   consistent with heel and toe being fed in as independent landmarks when they
   are rigidly linked.

### Known environment traps

- Launch from `~/ocs2_ros2_ws`. `libFolder` is the relative path
  `auto_generated/g1`; elsewhere the cached CppAD libraries are missed, the model
  regenerates, and generation segfaults.
- Use `scene_flat.xml` or `scene_ramp.xml` for locomotion work. `scene.xml` has a
  0.10 m riser at x = 0.75 m that silently ends every forward-walk test.
- Pass `baseCommandGui:=false` when driving from a script: the GUI publishes to the
  same topic at 50 Hz and its zeros interleave with the script's commands.
- Build with `NUM_JOBS=2`. Full parallelism OOMs (`cc1plus` killed) on this box.
- `ocs2_core` is a **static** library; changing it requires relinking dependents.
- Diagnostic CSVs are ~50-120 MB per run and filled the VM disk once. Clean up.
