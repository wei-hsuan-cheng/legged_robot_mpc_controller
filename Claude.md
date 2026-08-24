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

**Shipped configuration** (`config/g1/ros2_controllers.yaml`):

```yaml
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

1. **Backward walking remains the weak direction, even after the foothold fix.**
   The single fall in the N=6 height comparison was again `vx_neg10`, and the one
   remaining fall mode anywhere is that phase. `footCenterOffset` equalised the
   static margins at 0.09 m each, which is what removed most of it, but the gait
   is still forward-biased in ways this did not address - the swing trajectory and
   the step timing are both tuned around forward travel. Next step would be to
   measure the actual CoM excursion relative to the support polygon edge during
   the reversal rather than reasoning from the static geometry.
2. **Bound the estimator's absolute-z drift (Phase 2.5).** This is the blocker for
   terrain walking and the reason the terrain reference ships off. The right fix
   is a height pseudo-measurement *inside* the InEKF, so the filter's own z is
   corrected, instead of the blend currently applied outside it which leaves the
   internal z drifting uncorrected and lets `anchored` feed that drift back
   through the touchdown anchors.
3. **[SUPERSEDED - see item 1] Backward walking.** Every vx-ladder fall observed lands
   1.3-2.3 s after the reversal to -0.10 m/s; all forward speeds up to 0.30 m/s
   complete every time, with either arm posture. Look at the feedforward terms in
   the foot-placement path first (`baseAtTouchdown`, `stepLengthGain`) - they were
   derived for forward travel and reversal inverts their sign convention, while
   the capture-point correction itself is sign-symmetric.
4. **Retune pitch against the arms-down posture.** Moving 7 kg of arm mass down and
   back cost 10-20% in forward pitch RMS at every speed (table under "Arm
   posture"). The gains were fitted with the arms held forward.
5. **Re-measure the vy / yaw / combined ladders on the shipped configuration.**
   Only the vx ladder and the figure-eight were re-run after the weight was moved
   back to 100 with ICP on; the lateral and yaw limits quoted further down this
   file predate that and are stale. (The `trackingWeight: 1600` follow-ups that
   used to be items 2 and 3 here are withdrawn - that weight overfit the vx
   ladder and is no longer shipped.)
6. **The height command is not tracked during walking.** The figure-eight plot
   shows a commanded 0.77↔0.80 m sinusoid at 13 s producing only step-frequency
   bounce with a slow upward creep. Same family as the 13 mm squat offset, but a
   whole command being ignored rather than a small bias.
7. **Roll oscillates ±8° per step** at 0.22 m/s, much larger than expected, and is
   the precursor to most falls.
8. **`targetMomentum(5)` uses `ω_z/m` instead of `(I_G ω)_z/m`** — off by roughly
   the z inertia on a terminal component with weight 75.
9. **Arm-swing vs zero angular-momentum reference conflict** (Phase 3 item 1). Now
   the blocker for phase 2 of the arm work: the swing ships disabled precisely
   because of this, and it must be resolved before a momentum-regulating arm law
   can do anything. See "Arm swing: shipped off, with a seam for phase 2".
10. **Phase 2 contact model (B1):** double-support NIS is ~2x over-confident,
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
