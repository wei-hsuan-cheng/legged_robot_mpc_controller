# Centroidal MPC + InEKF tuning — working log

Companion to `Codex.md`. One section per phase, appended as work lands, so the
state of the effort is traceable without re-reading diffs.

Plan phases (agreed order):

| Phase | Content | Status |
|---|---|---|
| 0 | Instrumentation. No behaviour change. | **done, validated on real runs** |
| 1 | Estimator correctness: A1–A4, A7 | **done, validated** |
| 2 | Contact model (B1, A8) | not started |
| 2.5 | Height pseudo-measurement inside the filter | **not needed on flat ground** (measured drift ≈ 0) |
| 3 | Reference / cost consistency | **partly done — foot placement added, walking much improved** |
| 4 | Parameter sweeps | **done for foot placement; trackingWeight was the dominant knob** |
| 5 | Hardware readiness (A5, GT-free init) | not started |

## Headline result (superseded — see "Walking" below)

Two analysis bugs invalidated the first round of walking conclusions. Both are
fixed; the corrected picture is in the Walking section.

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

## Phase 3 — walking (next, and now the priority)

Walking fails at 0.10 m/s regardless of feedback source or build vintage. Since
the estimator is exonerated, the candidates are the ones from the original
Cassie-vs-G1 analysis that live in the OCP:

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

### Open

- Re-measure vy / yaw / combined / height ladders at the tuned weight.
- Test ICP at `trackingWeight: 1600`.
- The remaining failure is a forward **pitch** divergence at the top speed.
- `targetMomentum(5)` still uses `ω_z/m` instead of `(I_G ω)_z/m`.
- Phase 2 (contact model): double support NIS is ~2× over-confident.


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
