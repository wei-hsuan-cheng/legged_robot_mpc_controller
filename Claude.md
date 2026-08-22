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
| 3 | Reference / cost consistency (B4 first) | **now the priority — walking is broken** |
| 4 | Parameter sweeps | not started |
| 5 | Hardware readiness (A5, GT-free init) | not started |

## Headline result

**Walking is broken, and it is not the estimator.** The robot falls after ~10 s
of walking at 0.10 m/s. This reproduces:

- with the InEKF driving control, **and** with `ground_truth_state` driving it
  (estimator entirely out of the loop);
- on the **pre-Phase-0 baseline build** (controller `88e5e0a`, estimator
  `fe22857`), which falls at t = 31.2 s in the same `hold_010` phase that the
  current build falls in at t ≈ 32.9 s.

So it is pre-existing, and it is in the controller/gait, not the filter. The
fall-onset dump is unambiguous on the mechanism: `est_z`/`gt_z` agree to three
decimals and `est_pitch`/`gt_pitch` to ~0.01 rad right through the tipping; the
estimate only departs from ground truth *after* the pelvis has already dropped.
Pitch goes 0.03 → 0.13 → 0.49 rad in about 0.7 s.

Standing and squatting are solid: 197 s of stance with no fall, and the pelvis
follows a 0.7925 → 0.75 height command to 0.763 (13 mm short, reproducible in
both feedback modes) and returns cleanly.

Next work therefore belongs in Phase 3, on the OCP, not in the filter.

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
python3 scripts/command_sequence.py --sequence speed_ladder
python3 scripts/analyze_diagnostics.py /tmp/diag_<stamp> --timeline 5
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
