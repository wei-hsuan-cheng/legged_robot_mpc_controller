# Centroidal MPC + InEKF tuning — working log

Companion to `Codex.md`. One section per phase, appended as work lands, so the
state of the effort is traceable without re-reading diffs.

Plan phases (agreed order):

| Phase | Content | Status |
|---|---|---|
| 0 | Instrumentation. No behaviour change. | **code complete, not yet validated on a real run** |
| 1 | Estimator correctness: A1–A4, A7 | A1 landed early (see below); A2/A3/A4/A7 pending |
| 2 | Contact model (B1, A8) | not started |
| 2.5 | Height pseudo-measurement inside the filter | gated on Phase-0 data |
| 3 | Reference / cost consistency (B4 first) | not started |
| 4 | Parameter sweeps | not started |
| 5 | Hardware readiness (A5, GT-free init) | not started |

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

### Status / caveats

- Both packages **compile clean** (`colcon build`, Release).
- The analysis script is **smoke-tested end-to-end** against a synthetic log with
  the exact production schema — loader, every analysis section, and both plots.
- **Not yet run against real data.** I could not launch the sim in this
  environment (segfault during `Configuring controller`, right after
  `frameIndices: 103`); it reproduces at clean HEAD with diagnostics disabled, so
  it is environmental to my headless invocation and not caused by these changes.
  The first real run is the outstanding Phase-0 validation step.
- `bufferRows` 8192 at 200 Hz is ~40 s of slack against a stalled write. If
  deactivation reports dropped rows, raise it or lower `stateRate`.

---

## Phase 1 — estimator correctness (pending)

Remaining after A1:

- **A2** initial covariance is never configured. `InEKFState` defaults to
  `P = I(15)` — σ_attitude = 1 rad, σ_v = 1 m/s, σ_p = 1 m — on a state seeded
  from exact ground truth. The first contact correction with `N = (0.002 m)²`
  then yanks it. Needs a configurable initial covariance.
- **A3** `init()` calls `setState()` but not `clear()`, so
  `estimated_contact_positions_` survives with stale indices into a fresh 5×5 `X`
  → out-of-bounds on the next `CorrectKinematics`. `resetState()` now exists;
  `LeggedStateEstimator::init()` must call it.
- **A4** `base_ang_vel_world_estimate_` is only written under
  `dynamic_contact_estimation`, which is false — so it is permanently zero.
- **A7** `modeSchedule_` is read from the 1 kHz control thread and written from
  the solver thread without the `BufferedValue` every other cross-thread field in
  that class uses.

Gate: baseline flat-ground numbers no worse; injected bias converges;
deactivate→activate does not crash.
