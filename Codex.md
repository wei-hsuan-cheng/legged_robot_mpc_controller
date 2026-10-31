# WB-ID MPC Walking Robustness Handoff


## Current Commit Snapshot (2026-10-31 Asia/Taipei)

User requested committing and pushing both repositories without building or testing.
Preserve the current user-selected YAML: G1 and H2 both use `euler_zyx`, a 1.0 s horizon, a 0.02 s SQP grid and one SQP iteration.
Walking swing heights are G1 0.08 m and H2 0.10 m; gait-switch horizon fraction is 0.3.
H2 retains torqueConstraintNodes 10, torque barrier mu 0.5/delta 1.0 and friction barrier mu 0.2/delta 5.0.
`hardBounds` is omitted and defaults false; `keepInitialStateFixed` has been removed.
The earlier tangent/0.8 s/0.20 m candidate and its benchmark results below are historical, not the current YAML.
Do not automatically restore that candidate or present its measurements as validation of this snapshot.
No new build or tests were run for this commit; the initial-state-option removal has not been rebuilt.
Push OCS2 on `jazzy` and the controller on `feat/tangent_space_orientation` together because the optional hard-bound integration depends on the matching stack.
Matching OCS2 commit: `ff9823188` (`FEAT: add opt-in SQP hard inequality constraints`).

```text
FEAT: add opt-in SQP hard inequality constraints
```

```text
FEAT: add optional WB-ID torque bounds and solver diagnostics
```


## Initial-State Experiment Removed (2026-10-31 Asia/Taipei)

User explicitly requested reverting `keepInitialStateFixed` after comparing standard solver behavior.
Removed its OCS2 setting/loader/first-state override, controller schema/builder/H2 YAML entry, and dedicated native test/CMake target.
Preserved the original measured initial-state constraint, tangent warm-start rebasing, AD fixes and the separate disabled hard-inequality path.
The experiment had been false in all selected-profile trials, so removing it does not intentionally change the saved formulation or tuned numerical values.
Do not reintroduce it from chronological instructions below. Their pin-option settings/test counts describe historical experiments only.
CasADi distinguishes `subject_to` constraints from `set_initial` guesses; underlying solver globalization determines intermediate feasibility ([official docs](https://web.casadi.org/docs/)).
Crocoddyl DDP rolls out from the problem's supplied x0; FDDP can retain gaps in partial infeasible steps ([DDP source](https://github.com/loco-3d/crocoddyl/blob/master/src/core/solvers/ddp.cpp), [FDDP source](https://github.com/loco-3d/crocoddyl/blob/master/src/core/solvers/fddp.cpp)).
Thus pre-pinning every intermediate iterate is not a universal solver requirement, and its absence is not evidence that OCS2 forgot the initial-state constraint.
The general hard-constraint approach is standard, but our optional equality-projected HPIPM bridge remains a limited experimental extension, not a claim to reproduce every CasADi backend or Crocoddyl solver.
No rebuild or simulator test was started for this cleanup; rebuild matching OCS2/controller consumers before deployment because a settings/header and generated parameter schema changed.
Verification passed:8 activation checks,6 startup checks and the standalone parameter-contract script. Updated that script's stale model-key expectation to include the already-supported orientationRepresentation and asserted that the removed option is absent from schema/config/builder.

```text
CLEANUP: remove experimental initial-state pinning
```


## Saved Response-Priority Candidate (2026-10-31 Asia/Taipei)

After the user-requested two-hour wait, tested existing penalties first, then parameter-only timing/clearance changes.
Selected H2 candidate passed3/3 rough-field and3/3 bounded default-scene/ramp-entry trials, MPC80/MRT1000, InEKF, headless, GUI false.
Achieved75.53-76.66 Hz across those6 runs versus about68-70 Hz with the original1 s horizon.
Saved only three numerical changes from the start of this resumed session:

| H2 setting | Before | Saved | Reason |
| --- | --- | --- | --- |
| `ocs2.mpc.timeHorizon` |1.0|0.8|Fewer nodes, faster solves; preserve dt0.02 and one SQP iteration |
| `ocs2.gait.transitions.switchDelayHorizonFraction` |0.3|0.375|Keep the same physical0.3 s switch delay with the shorter horizon |
| `ocs2.gait.templates.walk.swingHeight` |0.10|0.20|Scaled apex0.15 m clears the first~8.6 cm tile; fallback0.10 and trot/run settings unchanged |

The source YAML at session start really had walk0.10, despite the older checkpoint describing0.20.
No new controller/OCS2 implementation or cost-weight edits in this resumed session; both experimental solver options remain false, torque window10 nodes/0.19 s, and all penalty values remain baseline.
Saved-YAML flat walk/vy/yaw/run regression PASSED4/4 in `/tmp/tangent_penalty_1031_saved`: rates76.48/76.10/78.17/76.88 Hz; mean advances11.92/12.16/11.56/11.90 ms.
Pose-derived tracking: vx0.256 for0.2 command, vy0.228 for0.25 (with forward drift0.083), yaw0.559 for0.6, running vx1.020 for1.0. Exact tracking is not claimed; the user prioritizes robustness and response.
Ten selected-profile trials completed with zero logged solver exceptions and torque-window overruns. Automated simulations are now stopped; no new commit/push or controller rebuild in this parameter-only round.
Do not claim hard feasibility, general terrain robustness, or reliable recovery from the default ramp's~35 cm end drop.
These are small sequential batches on the VM; timing below is headless and does not establish the same rate with RViz/MuJoCo rendering.
Raw selected terrain records: `/tmp/tangent_penalty_1031_horizon08_clearance020`; rejected profiles and limitations follow below.
All50 completed trials from this round are copied under local ignored `tuning_results/tangent_penalty_1031`; the separate ENOSPC-interrupted run is excluded.
Verified504 copied files against the12 remote trial directories by size, then replaced those owned Docker `/tmp/tangent_penalty_1031_*` directories with symlinks to the shared local copies to recover disk space and retain existing artifact paths.
Three additional older trial directories are preserved in `previous_trials_archive_3.tar.gz` before removing their Docker copies. The installed H2 config was checked and sees0.8/0.375/0.20 through symlink install; restart launch, no rebuild needed for these parameter changes.
Selected rough replay had zero crossing/opposite-leg overlap samples, min signed foot width0.133 m and final stagger0.011 m (latest-sample pose replay, not force truth).
Selected rough/ramp curves still show near torque/limit peaks1.90/2.13 and generalized accelerations2142/2291 over5-26 s; the ramp sample has a small planned negative world-z force down to-20.9 N. Soft feasibility is improved in these samples, not guaranteed.
Both anchors remain unit/constant; no new numerical orientation defect identified. Estimator height/tilt errors and rough-ground overspeed remain visible; no estimator tuning was performed here.

```text
TUNE: improve H2 tangent MPC update rate and walking clearance
```


## Resumed Penalty/Rate Investigation (2026-10-31 Asia/Taipei)

User manually reports current locomotion is good enough but achieved solve rate falls to about60 Hz; prioritize faster response rather than more SQP iterations/accuracy.
Waited the requested two hours before resuming; no user simulator was running at the initial process check.
Start with isolated existing penalty tuning and diagnostic-overhead measurement, not new solver/manifold implementations.
IMPORTANT: current on-disk H2 walk.swingHeight is0.10, matching HEAD, despite the previous handoff's0.20 statement; preserve current user settings as the new baseline.
H2 remains tangent, one SQP iteration, horizon1, dt0.02, nThreads5; hardBounds and keepInitialStateFixed remain false.
Default diagnostics.trackingRate is0; benchmark --record-tracking temporarily enables100 Hz tracking AND per-solve diagnostic cost/trajectory evaluation on the solver worker.
Printing solver statistics occurs only at solver destruction, not every solve; do not blame that flag for steady-state console overhead.
First baseline: `/tmp/tangent_penalty_1031_baseline`, two flat walk and two rough trials, MPC80/MRT1000, InEKF, headless, GUI false.
Baseline completed: flat2/2 at69.69/68.40 Hz; rough1/2 success at67.44/68.20 Hz (first fell9.97 s).
Flat LQ~5.77-6.00 ms and QP~8.03-8.08 ms dominate the budget; line search~0.34 ms, not extra SQP iterations.
Failed rough replay/curves show first tile-entry destabilization followed by excessive requests (near torque ratio33.32, acceleration37333, min planned world-z force-2660 N), finite plans and unit anchors; screenshots under local ignored `tuning_results/tangent_penalty_1031`.
At first node across ramp-in/walk samples, flat torque-ratio p95~0.589; rough failure p95~1.87. These are different from the maximum over first3 nodes.
Relaxed barrier h<=delta: gradient=mu*(h-2delta)/delta^2, curvature=mu/delta^2; outside that region gradient=-mu/h, curvature=mu/h^2.
Current torque mu0.5/delta1 gives boundary gradient-1 and curvature0.5; candidate mu0.75/delta1 gives-1.5/0.75; candidate mu1/delta2 preserves boundary gradient-1 with curvature0.25.
Current friction mu0.2/delta5 gives boundary gradient-0.08 and curvature0.008; candidate mu0.4/delta5 doubles both. Friction residual is in N, torque residual in Nm; numerical magnitudes are not interchangeable.
Queued serial diagnostic-off repeats, then isolated torque0.75, torque1/delta2 and friction0.4 profiles, initially one flat/rough trial each.
Do not save a penalty profile based on one successful trial. No controller or OCS2 implementation changes in this resumed session.
Diagnostic-off completed flat2/2 at66.82/68.28 Hz and rough2/2 at69.85/70.00 Hz; no consistent speed gain versus diagnostic-on, so do not claim diagnostics explains the user's60 Hz.
Storage interruption: the first torque0.75 rough run reached the end upright but could not save results (`ENOSPC`); exclude it as a completed trial. Stopped only its owned process group236147.
Archived older owned trial directories from Docker `/tmp` into verified local ignored `tuning_results/tangent_penalty_1031/previous_trials_archive{,_2}.tar.gz`, then removed only those archived originals to free space. Old raw-artifact `/tmp` links may now require extraction from these archives.
The retried torque0.75 rough run stayed upright but NO_TRACKING (pose-body vx0.497 for0.2 command), achieved60.84 Hz. Archive transfer overlapped part of this repeat, so its timing is not a clean penalty-only measurement.
Torque mu1/delta2 completed flat/rough at67.97/69.32 Hz, but rough mean vx0.339 for0.2 command. Friction-only mu0.4/delta5 completed both, rough70.07 Hz and vx0.241. No material speed gain from these isolated barrier changes.
Next existing-parameter tests: dt0.025/torqueNodes8 and dt0.03/torqueNodes7, all other parameters baseline. These retain horizon1 and one SQP iteration, with torque windows0.1875/0.195 s versus baseline0.19 s.
Do not extend the torque window inadvertently when comparing dt, since duration=(nodes-0.5)*dt. Rollout and gait timing are unchanged.
Initial mesh comparison: dt0.025/nodes8 passed flat and rough,75.50-76.17 Hz,12.12-12.20 ms, QP~6.36 ms versus baseline~8 ms. dt0.03/nodes7 passed flat but fell rough9.28 s,76.91-77.62 Hz; reject for saving.
Repeating dt0.025 on rough/ramp_entry, then comparing only friction mu0.4 with that mesh. No YAML values saved yet; all experiments are temporary harness overrides.
Completed bare dt0.025 repeats: rough1/3 success, ramp_entry1/3 success; reject as a standalone saved change despite speed gain. This corrects the earlier partial-log0/3 statement; completed JSON is authoritative.
dt0.025 plus friction mu0.4: rough1 success/1 upright NO_TRACKING, ramp_entry2/2 falls; also not promoted.
Next parameter-only pilots: horizon0.8 with dt0.02, torqueNodes10, switchDelayHorizonFraction0.375 (retain physical0.3 s switch delay); and dt0.025/nodes8 with only walk.swingHeight0.16 (scaled apex0.12 m, above first8.6 cm tile). Baseline penalties in both.
Horizon0.8 pilot passed flat, rough and ramp_entry at76.45/77.47/77.29 Hz, mean advance11.98/11.70/11.76 ms. Ramp-entry mean pose-body vx0.206 for command0.2; rough0.324. QP~6.2 ms, LQ~5.1 ms.
The clearance0.16/coarse-grid pilot still fell on ramp_entry11.46 s, so leave walking height unchanged.
Repeating horizon0.8 to three rough and three ramp-entry runs, then vy/yaw/run regressions, before saving any profile. Hardbounds/pinx0 and all cost/penalty weights remain baseline/off.
Horizon0.8 repeats are NOT uniformly successful: rough2/3 completed, ramp repeat2 fell. Keep those failures; do not promote based on the three pilot successes alone.
Next candidate retains dt0.02/horizon0.8 and tests the previously documented walk.swingHeight0.20 (scaled apex0.15), versus on-disk baseline0.10. This is a separate clearance comparison, not an orientation or solver implementation change.


## Stopped Checkpoint (2026-10-30, 15:52 UTC)

User requested stopping experiments and leaving a buildable checkpoint for manual testing.
The single-job Jazzy controller build passed (5 min 31 s); native inequality/filter tests passed 3/3, fixed-initial-state regression 1/1 and existing SQP regressions 8/8.
No owned simulator, RViz or test harness was running at the final process check; do not interfere with the user's subsequent manual tests.
No new commit or push was made for this unfinished investigation.

**Next session: fine-tune existing penalty parameters FIRST, rather than trying new implementations.**
Isolate torque and friction `mu`/`delta`, inspect residual units and gradient/curvature scales, change one group at a time and repeat matched Euler/tangent trials.
The prior combined stronger-barrier experiment caused early stance falls; it is not a systematic sweep or evidence that sensible penalty tuning cannot help.
Polynomial torque penalties have not been compared; do not add a new selector, hard-bound implementation or anchor redesign before exhausting the requested existing-parameter work.
Keep `jointTorqueLimits.hardBounds: false` and `solver.sqp.keepInitialStateFixed: false` unless the user explicitly redirects this priority.

Saved H2 candidate: `tangent_space`, walk swing height 0.20 m (0.15 m scaled apex), one SQP iteration, 1 s horizon, torque window 10 nodes, requested MPC80/MRT1000.
Fallback swing height remains 0.10 m; G1 and saved Q/R, friction/torque penalties, scenes, launch and estimator parameters are unchanged.
Best batch completed 3/3 isolated rough, 3/3 default-scene and four flat-motion cases; one surviving rough trial dipped to GT z=0.589 m.
Later saved-YAML long traversal fell at 30.41 s after the ramp-end drop; shorter ramp trials were Euler 3/3 versus tangent 2/3 plus a severe-collapse tracking failure.
Therefore this is a testable experimental checkpoint, NOT a reliable terrain fix or proof tangent outperforms Euler.
Actual instrumented solve rates were roughly tangent 67-70 Hz versus Euler 73-74 Hz at requested 80 Hz.

### Uncommitted Changes Versus HEAD

| Repository / HEAD | Current changes |
| --- | --- |
| Legged `feat/tangent_space_orientation`, `5e088ef` | H2 Euler-to-tangent default and walk height 0.10-to-0.20; plan/first-node diagnostics and capture/analysis tooling; native generated-constraint derivative regression; benchmark cases; optional hard-torque-bound wiring; documentation |
| OCS2 `jazzy`, `b4954f7a4` | Optional measured-x0 pinning; optional projected inequalities passed to HPIPM; inequality-aware filter/convergence; native regressions |

Both new solver options default off; hard bounds retain the existing torque RBF but have NOT been robot-tested.
The supported experimental hard path requires state/input equality projection; unprojected mixed constraints and state-only equalities are rejected explicitly.
Native tests do not establish nonlinear feasibility or recovery robustness after partial SQP updates.
Current uncommitted changes do not replace the tangent orientation math already committed in `3186b85`, change G1, or alter the estimator/launch/terrain.
Preserve the user's external `.gitignore` commit and OCS2 metadata commits.

Confirmed observations: generated RNEA/foot derivatives agree with independent checks (worst scaled error about 1.40e-7); failing plans remain finite with valid anchors but can request excessive torques/accelerations and negative contact forces.
Execution clips feedforward torque, creating plan/execution mismatch; Euler can also exhibit these violations and terrain falls.
No new current SO(3) singularity/NaN defect was identified by these checks.
Earlier sections below are chronological and contain superseded running/next statements; this stopped checkpoint takes precedence.

Possible later commit subjects, only after validation:

```text
FEAT: add opt-in WB-ID feasibility diagnostics and torque bounds
FEAT: support opt-in projected SQP inequalities
```


## Current H2 Checkpoint (2026-10-30, 15:04 UTC)

Saved H2 now selects `tangent_space` and `gait.templates.walk.swingHeight: 0.20` (15 cm reference apex after duration scaling).
G1 is unchanged and retains Euler; H2's fallback swing height remains 0.10, trot/run heights and all cost/gain/constraint values are unchanged.
Keep one SQP iteration, H2 torque window 10 (0.19 s) and `solver.sqp.keepInitialStateFixed: false`.
The clearance-only tangent profile completed six terrain trials (3 rough + 3 default scene) and four flat trials (vx=0.2, vy=0.25, yaw=0.6, run vx=1.0), requested MPC80/MRT1000, InEKF, GUI false/headless.
One rough trial recovered from a deep crouch (min GT z=0.589 m), and soft torque/friction penalties still permit infeasible optimized inputs. Do not claim the numerical/actuation-feasibility problem is solved or all runs are smooth.
Same-clearance Euler completed rough 3/3 and default 1/3; two late falls were in stopping after the ramp edge, which drops ~0.35 m at x=5.5, not first terrain entry. Both scenes remain unmodified.
Tangent achieved about 67-70 Hz with detailed diagnostics, versus Euler about 73-74 Hz; it is not faster on this VM.
Final no-override saved-YAML smoke `/tmp/tangent_numerical_saved_checkpoint` FELL at 30.41 s, x=6.956 m after walking off the ramp end during ramp-out. It did not repeat the early entry failure, but invalidates any claim that the long default-scene traversal is reliably solved.
The log still had walk active before this fall; no decreasing-to-stance event was logged, so do not blame premature stance without evidence.
The selected profile is a candidate checkpoint, not a fully robust terrain solution. A separately named `ramp_entry` case (vx=0.2, hold=10 s) is running three A/B repeats in `/tmp/tangent_numerical_ramp_entry` to isolate ramp ascent/stopping before the 0.35 m end drop. Do not replace or omit the failed long-traversal result.
Completed short-ramp matrix: Euler 3/3 success, tangent 2/3 success plus one NO_TRACKING with a large brief collapse (min GT z=0.383 m, pitch >1 rad). This invalidates promoting the clearance-only profile as reliably robust even before the ramp end.
Next compute-budget ablation `/tmp/tangent_numerical_short_rti`: horizon0.5, two SQP iterations, switchDelayHorizonFraction0.6 (keeps the old 0.3 s gait-switch delay), same clearance/gains, two repeats each of rough, short-ramp and long default. All requested80Hz. Do not confuse this with the rejected two-iteration/full-1s-horizon experiment.
Completed short-RTI profile: rough 2/2, short-ramp 2/2, long default 1/2; final long trial fell during overspeed. Actual rate ~65-67 Hz and hold speeds often 0.28-0.47 m/s for command0.2; do not promote unchanged.
New `/tmp/tangent_numerical_short_rti_vx60` repeats that profile with only running/terminal forward-velocity weights raised from30 to60, retaining every other weight. This tests braking/tracking without increasing lateral/yaw stiffness.
Pose-derived GT/estimated forward speeds agreed in sampled successful and failed records (RMS errors ~0.012-0.021 m/s), so the observed overspeed is not explained by a large estimator velocity bias in those records. The plotter now includes pose-derived GT vx.
The vx60 profile also had terrain falls, so it is not promoted. Current work is no longer assuming more gain tuning alone will close the gap.

Opt-in hard-torque-bound work is now in progress, NOT robot validated: `constraints.jointTorqueLimits.hardBounds` defaults false, adds the same torque-window constraint to the OCP inequality collection while retaining the existing relaxed-barrier penalty, and enables the new OCS2 SQP inequality path.
Reference `wb-mpc-locoman` uses hard torque bounds plus ordinary quadratic effort regularization, hard contact-force inequalities, and skips initial-node foot velocity constraints to avoid conflicting with the measured initial state. It does not use the same torque-limit RBF. Fatrop example allows up to10 iterations; our original path uses one SQP iteration.
Primary upstream references: [OCS2 issue30](https://github.com/leggedrobotics/ocs2/issues/30), [issue120](https://github.com/leggedrobotics/ocs2/issues/120), and [HPIPM example](https://github.com/giaf/hpipm/blob/master/examples/c/example_d_ocp_qp.c). SQP previously stored inequality linearizations but did not forward them to HPIPM; the existing line-search norm also ignored inequality SSE.
The new path uses existing equality projection, transformed state/input inequalities, HPIPM lower bounds with upper masks disabled, initial-state elimination, and opt-in inequality-aware line search/convergence. Legacy settings remain off.
Initial low-level bound/mask-reset and projected-SQP bounds tests pass. Unprojected mixed equalities/inequalities failed convergence; that unsupported combination is now rejected explicitly rather than advertised as supported. A trial of HPIPM equality metadata did not fix it and was removed.
Native tests also cover terminal bounds, rejection of the unsupported mode and opt-in filter behavior; the latter fixture needed merit as well as cost initialized (the filter compares merit). Final native regression and controller rebuild are running before any robot hard-bound experiment.
Hard bounds may expose genuinely incompatible contact/RNEA/actuator requirements; they do not make infeasibility impossible or guarantee nonlinear feasibility after partial SQP updates. Both a penalty and a hard bound can coexist, but the penalty does not enlarge the hard feasible set.
Detailed original and new tables, native validation, rejected ablations and limitations are in `docs/wb_id_tangent_orientation.md`.
To reproduce the original Euler baseline rather than the current profile, select Euler AND restore H2 walk.swingHeight to 0.10; selecting the flag alone does not undo tuning.


## Uneven-Terrain Numerical Investigation (2026-10-30)

Latest user report: H2 often falls violently just after starting uneven-terrain walking; investigate implementation/numerical/optimization errors, not only gain tuning.
User authorizes communication with Claude through the ignored `chore/agent_chat_box.md`; read it periodically and before ending, append ACKs/results, and do not disturb Claude's read-only workflow.
Existing user `.gitignore` edits are unrelated and must be preserved.
OCS2 local `jazzy` is behind origin by two metadata/VCS commits, with no runtime source difference; do not conflate those with this investigation.
Later status check: OCS2 HEAD is now `b4954f7a4`, matching origin/jazzy and including those metadata commits; local runtime investigation edits remain uncommitted. Preserve the externally updated metadata.

Initial actions: walking harness now saves `~/solver` per-solve named cost/equality metrics to `<prefix>_solver.csv` and copies the existing SQP ring log to `<prefix>_sqp.csv` after shutdown.
Fresh original-setting 80 Hz baseline (`/tmp/tangent_numerical_investigation_baseline`) completed rough Euler/tangent and default-scene Euler; default-scene tangent stayed upright but failed tracking (heading deviation), so it is not counted as a success.
Pointwise near-horizon RNEA residual SSE can exceed 1e6 even in surviving runs; time-integrated global SSE alone is insufficient, especially near tiny event intervals.
Existing OCS2 SQP ring logs show frequent half/quarter steps near failure with only one SQP iteration; no evidence of a thrown QP failure, and zero-step rejection was not present in the sampled old failures.
Do not yet call underconvergence or the measured-state boundary a confirmed root cause.

New native `wb_id_tangent_constraint_test.cpp` checks generated base/joint RNEA values against native Pinocchio and every state/input Jacobian column against independent perturbations.
The initial RNEA-only regression passed at phi=0, 1e-8 and 0.2, absolute pitch 0 and 1.2 rad, nonzero velocities/accelerations/contact wrenches; worst scaled derivative error 1.40e-7.
The regression is being expanded to stance/swing foot acceleration equalities.
Solver diagnostics are being extended with first-node full state/input, anchor norm/drift, maximum chart magnitude, minimum node interval and acceleration/torque/velocity extrema, without changing the OCP or gains.
Expanded stance/swing foot derivative checks also passed at all six native cases.
Three current tangent default-scene diagnostic trials (`/tmp/tangent_numerical_plan_diagnostics`) gave NO_TRACKING, SUCCESS and FALL (12.29 s).
The failing plan stayed finite, with unit/constant anchors to 2.3e-16 and chart norm below 1.1 rad, but requested generalized acceleration up to 17746, near torque/limit ratio 9.0, and negative normal force down to -2019 N.
The first planned joints differed by as much as 0.45 rad from interpolated same-time observations before 12 s; observation sampling/very high joint speeds also contribute, so native fixed-boundary regression is needed.
Two-iteration SQP ablation (`/tmp/tangent_numerical_two_sqp`) fell in all three trials and reduced achieved rate to about 32-34 Hz; reject this as a robustness fix.
Terrain inspection: near x=0.26, right/left y=-/+0.1 tile tops are 0.085964/0.053727 m above the clear floor. The 2-5 cm adjacent-tile bound explicitly excludes clear-patch edges. Current walking swing apex is only 0.075 m after duration scaling.
An opt-in OCS2 `sqp.keepInitialStateFixed` experiment now pins x0 to the measurement before linearization; legacy defaults remain false. This prevents a partial SQP step from diluting the initial-state correction, but does not by itself enforce nonlinear dynamics/contact/actuator feasibility.
Rebuild and compare that option before promoting it; no successful fix is claimed yet.
The deterministic SQP boundary regression now passes on a bilinear system with a nonlinear Gauss-Newton objective: the legacy path demonstrably returns x0 different from the new measurement under backtracking, while `keepInitialStateFixed=true` retains the measured x0 to 1e-12.
The 8 pre-existing SQP tests pass, and the Release rebuild with the opt-in option passed (controller 5 min).
Unchanged-gain 80 Hz tangent trials with only `keepInitialStateFixed=true` are running in `/tmp/tangent_numerical_fixed_initial` (three default-scene and three isolated rough-field repetitions).
Completed: fixed-initial-state default-scene trials fell 3/3; rough trials stayed upright 3/3 but only one passed tracking, with overspeed in the others. This option fixes the initial-boundary property, not terrain robustness; leave it false.
Removing only stance height/orientation restoration (`positionErrorGain_z=0`, `orientationErrorGain=0`) passed one default-scene run but fell on isolated rough ground; do not save this ablation.
Increasing torque/friction relaxed-barrier strength (`jointTorqueLimits.mu=10, delta=1`, `frictionCone.mu=2, delta=1`) caused six early stance falls, before walking. Reject it; stronger penalties are not automatically a feasibility/conditioning improvement.
A clearance-only ablation with `ocs2.gait.templates.walk.swingHeight=0.20` is running in `/tmp/tangent_numerical_clearance_020`; first three default-scene runs completed 36 s with progress, but rough-ground repeats and replay inspection are still needed before promotion.
All saved gains, terrain geometry and initial-state option remain unchanged at this point.
Completed clearance-only matrix: tangent 6/6 completed (three default scene, three isolated rough); one rough run recovered from a deep crouch (minimum GT base height 0.589 m), so do not describe all six as smooth.
Euler with the same clearance completed 3/3 isolated rough and 1/3 default-scene runs, with two late falls. This corrects the earlier live-tail reading that missed repeat 2's fall; the completed comparison JSON is authoritative. These small, sequential batches do not prove tangent is intrinsically superior.
Before the first 12 s, a representative successful tangent run reduced initial attitude/joint mismatch to 0.0011/0.0112 rad and avoided negative planned world-z force, compared with the original failed run's 0.043/0.449 rad and -2019 N.
However, the recovering rough run still produced near-term torque/limit ratio 20.27 and negative force later; clearance prevents many first strikes but does not enforce feasible plans.
An existing-parameter ablation with full-horizon joint RNEA (`torqueConstraintNodes=0`) plus 0.20 m clearance is running in `/tmp/tangent_numerical_full_rnea` (two rough and two default repetitions). First rough succeeded; second stayed upright but stalled/NO_TRACKING. Leave the saved H2 window at 10 (0.19 s); the diagnostic first-three-node metrics are not the torque-window length.
Completed full-horizon ablation: one success and one NO_TRACKING in each scene; the second default run barely advanced. Reject for promotion despite a smaller/faster projected QP.
Final clearance-only tangent flat regression is running in `/tmp/tangent_numerical_clearance_020_flat`: walk vx=0.2, vy=0.25, yaw=0.6 and run vx=1.0, all requested 80 Hz. The new `walk` benchmark case distinguishes low-speed walking from the existing `vx=0.5` trot case.


## Tangent-Space Orientation Experiment (2026-10-30, Opt-In Checkpoint)

Work on the existing `feat/tangent_space_orientation` branch; quaternion decision-variable experiments are explicitly deferred.
Latest user instruction: commit and push both repositories once this checkpoint is mature and validated.
The Euler fallback remains available; do not describe tangent terrain robustness as mature based only on flat-ground success.
The user authorizes building/testing in Parallels `ros_jazzy` and changes to `ocs2_ros2` if necessary.
All simulator tests use isolated scenes, headless execution and `baseCommandGui:=false`.

Current validation summary: both representations pass flat vx/vy/yaw/run and the mild 50 N push, but tangent is not yet an uneven-terrain robustness improvement.
Keep `euler_zyx` as the saved default for both robots; tangent remains selectable on this experimental feature branch, not a validated terrain replacement.
At MPC 80/MRT 1000, original-setting rough trials complete Euler 3/3 versus tangent 1/3; the default scene gives Euler one success, one tracking failure and one fall versus tangent three falls.
Three flat vx trials pass for both, with mean achieved rates 77.06/73.34 Hz and reported mean solve times 12.18/13.62 ms (Euler/tangent).
No solver exceptions or torque-window overruns occur in the 26 baseline comparison trials.
The native model tests now also verify identical physical RNEA results after Euler/tangent velocity, acceleration and force-coordinate conversion.
OCS2's new `tangent_math_tests` target passes three GTests, including generated complete-flow derivatives at zero and generated `atan2` Hessians on axes/quadrants.
The full scenario table, frame differences, caveats and reproduction commands are in `docs/wb_id_tangent_orientation.md`.
The paragraphs below are chronological development history: earlier statements about pending tests or superseded AD helper choices do not describe the current implementation.

Reference inspection: `wb-mpc-locoman/dynamics/dynamics_whole_body_torque.py` uses Pinocchio/CasADi `integrate` and `difference`; `optimization/ocp_whole_body_rnea.py` optimizes `DX_opt` of dimension `2*nv`, reconstructs `q = integrate(q_measured, dq)`, and advances fixed-anchor increments with explicit Euler `dq_next = dq + v*dt`.
The paper's quaternion is reconstructed storage, not a four-component free orientation decision with a unit-norm constraint.
Our experiment keeps measured-anchor three-component rotation increments, world linear velocity and body-local XYZ angular velocity; Pinocchio uses Translation + Spherical instead of Translation + SphericalZYX.
OCS2 currently carries the measured anchor in four zero-flow state slots fixed by the initial condition, with zero anchor cost; these are bookkeeping, not free quaternion decisions.
Warm-start states must be rebased to each new measured anchor before trajectory interpolation; an optional SQP warm-start transform was added for this purpose.
The exact chart flow uses the inverse SO(3) right Jacobian, rather than treating a fixed-anchor rotation increment's derivative as body angular velocity at arbitrary increments.
Do not claim this is an exact reproduction of the paper's approximate explicit-Euler transcription.

Baseline before modifications: H2, InEKF feedback, flat, vx 0.5 m/s, MPC 90/MRT 1000, 5 s settle + 3 s ramp + 25 s hold + 3 s stop + 5 s rest.
`/tmp/tangent_h2_euler_flat`: SUCCESS, 41 simulation seconds, hold mean vx 0.5361 m/s, minimum base height 0.9917 m, max roll 0.01265 rad, max pitch 0.02729 rad, max heading error 0.06285 rad.
These files are inside the Docker container; no tangent walking result exists yet at this checkpoint.

First native build failed in Pinocchio `log3/Jlog3` for `CppAD::AD<CppAD::cg::CG<double>>`, at Boost's pi/numeric-limits machinery.
This is not evidence that Pinocchio cannot do AD manifolds: the reference repo uses CasADi, and upstream Pinocchio has CppAD and CppADCodeGen specializations.
Upstream references checked: [AD spatial tests](https://github.com/stack-of-tasks/pinocchio/blob/devel/unittest/cppad/spatial.cpp), [AD integrate/difference tests](https://github.com/stack-of-tasks/pinocchio/blob/devel/unittest/cppad/joint-configurations.cpp), and [CppADCodeGen backend](https://github.com/stack-of-tasks/pinocchio/blob/devel/include/pinocchio/codegen/cppadcg.hpp).
A native probe with the backend include first compiled and ran generated-library validation.
Direct matrix `Jlog3(Exp(phi))` produced NaN generated derivatives at phi=0 despite finite values; the quaternion log path itself had finite derivatives.
Pinocchio quaternion `exp3/log3` plus `Jexp3(phi).inverse()` passed generated derivatives at zero, tiny angles, a large tilt near pi/2 and a 2.8-rad rotation, with values compared against native Pinocchio `log3/Jlog3`.
Use this existing upstream Pinocchio path; no custom SO(3) math helper or finite-difference MPC derivative remains.
Temporary custom AD-compatible SO(3) helpers/tests were removed after the successful upstream probe.
Reproduce the native code-generation test in a sourced workspace with `python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py`.
There are no finite-difference derivatives in the MPC; numerical checks are tests only.
The first full main rebuild passed with the temporary helpers; the rebuild with the selected Pinocchio-only math is running.
`tests/wb_id_tangent_model_test.cpp` passed native H2 FK, world-linear/body-angular twist mapping, local-chart integration against `pinocchio::integrate`, generated flow Jacobian finiteness, analytic Pinocchio-state-map Jacobians against independent perturbations, and physical pose preservation under rebasing.
The native cases include zero rotation and absolute attitudes near/past +/-pi/2 pitch; these are model-math checks, not a claim that a humanoid can walk pitched 90 degrees.
Run with `python3 src/legged_robot_mpc_controller/tests/run_tangent_native_tests.py --test wb_id_tangent_model_test`.
No tangent closed-loop result exists yet; do not call this checkpoint ready for user testing.

First tangent simulator attempt `/tmp/tangent_h2_flat_v1` ended with STARTUP_ERROR during base-cost code generation (`GreaterThanOrZero cannot be called for non-parameters`), before MPC takeover.
This is a backend scalar-dispatch/include-order failure, not a walking instability.
The selected backend must be included before Pinocchio math template definitions; the next patch makes that explicit in WB-ID public entry headers and its controller translation unit, without forcing a new include on unrelated Pinocchio users.
Also corrected the startup upright target to use model accessors instead of directly zeroing Euler indices, which mean local rotation increments in tangent mode.
The following two-job rebuild was killed in `cc1plus` while another native test was also compiling; retry with one build job and no concurrent compiler workloads.
Inspection also identified a concrete illegal CG branch: Eigen's quaternion `normalize()` delegates to a vector normalization that tests `squaredNorm() > 0` in ordinary C++.
Replaced it with algebraic normalization in the tangent state decoder; no custom manifold formula or finite-difference derivative is involved.
The 23 WB-ID parameter/activation/startup/torque contract tests pass in the Jazzy container.
Full local `test_*.py` discovery still has the five pre-existing `test_none_removes_every_environment_module` include-order subcase failures (56 tests, 13 skips); do not claim a clean full-suite run.
The old flat replay has zero opposite-leg overlap/crossing samples, minimum signed foot width 0.125 m and final stagger 0.033 m; native pose replay is latest-sample paired rather than force-contact truth.
Added `tests/tangent_orientation_benchmark.py` to compare unchanged gains under Euler/tangent modes serially, with vx/vy/yaw/run, rough-only/default-scene terrain and an optional existing 50 N pelvis-push case.
It writes raw harness outputs plus comparison CSV/JSON and reports solve timing, exceptions and torque-window overruns; startup failures abort the matrix instead of being mislabeled as walking failures.
The single-job Release rebuild passed (13 min 42 s after broad header recompilation); no simulator ran during the rebuild.
Next gates are the expanded native base-cost generated-gradient/gauge checks and a fresh tangent simulator startup.
Expanded native testing caught NaN base-cost orientation gradients at exact upright with the matrix-to-quaternion error conversion (cost value itself was zero).
The correction forms full/heading-removed relative quaternions directly and calls Pinocchio quaternion `log3`; no matrix-to-quaternion conversion is needed in this base objective.
The cost cache name advances to `tangent_base_motion_v2` to prevent reuse of the failed expression.
Further compiler diagnostics exposed an actual initialization bug in that prototype: plain integer zeros in an Eigen comma initializer with the nested AD/CG scalar were not populating the heading matrix as intended.
Replaced it with a fully initialized AD identity matrix plus explicit entries, and explicitly typed the quaternion zero coefficients.
Therefore, do not attribute the earlier base-cost NaNs to an upstream matrix-to-quaternion defect; the standalone upstream matrix-Jlog3 derivative failure is separate and remains directly reproduced.
The subsequent isolated generated `atan2(y,x)` probe reproduced NaN Jacobians at `(0,1)`, while raw Pinocchio quaternion log had the correct finite Jacobian at identity.
OCS2 bundles an older CppAD `atan2` with `fabs` and eager divisions; upstream CppAD uses conditional absolute values and corrected quadrant selection ([source](https://github.com/coin-or/CppAD/blob/master/include/cppad/core/atan2.hpp)).
Backported that upstream implementation in `ocs2_thirdparty/include/cppad/core/atan2.hpp`, additionally guarding only inactive denominators for code generation (the selected ratio is unchanged).
The generated axis derivatives now match analytic `atan2` derivatives on all four axes and a non-axis point; no finite differences are used in control.
The expanded native model test, compiling the current cost source directly, now passes identity/nonzero base-cost gradients, quaternion-sign invariance and common planar-gauge invariance, in addition to FK/integration/mapping/rebasing checks.
Code-generation cache folders advance to `wb_id_tangent_v3_<robot>` and base cost to `tangent_base_motion_v3` so previous broken generated functions are not reused.
The next step is a dependency rebuild and the first successful closed-loop attempt; no tangent walking success is claimed yet.
The full-pose (`base_pose`, not relative walking) base residual now uses world-frame velocity errors and bypasses heading extraction, so that cost need not reintroduce a heading singularity at vertical pitch.
Added generated-cost finiteness cases at +/-90 degrees and beyond, and a native Euler-rate conditioning comparison; these expanded checks are queued after the rebuild.
The 23 WB-ID contract tests passed again during the dependency rebuild.
Full dependency build succeeded: 21 packages, 19 min 1 s wall time, one compile job; controller package 14 min 49 s.
Both native regressions passed against the rebuilt libraries, including full-pose residual Jacobians at +/-90 degrees and beyond, plus analytic generated `atan2` Jacobian/Hessian checks in every quadrant and on the axes.
Native rate-map condition numbers at absolute pitch 0/60/80/89/89.9 degrees: Euler 1/3.732/11.430/114.589/1145.92; tangent at each newly measured anchor (phi=0) is 1.
This is coordinate conditioning evidence, not evidence of walking or recovering with the humanoid pitched 90 degrees.
The first current closed-loop test is `/tmp/tangent_orientation_benchmark/h2_vx_tangent_space_1` in Docker; no gains were changed.
That attempt compiled/configured all generated libraries, but stayed in the stand hold before receiving any policy.
A debugger sample found the solver inside HPIPM QP residual computation, not waiting on the observation handoff; the run was stopped before any walking.
Investigating QP dimensions/finiteness with the existing `printSolverStatus` flag before changing gains; do not classify this as a walking fall or claim tangent closed-loop success.
The QP diagnostic isolated nonfinite discrete dynamics A matrices before equality projection; equality values/Jacobians and costs were finite.
An expanded native regression reproduced this directly: flow Jacobians at phi=0 are NaN for the composed Pinocchio `Jexp3(phi).inverse()*omega`, including omega=0, despite the earlier matrix-only generated probe passing.
Thus the earlier probe was insufficient to validate the flow at zero; RK4 sensitivity checks at zero are now mandatory.
Upstream Pinocchio's current `Jexp3` still forms `1/norm(phi)` before selecting its small-angle branch ([source](https://github.com/stack-of-tasks/pinocchio/blob/devel/include/pinocchio/src/spatial/explog.hxx)).
A small `ocs2_robotic_tools/common/So3Tangent.h` adapter now uses the standard inverse-dexp formula and near-zero series documented in [GTSAM SO3.cpp](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SO3.cpp), with a guarded inactive norm before sqrt/division for AD/codegen.
Pinocchio exp/log and spherical FK/RNEA remain in use; no finite-difference control derivative is introduced.
The flow cache changes to `tangent_dynamics_v2`; revalidate native zero-flow/RK4 tests before another simulator attempt.
The expanded native zero-increment flow/RK4 tests now pass at omega scales 0, 1e-20, 1e-15, 1e-10, 1e-5 and 1, together with the previous model/base-cost checks.
Temporary QP diagnostic prints/checks were removed; the remaining SQP change is only the optional warm-start transform.
Incremental build passed (23.3 s).
First current tangent closed-loop SUCCESS: `/tmp/tangent_orientation_benchmark/h2_vx_tangent_space_1`, H2 flat, InEKF, vx=0.5, MPC90/MRT1000, 41 s including start/stop; mean vx=0.53083, min z=0.99154 m, max roll=0.01229 rad, max pitch=0.02472 rad, max heading error=0.04570 rad.
Reported solve averages mean=13.253 ms, worst logged solve=57.612 ms, mean actual rate=75.21 Hz; zero solver exceptions and torque-window overruns.
No tuning gains changed; the next check is tangent rough ground, then paired Euler/tangent scenarios.
The preliminary 90 Hz tangent rough-only run also passed 36 s including stop/rest at vx=0.2: mean body vx=0.28367, min z=0.92443 m, max roll=0.14591 rad, max pitch=0.09600 rad; zero exceptions/torque-window overruns.
Logged maximum dynamics SSE was 291.5 on that rough run, so survival must not be described as a tightly converged terrain solution.
Latest user instruction: all further H2 runs at requested MPC 80 Hz, compare achieved solve rates for both modes, and separate Euler/tangent YAML comments.
The authoritative paired matrix is now `/tmp/tangent_orientation_benchmark_80hz`; keep the 90 Hz runs as preliminary data, not mixed into its timing comparisons.
First 80 Hz pair (vx=0.5, 41 s start/walk/stop): both SUCCESS; Euler mean vx=0.53312, achieved=75.665 Hz, reported mean solve=12.696 ms; tangent mean vx=0.53223, achieved=72.617 Hz, reported mean solve=13.746 ms.
Tangent was about 4% lower in achieved rate in this first pair, not faster; repeated measurements are needed before a broad timing claim.
SQP per-iteration breakdown: Euler LQ/QP=5.337/6.891 ms; tangent LQ/QP=5.632/7.727 ms, so the larger QP (four fixed anchor state slots) accounts for most of this observed overhead.
Both had zero exceptions and torque-window overruns; max heading error Euler=0.06257 rad, tangent=0.20219 rad, showing one trial does not establish better heading behavior.
Current YAML comments separately describe Euler vs tangent state, base costs and base-acceleration conventions; user-added inline mode-choice comment is preserved.
Benchmark measurement caveat: H2's `/mujoco/ground_truth/odom` twist is not aligned with its published pose axes.
The publisher calls `mj_objectVelocity(..., mjOBJ_BODY, ..., local=1)`, which returns inertial-body axes; the pelvis inertia principal axes are rotated roughly 90 degrees about X.
Local synthetic check at identity pose with true world vy=0.25 and wz=0.6: `mjOBJ_BODY` returned angular approximately [0.0088,0.5999,0.0013] and linear [-0.0029,0.00058,-0.2501], while `mjOBJ_XBODY` returned exactly [0,0,0.6,0,0.25,0].
Do not use the legacy raw `walk_mean_vy/wz` fields as physical tracking scores for these runs; the comparison/plots now derive body-horizontal speeds and yaw rate from the GT pose trajectory.
This does not change the tested control loop (InEKF feedback) and the simulator repository remains unmodified; investigate/fix its body-vs-inertial convention separately before using GT twist as controller feedback.
80 Hz lateral pair, vy command 0.25, 41 s: both SUCCESS; pose-world mean vy Euler=0.23884, tangent=0.23575 m/s; rates=75.907/70.263 Hz, reported mean solve=12.738/14.304 ms, zero exceptions/window overruns.
80 Hz yaw pair, command 0.6 rad/s, 46 s: both SUCCESS through repeated heading wraps; total rotations Euler=18.32657, tangent=18.34341 rad; maximum translation drifts=0.15788/0.12453 m.
Yaw pair achieved rates=75.367/72.359 Hz; mean solve=12.919/13.806 ms; max roll/pitch below 0.014 rad in either trial, zero exceptions/window overruns.
These are single trials; the direction of yaw drift and rare solve-time spikes must not be treated as statistically established representation advantages.
Latest user now authorizes commit/push of both legged and OCS2 once the checkpoint is mature; this supersedes the earlier no-commit instruction.
Frames: Euler angles parameterize the world-referenced attitude, but Euler rates are not physical world angular velocity; they are a configuration-dependent transform of body gyro velocity.
Tangent reconstructs the same world attitude using an anchor-local right increment and uses physical body XYZ angular velocity, so it is not an entirely local/world-free controller.
Same numeric angular weights do not define the same physical metric between these paths; tangent removes Euler-rate conditioning/noise amplification, not actual estimator uncertainty.
Detailed comparison and limitations were added to `docs/wb_id_tangent_orientation.md`.
80 Hz rough pair: Euler survived the 36 s trial, tangent fell at 9.18 s after takeover (about 1.18 s into the full command), min z=0.260 m, max roll=1.338 rad.
No solver exceptions/window overruns occurred; tangent's preliminary 90 Hz rough run survived, so investigate partial footholds/clearance and repeat rather than claiming orientation alone improves terrain robustness.
User specifically requests repeated uneven-terrain trials after falls; one successful/failed run is not enough to rank the representations.
User prefers the physical body-angular-velocity state; clarify that it is not generally phidot (the inverse-right-Jacobian map is required).
Setting `orientationRepresentation: euler_zyx` selects the retained baseline model/dynamics/base cost; no saved tuning gains have changed, although shared dependency fixes remain and timing makes trajectories nondeterministic.
Terrain swing-height experiments are temporary `--set` overrides applied equally to both representations, not saved YAML changes.
Push harness cleanup exposed an existing bug: spinning after closing CSV dispatched an odometry write to the closed file; cleanup now sends zeros without dispatching callbacks, and shutdown is guarded against already-shutdown ROS contexts.
The aborted first push attempt had completed the physical trial, but its cleanup prevented a result JSON; it is rerun, not counted as a validated recovery.
The first rough tangent failure replay has zero opposite-leg mesh overlap samples and minimum foot-point distance 0.187 m; signed-width crossings appear only as the robot is already falling.
Snapshots around t=7.5 s show feet at the boundary of the clear floor opening; the first swing rises only about 7.5 cm (0.10 m template height scaled by the 0.3 s swing), consistent with an entry-step/partial-foothold problem rather than leg collision, but no contact-force truth was recorded for that diagnosis.
Trying a shared temporary walk swing height of 0.15 m for both modes, in `/tmp/tangent_orientation_80hz_clearance`, without changing saved gains or gait YAML.
80 Hz 1.0 m/s run/start/stop pair: both SUCCESS for 46 s; Euler/tangent mean body vx about 1.018/1.006 m/s, achieved rates 77.263/72.313 Hz; min z 0.93781/0.93634 m; zero exceptions/window overruns.
80 Hz verified 50 N lateral pelvis push during vx=0.3: both SUCCESS, 41 s; rates Euler/tangent 76.327/70.930 Hz; min z 0.94556/0.91555 m. These are mild-push trials, not recovery-envelope limits.
Baseline default-scene first pair: Euler SUCCESS, tangent FALL at 11.07 s; repeat before ranking terrain robustness.
Temporary 0.15 m walk swing trials: rough Euler SUCCESS, tangent NO_TRACKING (upright but blocked); default scene Euler FALL near 35 s, tangent NO_TRACKING (upright but little progress).
Do not promote that swing-height change: it has not demonstrated a reliable improvement and survival without commanded progress is not a terrain-walking success.
Next batch repeats original saved settings at 80 Hz for vx timing and both terrain scenes, alternating A/B order between repeats.
Repeated flat vx runs now pass three of three in both representations; tangent repeat 2/3 achieved 72.659/74.745 Hz with mean solve 13.760/13.339 ms.
The second original-setting rough tangent trial fell at 10.16 s, while the second Euler trial completed 36 s; no solver exception or torque-window overrun occurred in either.
Continue the counterbalanced terrain repeats before drawing a robustness conclusion or pushing a final checkpoint.
Those repeats are now complete; see the current summary at the beginning of this section and the full table in the tangent document.
A further temporary tangent-only contact-compliance profile used foot height/tilt equality gains 25/50 instead of 50/100: rough falls at 8.75/13.11 s, default-scene falls at 22.40/9.22 s.
Reject that profile too; no gain or gait-height changes from the baseline are saved.
The integration is an opt-in experimental checkpoint with native/flat-ground validation, not a successful terrain-robustness upgrade; retain that qualification when committing or handing off.
Final single-job Release rebuild passed for `ocs2_sqp` and the controller (3 min 32 s); 42 focused Python tests, both native model/codegen executables and the three OCS2 math GTests passed.
Fresh post-build H2 80 Hz flat vx=0.5 smoke pair completed 41 s each: Euler/tangent achieved 78.24/75.64 Hz, mean solve 11.74/13.21 ms, pose-derived body vx 0.5359/0.5357 m/s, zero exceptions/window overruns.
Artifacts: Docker `/tmp/tangent_orientation_final_smoke`; baseline 26-trial matrix `/tmp/tangent_orientation_benchmark_80hz`; rejected profiles `/tmp/tangent_orientation_80hz_clearance` and `/tmp/tangent_orientation_80hz_soft_contact`.
Commit/push destinations: legged `feat/tangent_space_orientation`, OCS2 `jazzy`; do not merge the experiment into main or claim better terrain robustness from these results.
Required OCS2 dependency checkpoint: `ffdde1d7e` (`FEAT: support tangent-space SQP models`).

Commit messages:

```text
FEAT: support tangent-space SQP models

Add optional warm-start rebasing and codegen-safe SO(3) derivatives.
Backport the CppAD atan2 axis fix and cover generated derivatives.
```

```text
FEAT: add opt-in tangent-space WB-ID orientation

Use measured-anchor rotations and physical body angular velocity.
Retain Euler defaults and document repeated 80 Hz benchmark limits.
```


## Unified RViz Environment (2026-10-26)

- `description/urdf/env.urdf.xacro` is the RViz counterpart of a robot's MuJoCo `scene.xml`: one `world` link plus `<xacro:include>`s of `terrains/stairs`, `terrains/ramp`, `terrains/uneven_terrain` and `objects/boxes` `.urdf.xacro`; comment or uncomment to match the scene (default uneven terrain + ramp, as in H2's committed scene; G1's scene has no terrain).
- Each piece is a self-contained include: it attaches to `world`, keeps its properties inside a macro it calls itself (no collisions between pieces), and loads its params YAML by a path relative to itself (xacro resolves `load_yaml` from the including file), so no `*_params_file` args are needed. The uneven-terrain generator no longer emits a `world` link.
- `robot.launch.py`: the `environmentDescription` argument and the params-file plumbing are removed; the environment `robot_state_publisher` always publishes `env.urdf.xacro` as `/environment/robot_description`.
- Tests: `tests/test_robot_launch.py` checks `env.urdf.xacro` expands and that all four pieces combine with unique links under one `world`; launch, uneven-terrain and shared-terrain tests pass (16 passed, 2 skipped). Docs updated (README, `description/README.md`, `description/mujoco/README.md`, `config/terrains/README.md`, `config/objects/README.md`, `docs/h2_wb_id_mpc.md`).


## Torque Contact Detection and Hidden Height Drift (2026-10-26)

Goal set by the user: `contact.source: torque`, H2 walking and running robustly on flat and uneven terrain, proprioceptive only. Nothing committed yet.

New pieces (uncommitted):
- `mujoco_ros2_control` (repo `~/src/mujoco_ros2_control`): contact-wrench sensor. A ros2_control `<sensor>` with `mujoco_contact_body_name` exports the net contact wrench on that body (`mujoco_contact_include_subtree`, `mujoco_contact_frame` world|body, torque about the body origin), `normal_force` and `contact` (above `mujoco_contact_threshold`). Computed in a new `MujocoSystemInterface::post_step()` hook after `mj_step2`; `read()` runs after the next `mj_step1`, when contacts belong to the new step but forces are unsolved. H2 xacro: `left/right_foot_contact` on `*_ankle_pitch_link`, body frame (`use_contact_sensors`).
- `legged_state_estimator` (repo `~/src/mpc/legged_state_estimator`): `ContactEstimator` rewritten robot-agnostic. Solves all contact forces at once, `f = -(Jc Jc^T + lambda I)^-1 Jc (tau - tau_ID)` over the stacked 3-D contact Jacobians (no joint layout assumed); `contact_groups` (foot index) give one decision per rigid body on the summed normal force; `contact_release_probability_threshold` hysteresis; optional `contact_point_force_ratio` (per-point stance). Inverse dynamics now uses the estimated attitude; the dynamic mode no longer double-counts gravity (still unusable, see below). The detector runs every tick, also when contacts are supplied, so it can be scored in shadow.
- Estimator: `contact.source: sensor` reads per-foot flags from `contact.sensorInterfaces` (sim truth or foot sensors); torque mode feeds the detector's per-contact flags into the height logic (before, every contact counted as stance there). New diagnostics: `foot_stance.*`, `foot_normal_force.*`, `foot_torque_stance.*`, `point_normal_force.0-3`, `point_stance.0-3`. New params `contact.releaseThreshold`, `contact.pointForceRatio`, `contact.sensorInterfaces`.
- Harness: `<prefix>_internals.csv` at 200 Hz via a spawned `state_interfaces_broadcaster` (estimator internals, sim foot wrench, pelvis truth). Spawner on Jazzy takes the controller type from the params file (`type:`), not `-t`. Analysis: `tests/tuning/analyze_contact_estimation.py`.

Findings (H2 run circle vx 1.0 / yaw 0.7854, 60-90 s each, `tuning_results/inekf_tuning/`):
- Torque force estimate (non-dynamic) tracks the true foot force in stance (600-1300 N); swing reads -180..+280 N. The shipped logistic (50% at 29 N) gave 29% false stance. 50% at 350 N (`beta1 0.02, beta0 -7`), release ~200 N (`releaseThreshold 0.05`), `lpf.tauJCutoff 50`: shadow agreement with truth 97-99%, latency 0-5 ms, same as the schedule.
- `dynamicEstimation: true` is broken (swing reads ~2300 N, 64% agreement); suspects: finite-difference angular acceleration and the legacy `ddqJ` formula. Keep false.
- The schedule already agrees 97-99% with true contacts, and true per-foot contacts cut the internal height drift only from ~90 to ~58 cm/min.
- Against the 200 Hz pelvis truth the internal height drift (+140 cm in 90 s, scheduled) builds up in stance, not flight: +4.4 mm in the first ~25 ms after each touchdown, then flat (vz error spike +3.6 cm/s there). Likely a heel-strike effect: per-foot contact pins the toe landmark while the toe is still descending. Earlier "drift per flight" numbers measured against the kinematic height were an artefact (that reference is off at heel strike and toe-off).
- Published height with `holdThroughFlight` is fine regardless (p95 1.2 mm vs truth).
- Torque contacts in the loop: one 90 s run and one 90 s walk survived (internal drift +15 cm/min); the repeat and the per-point sweep (`pointForceRatio` 0 / 0.10 / 0.25) all fell (18-45 s). Failure chain: late liftoff release (stance held below 200 N through the torque filter) pins a lifting foot, vz bias -5..-12 cm/s from the start, rougher gait, swing force readings rise past the threshold, false stance 12%, fall. Per-point stance made it worse.
- `contact.forceCovarianceAlpha` is unscaled for this robot: measurement variance alpha*(dF per tick)^2 with alpha 10 makes even 1 N changes ~10 m^2, so in torque mode the legs are almost ignored.
- With alpha 1e-8 torque contacts work in the loop (flat). Release 200 / 300 / 350 N all survived 90 s running; vz error RMS 2.4-2.8 cm/s, better than the schedule (4.7). Then 10 min each, release 200 N:

| 10 min, flat, torque contacts | run circle vx 1.0 / yaw 0.7854 | walk circle vx 0.5 / yaw 0.7845 |
| --- | --- | --- |
| result | SURVIVED hold and stop | SURVIVED hold and stop |
| agreement with true contact (L / R) | 0.987 / 0.976 | 0.974 / 0.978 |
| touchdown / liftoff latency | 0-5 / 0 ms | 10-15 / 5 ms |
| vz error RMS | 2.7 cm/s | 0.6 cm/s |
| reported height p95 vs truth | 1.2 mm | 0.4 mm |
| control first vs last minute | vx 0.94, tilt 1.8 deg, unchanged | vx 0.51, tilt 0.6 deg, unchanged |
| internal height drift (hidden) | +48 cm/min | +4 cm/min |

Working torque settings (applied with `--set`, not yet in `config/h2/ros2_controllers.yaml`): `contact.source torque`, `beta0 [-7]x4`, `beta1 [0.02]x4`, `releaseThreshold 0.05`, `forceCovarianceAlpha 1e-8`, `lpf.tauJCutoff 50`, `dynamicEstimation false`, `pointForceRatio 0` (0.15 cut the internal drift only 45 -> 35 cm/min).
- The internal drift is the heel-strike effect above; it does not reach the MPC on flat ground (kinematic height + `holdThroughFlight`). A terrain-referenced height measurement inside the filter would bound it, but a proprioceptive terrain prior cannot predict the next tile, so it only helps once the per-step bias is small.
- Contact settling (new, library `contact_settle_time` / `contact_settle_position_noise`, params `contact.settleTime` / `settleNoise`): a contact uses 5 cm measurement noise for its first `settleTime`, also for the new landmark's covariance, so a landing foot moves its own landmark instead of the base. Run circle 90 s: internal drift torque 45 -> 19 cm/min, schedule 90 -> 22.5; vz error RMS 2.5 -> 1.5 (torque) and 4.7 -> 1.5 (schedule). 30 and 60 ms equal; ~1.4 mm per stance remains, in the first 20% of stance.

Terrain (blind, `terrain=rough`: 0.5 m tiles 0-0.10 m, 2-5 cm neighbour steps; harness `--terrain`, case token `terrain=`, MPC overrides `mpc.<path>`):
- Current setup falls in 13-15 s on the tiles with either contact source; the schedule then agrees with true contact only 55-78% while the torque detector stays at 96-99%.
- With `height.source kinematic` every stance foot reads z = 0, so `ocs2.reference.groundHeight.useEstimate` (swing aimed at the stance-foot height, body height target shifted with it) is inert. With `height.source inekf` it works, but the commanded pelvis height was absolute: `ProceduralMpcMotionManager` regenerates velocity targets every solve at z = command, and `adaptToCurrentGroundHeight` only applied the change of the ground estimate since the last solve. A fix (`SwitchedModelReferenceManager::setTargetTrajectories` marking targets fresh so they are lifted by the whole estimate once) was tested and then reverted at the user's request; it is not in the code, so `inekf` + `useEstimate true` still has an absolute pelvis height. Flat run circle with `inekf` + `useEstimate`: SURVIVED with true height 0.958 m steady although the filter frame drifted 26-34 cm (before the fix: true height sank 0.957 -> 0.845 and it fell at 97 s).
- Tile failures are partial footholds: a foot lands on its toe tip (CoP +0.14 m, toe contacts at +0.15) on a tile edge and carries 190-370 N while the other foot lifts; the WB-ID still plans a full flat 6-DoF contact.
- Walk swing height 0.10 -> 0.15 and `swingTrajectory.touchDownVelocity` 0 -> -0.1: vx 0.3 walked the full 120 s hold on tiles, then fell 5 s after stopping (both CoPs crept from +0.07 to +0.13 m over 4 s while standing; the zero-command base target re-anchors at the current pose, so nothing pulls the CoM back over the feet); vx 0.5 fell at 58 s (toe landing, 1.7 kN impact, heel loading, both feet on toes).
- Open for terrain: the actual support region under each foot (measured CoP / per-point force from joint torques) is not given to the MPC/WBC (the CoP bounds are constants compiled into `ContactMomentXYConstraintCppAd`; making them per-solve parameters needs CppAD regeneration).
- Ankle compliance: `control.jointEffortPd.kp` also drives the stand-up, so ankle kp 150 tripped the 0.8 rad tilt fault there. Locomotion-only gains (`control.jointEffortPd.locomotionKp/Kd`) were tested and then removed at the user's request. Locomotion ankle kp 150/300 changed nothing on the tiles (vx 0.3 full hold then fall at rest; vx 0.5 fell 57 s / 25 s) and did not hurt a flat 90 s run.
- A/B, torque contacts + 30 ms settling: torque only (`kinematic`, `useEstimate false`) vs torque + `inekf` + `useEstimate true`. Flat 10 min run and walk: all four SURVIVED with identical control (run vx 0.937, z 0.958 m, tilt 1.5 deg; walk vx 0.513, z 1.008 m) and vz error 1.3 / 0.6 cm/s; with `inekf` the reported height drifts up to 1.8 m while the true height holds (relative pelvis height works). Rough tiles vx 0.3, swing 0.15: both walked the full 120 s and fell after stopping. So on tiles the swing height mattered, not the height source.
- Repeats (torque only, swing 0.15, `tests: rep03_*/rep04_*`): vx 0.3 walked the full 120 s 3/3, vx 0.4 2/3 (one fell at 27 s); every run that completed the walk fell 6-14 s into the zero-command rest. Walking straight 8 s onto the tiles then standing 60 s survived 2/2 (per-point stance 0.15 fell while walking).
- Rest falls after the circle: true forward creep +0.7 cm/s while the estimate reads ~0, pitch +1.4 -> +3.3 deg over 10 s. Tilt estimate error is ~0 deg during the circle and jumps to pitch -0.5..-0.9 / roll -0.5..-1.2 deg in the first 4 s of rest (flat walk: < 0.1 deg). Same signature as the G1 turn-stop tilt error (gyro bias hidden as tilt while turning); the simulated gyro has no bias. Bias settings are now written out in the H2 config at their parameter-file defaults so `--set` can reach them.
- Gyro-bias freedom confirmed as the cause (tiles vx 0.3, 120 s walk + 30 s rest): `noise.gyroscopeBias` 1e-7 alone still fell at rest (pitch error -0.84 deg); adding `initialCovariance.gyroscopeBias` 0.01 -> 0.001 survived; `estimateImuBias false` survived (sim-only evidence, the sim gyro is unbiased). Repeats with gyro-bias rw 1e-7 + initial std 0.001: vx 0.3 4/4 SURVIVED walk and rest, vx 0.4 2/3 (one fell at 26 s while walking: right foot repeatedly landing on its toe tip, CoP +0.13..+0.16 m with 20-125 N, roll -2 -> -16 deg). Flat 10 min run circle with the same settings: SURVIVED, vx 0.94, z 0.960, tilt 1.5 deg. On hardware the small initial gyro-bias std assumes the gyro bias is calibrated at standstill before walking.
- Speed limit on the tiles with the gyro fix: vx 0.4 swing 0.18 2/3 (one fell 5 s into rest); vx 0.5 swing 0.15 0/3 (27-60 s while walking). At 0.5 m/s H2 is in the trot rung (walk band ends at 0.45 m/s; trot has no double support, so the walk swing height does not apply); keeping walk with `transitions.walk.linearVelocity [0.02, 0.65]` and `trot [0.6, 0.75]`: 1/3 (falls at 34 / 50 s). Remaining failures are partial footholds.
- Torque-estimated CoP from the heel/toe split vs the true CoP (body-frame foot sensor, `-torque.y / force.z`): RMS 2.7-3.9 cm, correlation 0.6-0.85; heel-only loading recognised 92-99%, toe-only only 35-65%. A support-region-aware CoP constraint (four `ContactMomentXYConstraintCppAd` bounds as CppAD parameters from the reference manager, measured support for the current stance only) is the next candidate; it needs a reliable partial-support signal (probably foot tilt as well) so a normal toe push-off does not shrink the support on flat ground.
- 10 min walks on the tiles, vx 0.3, best setup: one fell at 332 s while walking; one walked the full 600 s and fell 6 s into the rest. So the turn-stop tilt problem is reduced, not gone, for long circles, and partial footholds still end some walks at 0.3 m/s.
- In `config/h2/ros2_controllers.yaml` (user's final choice): torque-detector settings `beta0 [-7]x4`, `beta1 [0.02]x4`, `forceCovarianceAlpha 1e-8`, `releaseThreshold 0.05`, `pointForceRatio 0`, `lpf.tauJCutoff 50`, `sensorInterfaces`; tuned covariances `settleTime 0.03`, `noise.gyroscopeBias 1e-7`, `initialCovariance.gyroscopeBias 0.001`; `contact.source` is back to `scheduled` (set `torque` to use the detector). The walk swing 0.15 / `touchDownVelocity -0.1` were reverted (back to 0.10 / 0). With `scheduled`, the estimate-to-control path equals the earlier one except the three tuned values (`settleTime 0` and the old gyro values restore it exactly); the torque detector then runs only as a diagnostic. Height `kinematic`, `groundHeight.useEstimate` false. G1 config unchanged.
- Best tile setup as tested (through `--set`; the swing part is not in the config): torque contacts (350 N on / 200 N off, alpha 1e-8, tauJ 50 Hz, settle 30 ms), gyro-bias rw 1e-7 + initial std 0.001, walk `swingHeight` 0.15, `touchDownVelocity` -0.1; `kinematic` height and `useEstimate false` measured the same as `inekf` + `useEstimate true` on these tiles.


## InEKF Height Hold Through Flight (2026-10-26)

Fixes InEKF failure mode 1 from the durability matrix below (H2 run gait, fell within 3.5 min).
`height.holdThroughFlight` (new, `humanoid_inekf`; default false in the parameter file, true in the H2 and G1 configs):
- With a non-`inekf` height source, every tick with a scheduled stance foot records `reported - filter height`.
- With no stance foot (flight), the reported height is `filter height + that offset`: the last stance height plus the filter's change since liftoff.
- The filter's absolute height is a gauge state and drifts without bound; only its change over one flight (about 0.1 s of IMU) reaches the MPC now.
- No effect while any foot is in stance, so walking and standing are unchanged. The internal filter height still drifts.
- Code: `InekfFloatingBaseEstimator::updateImpl` (`src/humanoid_state_estimation/inekf_floating_base_estimator.cpp`), offset reset in `initialize`.

H2 run circle (vx 1.0, yaw 0.7854), InEKF, 10 min hold; flight-phase height error = per-minute 95th percentile of |height error| (`z_err_p95`):

| change | flight height error per minute | outcome |
| --- | --- | --- |
| none (baseline) | 36 -> 68 -> 143 cm | fell at 3 min |
| `noise.contactPosition` 0.002 -> 0.01 | grows faster each minute, about 16 m by min 9 | full hold, fell 2 s after the stop |
| `noise.contact` 0.01 -> 0.1 | not reached | never started: the estimator's own height-std readiness guard (0.10 m) blocks the hand-off |
| `noise.accelerometerBias` 1e-4 -> 1e-6 | same accelerating growth, about 25 m by min 9 | full hold, fell 2 s after the stop; rules out a wandering accelerometer bias |
| `height.holdThroughFlight: true` | 0.25-0.29 cm, flat | SURVIVED hold and stop; true height 0.957 m, tilt RMS 2.3 deg, vx 0.94 m/s, MPC 83 Hz (matches the linear KF) |

Notes:
- Surviving the hold with metre-scale flight heights is not robustness; the controller appears to stop reacting to absurd heights, and every such run fell at the stop.
- The noise parameters cannot bound the drift: with contact-only corrections nothing measures absolute height, so each flight's integration error becomes permanent at the next touchdown.
- A 10 min walk check with the hold on was stopped at 600 s of 615 (upright, height error 1 mm); the stop phase was not reached.
- Not yet run: the 30 min run and walk confirmation (the user is running it), and any G1 run with the hold on.
- `launch/robot.launch.py` now defaults `floatingBaseEstimator` to `humanoid_inekf` (user change).

Open, deliberately deferred:
- `contact.source: torque` is structurally wrong for humanoids: `legged_state_estimator/src/contact_estimator.cpp:70-71` inverts a 3x3 Jacobian block over joints `3i..3i+2` per contact (12-joint quadruped layout), so on H2 heel force comes from hip torques and toe force from knee/ankle torques. Planned: a robot-agnostic detector configured with contact count, which contacts share a rigid body, and the joints driving each body (per-body 6-D wrench from the full Jacobian, one stance flag per body).
- Cheap sizing experiment first: feed simulator-true foot contacts as the contact source; if the internal drift mostly disappears, schedule mismatch dominates and the detector is worth it, otherwise the in-filter height measurement (Phase 2.5) matters more.

Test tooling: `tests/estimator_durability_test.py --set path=value` overrides one existing controller YAML value per flag (rejects unknown keys); `analyze_estimator_durability.py` adds `z_err_p95`; `plot_estimator_durability.py matrix` plots it.
Data and the case runner: ignored `tuning_results/inekf_tuning/` (`claude_cases.sh CASEFILE`, `claude_screen_run.txt`, `claude_fix_10min.txt`, `progress.txt`, `claude_tune_*`).


## Estimator Durability Matrix, H2 and G1 (2026-10-25/26)

Question: can WB-ID + each floating-base estimator run for 1 h without the estimate degrading control into a fall?
Matrix per robot: {stand, walk circle vx 0.5 + yaw 0.7845, run circle vx 1.0 + yaw 0.7854} x {humanoid_inekf, humanoid_linear_kf}, 1 h hold each, flat scene, MPC 90 / MRT 1000, headless, `state_estimator` feedback.
The linear KF takes orientation from simulator truth; the InEKF integrates raw IMU acceleration and angular rate, so the InEKF runs are the realistic case.
Run-circle tasks retry with yaw 0.6 / 0.45 / 0.3 if they fall within 10 min (user rule for over-aggressive commands); later falls are kept as results.
New tools:
- `tests/estimator_durability_test.py`: one run; 20 Hz truth + estimate to `<prefix>.csv`, minute status with process RSS, stops on a fall, verdict in `<prefix>.json`.
- `tests/tuning/analyze_estimator_durability.py PREFIX...`: frame alignment, per-minute errors, error-growth class (bounded / random walk / steady drift, with sign statistics), control outcome, MPC rate; writes `<prefix>_durability.png` / `.json`.
- `tests/tuning/plot_estimator_durability.py matrix PREFIX... --out F`: several runs per hold minute on shared axes (tilt and height estimate error, true tilt RMS, true height; InEKF solid, linear KF dashed, x = fall). Produced `h2_matrix_summary.png` and `g1_matrix_summary.png`.
- `tests/tuning/plot_estimator_durability.py stop PREFIX... --out F`: true vs estimated attitude, estimate error, yaw rate and height around the commanded stop, one column per run. Produced `inekf_stop_tilt.png`.
- Analysis Python needs numpy and matplotlib (not in the ros_jazzy container); prefixes are paths without extension, and the matrix mode also accepts a shell glob of run files.

Reproduce one run (inside ros_jazzy, from `/root/ocs2_ros2_ws`, about 1 h): `python3 src/legged_robot_mpc_controller/tests/estimator_durability_test.py --robot-name g1 --estimator humanoid_inekf --vx 0.5 --yaw-rate 0.7845 --duration 3600 --prefix /tmp/claude_dur_g1_walk_inekf`.
Batch scripts and all data: ignored `tuning_results/estimator_durability/` (`claude_batch_all.sh`, `claude_batch_run.sh`, `<robot>_progress.txt`, `claude_dur_<robot>_<task>_<estimator>*`, `h2_matrix_summary.png`, `g1_matrix_summary.png`, `inekf_stop_tilt.png`).

Verdict: both estimators keep standing and walking upright for 1 h, but the InEKF has two estimate-corruption failure modes that end in falls, and neither shows up as a large error before it bites:
1. Scheduled flight phases (H2 run gait): the published height switches to the drifting internal filter height and the MPC crouches the robot into a fall within 3.5 min.
2. Long constant turns (G1 walk and fast circles): a roll/pitch error of 1.5-1.9 deg builds up over the hour without changing sign; at the stop it grows to 4 deg in 5 s and the robot falls backward 2 s after stopping. H2 shows the same effect at 0.8 deg and survives.
The linear KF (truth orientation, so it cannot show mode 2) survived every task its controller could do: 3/3 hours on H2 and 3/3 on G1 (G1 at vx 1.0 excluded; neither estimator lasts 15 s there).

| H2 task | Estimator | Wall time (UTC) | Result | Estimate error over the hour | Control first 10 min -> last 10 min |
| --- | --- | --- | --- | --- | --- |
| stand | InEKF | 15:40-16:41 | SURVIVED 60 min | height 0.001 mm, roll < 0.001 deg, yaw +0.16 deg | z 1.0142 -> 1.0142 m, tilt 0.21 deg, MPC 83.2 -> 83.7 Hz |
| stand | linear KF | 16:41-17:42 | SURVIVED 60 min | all bounded, max velocity error 0.0003 m/s | unchanged; MPC 83.4 -> 83.7 Hz |
| walk circle | InEKF | 17:42-18:43 | SURVIVED 60 min | height <= 6 mm bounded, roll/pitch <= 0.5 deg bounded; yaw steady drift -38.7 rad (about 6 turns, ~1% turn-rate deficit) | vx 0.511 -> 0.512, wz 0.741 -> 0.735, z 1.009 m, tilt 0.64 -> 0.52 deg, MPC 86 Hz |
| walk circle | linear KF | 18:43-19:44 | SURVIVED 60 min | height offset -0.8 mm bounded, yaw <= 1 deg | vx 0.515 -> 0.517, wz 0.738, z 1.011 m, tilt 0.72 deg |
| run circle | InEKF | 19:44-19:57 | FALL at yaw 0.7854 / 0.6 / 0.45 / 0.3 after 183 / 175 / 212 / 160 s | see root cause below | true height sank 0.95 -> 0.78-0.83 m and speed crept 0.96 -> 1.35 m/s before each fall |
| run circle | linear KF | 19:57-20:57 | SURVIVED 60 min at yaw 0.7854 | height <= 1.6 cm bounded with sign flips; vx bias +0.036 -> +0.011 m/s | vx 0.958 -> 0.961, wz 0.720 -> 0.717, z 0.958 m, tilt 2.3 deg, MPC 84 Hz |

Total H2 batch 5 h 17 min. No solver exceptions or torque-window overruns in any run; MPC rate did not degrade over any hour.

Root cause of the InEKF running falls (estimator corruption, not command aggressiveness; it is independent of yaw rate):
- `InekfFloatingBaseEstimator` with `height.source: kinematic` replaces the reported height with stance-foot kinematics only when `kinematicBaseHeight(...)` succeeds (`src/humanoid_state_estimation/inekf_floating_base_estimator.cpp:485-499`).
- With `contact.source: scheduled` the run gait's scheduled flight (0.04 s of every 0.30 s step) has no stance foot, so the published height falls back to the filter's internal height, which the code itself documents as drifting uncorrected.
- In the 20 Hz data the median height error stays 0 mm, but about 13% of samples (the scheduled flights) are too high, and the spikes grow: 95th percentile 19 cm (20-40 s), 64 cm (80-100 s), 124 cm (130-150 s).
- The 15 s mean error climbs about 1.2 cm per 15 s from the first running step; the MPC lowers the body against the inflated height until the robot overspeeds and falls at a mean error of 21-24 cm.
- Walking always has a stance foot, so the same drift stays hidden there (InEKF walk circle: height error <= 6 mm for 1 h). The linear KF does not show it.
- Fix candidates (not implemented): carry the last kinematic height through no-contact phases with the vertical velocity instead of switching to the internal height; add the height pseudo-measurement inside the filter (Phase 2.5 in `Claude.md`); detect contact instead of trusting the schedule.

Other findings:
- InEKF yaw drift is proportional to rotation (smoke test 0.5% turn-rate deficit; hour-long walk circle about 1%) yet did not affect control, since WB-ID tracks body-frame twist and yaw is a gauge state.
- While moving, per-minute lateral/vertical velocity error is 0.13-0.20 m/s RMS (forward 0.02-0.14 m/s) for both estimators; it is step-frequency error that does not grow, with only 6 ms median estimate-to-truth pairing lag.
- Simulator process RSS was about 1.4 GB and grew +11 MB/h standing and +23 MB/h moving (fresh process per run); not limiting at 1 h, worth heap profiling before multi-day use.
- `mujoco_ros2_control` ground-truth odometry reports about 0 angular z while the robot turns at 0.74 rad/s, although the code looks correct; the analyzer derives the true yaw rate from the yaw angle instead.
- The early-fall yaw-retry rule assumed early falls meant an over-aggressive command; these falls were estimator-driven, so lower yaw rates could not help.

G1 matrix (started 2026-10-25 20:58:50 UTC, same tasks):

| G1 task | Estimator | Wall time (UTC) | Result | Estimate error over the hour | Control first 10 min -> last 10 min |
| --- | --- | --- | --- | --- | --- |
| stand | InEKF | 20:59-21:59 | SURVIVED 60 min | all negligible (height 0.00 cm, yaw 0.9 deg) | z 0.751 m, tilt 0.02 deg, MPC 88 Hz |
| stand | linear KF | 21:59-23:00 | SURVIVED 60 min | all bounded about zero, with sign flips | unchanged; MPC 88 Hz |
| walk circle | InEKF | 23:00-00:01 | FALL 2 s into the rest after the full 60 min hold | roll and pitch error grew 0 -> 1.0 / 1.1 deg with no sign change; yaw steady drift -91 rad (3.4% turn-rate deficit) | vx 0.411 -> 0.412 (command 0.5), wz 0.737 -> 0.750, z 0.757 -> 0.762, tilt RMS 1.13 -> 1.84 deg |
| walk circle | linear KF | 00:01-01:02 | SURVIVED 60 min and the stop | all bounded; constant vx bias -0.010 -> -0.012 m/s, height 0.2 mm | vx 0.423 -> 0.426, wz 0.736, z 0.758 m, tilt RMS 1.31 deg unchanged, MPC 88 Hz |
| run circle vx 1.0 | InEKF, linear KF | 01:02-01:07 | FALL 4-12 s into the hold at every yaw rate (0.7854 / 0.6 / 0.45 / 0.3) | accurate up to the fall: pitch error < 0.006 rad, vx error < 0.03 m/s | G1 controller limit, not the estimator: it falls within seconds of reaching about 0.7 m/s, where the `run` rung engages |
| fast circle vx 0.65, yaw 0.7854 | linear KF | 01:07-02:08 | SURVIVED 60 min and the stop (first candidate) | all bounded; constant vx bias -0.016 -> -0.015 m/s, height 0.1 mm | vx 0.536 -> 0.532, wz 0.730, z 0.759 m, tilt RMS 1.65 -> 1.66 deg, MPC 88 Hz |
| fast circle vx 0.65, yaw 0.7854 | InEKF | 02:08-03:09 | FALL 2 s into the rest after the full 60 min hold | roll/pitch error grew 0 -> 1.9 deg (pitch 1.64 deg) with no sign change; vx bias -0.001 -> -0.022 m/s; yaw drift -75 rad | vx 0.524, wz 0.732 -> 0.757, z 0.759 -> 0.767, tilt RMS 1.45 -> 1.95 deg, MPC 88 Hz |

Total G1 batch 6 h 10 min (20:58:50-03:08:43 UTC). No solver exceptions or overruns; MPC 87.5-88 Hz throughout; simulator RSS +11 to +36 MB/h.

G1 cannot run at 1.0 m/s with this controller, so the durability question moves to a fast task below the run threshold (`claude_batch_fast.sh`): the linear KF tries vx/yaw 0.65/0.7854, then 0.65/0.5, then 0.6/0.4 and keeps the first it sustains past 10 min; the InEKF then runs that condition with no retry.

G1 InEKF walk- and fast-circle falls: the hidden tilt error appears at the stop (`inekf_stop_tilt.png`, `g1_matrix_summary.png`).
- During the hold the roll/pitch estimate error grew steadily (walk 1.5 deg, fast 1.9 deg, faster circle grows faster, still rising at 60 min) without changing sign, and the true tilt RMS rose with it (1.1 -> 1.9 and 1.4 -> 2.0 deg) while the linear KF runs stayed flat; it never looked like a failure.
- The fast circle failed exactly like the walk circle: pitch error 1.9 -> 4 deg during the 5 s stop ramp, true pitch leaning back while the estimate stayed level, fall 2 s into the rest.
- As the yaw rate ramped from 0.74 to 0 rad/s, the pitch error grew at about 0.01 rad/s (1.1 -> 3.8 deg in 5 s). The estimated pitch stayed between 0 and -1 deg, the controller held that, and the true pitch leaned back to -5.4 deg, then the robot fell backward 2 s into the rest.
- Same signature on H2 (InEKF walk circle), smaller and converging: pitch error 0 -> -0.8 deg over 20 s after the stop, true pitch following to +1.2 deg; it survived. The linear KF (truth orientation) shows none of it.
- Likely mechanism (inference, biases are not logged by this harness): while yawing at a constant rate with constant body tilt, a gyro x/y bias error integrates into a body-fixed tilt error of about bias/yaw-rate, offset by an accelerometer bias error, which kinematic contact updates cannot see. Removing the turn breaks that balance and the bias error integrates into tilt. Observed size fits: 0.0125 rad/s drift at the stop vs 0.75 rad/s turning gives 0.017 rad hidden tilt; observed 0.027 rad.
- Next test item: walk circle -> stop with turning times of 5, 15 and 30 min, logging the InEKF gyro/accelerometer bias, to find how long a turn the InEKF tolerates before the stop is fatal and to confirm the bias mechanism; then the bias random-walk noise and the estimator's own stance/turn handling as fixes.
No controller, estimator or config change was made for these tests; nothing committed.


## Uneven RViz Filename Fix (2026-10-25)

The user selected `uneven_terrain.urdf.xacro` as the launch default, but the generator previously produced `uneven_terrain.urdf`; the missing file caused the reported xacro launch exception.
Renamed the generated RViz output to `description/urdf/terrains/uneven_terrain.urdf.xacro` and updated the generator default, launch help, tests and docs.
Preserved the user's current uneven-terrain environment default and H2 scene include ordering/content; the earlier empty-default notes below are historical.
The generated file is valid xacro-compatible URDF; Python still generates its geometry alongside MJCF after YAML edits, rather than loading that YAML dynamically at launch.
Generator-only temporary-output checks passed for current and moved/rotated rectangular configurations, matching formats, height bounds, reproducibility, stale-file detection and invalid-input rejection.
The earlier local full suite reported five subcase failures in one scene include-order assertion; do not interpret the generator checks as a full-suite pass.
ROS-specific launch tests require the user's build/install to expose the newly named file.
No rebuild or new simulator run was performed; the user is testing manually, and subsequently requested committing and pushing this checkpoint.


## Single Scene and Per-Box Configuration (2026-10-25)

Current branch is `jazzy`; no commit or push is requested for these changes.
Each robot now ships only `description/mujoco/robots/<robot>/scene.xml`; removed the flat, ramp and uneven scene variants without changing either retained scene's contents.
`mujocoModelFile` still accepts a robot-local filename or absolute custom XML path, with no fixed choices.
`environmentDescription` now defaults to empty, disabling the optional RViz environment publisher; its argument description lists stairs, ramp, uneven terrain and boxes under the package's `description/urdf/` directory.
The RViz environment is independent of the MuJoCo scene: selecting it never changes simulator physics.
Existing automated flat/ramp/rough tests now use `tests/mujoco_scene.py` to generate fixtures outside the robot directories with absolute include and compiler asset paths.
The walking harness generates `<prefix>_scene.xml` for flat ground when `--scene` is omitted; explicit scene overrides remain untouched, and startup/ghost/benchmark/replay tools no longer reference deleted variants.
Previously recorded `scene_flat.xml`, `scene_ramp.xml` and `scene_uneven.xml` names in historical results below identify the old fixtures, not current shipped files.
`config/objects/boxes_params.yaml` now contains `boxes.box_1`, `boxes.box_2` and any additional named entries rather than count plus parallel arrays.
Each entry owns position, yaw, size, mass, inertia, color and friction; optional `name` preserves the simulator/RViz name and falls back to the entry key if omitted.
Updated both boxes xacros and regenerated MuJoCo XML; body/geom/inertial attributes are identical to HEAD, preserving `purple_box`, `orange_box` and every physical value.
Tests cover adding/removing entries, optional names, independent friction, empty box sets and matching MuJoCo/RViz attributes.
Validation passed:55 local discovery tests with five ROS-only skips, five real Jazzy launch-resolution tests,23 focused WB-ID contract checks and seven standalone terrain-reference checks.
The Jazzy launch tests verify the empty environment default, unrestricted custom scene/environment paths and the listed URDF/xacro examples; `ros2 launch ... --show-args` also passed without starting nodes.
Native MuJoCo compiled both retained scene files and all generated test terrain fixtures, including relocated description trees; the box migration preserved every body/geom/inertial attribute compared with HEAD.
No C++ controller code or tuning changed, and no new build or locomotion trial was run for this cleanup.
Handoff and two commit-message snippets are updated; no commit or push was performed.


## Parameterized Uneven Terrain (2026-10-25, Ready For Manual Testing)

Current branch is `jazzy`; do not commit or push this task.
Renamed the fixed `uneven_terrain_10x10.xml` module to `description/mujoco/terrains/uneven_terrain.xml` and updated the scene creator, exporter tests and docs.
New shared YAML is `config/terrains/uneven_terrain_params.yaml`; run `python3 description/mujoco/terrains/generate_uneven_terrain.py` to generate that MJCF and matching `description/urdf/terrains/uneven_terrain.urdf.xacro`.
The generator's `--check` verifies current output; `--params`, `--mjcf` and `--urdf` support alternate files without editing the script.
Added `scene_uneven.xml` for G1/H2 without changing their existing default, flat or ramp scenes, controller parameters, estimators or launch code.
The field is simulation geometry only, not an MPC foothold map; ordinary `base_twist` remains blind to the generated heights.
Default field is 10x10 m, grid20x20 (0.5 m pitch), seed7, a local 0-0.12 m tile-top envelope, triangular nominal step sampling with mode0.03 m, bounded independent jitter0.002 m and a broad sinusoidal directional tendency.
Final logical edge-neighbor differences are checked within 0.02-0.05 m; generated values are min0.0229622, max0.0466639, mean0.0331671 and standard deviation0.00477179 m.
Generated tile tops span0.006073-0.103840 m, avoiding the old negative tile tops that were masked by the flat floor.
The exact0.5x0.5 m clear patch clips cells around its boundary, producing404 box pieces from400 logical cells; pieces of one cell share a height.
Bounds apply to X/Y grid neighbors, not diagonals, same-cell clip pieces, outer-floor edges or the clear-patch boundary; floor entry steps can exceed5 cm and are a separate difficulty from neighboring tile differences.
Parameters also expose center/yaw, rectangular size/grid, seed, profile amplitude/wavelength/strength, jitter, patch center/size, block depth, optional gaps, friction and RViz gray.
Two bounded correlated axis walks plus per-cell jitter give structured variation; the distribution is conditioned near the envelope and is not an exact independent triangular density over all realized differences.
MuJoCo retains the shared stair texture and old field friction [0.9,0.005,0.0001]; RViz uses the same generated box geometry with gray materials.
Tests cover60 seed/distribution combinations, reproducibility, shape/profile changes, exact patch/footprint, XML/URDF agreement, invalid/infeasible inputs, relocatable CLI, current generated files and native MuJoCo ray heights.
Jazzy two-job Release build passed in VM Docker `ros_jazzy` (3.30 s); log is `/tmp/uneven_terrain_build.log` in the container.
Local discovery passed52 tests with four ROS-only skips, including nine uneven-terrain tests; seven standalone terrain-export/reference tests also passed.
All eight G1/H2 scenes compiled with native MuJoCo; native ray tests verified tile-top heights, the clear floor opening, moved centers and yaw rotations.
The installed-package generator `--check` passed in Docker, and its generated URDF passed the existing xacro command with the launch's three environment parameter mappings.
Rendered both robot scenes and inspected the height map, adjacent-step histogram and measured-state replay images.
No RViz GUI session was opened; RViz validation covered generated URDF geometry and xacro compatibility, not an interactive display test.
Closed-loop smoke tests used WB-ID, `state_estimator`/`humanoid_linear_kf`, MPC90/MRT1000, headless MuJoCo, RViz false, `baseCommandGui:=false`, vx0.10 m/s, settle3/ramp2/hold5/ramp-out2/rest3 s and unchanged robot/controller YAMLs.
G1 uneven trial `/tmp/uneven_terrain_g1_smoke` started MPC and walked, then fell at10.65 s after takeover during ramp-out; final base z0.274 m, max roll1.342 rad and max pitch0.940 rad.
H2 uneven trial `/tmp/uneven_terrain_h2_smoke` completed15 s and stopped upright, but a transient base-height drop to0.473 m and max yaw error0.716 rad made the verdict `NO_TRACKING`; final z1.079 m, roll0.021 rad and pitch0.006 rad.
H2 replay shows a deep near-fall/crouch around7.09 s followed by recovery; this is not evidence of clean or reliable rough-terrain walking.
G1 flat comparison `/tmp/uneven_terrain_g1_flat_control` passed the same15 s command profile and stopped upright, with max yaw error0.00781 rad; unlike the uneven trial, this comparison did not enable joint/high-rate tracking recording, so timing/load was not matched exactly.
G1/H2 uneven JSON, YAML, CSV, tracking, logs and replay data are copied to ignored `tuning_results/uneven_terrain/`; the flat comparison remains in Docker `/tmp/`.
That local directory also contains `height_profile.png`, `g1_overview.png`, `h2_overview.png`, `g1_smoke_curves.png`, `h2_smoke_curves.png`, `g1_smoke_replay.png` and `h2_smoke_replay.png`.
The existing headless walk harness retains the default RViz environment description, which is irrelevant with RViz off; for manual visualization use the matching generated uneven URDF shown in `config/terrains/README.md`.
No controller tuning was performed, no terrain map was fed to MPC, and no terrain parameters were reduced to mask the failed/near-fall trials.
Stopped further work at the user's request for manual testing; no owned simulations or builds remain running, and nothing was committed or pushed.
Commit-message snippet is in `/Users/whcheng/src/mpc/commit_msg.md`.


## H2 Running to 2 m/s (2026-10-25, Latest)

H2 now holds a 2.0 m/s command: 6/6 trials passed (four 12 s holds, one 30 s hold, one 12 s ramp), plus 1.5, 1.0 (x2), 0.75, 0.3, vy ±0.3 and yaw 0.5 regressions.
Before: a 2.0 m/s command fell 1.6 s into the hold; a slow ramp fell at about 1.75 m/s; steps alternated long/short above 1.5 m/s.
Protocol: `tests/wb_id_walk_test.py --robot-name h2 --scene scene_flat.xml --source state_estimator --estimator humanoid_linear_kf --mpc-rate 90 --settle 4 --ramp 4 --hold 12 --rest 6 --record-joints --record-tracking`, in Docker `ros_jazzy`; analysis on the Mac with `analyze_running.py`, `analyze_leg_clearance.py` and the new `tests/tuning/analyze_gait_regularity.py`.
Raw outputs, 81 trials, config variants and the trial log (`claude_trials.md`) are in ignored `tuning_results/h2_run_2ms/`.

Root cause chain, each step measured:

- above 1.5 m/s the 3-node torque window let the plan demand ankle-pitch 78-125 Nm (limit 66.9) and hip 356-423 Nm (limit 360); actuators clipped and execution departed from the plan;
- the friction cone (mu 0.4) became binding exactly where the gait degraded (planned |Fxy|/Fz > 0.36 on 2% of loaded samples at 1.25-1.5 m/s, 44% at 1.5-1.75, 82% at 1.75-2.0);
- with 0.32-0.34 s steps the step length alternated (CV 0.39-0.51); 0.30 s steps were regular (CV 0.02-0.06);
- even the regular gait landed the foot only +0.10-0.15 m ahead of the CoM (knee 0.9-1.1 rad) against -0.28..-0.44 m at lift-off, netting propulsion every stance; the robot ran away past ~1.8 m/s within ~3 s. Swing hip torque was not saturated, so this was the optimizer's choice.

Sixty YAML-only trials could not fix the touchdown placement (weights on pitch/height/vx/knee/hip/foot velocity/contact moments, torque window 10/25, torque barrier shapes, cadence 0.28-0.34 s, swing height 0.12-0.20, friction 0.6, height 0.95, arm swing).
The existing capture-point foothold in `SwitchedModelReferenceManager` was hard-coded off in `whole_body_inverse_dynamics_mpc_config_builder.cpp`.
C++ change (default off, so other configurations are unchanged):

- WB-ID schema `ocs2.costs.captureFootPlacement.*` and builder mapping;
- per-foot touchdown and stance time from the solve's mode schedule (flight-safe; walking gives the old next-event time);
- `stanceTimeGain` (land `stanceTimeGain * v * T_stance` ahead of the predicted base) and `currentSwingOnly` (target only until that foot's touchdown).

H2 YAML: `maxDisplacementVelocityX` 1.0 -> 2.0; run 0.26 s support / 0.04 s flight; `torqueConstraintNodes` 3 -> 10; torque barrier `mu/delta` 0.1/5.0 -> 0.5/1.0; foothold enabled (`gain` 0, `stanceTimeGain` 0.5, `trackingWeight` 50, `stepWidth` 0.26, `currentSwingOnly` true).
Ablations at 2.0 m/s: without the torque window, the cadence or the barrier-less combination it fell; without the barrier change it passed with twice the lean, a 0.84 m dip and 6.5 m drift; the foothold alone on the old settings fell.
Results with the source YAML: 30 s hold 1.87 m/s, hold roll RMS 0.010, pitch +0.010 ± 0.013 rad, base 0.922 ± 0.009 m, step period/length CV 0.03/0.03, touchdown +0.20-0.23 / lift-off about -0.31 m vs CoM; no crossing, overlap or self-contact; MPC 82 Hz.
Limitations: grounded run (physical flight ≤ 10 ms against 40 ms windows); held speed 3-7% under command; heading drift 0.1-0.4 rad per run (0.40 rad and 16.9 m over the 64 m long run); one walk-to-trot transient in ten ramps (0.85 m, 0.12 rad roll, 12° heading jump); G1 untouched and not re-tested.
`tests/test_wb_id_running.py` updated for the new H2 run timing plus a foothold-settings check; 41 discovery tests (8 native skips) and the WB-ID contract scripts pass.
Package build (2 jobs, Release, `--packages-select legged_robot_mpc_controller`) passed in 5 min 10 s; no new warnings.
Details and the reproduce command: [H2 to 2 m/s](docs/wb_id_running.md#h2-to-2-ms). No commit or push.


## Per-Gait Swing Clearance (2026-10-25)

Added optional `ocs2.gait.templates.<name>.swingHeight` [m]; omitted or -1 inherits `ocs2.gait.swingTrajectory.swingHeight`, while zero is an explicit zero-clearance request.
H2 overrides are walk0.10, trot0.15 and run0.20; all other H2 templates inherit0.10.
G1 follows the same YAML style with walk0.08, trot0.09 and run0.10; its default fallback and all other templates remain0.08.
These are the user's latest YAML edits at commit time, superseding the initially requested trot0.12/run0.16 values; no new simulation evidence is claimed for the revised values.
This G1 follow-up changes configuration only, not C++, gait timings, costs or constraints; both robots' height overrides are covered by the configuration regression.
The existing swingTimeScale0.4 remains unchanged: steady H2 walk0.30 s gets0.075 m clearance, trot0.50 s gets0.15 m, run0.40 s gets0.20 m.
GaitSchedule carries per-phase heights through insertion, truncation and horizon extension; the swing planner chooses the lift-off phase's height for the entire contiguous swing, including flight modes.
Both parameter schemas support the property because the gait-map builder is shared; centroidal YAML values are unchanged.
Tests cover per-phase metadata alignment, old/future gait coexistence, future replacement, fallback, zero/invalid overrides, duration scaling, terrain endpoints and spline continuity.
Validation passed: two-job Jazzy Release build in Docker `ros_jazzy` (6 min20 s), both native AddressSanitizer regressions including100 gait-transition cycles,37 focused WB-ID tests,40 discovery tests (four ROS-only skips) and the parameter contract.
No walking/running simulation with these higher G1/H2 heights is claimed, and earlier running results used the universal0.08/0.10 m heights respectively.
Stopped after the build and regressions at the user's request to test manually; no simulation was started for this change.
The0.20 m run target may reduce robustness through higher velocity/torque/collision demands; no costs, constraints, base targets or gait timing were retuned in this change.


## G1/H2 Four-Stage Gait Ladder (2026-10-25, Untested)

Both WB-ID controllers now select `[stance, walk, trot, run]`; this supersedes the six-stage selection described below.
Both robots' linear bands are stance [-0.10,0.03], walk [0.02,0.45], trot [0.40,0.75], run [0.70,10.0] m/s, preserving adjacent overlap and run entry at 0.75 m/s.
Walk yaw minimum is reduced from 0.15 to 0.05 rad/s to overlap stance's 0.10 rad/s upper boundary; otherwise direct stance/walk selection could chatter for small turning commands.
Inactive slow_walk and slow_trot transition settings/templates remain available but are not selected by either ladder.
G1's stance upper speed also changes from 0.10 to 0.03 m/s to match H2; its centroidal YAML is unchanged.
No costs, constraints, gait timings, measured margins, core C++ or command limits changed in this follow-up.
Configuration tests were updated for the common four-stage ladder; no build or simulation validation is claimed for this revised selection policy.
Required OCS2 dependency fix: `a68c8a6cb` on `ocs2_ros2` branch `jazzy`, preserving warm-start samples before trajectory truncation.


## Six-Stage Gait Ladder (2026-10-25, Untested)

At the user's request, both WB-ID ladders now select `[stance, slow_walk, walk, slow_trot, trot, run]`; the unused `slower_trot` transition entry is removed but its catalog template remains available.
Retain each robot's stance threshold and slow_walk minimum; set slow_walk maximum0.25, walk[0.20,0.45], slow_trot[0.40,0.60], trot[0.55,0.75], run[0.70,10.0] m/s.
Adjacent speed intervals overlap for hysteresis; run entry is0.75 m/s, run exit below0.70 m/s, subject to existing measured-speed/dwell/schedule guards.
The shared faster-gait guard now includes the linear upper boundary with1e-9 m/s tolerance so an exactly0.75 m/s command is not held below the threshold by command-filter roundoff.
Yaw bands, measured margins, gait timings, costs, constraints, command limits and servo/estimator settings are unchanged in this follow-up; no gait-specific cost profiles have been introduced.
G1's centroidal YAML is unchanged, but the inclusive linear-boundary comparison is in the common gait manager used by both formulations.
This new six-stage sequence has not been built or simulation-tested; the successful running trials below used the earlier four-stage ladder and must not be attributed to this new selection policy.


## Running With Flight (2026-10-25, Latest)

Completed checkpoint: flight-phase running at forward commands 0.75-1.0 m/s, validated G1 before H2 and retained successful settings.
All trials below use `scene_flat.xml`, WB-ID, requested MPC90/MRT1000, `state_estimator`/`humanoid_linear_kf`, headless MuJoCo, RViz off and `baseCommandGui:=false` in VM Docker `ros_jazzy`.
Selected candidate tuning is now saved in both source YAMLs; trial-specific YAML snapshots and raw logs are in Docker `/tmp/<prefix>*`, with copies under ignored `tuning_results/wb_id_running/`.
Saved G1 changes are swing height 0.08, supported walk0.30/0.10, run 0.25/0.08, collision lateral separation 0.09/mu 3000/delta 0.02 and earlier gait switching0.3.
Saved H2 changes are run 0.28/0.06 and Q/Qf base z 50->300, vx 60->30, vz 5->1; retain swing height 0.10, collision parameters, servo gains and all estimator settings.
Both now use the stance/slow_walk/walk/run ladder, forward run entry 0.70/exit 0.65 with measured-speed guards, and supported walking for pure yaw commands; their original slow_walk timing remains unchanged.

| Trial | Temporary changes | Observation |
| --- | --- | --- |
| `g1_run_initial075` | Ladder stance/slow_walk/run, run entry above 0.65 m/s, exit below 0.60, switch delay 0.3; original run 0.30 s support/0.10 s flight | Held 0.75 command for 12 s at pose-derived vx 0.745 m/s, then fell during stopping; sampled physical flight 28 bouts, 20% of hold, longest 0.10 s; planned flight force zero. |
| `g1_run_quick075` | Same, slow_walk shortened to 0.30 s support/0.10 s double support | Stayed upright through deceleration, then process aborted with `free(): corrupted unsorted chunks` near stance switch; not a pass. |
| `g1_run_continuous075` | Previous quick setup plus continuous swing spline fix | Complete 28 s including stopping and 6 s rest passed; vx 0.637 m/s, max roll/pitch0.0281/0.0547 rad; physical flight 30 sampled bouts, 7.25% of hold, longest 0.05 s. |
| `g1_run_continuous100` | Preserve original slow_walk, add quick walk rung, run entry 0.70/exit 0.65 | Fell at 10.49 s; original 0.30/0.10 running timing is not robust at 1.0 m/s. |
| `g1_run_short100` | Run 0.25 s support/0.05 s flight, swingHeight 0.07, same four-rung ladder | Held 12 s at vx 0.874 m/s, max roll/pitch0.0231/0.0433 rad; process aborted with `malloc(): unaligned tcache chunk detected` at run-to-walk switch; not a pass. |
| `g1_run_gdb100` | Same short-flight parameters under GDB, 6 s hold | Reproduced allocator crash at run-to-walk switch; crashing thread was in solver warm-start initialization. |
| `g1_run_flight080100` | Post-dependency-fix, 0.25 s support/0.08 s flight, swingHeight 0.10 | Fell at 13.0 s; actual flight existed (15 bouts, longest 0.05 s), but signed-width crossed during collapse; one sampled foot self-contact near collapse. |
| `g1_run_flight080_nodes6_100` | Same 0.25/0.08 timing, swingHeight 0.07, torqueConstraintNodes 6 | Fell at 11.45 s; extending the near-term torque constraint window alone did not help. |
| `g1_run_sep100` | 0.25/0.08 timing, swingHeight 0.08, G1 minimumLateralSeparation0.09, collision mu 3000/delta 0.02 | Passed 28 s including stopping/rest; pose vx 0.870 m/s, max roll/pitch0.0362/0.0707 rad; 19 physical flight bouts, longest 0.02 s; no sampled self-contact/crossing, minimum signed width 0.1156 m. |
| `g1_run_sep075` | Same candidate, command 0.75 | Passed 28 s including stopping/rest; vx 0.656 m/s, max roll/pitch0.0366/0.0455 rad; physical flight 36 bouts, longest 0.05 s, no sampled foot self-contact. |
| `h2_run_initial100` | G1 0.25/0.08 running timing and 0.08 swing height, H2's existing collision/servo/cost values | Fell 11.25 s; forward overspeed and steadily deepening knees/base-height loss before collapse; not an estimator-only offset diagnosis. |
| `h2_run_height300100` | H2 Q/Qf z50->300, vx 60->30, vz 5->1; same gait | Fell 13.24 s; greater height priority delayed but did not prevent the failure. |
| `h2_run_short100` | Original H2 costs, support 0.28/flight 0.04, swingHeight 0.10 | Passed 28 s and stopped; pose vx 1.190 m/s, minz 0.762 m (deep crouch), physical flight 16 bouts/longest 20 ms, no sampled foot self-contact; not yet a sufficiently validated final tune. |
| `h2_run_short_height300100` | Short H2 gait with the height 300/vx 30/vz 1 objective | Passed 28 s including stopping/rest; vx 1.004 m/s, minz 0.951 m, max roll/pitch0.0174/0.0329 rad; no sampled crossing/self-contact, min width 0.1434 m, but just one10 ms physical flight bout; not a satisfactory sustained-flight checkpoint. |
| `h2_run_flight60_height300100` | Keep the improved support-height objective, lengthen H2 flight 40->60 ms with 0.28 s support | Passed 28 s and stopped; vx 1.020 m/s, max roll/pitch0.0300/0.0378 rad, minz 0.897 m; physical flight 26 bouts/longest 50 ms, zero sampled self-contact/crossing/leg overlap; saved as H2 candidate. |
| `h2_run_saved100_long` | Saved H2 source YAML, no tuning overrides, 30 s hold | Passed 46 s including stopping/rest; world vx 1.026 m/s, minz 0.904 m, max roll/pitch0.0352/0.0355 rad; physical flight 53 bouts/longest 40 ms; no sampled foot self-contact/crossing/leg overlap, min width 0.1417 m. |
| `g1_run_saved100_long` | Saved G1 source YAML, no tuning overrides, 30 s hold | Passed 46 s including stopping/rest; world vx 0.875 m/s, max roll/pitch0.0441/0.0650 rad, heading error 0.0881 rad; physical flight 34 bouts/longest 30 ms, no sampled self-contact/crossing/leg overlap, min width 0.1088 m. |
| `h2_run_saved075` | Saved H2 source YAML, command 0.75, 12 s hold | Passed 28 s including stopping/rest; world vx 0.762 m/s, minz 0.939 m, max roll/pitch0.0271/0.0367 rad; flight 30 bouts/longest 30 ms, no sampled self-contact/crossing/leg overlap, min width 0.1452 m. |
| `g1_running_walk_regression` | Saved G1 source YAML, command 0.2, 6 s hold | Passed 18 s including stop/rest; pose vx 0.180 m/s, max roll/pitch0.0067/0.0329 rad. |
| `h2_running_walk_regression` | Saved H2 source YAML, command 0.2, 6 s hold | Passed 18 s including stop/rest; pose vx 0.227 m/s, max roll/pitch0.0150/0.0111 rad. |
| `g1_run_synchronized100` | Saved G1 source YAML, command 1.0, 12 s hold; timestamp-paired recorder | Passed 28 s and stopped; world vx 0.868 m/s; 12 sampled flight bouts/longest 30 ms, all hold pairs within 2 ms; zero sampled self-contact/crossing/leg overlap. |
| `h2_run_synchronized100` | Saved H2 source YAML, command 1.0, 12 s hold; timestamp-paired recorder | Passed 28 s and stopped; world vx 1.052 m/s; 22 sampled flight bouts/longest 20 ms, 99.92% of hold pairs within 2 ms; zero sampled self-contact/crossing/leg overlap. |

Confirmed code defect: `SwingTrajectoryPlanner` previously changed spline shape between consecutive airborne contact modes, although their lift-off/touchdown times span one complete swing.
It now builds the same complete lift-off/apex/touchdown spline across all those modes, using endpoint heights from the first/last swing phases; single-mode walking is unchanged.
Stance-only schedules no longer read nonexistent event times.
Two-job Jazzy Release build passed; standalone compiled `tests/swing_flight_test.cpp` passed, covering flight-mode continuity, ordinary walking and event-free stance.
`tests/wb_id_walk_test.py --record-tracking` captures planned modes/wrenches/feet alongside measured-pose replay; high-rate data stays best effort, while the subscriber matches the existing reliable latched metadata publisher.
`tests/tuning/analyze_running.py PREFIX --robot-name g1|h2` checks both feet's active collision geometry above 2 mm and absence of sampled floor contacts, and plots flight, CoM/base height and planned forces.
Replay uses latest-sample-paired measurements: it is geometric flight evidence, not measured contact forces or a continuous-time guarantee.
The final recorder now stores joint timestamps and repairs recent odometry/joint pairs when matching messages arrive out of order; the analyzer accepts flight only for pairs within 2 ms when timestamps exist.
Earlier trial replay files did not store joint timestamps, so their flight counts remain weaker latest-sample-pair evidence; the final timestamp-paired repeats on both robots confirm brief airborne intervals rather than relying on old counts alone.
`g1_run_short100` had scheduled flight but no sampled physical flight, so that shorter-flight trial is not a running success even before considering its crash.
`g1_run_gdb100` reproduced the allocator abort at the run-to-walk switch, with the solver warm-start initialization on the crashing thread.
The standalone `tests/running_gait_transition_test.cpp` reproduced a heap-use-after-free under AddressSanitizer in OCS2 `TrajectorySpreading::adjustTrajectory` across 200 synthetic run-to-walk splice phases.
The dependency header `/Users/whcheng/src/mpc/ocs2_ros2/core/ocs2_oc/include/ocs2_oc/trajectory_adjustment/TrajectorySpreading.h` now snapshots source values before truncation, limits writes to the retained interval and truncates last.
The same ASan regression passes after that change; rebuilding `ocs2_oc`, `ocs2_ddp`, `ocs2_sqp` and the controller passed in 16.3 s.
The upstream trajectory-spreading suite also passed 19 tests, including a new200-phase flight-to-walk regression.
This necessary dependency change is outside the legged controller Git repository and must be carried with its final checkpoint.
Post-fix native G1/H2 run/stop trials at 0.75 and 1.0 m/s, source-value46 s endurance trials and 0.2 m/s supported-walking regressions all passed.
Actual flight in this conservative candidate is much shorter than its prescribed 80 ms flight window; do not conflate planned contact flags with achieved airborne duration.
H2's saved run prescribes60 ms flight; its long trial achieved a 2.33% sampled airborne fraction versus 17.57% planned flight, with zero planned force in flight.
H2 remained balanced during 46 s but heading drift reached0.461 rad and world lateral displacement8.14 m; do not describe this as accurate straight-line heading control.
User clarified that commands above 1.0 m/s should still select `run`, but only up to 1.0 m/s needs testing; no higher-speed stability claim is required.
User additionally requires natural-looking running without kicking other body parts; verify physical clearance/contact and measured-pose replays, not only velocity or height.
Long-trial logs show zero solver exceptions, typically G1 around 86-88 Hz and H2 around 83-85 Hz actual MPC, with transient lower rates and some torque-window fallbacks; requested90 Hz is not a timing guarantee.
Nonlinear equality residuals and actuator limits are not certified feasible by these successful tests; no claims are made about rough terrain, push recovery, mixed-axis running, InEKF feedback, speeds above 1.0 or hardware.
In the failed initial H2 run, paired 0.3 s averages at 10 s after takeover show pose-derived vx 1.52 m/s versus observed vx 1.08 m/s, while observed and physical z both approximately 0.82 m.
This is evidence of a significant velocity-feedback discrepancy during the failing motion, not proof of the sole root cause; inspect slip/contact timing and consider an isolated ground-truth-feedback diagnostic before changing estimator core logic.
No estimator core, estimator parameters, servo gains, launch structure, robot physical models or G1 centroidal YAML changed in this checkpoint.
Final controller build passed in 3.33 s; logs are Docker `/tmp/wb_id_running_build.log` and `/tmp/wb_id_running_final_build.log`.
Final verification passed 37 focused WB-ID checks,38 discovery tests (four native ROS tests skipped locally), the parameter contract, the standalone compiled swing test,200 ASan transition phases and 19 upstream trajectory-spreading tests.
Commit snippets for both repositories are at `/Users/whcheng/src/mpc/commit_msg.md`; publication was requested after the gait-ladder edits, so consult Git history for the final controller checkpoint.
The running/clearance plots and side/front frame sheets are in ignored `tuning_results/wb_id_running/`; see [Running Checkpoint](docs/wb_id_running.md) for the test command and limitations.
All owned simulation, debugger and auto-test processes were stopped after verification; Docker `ros_jazzy` remains available for manual tests.


## Future Robot Mesh Ghosts (2026-10-24, Latest)

G1 and H2 now have legacy-style future mesh ghosts in the existing performance visualization worker, with optional terminal-state and terminal-target MarkerArray topics.
Only controller YAML parameters were added: `visualization.optimizedStateRobotModelsActivate=true` and `visualization.optimizedStateRobotModelsInterval=0.2`; no launch arguments or MPC tuning values changed.
Sampling uses the actual policy time range (following `ocs2.mpc.timeHorizon`), not a second visualization horizon; as in the legacy code, round horizon/interval to the count and distribute poses equally including the endpoint.
Future color is RGB `[0.765,0.665,0.905]`, alpha `0.22 -> 0.08`; terminal state uses `[0.180,0.220,0.300]`/`0.40`, terminal target uses the future color/`0.30`.
The new `FutureRobotModelVisualization` handles URDF origins/scales, multiple visuals and primitives, with private Pinocchio data and `MpcRobotModelBase::getGeneralizedCoordinates` for both state layouts.

The worker polls subscribers and only requests/copies snapshots when a display subscribes; it processes fresh snapshots rather than repeating FK for unchanged data.
Ghost topics `/performance_visualization/{optimizedStateGhosts,terminalStateGhost,terminalTargetGhost}` use best-effort volatile QoS; the user's saved `humanoid.rviz` enables future ghosts by default, while terminal ghost topics can be added as optional MarkerArray displays.
With no subscribers, no periodic state/target copies, ghost FK, interpolation or marker generation occur; one-time setup and light subscription polling remain.
Snapshot handoff uses nonblocking `RealtimeThreadSafeBox::try_set/try_get`; copying still costs CPU and allocations, so this is not a hard real-time or zero-overhead guarantee.
WB-ID snapshots are copied from the control-thread-owned active MRT policy; centroidal snapshots are copied between solves on its `updatePolicy()`-owning solver thread to avoid adding cross-thread policy reads.
All FK and marker publishing run on the visualization thread, never in the solver or control callbacks.

Jazzy two-job Release builds passed (initial 2 min 29 s, incremental 2 min 15 s); the final policy-owner object rebuild passed in 1 min 54 s.
Logs are `/tmp/robot_ghosts_build.log`, `/tmp/robot_ghosts_final_build.log` and `/tmp/robot_ghosts_owner_build.log` in Docker `ros_jazzy`.
Native headless WB-ID topic/walking checks passed: G1 horizon1.0 produced five robots x35 visual elements; H2 horizon1.4 produced seven robots x34 visual elements.
The checks verified the expected marker counts, finite poses/normalized quaternions, mesh resource paths, colors/opacity, terminal topics, best-effort QoS and subscription-free startup; both short flat walks stopped upright.
Native test artifacts are `/tmp/{g1,h2}_future_ghosts*`; these topic checks did not launch the RViz GUI or establish a measured visualization CPU budget.
Five new static visualization checks and the existing 37 focused WB-ID checks passed; discovery ran 36 tests with four native ROS skips locally, and the parameter contract passed as well.
See [implementation and thread ownership](docs/humanoid_whole_body_inverse_dynamics_mpc_impl.md#12-future-robot-mesh-ghosts) for YAML controls.
All owned simulator and test processes were stopped; these changes are included with this handoff checkpoint.


## H2 Natural Stance and First Push Test (2026-10-24, Latest)

The current follow-up is on `tune/h2`, after commits `f80044c` and `2da0ac1`; walking parameters and geometry checks are committed as `e1919de`, with push-test tools and documentation in the accompanying commit.
Read [current tuning and first push result](docs/h2_wb_id_mpc.md#more-natural-foot-placement-2026-10-24-current) before the historical sections below.
The user's priorities are robustness, no leg crossing, less tandem foot placement while turning/lateral walking, and recovery rather than exact heading/speed tracking.
They explicitly accept about 0.45 m/s achieved from a 0.5 m/s lateral command and require retaining the original lateral slew of 0.5 m/s2.
They requested one first push test only, with no push-based tuning.

Saved H2-only parameter changes from `f80044c`: left/right hip-pitch joint-position Q/Qf `0.05 -> 3.0`, foot sphere radius `0.10 -> 0.085 m`, collision polynomial `mu 3000 -> 1000`, `delta 0.03 -> 0.02 m`.
Keep knee radius 0.10 m, minimumLateralSeparation 0.10 m, command maxima `[1.0,0.5,0.4,0.7854]`, original command slew, gait timing and earlier switching from the committed checkpoint.
These are soft posture/collision preferences, not PD gain changes or an explicit side-by-side foothold target.
No controller C++ architecture, G1 YAML, physical model, estimator tuning or terrain planner changed in this follow-up.
The user's unrelated `description/mujoco/robots/g1/scene.xml` edit (ramp and uneven terrain) was not used in these tests, modified by this work or included in these commits.

Geometry audit: three sole-center/fore/aft collision proxies need about 0.0692 m radius to cover the sampled H2 collision-sphere extents/box corners; 0.085 m leaves about 1.6 cm padding, comparable to G1's approximately 1.7 cm sampled padding with its unchanged 0.065 m radius.
Near-knee mesh samples reached approximately 0.090 m horizontal radius, so the H2 knee proxy stays 0.10 m.
These checks are not complete mesh-enclosure or continuous-time collision guarantees.
`tests/test_h2_model.py` checks sampled active foot primitives and permits only the intended hip-pitch deviations from copied G1 joint cost weights.

Final saved-value-equivalent flat step tests: vx+1.0 for 40 s, vy+/-0.5 for 30 s each, yaw+/-0.7854 for 22 s each; all stopped upright with no crossing or sampled opposite-leg overlap.
Forward travel was 29.91 m versus 30.0 requested, pose-derived world vx0.951 m/s.
Lateral world vy was +0.443/-0.449 m/s; mean fore/aft stagger fell from 27.90 cm in the earlier positive lateral baseline to 10.57 cm, and negative lateral mean was 9.02 cm.
Turning mean stagger fell from 28.55 cm to 3.73 cm positive and 3.54 cm negative; final positive turning stagger was 1.09 cm.
Stagger is the absolute left/right sole-center difference along pelvis-heading forward, not distance travelled per step.
The smaller 0.075 m foot radius with hip-pitch weights 3.0 was also viable, but saved 0.085 m has greater geometry padding.
Reject radius0.085/mu6000/delta0.02: it stayed upright but caused 31.06 cm turning stagger.
G1 comparison vy0.2 looked natural (1.32 cm stagger) but achieved only 0.062 m/s, so do not claim matched-speed equivalence.

The first and only push experiment used the saved source YAML with no overrides: H2 vy+0.5, two world-frame pelvis +/-80 N lateral forces for 0.15 s, starting at 10.00/14.15 s after takeover.
Applied feedback confirmed both pulses (16/14 samples, 0.15/0.13 s measured span), and confirmed zero force at the end.
It completed 30 s, continued walking and stopped upright: peak roll/pitch1.44/1.01 degrees, minimum z0.942 m, pose-derived world vy0.448 m/s, minimum signed width16.27 cm, no crossing or sampled inter-leg overlap.
Final z1.008 m and roll/pitch0.141/-0.046 degrees; no push-driven tuning was made afterward.
This is one moderate walking-push result, not a push-force limit or proof of stance recovery stepping.

New installed command: `ros2 run legged_robot_mpc_controller push_recovery_test.py`, optional `--sequence /path/to/pushes.yaml --delay 3`, `--dry-run`.
Default list `config/tests/push_recovery.yaml` specifies link/frame/force/torque/duration/wait_after; applied at body CoM through the existing custom MuJoCo wrench topic, simulation-time based, best effort, zeros on completion/exit.
`tests/wb_id_walk_test.py --wrench-sequence ... --wrench-delay 10` uses the same scheduler and saves applied body-frame feedback in `_wrenches.csv`; magnitude/timing checks reject an unverified push.
`tests/tuning/analyze_leg_clearance.py` now supports `--robot-name g1|h2`, reads each test's YAML snapshot, measures heading-relative fore/aft stagger, fixes G1 actuator-bound lookup and adds `_push.png` when wrench feedback exists.
Plots/replays use latest-sample-paired base/joint states; geometric replay contacts are not measured contact forces.
Use pose-derived speeds; GT twist convention was not revalidated here.

Rebuild/install passed in 3.81 s total, log Docker `/tmp/h2_natural_build.log`; installed script dry run passed.
All 37 focused WB-ID tests and the parameter contract passed; model/layout discovery ran 31 tests with four native ROS skips locally, including five new wrench sequence checks.
All walking/push tests were flat, headless, no RViz/GUI, requested MPC90/MRT1000, base_twist, state_estimator/humanoid_linear_kf (not InEKF).
Artifacts: Docker `/tmp/h2_natural_*`, local ignored `tuning_results/h2_crossing/`; main selected prefixes `h2_natural_foot085_mu1k_{vx100,vy050,vyn050,yaw079,yawn079}` and `h2_natural_push_first`.
No mixed-axis full-speed, backwards full-speed, rough-terrain revalidation, stronger push sweep, InEKF feedback or hardware validation was done in this follow-up.
All owned simulation and auto-test processes were stopped after verification; Docker `ros_jazzy` remains available for manual testing.


## H2 Faster Walking and Foot Crossing (2026-10-24, Historical)

The tested implementation and tuning were committed and pushed as `f80044c` on `tune/h2`.
For a simple part-by-part explanation, read [Changes in Plain Words](docs/h2_wb_id_mpc.md#changes-in-plain-words), including diagnostic-only changes and rejected tuning trials.
In short: give the legs room, discourage crossover, take quicker supported steps and switch gaits sooner when commands change.
The original command speed limits and 0.5 m/s2 lateral slew remain unchanged; no retained change to PD, Q/Qf/R, estimator tuning or physical models was needed in this pass.

Branch is now `tune/h2`, based on user commits `20e6217`/`11b3950` after shared launch commit `23f3c68`.
This checkpoint supersedes the conservative H2 command-range statements in the historical sections below.
The user's priority is balance, recovery to stance and foot clearance, not precise yaw tracking.
Read [H2 checkpoint](docs/h2_wb_id_mpc.md#faster-walking-and-anti-crossing-checkpoint-2026-10-24) for the complete trial table and [collision math](docs/humanoid_whole_body_inverse_dynamics_mpc.md#55-joint-and-collision-margins) for the added row.

At entry, the user had already set both H2 sphere radii to 0.10 m, reduced collision `mu` to 10 and `delta` to 0.00001, restored maxima `[1.0,0.5,0.4,0.7854]`, and commented out stairs in H2 `scene.xml`.
The scene edit is the user's existing change and was not changed by this work; all current tests explicitly select `scene_flat.xml`.
No G1 YAML, robot physical model, PD gain, estimator tuning, Q/Qf/R weight, launch structure or terrain planner change is retained in this checkpoint.

Saved H2 changes in `config/h2/ros2_controllers.yaml` are collision `mu=3000`, `delta=0.03`, the user's 0.10 m sphere radii, `minimumLateralSeparation=0.10`, `walk` times `[0.0,0.3,0.4,0.7,0.8]`, selected ladder `[stance,slow_walk,walk]`, and gait switch delay fraction 0.3 instead of 0.7.
The shorter single support and retained double support were crucial: stronger collision penalties alone still fell, a 0.40 s single-support walk passed 0.4/0.6 m/s but fell at 1.0 m/s, and 0.30 s single support passed 1.0 m/s.
Do not claim the no-double-support and flight templates left in the template catalog have been validated; they are no longer selected by the saved ladder.
Do not reduce the user-requested maxima to hide failure; they are normalizers, not physical feasibility guarantees.

The existing `FootCollisionConstraint` now optionally adds a signed left-minus-right foot-center margin along pelvis-heading lateral, so lifting or advancing a foot does not hide crossing.
It is a soft piecewise-polynomial penalty sharing `legSelfCollision.mu/delta`, not an RBF radial-basis penalty, hard geometric barrier, fixed pelvis midline or full-body mesh collision checker.
It stays active in double support while the original 16 3D distance rows remain masked off there.
The extra row is enabled only for positive `minimumLateralSeparation`, defaults to zero, and uses a separate `_lateral_v1` AD cache with 17 rows/four parameters versus the original 16 rows/two parameters.
Setting the value to zero restores the original collision formulation/gating with otherwise identical settings, but does not undo the separate H2 gait/penalty tuning or promise bit-identical multithreaded solver output.
The parent `legSelfCollision.activate` still disables the whole group; restart/reconfigure for either change.
G1 and centroidal configurations retain zero/default behavior; the clone now also copies the existing `isActive_` flag correctly.

The following first-checkpoint comparisons used the original 0.7 gait switch delay; abrupt-transition follow-ups motivated the final 0.3 value.
Measured baseline vx=0.4 fell at 9.45 s after takeover, with signed foot width -2.16 cm, inter-leg overlaps before the fall and minimum base z0.463 m.
Saved-settings vx=1.0 completed 36 s and stopped upright: 23.34 m versus 23.0 m requested, minimum foot width 14.28 cm, minimum base z0.901 m and maximum roll/pitch 1.19/4.50 degrees.
Saved vy=+0.5/-0.5 each completed 36 s and stopped upright, with minimum signed widths 11.52/10.49 cm and no sampled inter-leg overlaps.
The positive lateral trial remains `NO_TRACKING` under the test's strict heading criterion (0.608 rad maximum yaw drift), not `FALL`; the negative lateral trial passed the complete tracking verdict.
Saved yaw=+0.7854 completed 28 s and stopped upright, minimum width 10.94 cm, turning 11.04 rad versus 11.78 requested and translation drift below 7.5 cm.
A step-to-vx1.0 test with no publisher ramp completed 40 s including a 30 s hold and step-to-stop, travelling 30.40 m versus 30.0 m requested with minimum width 14.26 cm and no sampled inter-leg overlaps.
Configured command slew limits remain active even when the publisher ramp is zero.

An abrupt vy=+0.5 publisher step with 0.7 gait delay fell at 6.02 s without crossing (minimum width 13.71 cm) or sampled inter-leg overlap.
Failure render/curves show deep knee collapse, base z0.579 m, lateral speed overshoot near 0.8 m/s, both ankle-roll actuators hitting 19 Nm, left knee hitting 360 Nm and large nonlinear equality residuals; this is a different mechanism from the original crossover trip.
Slowing ay from 0.5 to 0.15 m/s2 made both lateral step trials survive but was explicitly rejected by the user.
Keep `velocityCommandRateLimits=[0.5,0.5,0.5,2.0]`; the user wants the solver/contact schedule to handle command changes without hiding them with additional slowing.
With only gait delay fraction changed to 0.3, both +/-0.5 lateral step commands completed 30 s and stopped upright; positive minimum width was 11.69 cm and base z0.893 m, negative base z0.951 m.
The positive earlier-switch test remains `NO_TRACKING` only because heading error reached 0.513 rad, while the negative trial passed the full verdict.
Knee saturation was absent in the successful positive comparison, but ankle-roll saturation still occurs occasionally; do not claim actuator limits or nonlinear equalities are now guaranteed satisfied.
Final no-override step-command verification with the saved 0.3 gait delay completed vx+1.0 for 40 s (30 s hold), vy+0.5 twice and vy-0.5 for 30 s each (20 s holds), and yaw+0.7854 for 22 s (12 s hold), all ending upright.
Final forward travel was 30.52 m against 30.0 requested, minimum width 14.84 cm; lateral minimum widths were 11.69/11.85 cm for positive runs and 11.59 cm for negative.
The positive repeat still returns `NO_TRACKING` due to 0.726 rad heading error, not falling; balance was not traded for heading accuracy.
Final turning achieved 8.94 rad against 9.42 requested with less than 6.8 cm translation drift.
Final turn minimum foot width was 10.38 cm; no crossing or sampled inter-leg overlap was found in any final forward, lateral or turn step test.
These publisher step tests preserve the original internal slew limits and do not claim unrestricted instantaneous actuator response.
All owned simulation/test processes were stopped at handoff; Docker `ros_jazzy` remains running and ready for manual testing.

The Jazzy two-job Release rebuild passed in 3 min 12 s; log `/tmp/h2_cross_build.log` in Docker `ros_jazzy` on VM `10.211.55.17`.
All trials used requested MPC90/MRT1000, `state_estimator`/`humanoid_linear_kf`, `base_twist`, flat ground, no GUI and no RViz.
The log shows actual MPC commonly around 80-84 Hz, zero solver exceptions in the saved forward trial and some torque-window fallback ticks; solver constraints are not certified feasible just because the robot stayed upright.
The five new reference-math/wiring tests are included in 37 passing focused WB-ID checks; four native ROS launch tests and 25 model/layout/launch discovery checks also passed (the four ROS tests skip locally and pass in Docker).
An overly broad local `wb_id_*test.py` discovery also imported the ROS integration executable and failed only because local macOS has no `rclpy`; use named contract tests locally or the sourced Jazzy environment for integration scripts.

`tests/wb_id_walk_test.py --record-joints` records latest-sample-paired joint/base poses and estimator orientation/twist snapshots to `PREFIX_replay.json`.
`tests/tuning/analyze_leg_clearance.py PREFIX --render` reconstructs H2 FK, signed width, point distances, replay geometric contacts, pose-derived velocity curves, knee/ankle/hip support-effort curves and measured-pose PNGs.
Use pose-derived speed for interpretation; the GT twist-axis convention was not revalidated during this session.
Replayed contacts are geometric overlap checks, not measured forces, and sample-based absence is not a continuous-time collision guarantee.
Artifacts are in Docker `/tmp/h2_cross_*` and locally under ignored `tuning_results/h2_crossing/`.
The older low-speed ramp trial `/tmp/h2_ramp_baseline.json` completed 45 s at vx0.15 on the 3-degree ramp and stopped upright, but the current faster-walking changes have not been revalidated on ramps.
Mixed-axis full-speed commands, external push recovery, complete height sweeps, InEKF feedback and real hardware remain unvalidated; avoid claiming equivalence to G1 across all operating conditions.


## Robot-Agnostic Launch and Robot-Local Scenes (2026-10-24)

The current entry point is `robot.launch.py robotName:=g1|h2`; the former `g1.launch.py` and `h2.launch.py` files have been removed.
All 31 existing launch argument names remain available, with only `robotName` added, and the original multiline `mpcControllerName` declaration/choices style is retained.
Robot selection resolves the controller YAML, URDF, ros2_control xacro, initial pose, terrain variants and separate `auto_generated/<robot>/` cache together.
G1 supports WB-ID and centroidal, while H2's YAML remains WB-ID only; an unsupported controller or H2 lidar request fails early with an explicit error.

Each robot now owns `description/mujoco/robots/<robot>/scene.xml`, `scene_flat.xml` and `scene_ramp.xml`, so `mujocoModelFile` uses identical names for both robots.
Mesh compiler paths and texture directories were adjusted for the new top-level scene location; shared backgrounds, terrains, objects and textures remain in `description/mujoco/`.
Compiled models for all six moved scenes were compared against commit `332a369`: body masses/poses/inertias, joint/actuator bounds, geometry/contact properties and qpos0 are unchanged.
The scene generator supports `--robot-name g1|h2`, writes scenes beside the selected robot module and keeps generated textures/backgrounds in their shared directories.

The controller YAML now owns numeric `ocs2.reference.defaultBaseHeight`: 0.795 m for both G1 MPC blocks and 1.02 m for H2 WB-ID.
The existing launch parameter resolver reads that selected controller value and passes the same height to the GUI through `guiRefBaseHeight`.
`refBaseHeight` remains an optional argument with an empty default; when specified it updates only the selected controller in the existing launch-generated temporary YAML and the GUI together, never the source YAML.
Native ROS tests passed for both defaults, controller selection, custom YAML height, explicit overrides, untouched source files and invalid heights/controller/lidar choices.
No OCP, reference-generation C++, estimator or tuning values changed; the explicit YAML height replaces the former launch substitution with the same effective values.

The two-job Jazzy Release build passed in 3.40 s (`/tmp/robot_launch_build.log`).
The 21 model/scene/shared-terrain checks passed locally, along with 23 WB-ID contract checks, nine torso checks and the standalone parameter contract; all four native ROS launch tests passed in the sourced container.
Both robots completed 31 s headless flat-ground trials at vx=0.15 m/s, with no GUI, no RViz and no height override, then stopped upright without falling.
H2 used 100 Hz requested MPC, travelled 2.893 m, had maximum roll/pitch 0.04683/0.02700 rad and minimum z0.9707 m; G1 used 90 Hz requested MPC, travelled 2.247 m and had maximum roll/pitch 0.00717/0.03255 rad.
Raw artifacts are `/tmp/robot_launch_{g1,h2}_flat.{log,yaml,csv,json}` in Docker `ros_jazzy`; these are ordinary `base_twist` tests, not a mapped-terrain capability claim.
The README has one shared launch example and a supported-robots table distinguishing physical actuated joint DoFs (G1 29, H2 31) from MPC-active joint DoFs (G1 23, H2 25), excluding the floating base.


## Shared Terrain Paths and H2 YAML Formatting (2026-10-24)

Terrain planner files moved out of `config/g1/terrain/` and `config/h2/terrain/` into shared `config/terrains/stair_climbing/{g1,h2}.yaml` and `config/terrains/terrain_walking/{g1,h2}.yaml`.
Both launch files select their corresponding variant automatically; existing `stairClimbingFile` and `terrainWalkingFile` override arguments and the C++ YAML loaders are unchanged.
Variants remain necessary because five robot-dependent height/foot-spacing/margin settings differ; the migration does not replace H2's 1.03 m terrain-height ceiling with G1's 0.80 m value.
See [Shared Terrain Configuration](config/terrains/README.md) for the layout, preserved differences and external-path migration.

H2's controller YAML now follows G1's blank lines, grouped joint-name lists, wrapped numerical arrays, inline gait entries and explanatory comments, with the two head joints grouped separately before the arms.
Robot-specific comments and the commented torque-limit example were adapted to H2 rather than copying G1 values into its documentation.
A parsed comparison against commit `5badc22` confirmed exact equality of all values in both controller YAML files and all four moved terrain configurations; no controller block or tuning value was added or removed.
Only launch paths, schema/header descriptions, documentation, formatting and regression tests changed; no OCP, estimator, target generation, runtime C++ behavior or generated-model signature changed.

The two-job Jazzy Release symlink-install build passed in 2 min 22 s; log `/tmp/shared_terrain_build.log` in Docker `ros_jazzy` on VM `10.211.55.17`.
Twenty-one model/scene/shared-terrain checks, 23 WB-ID contract checks, nine torso-cost checks and the standalone parameter contract passed locally.
The new shared-terrain tests verify both launch defaults, absence of the former YAML locations, robot-specific dimensions and consistency with `stairs_params.yaml`.
After rebuilding, both WB-ID controllers passed isolated 31 s flat-ground forward trials at 0.15 m/s with the saved YAMLs and no parameter overrides, using estimator feedback, headless MuJoCo, no RViz and `baseCommandGui:=false`.
Each trial held the full command for 15 s, ramped in/out for 3 s each and returned to stance for 6 s after the initial 4 s settle interval.
G1 travelled 2.248 m against 2.700 m requested, with maximum roll/pitch 0.00931/0.03167 rad and heading error 0.03096 rad; its minimum z 0.6387 m includes the initial rise from the crouched posture, with final z 0.7504 m.
H2 travelled 2.932 m against 2.700 m requested, with maximum roll/pitch 0.03422/0.02118 rad, heading error 0.20716 rad, minimum z 0.9685 m and final z 1.0022 m.
Both returned `SUCCESS` and stopped upright without a fall; the tests validate bring-up, path loading and ordinary `base_twist` walking after migration, not stair-climbing or mapped-terrain performance.
Container artifacts are `/tmp/shared_terrain_{g1,h2}_walk.{log,yaml,csv,json}` and `/tmp/shared_terrain_build.log`, copied locally to ignored `tuning_results/shared_terrain/`.


## H2 WB-ID Integration (2026-10-24)

Current branch is `feat/h2`, created from `feat/wb_id_mpc` at `f56ae9a`; the older sections below describe G1 history unless explicitly stated otherwise.
H2 now has `launch/h2.launch.py`, `config/h2/`, `description/{meshes,urdf,mujoco}/robots/h2/` and the `scene_h2*.xml` entry points, without changing runtime C++, the G1 launch/config/models or the WB-ID architecture.
The H2 YAML has WB-ID only, with both existing estimators available and `humanoid_linear_kf` selected by default; no centroidal controller block was added.
See [H2 Integration and Validation](docs/h2_wb_id_mpc.md) for the complete tuning table, results and commands, and [MODEL_SOURCE.md](description/urdf/robots/h2/MODEL_SOURCE.md) for official Unitree source revisions and integration changes.

The official H2 model has 31 physical joints, with six wrists locked in the MPC model and held by the existing servo path, leaving 25 active joints, 62 states and 68 inputs.
Both head joints remain active and use the existing generic Cartesian cost on `head_yaw_link`, with orientation/angular-velocity weights only; torso stabilization remains `torsoTiltCost`, not another generic torso frame cost.
H2 ankle order is roll then pitch, its contact parent is ankle pitch, and its head joints occur before the arms in Pinocchio order; all robot-dependent arrays were mapped by joint name.
The ankle-pitch frames were translated along their own pitch axes with opposite inertial/geometry translations so both feet use one symmetric contact offset without changing physical geometry or axes.
Contact translation is `[0.035,0,-0.058]`, rectangle `[0.105,-0.105,0.03,-0.03]`, and nominal sole centers are y=+/-0.14 m.

Do not undo the explicit MJCF pelvis inertia correction: Unitree's MJCF omitted it and inferred 4.248 kg, while its URDF specifies 8.5477 kg; the integrated total mass matches at 75.58746 kg.
Do not replace distal wrist-yaw MJCF inertias with URDF wrist-only inertias, because the former already include the fixed hands.
The pelvis IMU site and URDF IMU frame are matched at `[-0.055,0,-0.0589]`.
G1's raw crouch angles are invalid for H2's different hip axes; retained spawn and standing postures were solved with level soles and support midpoint aligned with the model CoM in x.
Spawn height is 0.882 m and standing reference is 1.02 m; the support midpoint x is approximately 0.04961 m in the crouch and 0.01771 m in standing.

The original G1 PD gains failed during H2 stand-up even after posture balancing, while higher gains alone did not fix the initially unbalanced settling posture.
Retained PD is kp600 for all active joints, kd40 except knees kd20; Q/Qf base velocity is `[60,300,5,30,30,30]`; per-foot contact Mz weight is 100 instead of 5 with the inherited `rScaling=0.001`.
The Mz change prevented the observed +0.20 m/s forward fall, but it is a soft regularizer rather than a new torsional-friction constraint.
Raising yaw-rate Q/Qf weight from 30 to 300 caused an earlier fall and was rejected.
Stance exits at commanded linear speed 0.03 m/s and slow_walk exits at 0.02 m/s, because the inherited 0.10 threshold kept a 0.08 lateral command in stance.
Command normalization maxima are conservatively 0.15 m/s forward, 0.08 m/s lateral and 0.15 rad/s yaw; normal GUI/topic commands are still normalized, not SI-valued messages.

Measured flat-ground runs completed and stopped upright with estimator feedback: vx+0.15 for 41 s, vx-0.10 for 36 s, vy+/-0.08 for 36 s, and yaw+/-0.15 for 31 s.
The +0.15 forward run travelled 4.428 m against 4.200 m requested, with maximum roll/pitch 0.0546/0.0218 rad, minimum z 0.955 m and maximum heading error 0.158 rad.
Lateral travel was +1.435 m and -1.375 m against +/-1.840 m requested, so lateral tracking remains imperfect even though the robot stayed upright.
One +0.20 m/s run also completed 36 s with stronger Mz regularization, but had 0.359 rad heading error; the saved command maximum intentionally remains lower.
The final two-job Jazzy Release symlink-install build passed in 3.92 s; log `/tmp/h2_final_build.log` inside Docker `ros_jazzy` on VM `10.211.55.17`.
The subsequent saved-configuration negative-yaw test passed without overrides, turning -2.153 rad against -2.700 requested with maximum translation drift 0.159 m, then stopped upright.
Seven H2 model/xacro/array/order/limit tests, 11 scene-layout tests, 23 WB-ID contract tests, nine torso-cost tests and the standalone parameter contract passed locally.
No tests establish arbitrary-duration stability, simultaneous command robustness, high-speed walking, terrain capability, InEKF feedback quality or real-hardware readiness.

Trials used `mujocoModelFile:=scene_h2_flat.xml`, 100 Hz requested MPC, 1000 Hz MRT, `baseCommandGui:=false`, `rviz:=false` and headless MuJoCo.
Typical MPC timing was 82-86 Hz with approximately 12 ms advances, with occasional longer spikes; do not claim the VM achieved a sustained 100 Hz.
H2 CppAD libraries live in the separate workspace-relative `auto_generated/h2/wb_id_mpc_h2/` cache; the first launch compiled them, while subsequent runs reused them.
The test harness now accepts robot-specific launch/scene/height settings, waits for actual locomotion takeover and detects initialization errors before waiting for odometry.
All raw trial files are in container `/tmp/h2_*` and copied locally to ignored `tuning_results/h2/`, including plots and a rendered replay from measured joint/base state.

Important diagnostics caveat: the existing simulator odometry publisher asks `mj_objectVelocity` for local `mjOBJ_BODY` velocity, which is in the body's inertial axes rather than the pelvis link axes.
H2 has a strongly rotated pelvis inertia frame, so reported GT vy/yaw-rate values can look nearly zero despite real lateral/turning displacement.
This was reproduced locally with a known `[1,2,3]` world velocity; no external simulator code was changed in this task.
Use recorded pose displacement/unwrapped orientation for validation; the harness now also reports position-derived world velocities.
The default H2 estimator-control loop does not consume that published GT odometry topic.

## WB-ID Constraint Naming Cleanup (2026-10-24)

The current WB-ID parameter groups are `ocs2.constraints.footMotionEquality`, `swingFootZeroWrench` and `legSelfCollision`, replacing `footConstraint`, `zeroWrench` and `footCollision` respectively.
Generated schema, builder, controller warnings and WB-ID activation fields use the new names; external overrides must migrate, and the controller needs a rebuild before loading the renamed YAML.
This is a naming-only change: all gains, activation defaults, bound scalings, OCP equations, term names and CppAD cache identities remain unchanged, as do the centroidal configuration and shared constraint classes.
`legSelfCollision` checks selected opposing foot/ankle/knee separations and is inactive in double support; it does not check arm/torso/thigh collisions or provide full-body collision geometry.
The renamed code rebuilt successfully with two jobs in Jazzy Release mode in 1 min 56 s; build log: `/tmp/wbid_constraint_names_build_20261024.log`.
The flat-scene headless smoke test returned `MPC_RUNNING` after 11.69 s, then immediately stopped the launch group; log and JSON verdict: `/tmp/wbid_constraint_names_startup.log` and `.json`.
The run used 100 Hz MPC, 1000 Hz MRT, the default estimator, `rviz:=false` and `baseCommandGui:=false`; it did not test walking or disabled-constraint configurations.
The parameter contract, 23 static wiring checks and nine torso-cost checks passed; 164 source-line links in the two WB-ID guides were checked for file existence, anchor bounds and numeric-label consistency, with stale registration/YAML links and the friction-normal setter description corrected.


## WB-ID Constraint Activation and Bound Scaling (2026-10-24)

Every WB-ID `ocs2.constraints` group now has `activate: true`, including an explicit `swingFootZeroWrench` block for the existing swing-wrench equality; none of the original eight groups was an unused shell.
The switches gate OCP term construction/registration during configuration; changing them requires a restart or reconfiguration, and disabling dynamics/contact equalities is diagnostic-only with a startup warning.
`jointLimits` is renamed to `jointPositionLimits`; its existing shared factory already reads URDF lower/upper position bounds and remains unchanged for centroidal.
Velocity limits retain their URDF source, with `jointVelocityLimits.scale` renamed to `velocityScaling`; `jointTorqueLimits.torqueScaling` scales URDF or override torque bounds before sharing them with the OCP and runtime clamp.
Both scalings default to 1.0 and must be finite with `0 < scaling <= 1`; use activation flags rather than zero scaling to disable OCP terms.
Disabling the OCP torque penalty does not disable feedforward clipping, and disabling inverse dynamics selects measured-state RNEA instead of executing unconstrained policy torque inputs; this does not restore physical feasibility to an ablated plan.
All prior numerical settings and the centroidal YAML were compared against HEAD and preserved after normalizing the renamed paths and new default flags/scaling.
The parameter contract, 23 static wiring checks and nine torso-cost checks passed; the activation tests cover all 512 switch combinations statically, not their runtime feasibility.
The two-job Jazzy Release rebuild passed in 1 min 35 s (`/tmp/wbid_constraint_activation_build_20261024.log`), and the flat-scene headless smoke test returned `MPC_RUNNING` after 11.37 s before immediately shutting down its launch group.
The startup log and JSON verdict are `/tmp/wbid_constraint_activation_startup.log` and `.json`; the run used 100 Hz MPC, 1000 Hz MRT, the current estimator defaults, no command GUI and no RViz, with no walking command or post-takeover robustness trial.


## WB-ID Shared Torque Limits (2026-10-24)

`control.torqueLimit` moved to optional `ocs2.constraints.jointTorqueLimits.limits`, alongside the unchanged `mu: 0.1` and `delta: 5.0`.
The YAML now omits `limits`, meaning URDF joint effort limits; a full positive finite array in joint order overrides those defaults.
Do not write an explicit `limits: []` in the runtime YAML: Jazzy converts it to an unset override and rejects initialization with `parameter_value_from failed` / `No parameter value set`; the generated schema's typed empty-vector default works when the override is omitted.
All 23 current URDF efforts match the former YAML array, so the effective G1 limits are unchanged.
The interface resolves the limits from its reduced Pinocchio model or override once, validates them, and provides the same vector to the OCP penalty and runtime feedforward clamp through `getTorqueLimits()`.
The OCP uses soft margins `limits - tau` and `limits + tau` with a relaxed log barrier, active only inside the inverse-dynamics torque window (currently 3 nodes, approximately 0.05 s).
Runtime clamps only the computed MPC feedforward torque; the downstream effort_pd feedback can increase the total torque beyond these bounds, and final saturation is a separate hardware/simulator responsibility.
`mu` is penalty strength and `delta` is the relaxation transition margin, not an allowed overrun; neither affects the hard feedforward clamp.
An empty array now selects URDF bounds instead of disabling both uses; invalid resolved limits fail configuration, and the centroidal controller retains `control.torqueLimit`.
No dynamics, CppAD signature, tuning values or launch behavior changed; external WB-ID YAML files and overrides need the new path and a controller rebuild.
The two-job Jazzy Release symlink-install build passed in 1 min 22 s; log: `/tmp/wbid_urdf_torque_limits_build_20261024.log`.
The parameter contract, 16 static wiring checks and nine torso-cost checks passed, including URDF-default and override checks; no simulation or walking trial was launched for this change.
The subsequent startup-error reproduction is saved in `/tmp/wbid_urdf_startup_before.log` inside the Jazzy container; that real launch exposed the empty-YAML-override issue despite the earlier successful build and static tests.
`tests/wb_id_startup_smoke_test.py` now runs a real flat-scene launch without the GUI or RViz, detects initialization failures, records its result and shuts down immediately on MPC takeover; this is startup validation, not a walking robustness test.
After omitting the runtime `limits` override, the two-job Release rebuild passed in 1 min 14 s (`/tmp/wbid_urdf_startup_fix_build_20261024.log`).
The rebuilt flat-scene startup smoke test returned `MPC_RUNNING` after 11.28 s and immediately stopped its launch group; results are `/tmp/wbid_urdf_startup_after.log` and `.json` in the Jazzy container.
The trial used the existing estimator defaults, `mpcFreq:=100`, `mrtFreq:=1000`, `mujoco_headless:=true`, `rviz:=false` and `baseCommandGui:=false`; no walking commands or subsequent robustness tests were run.


## WB-ID Parameter Ownership Cleanup (2026-10-24)

WB-ID now groups SQP and rollout settings under `ocs2.solver.sqp` and `ocs2.solver.rollout`, while keeping `ocs2.mpc` at its existing location.
`control.policyTimeOffset` moved to `ocs2.solver.rollout.policyTimeOffset`; both policy-coverage validation and execution use the same value, which is still 0.000 in the G1 YAML.
Code-generation flags moved to `ocs2.codegen`; physical contact geometry to `ocs2.model.contactGeometry`; torque-window nodes to `ocs2.constraints.inverseDynamics`; transition stance timing to `ocs2.gait`; and ground-height/arm-name settings to `ocs2.reference.groundHeight` and `.armSwing`.
See [Implementation Guide, Section 10](docs/humanoid_whole_body_inverse_dynamics_mpc_impl.md#10-wb-id-parameter-layout-migration) for the complete old/new path table and external-YAML migration instructions.
All configured values, schema defaults/types/validation, `ocs2.mpc` settings and every other controller's YAML block were compared against the previous revision after normalizing these path moves and found unchanged.
No launch, estimator, shared OCP equation or CppAD signature changed, and the common model still resolves the same arm joint indices even when arm swing is disabled.
The shared torque-limit roles and the pre-existing zero acceleration-gain schema defaults were not changed as part of a layout-only task.
External WB-ID overrides need the new names and a rebuilt controller; the centroidal controller deliberately retains its existing paths.
The two-job Jazzy Release symlink-install build passed in 1 min 19 s with `BUILD_TESTING=OFF`; container log: `/tmp/wbid_parameter_layout_build_20261024.log`.
The parameter contract, six startup checks, six cost-activation checks and nine torso-cost checks passed, with YAML/schema value-preservation checks confirming a migration-only change.
No simulation was started for this structural cleanup, so these checks do not add a new walking validation.

## WB-ID Foot-Constraint Parameter Migration (2026-10-24)

The WB-ID YAML and generated parameter schema now place the eight foot-stabilization gains at `ocs2.constraints.footMotionEquality`, replacing `ocs2.model.footConstraint`.
The builder reads the new path and still fills the same internal `ModelSettings::footConstraintConfig` and `footAccelerationGains` fields; all configured gains and schema defaults are preserved.
External WB-ID parameter files must migrate the block, and the controller needs a rebuild for the new generated parameter structs before running the new YAML.
The centroidal controller and schema retain their existing layout, and no launch, OCP equation, CppAD model signature or other parameter ownership changed in this migration.
Foot stance/swing rows are hard OCP equalities with stabilization feedback, not soft task-space costs.
Other parameter-layout changes remain proposals pending the user's approval.

## Blind-Walking Tuning Guide

[Formulation and Signal Flow, Section 9](docs/humanoid_whole_body_inverse_dynamics_mpc.md#9-blind-walking-robustness-and-tuning) records the ranked tuning table, measured ramp outcomes, correction-gain equations, experimental procedure and limitations.
[Implementation Guide, Section 9](docs/humanoid_whole_body_inverse_dynamics_mpc_impl.md#9-blind-walking-tuning-and-validation) supplies current parameter/code line references.
Only the paired foot-stabilization gain reduction was isolated in A/B trials; other retained settings have engineering rationale but no independent ramp ablation.
Do not promote the shallow-ramp results to a formal stability guarantee or a validated arbitrary uneven-terrain capability.

## Launch Scope Correction (2026-10-24)

The user wants unknown-terrain accommodation in ordinary locomotion through C++/parameter changes, not a simulator-geometry pipeline or the dormant `terrain_walk`/`stair_climb` modes.
`launch/g1.launch.py` was restored exactly to HEAD (`3ffceca`), removing the `terrainFromScene` argument, XML-to-YAML hook and temporary-file lifecycle changes.
Omit `terrainFromScene` from future launch commands; the prior successful trials passed it as false, so the removed hook was already inactive in those tests.
The tested `positionErrorGain_z: 50.0` and `orientationErrorGain: 100.0` remain unchanged.
The standalone `launch/terrain_scene_model.py` exporter and mapped-terrain C++ extensions from the earlier checkpoint are still present but unused by normal `base_twist`; they are not required for the measured improvement.
The explicit `TerrainFootholdPlanner` itself already existed in HEAD; the earlier changes extended its geometry, normals, reach checks and reference generation, rather than introducing the planner for normal locomotion.
The archived terrain-checkpoint document is not a current launch instruction.
Only launch/documentation changed in this scope correction; Python syntax and exact launch equality to HEAD were checked, but the full simulation suite was not rerun.

### Change Inventory Relative to HEAD

- Normal WB-ID parameter changes: Q/Qf base z weight 100 to 50 and vz weight 10 to 5; foot xy velocity weights 10 to 5; torso free-tilt corridor 5 to 10 degrees; extra leg-load cost disabled; swing clearance 0.08 to 0.10 m; foot stabilization gains 250 to 50 vertically and 1000 to 100 for orientation.
- The final two foot-stabilization gains were isolated in the ramp A/B tests; the other parameter changes were already present in both baseline and candidate, so those tests do not establish their individual effects.
- Mapped-planner extensions: inclined-region height/normal queries, scene-style region YAML loading, reach/height/occlusion candidate checks and retaining the previous foothold when no candidate is valid.
- Mapped-reference extensions: smooth anticipated support transfer, matching base vertical velocity and denser horizon reference samples.
- Contact wiring: mapped normals feed the foot cost, stance tilt equality and friction cone with rotated analytic derivatives; stance/swing AD models use 14 parameters and distinct `_v2` cache names.
- Those mapped-planner/reference changes only provide their terrain capability when `terrain_walk` is active; ordinary `base_twist` still uses a world-up normal and its existing ground-height fallback.
- Offline tooling: scene-geometry exporter and geometry/reference regression tests; tuning harness duration/readiness improvements; full-run tracking plots and recorded-state MuJoCo replay images.
- No new perception, terrain-state estimator, unknown-surface normal estimator or contact-triggered touchdown adaptation was implemented, and no new inverse-dynamics model was introduced.
- The explicit foothold planner predates these changes; the launch hook was added for simulation-known geometry and has now been removed because it does not meet the user's unknown-terrain goal.

## Ordinary Base-Twist Ramp Tuning (2026-10-24)

This is the latest task: the user is NOT using `terrain_walk` or `stair_climb` and requested ordinary `base_twist` walking on the ramp included in `description/mujoco/scene.xml`.
Do not assume the older mapped-terrain checkpoint below solves this path.
No legacy-mode implementation was changed in this tuning pass, and all trials explicitly use `terrainFromScene:=false`.
The scene remains the user's 3-degree ramp, beginning near x=1.5 m and ending near x=5.49 m with an abrupt drop to the floor.

Only two WB-ID controller parameters changed from the user's current configuration:

```yaml
ocs2:
  reference:
    groundHeight:
      useEstimate: false  # unchanged
  constraints:
    footMotionEquality:
      positionErrorGain_z: 50.0     # previously 250
      orientationErrorGain: 100.0  # previously 1000
```

All acceleration gains remain 1 and velocity gains remain 50.
These are stabilization gains inside hard acceleration equalities, not soft costs or relaxed contact constraints.
The original orientation term demanded rapid correction toward a world-horizontal foot even on the incline; lowering it reduces that conflict while retaining angular damping.
The vertical term likewise corrects height error more slowly instead of fighting the contact surface as aggressively.
Q/Qf/R, servo gains, foot task-space weights, estimator parameters and gait timing were not changed in this pass.

Trials use Parallels container `ros_jazzy`, MPC requested at 100 Hz, MRT 1000 Hz, command GUI disabled, default `base_twist`, `humanoid_linear_kf` with `floatingBaseSource:=state_estimator` unless explicitly marked otherwise.
The command schedule is 4 s stand, 3 s ramp to vx=0.3 m/s, 20 s hold, 3 s ramp down and 5 s stop; the published command is normalized using the configured velocity scale.
The probe schedule is wall-clock based; recorded simulator timestamps are used for plots and velocity metrics.

| Case in `tuning_results/` | Change | Outcome |
| --- | --- | --- |
| `ramp_baseline` | Original gains, RViz and visible MuJoCo | Fell 27.4 s after command onset, max x=3.71 m, mean walking vx=0.145 m/s. |
| `ramp_baseline_headless` | Original gains, both visualizers off | Fell again 27.4 s after command onset, mean vx=0.158 m/s. |
| `ramp_gt_feedback` | Reduced gains, height adaptation on, simulator feedback for isolation only | No fall, stopped at x=5.00 m, mean vx=0.219 m/s. |
| `ramp_est_feedback` | Reduced gains, height adaptation on, estimator feedback | No fall, stopped at x=5.26 m, mean vx=0.226 m/s. |
| `ramp_est_gains_only` | Reduced gains only, height adaptation off, estimator feedback | No fall, stopped at x=5.25 m, mean vx=0.226 m/s; max absolute roll/pitch over the recording 0.87/1.43 degrees. |

The first trials above used the previously installed binaries.
The two-job Jazzy Release symlink-install build of the current tree then passed in 5 min 24 s, with existing compiler/deprecation warnings only; its container log is `/tmp/wbid_base_twist_ramp_build_20261024.log`.
The rebuilt process loaded the current `_v2` stance/swing AD libraries, so the final comparison is not based on stale installed constraints.

| Post-build case | Command | Result |
| --- | --- | --- |
| `ramp_rebuilt_baseline` | vx=0.3 m/s, original gains, headless | Fell 27.9 s after command onset, max x=3.91 m, walking vx=0.158 m/s. |
| `ramp_final_headless` | vx=0.3 m/s, reduced gains | No fall, final x=6.25 m, walking vx=0.244 m/s. |
| `ramp_final_slow` | vx=0.2 m/s, reduced gains, 25 s hold | No fall, stopped on the ramp at x=4.17 m, walking vx=0.149 m/s. |
| `ramp_final_rviz` | vx=0.3 m/s, reduced gains, RViz enabled and MuJoCo headless | No fall, stopped on the ramp at x=5.21 m, walking vx=0.226 m/s. |

Parsed trial YAML comparison confirmed exactly two changed controller values between the rebuilt baseline and final tests: the vertical position and orientation stabilization gains.
No final test uses simulator-state feedback for control; that was only the explicitly named earlier isolation run.
The headless final run went past the far edge and landed without falling, but its full-run plot shows a roughly 1.1 m/s forward speed spike and a sharp height drop there.
Do not interpret survival of that event as validated controlled descent; the slower run and RViz repeat provide cleaner on-ramp stopping evidence.
Walking-window statistics deliberately exclude most of that edge event, so always inspect the full-run `ramp.png` as well as `metrics.json`.
Total equality SSE changes scale when stabilization gains change; smaller SSE alone is not proof of improved physical feasibility.
The parameter contract, nine torso-cost checks, six cost-activation checks, six startup checks, Python compilation and shell syntax validation also passed.
All test-owned simulation/recording processes were stopped after the trials.

The headless baseline confirms that graphical overhead alone does not explain the failure.
The gain-only ablation shows that enabling FK height adaptation is not needed for this improvement.
This is improved shallow-ramp ascent and stopping, NOT a general controlled-descent or arbitrary uneven-terrain result.
Tracking is still below the 0.3 m/s command, and high-frequency leg torque content did not improve; prioritize not falling before increasing tracking stiffness.

The existing linear KF still observes every contact height as zero, so its absolute base height misses the ramp rise by approximately 0.1 m on average during these trials and nearly 0.2 m at the end.
Do not present successful walking as accurate terrain-height estimation.
Simply enabling `useTerrainHeightEstimate` cannot recover terrain height already removed by this estimator assumption.
Separately, the ordinary ground-height adapter only adds the difference from its previous support-height estimate, while the command generator supplies fresh trajectories each solve; accumulated-offset bookkeeping remains a follow-up if this option is enabled with a terrain-capable estimator.
It was deliberately left disabled rather than adding unrelated reference-manager or estimator changes without validation.

Tools: `tests/tuning/run_case.sh` now recognizes first-policy startup for simulator-feedback isolation runs and permits longer runs; `HOLD_SECONDS` configures the probe hold duration.
`tests/tuning/plot_ramp.py <case> [--render]` writes full-run `ramp.png`, `ramp_metrics.json` and optional `replay0.png` through `replay4.png`.
Replay images reconstruct recorded physical base pose plus joint encoders in MuJoCo; they are not live screenshots or estimated robot poses.
`replay4.png` is the last recorded state, including the fallen baseline.
Existing `SHOTS=1` live X11 screenshots were also captured in the baseline but overlap with desktop windows, so use the replay images for an unobstructed inspection.
The default RViz environment still depicts an independent staircase rather than the selected MuJoCo ramp; this is a visualization mismatch, not the simulated collision terrain, and is another reason to use the recorded-state MuJoCo renders for judging physical contact.
Result folders are git-ignored, remain on the shared filesystem, and contain exact trial configs, logs, arrays and plots.

## Terrain Reference Checkpoint (2026-10-24, Jazzy build passed; terrain trial pending)

See [wb_id_terrain_checkpoint.md](docs/wb_id_terrain_checkpoint.md) for the launch/mode commands, tuning table and supported geometry.
WB-ID MuJoCo launch now defaults `terrainFromScene:=true`: static box-top regions and the background floor are exported from the selected scene to a temporary terrain YAML, without modifying the user's `scene.xml` edits.
`terrain_walk` must still be selected explicitly; geometry export does not change the default `base_twist` mode.
The planner handles inclined planar heights/normals, scene-derived candidate reach bounds and the background floor under below-floor tiles; legacy staircase heuristics remain isolated to the old stair model.
Terrain normals now feed the shared foot cost, WB-ID stance tilt and the friction cone, including its analytic derivatives.
The WB-ID stance/swing AD models use 14 parameters and `_v2` cache names, avoiding stale 11-parameter libraries.
The terrain base-height path uses smooth anticipated support transfer with matching analytic vertical velocity, sampled every 0.05 s.
Initial WB-ID tuning: Q/Qf height 50 and vz 5; foot tilt 300 and xy velocity 5; extra leg-load cost disabled; torso free lean 10 degrees; swing clearance 0.10 m; planner height cap 0.80 m to avoid an entry crouch at the default 0.795 m command.
Existing `useTerrainHeightEstimate: false` and `maxTerrainHeightStep: 0.002` remain unchanged; these control only the ordinary FK-ground-height fallback, not explicit mapped terrain.
Seven geometry/reference checks passed before the user's final build-only instruction, including ramp agreement with MuJoCo compilation and ray casts, all 440 tile tops, empty flat-scene geometry, smooth support derivatives and normal/cache wiring.
The WB-ID parameter contract and nine torso reference-math/wiring checks also passed.
No walking experiment was run for this checkpoint, and no simulation is to be launched: the user explicitly requested build only and will test themselves.
The current linear KF's contact-height observations remain zero, so first isolate terrain control with `floatingBaseSource:=ground_truth_state`; validating a non-flat estimator-height path is still required afterward.
This is a simulation-known-geometry checkpoint, not perception, drift-corrected map registration, general mesh/heightfield support or a terrain stability guarantee.
Build log in `ros_jazzy`: `/tmp/wbid_terrain_checkpoint_build_20261024.log`.
The two-job symlink-install Release build completed successfully in 5 min 52 s with `BUILD_TESTING=OFF`; only existing compiler/deprecation warnings were reported.
No simulator or MPC process was started after this build, as requested.

## Gravity-Relative Torso Stabilization (2026-10-24, build and startup verified)

Added `WbIdTorsoTiltCost`, registered as the running OCP cost `torsoTiltCost` when `ocs2.costs.torsoTiltCost.activate` is true.
G1 enables it on `torso_link` with `freeTiltAngle: 0.08726646259971647` rad (5 degrees), `tiltWeight: 10.0` and `angularVelocityWeight: 5.0`; these are initial settings, not tested gains.
The generic `taskSpaceCosts.torso` tracker is disabled to avoid competing objectives; the new schema default is opt-in (`activate: false`) for external parameter files.
The residual penalizes tilt beyond the free corridor and physical angular velocity perpendicular to model gravity, with no pose trajectory, position target or yaw target.
The corridor and weights are AD parameters rather than frozen generated-code constants; the model has a distinct `wb_id_torso_tilt_v1_<link>` cache name.
The cosine is clamped before `acos` so upright AD derivatives stay finite; a lower clamp at `-1+1e-9` bounds the singularity near inversion.
Bending remains possible because this is soft, but picking tasks should widen the corridor or reduce/disable this cost and revisit existing pelvis/waist posture penalties.
This is not a CoM stability guarantee, and does not add upper-body collision avoidance.
No contact/dynamics constraint, Q/R weight, estimator, servo or centroidal-controller behavior was changed; retain the user's concurrent foot-orientation weight tuning.
Reference-math and static wiring tests cover the free corridor, yaw invariance, transverse damping, finite upright residual derivatives, bending and schema/config registration.
Built successfully in Parallels container `ros_jazzy`, workspace `/root/ocs2_ros2_ws`, with two jobs, Release and `BUILD_TESTING=OFF`.
The first build exposed a Pinocchio header compatibility issue; `frameId_` now uses `std::size_t` rather than relying on the visibility of `pinocchio::FrameIndex`.
Launched `scene_flat.xml`, MPC 100 Hz / MRT 1000 Hz, `rviz:=false`, `mujoco_headless:=true`, `baseCommandGui:=false`, with the default `state_estimator` source and `humanoid_linear_kf` backend.
OCS2 generated and loaded `wb_id_torso_tilt_v1_torso_link_lib.so` at runtime with `recompileLibrariesCppAd: false`; the cache symlink resolved to `/root/src/mpc/ocs2_auto_generated/g1/wb_id_mpc_g1/wb_id_torso_tilt_v1_torso_link/cppad_generated/`.
The first MPC advance completed in 20.588 ms, the policy was accepted, and bring-up transitioned through blending into locomotion.
Stopped immediately at locomotion as requested, sent no motion commands, and verified that no simulation or ROS test processes remained.
This is startup validation only, not a walking, robustness, bending or hardware validation; the user will test those next.
Container logs: `/tmp/wbid_torso_tilt_build_20261024.log` and `/tmp/wbid_torso_tilt_startup_20261024.log`.

## Optional Cost Activation (2026-10-23, needs build and runtime test)

WB-ID now follows `mpc_controllers`' configure-time cost-registration pattern with `ocs2.costs.taskSpaceFootCost.activate`, `ocs2.costs.taskSpaceCosts.<name>.activate` and `ocs2.costs.legTorqueCost.activate`.
All defaults and shipped G1 flags are `true`, and all existing weights are unchanged.
The WB-ID foot vector moved from `taskSpaceFootCostWeights` to `taskSpaceFootCost.weights`; update external WB-ID parameter files accordingly.
Disabled costs are not constructed or registered, avoiding their CppAD model creation/loading; Q/R and all contact/dynamics constraints remain active.
The centroidal schema and costs are unchanged.
The task-space objectives are not algebraically identical to Q/R, and the load cost regularizes selected entries of `J^T W`, not the torque decision variable.
Disabling a foot cost does not remove the hard stance orientation or vertical contact constraints, so this is an ablation switch, not a terrain-support fix.
Static activation and parameter-contract tests cover schema defaults, all eight switch combinations, config wiring and constructor guards; no ROS build or simulator test was run.

## WB-ID Startup Horizon Recovery (2026-10-23, needs build and runtime test)

The intermittent takeover failure showed control time around 15.7 s while the last plan ended at 9.5024 s.
Observations were only forwarded from `compute_mpc_joint_action()`, so startup could keep solving the initial snapshot and then accept an expired first policy.
OCS2's `MPC_BASE::run()` refuses a new start time beyond its previous final time until reset, making the failure persistent.

The fix follows the existing arm-controller OCS2 pattern: `setCurrentObservation()` on every valid control tick, `updatePolicy()` on the control thread, and `advanceMpc()` on the worker.
The deferred worker waits for a post-construction observation before its first solve.
Policy evaluation is gated on finite timestamps and interval coverage with two control periods of end margin.
An atomic warm-start reset request is consumed by the worker through `resetMpcNode()`, as in `dynamics_mpc_controller`.
No custom observation mailbox, new tuning parameter, OCS2 library change or weight/gain change remains in this patch.
Startup holds and retries; a blend interruption returns to startup, while policy loss after estimator-driven locomotion latches `Fault` and requires controller reactivation.
The ground-truth path holds the last commanded posture until a usable plan arrives.

The arm controllers normally compute their initial policy synchronously in activation, whereas this legged controller must keep standing and estimating while constructing/starting MPC asynchronously.
The centroidal controller has the same latent observation-delivery gap and is not inherently immune, but is unchanged in this WB-ID checkpoint.
Six static startup/thread-ownership regression checks and the WB-ID parameter contract pass; no ROS build or simulator validation was performed for this patch.
Test repeated launches at the usual rates and inspect the new construction/first-advance logs, rejected-policy timestamps and warm-start reset messages.

## Description Layout (2026-10-23)

The current layout is format-first: `description/meshes/robots/g1/`, `description/mujoco/robots/g1/` and `description/urdf/robots/g1/`.
Shared MuJoCo scene entry points and the generator remain at `description/mujoco/`, with reusable presets in `common/backgrounds/` and `common/materials/`, plus `terrains/`, `objects/` and `textures/`.
The RViz staircase xacro is `description/urdf/terrains/stairs.urdf.xacro`, not a MuJoCo resource.
Both staircase xacros use `config/terrains/stairs_params.yaml`, which is format-neutral and installed alongside the description tree.
Launch defaults, robot mesh URIs, scene includes and documentation links were updated; `mujocoModelFile:=scene.xml` and other scene basenames remain unchanged.
Compiled numerical model arrays and simulation options match the pre-move baselines for all three scenes and all optional terrain/object modules; only resource paths changed.
Eleven layout checks pass, including both staircase xacros expanded from a relocated copy of the installed-style description/config trees.
See [the layout guide](description/README.md), [the MuJoCo guide](description/mujoco/README.md) and `tests/test_mujoco_scene_layout.py` for regression coverage.

## Tuning Benchmark and Current Gains (2026-10-23, uncommitted)

- `docs/wb_id_tuning_benchmark.md` has the current state in its section 0, plus the instrumentation, the tools, how to run and read a case, every result, the measurement biases found, and the open items.
- **Instrumentation:** `diagnostics.trackingRate > 0` publishes `~/tracking` (plan / command / measurement per tick) and `~/solver` (per-solve metrics, constraint violation including over the first 3 nodes, and cost terms).
- **Tools and results:** the tools are in `tests/tuning/` (recording via `ros2 bag`, not Python subscribers); results go to `tuning_results/<tag>/` (git-ignored).
- **Shipped WB-ID values:**
  - `jointEffortPd` kp 200 / kd 20;
  - `q/qFinal.baseVelocity` vz 10 and pitch/roll rate 30;
  - arm `q.jointVelocities` 0.06;
  - `sqp.useFeedbackPolicy: false`;
  - `sqpIteration` stays 1.
- **`mujoco_ros2_control` (jazzy, uncommitted): `controller_time_from_sim` (default true).** The controller manager's clock is set from MuJoCo time every update, with its `/clock` remapped away. Before, controllers saw time in 10 ms steps, the default `mujoco_publish_rate`.
- **Net, same 0.5 m/s walk:** bounce 14.4 → 11.5 mm; tracking error legs 0.39 → 0.13°, arms 0.11 → 0.02°; leg / arm torque jitter 5.9 → 4.2 / 1.7 → 0.9 Nm; MPC 78 → 97 Hz.
- **Knee kd 10** has since halved the touchdown spikes. Robustness passed: lateral, turning, stop cycles, 0.25 and 0.75 m/s, with no falls.
- **Resume:** from `docs/wb_id_tuning_benchmark.md` → "Resume here". Round 7 (`walk` gait timing) was in progress at handoff.

## Servo Gains: Controller-Written Per Joint for WB-ID (2026-10-23, implemented, needs build)

This supersedes the launch-argument proposal that was here. Launch has no gain arguments any more; gains are set in YAML or Xacro only.

- **Hardware (`mujoco_ros2_control`, jazzy branch).** Each effort_pd joint that declares `<command_interface name="kp"/>` / `"kd"` exports its `pd_kp` / `pd_kd` as command interfaces. They start at the joint's `effort_pd_kp` / `effort_pd_kd` parameters. A non-finite or negative written gain counts as 0. This mirrors Unitree `LowCmd`, which carries kp/kd with every motor command.
- **Xacro.** `g1.ros2_control_macro.xacro` declares kp/kd on every effort_pd joint. The hardware defaults stay `effort_pd_kp/kd` args of `g1.ros2_control.xacro` (1200/10; wrists 100/1). They apply to centroidal, to the wrists, and to WB-ID before activation.
- **WB-ID.**
  - `control.jointEffortPd.kp/kd` are arrays index-aligned with `robot.jointNames`, currently 1200 / 20 uniform.
  - The controller claims `<joint>/kp`, `<joint>/kd` after the joint and wrist commands, so the `[3 * i + k]` layout is unchanged. It finds their indices by name at activation and writes them every tick in every bring-up phase.
  - On deactivation it leaves them in the hardware rather than zeroing them, which would leave the joints limp.
  - Empty arrays mean the controller doesn't claim them and the hardware gains stay in effect.
  - The contract test checks the lengths.
- **Centroidal** is unchanged: it doesn't claim kp/kd.
- **Not done yet:** per-phase gains (e.g. bring-up vs walking) and a real G1 hardware interface. That interface should map the same kp/kd command interfaces onto `LowCmd`.

## Tested Servo Gain Applied (2026-10-23)

WB-ID runs Kp 1200 / Kd 20 on the 23 MPC joints. This was first set through launch arguments and is now `control.jointEffortPd`, written by the controller (see above). Centroidal stays 1200/10 from the Xacro; all six fixed wrists remain 100/1. The new MJCF retains damping 0.05, its armature and friction; no Q/R or estimator settings were changed. This promotes the temporary Kd-20 test below, not the artificial passive-damping-10 ablation. Forward stability improved in the short test; lateral speed tracking remains unresolved. Restart the launch to apply hardware gains.

## New-Model Fall Investigation (2026-10-23)

Analysis-only session against HEAD `5fe1d79` plus the user's uncommitted rev_1_0 model/launch changes. No production code, model, or parameter settings were changed by this investigation; only this handoff was updated. No build was run.
Since then the rev_1_0 model has replaced `g1_29dof.xml` / `g1_29dof.urdf` in place, so every scene and the launch default use it (see "G1 model" below).

### Controlled Experiments

Ran sequentially in Parallels `ros_jazzy`, flat scene, MPC 100 Hz / MRT 1000 Hz, MuJoCo headless, RViz and base-command GUI off. All cases use the new URDF/MJCF and the new CppAD cache; numerical controller settings are the current checkout's settings. Estimator cases use the default `humanoid_linear_kf`, not InEKF.
The standard sequence settles for 10 s after the harness detects locomotion, ramps for 3 s, holds for 5 s, ramps out for 3 s and rests for 5 s. The lateral test holds for 15 s instead. Fall times are relative to harness start, not process launch; GT cases start at the first observed odometry, estimator cases after the locomotion log.

| Container artifact prefix under `/tmp/` | Change | Result |
| --- | --- | --- |
| `g1_audit_new_kf0795` | Stock new model, height 0.795, KF | Fell at 8.88 s, before walking command. |
| `g1_audit_repeat_kf0795` | Same, repeat with joint logging | Fell at 6.58 s, before walking command. |
| `g1_audit_new_gt0795` | GT feedback, height 0.795 | Fell at 13.95 s, during walking. |
| `g1_audit_new_gt` | GT feedback, height 0.6 | Fell at 16.54 s, during walking. |
| `g1_audit_d10_kf0795` | Only passive damping restored to 10; retain new mass, armature, friction and integrator | Passed 26 s forward sequence; 1.310 m / 2.0 m expected, mean walking vx 0.179 m/s for 0.25 commanded, peak roll/pitch 0.030/0.026 rad. |
| `g1_audit_kd20_kf0795` | Passive damping stays 0.05; only hardware servo Kd 10 -> 20 on the 23 MPC joints | Passed 26 s forward sequence; 1.636 m / 2.0 m expected, mean walking vx 0.218 m/s, peak roll/pitch 0.014/0.032 rad. |
| `g1_audit_kd20_vy_kf0795` | Same Kd 20, lateral 0.15 m/s | Upright for 36 s, peak roll/pitch 0.011/0.017 rad; mean vy 0.050 m/s, 0.909 m / 2.7 m expected. Verdict NO_TRACKING, not success. |

These are short diagnostic trials, not a robustness certification or hardware validation. No fresh old-model control run was performed; the clean ablation retained the entire new model and changed damping alone.
Actuator limits remained enabled. Kp stayed 1200; fixed wrist gains stayed 100/1. Runtime `/robot_state_publisher` robot_description confirmed Kd values 20 and 1 in the active-gain test.

Joint-state evidence, same KF setup, the 3-to-5-second window after locomotion starts, before any velocity command (RMS joint speed, rad/s):

| Joint | Stock new model / Kd 10 | New model / Kd 20 | New model / passive damping 10 |
| --- | ---: | ---: | ---: |
| Left shoulder yaw | 0.6047 | 0.0009 | 0.0002 |
| Waist pitch | 2.7612 | 0.0188 | 0.0010 |
| Left hip pitch | 2.6129 | 0.0442 | 0.0022 |
| Left knee | 1.4226 | 0.0350 | 0.0047 |

Stock waist-pitch and left-hip-pitch efforts were at >=99% of their actuator limits for 2.63% and 1.62% of samples in that window; both were zero with Kd 20. Large joint oscillations exist before gait switching. GT feedback also fails, so an estimator error alone does not explain the regression.

### Interpretation and Next Fix

The dominant demonstrated change is the loss of passive damping: old MJCF damping 10 -> new per-joint damping 0.05, while the hardware servo stayed Kp 1200 / Kd 10. Servo torque is `Kp*(q_des-q) + Kd*(dq_des-dq) + tau_ff`, evaluated against live MuJoCo state every physics step; the passive term additionally contributes `-b*dq`. In a local scalar approximation, velocity dissipation changes from Kd+b=20 to 10.05. This is not a full closed-loop stability analysis: targets move, actuator saturation and contact coupling matter, and armature also changed.
Increasing active Kd to 20 while retaining the low physical damping is the first tested controller-side candidate. Do not silently restore damping 10 and call the new model hardware-accurate. Expose per-joint servo gains in a model-specific hardware configuration, start from the tested gain, then check longer forward/lateral/yaw/reversal tests, saturation and transient tracking. Kd is not currently a WB-ID YAML cost/control gain; it lives in the ros2_control xacro defaults.
Only after the inner-loop oscillation is removed should Q/R and gait tuning address the poor lateral speed. Both successful-upright variants settle near GT height 0.764 for target 0.795, leaving about 31 mm height error. Kd 20 also still produces saturation during some early transition samples; do not claim saturation has disappeared globally.

Other model-consistency findings:

- Pinocchio URDF mass = 33.34114202 kg; MJCF declared mass sum = 33.341142 kg. Old equivalents were 35.11514202 and 35.112142 kg. At identical crouched q, ankle frame positions and rotations in old/new URDFs are identical. New mass/leg kinematics do not show an obvious mismatch in these checks; a full inertial-tensor comparison was not performed.
- MuJoCo has reflected rotor inertias, damping and friction; loaded Pinocchio armature is zero and the WB-ID RNEA constraint adds no explicit actuator-dynamics correction. Model rotor inertia and losses consistently in MPC, fallback inverse dynamics and torque feasibility after validating their physical values. Do not treat the handoff's earlier "small torque mismatch" as measured evidence.
- Hip-roll torque limits were raised to 139 in URDF/MJCF/controller YAML, but the ros2_control macro still declares 88 on both hips. Align them when implementing the fix. This inconsistency was not isolated as a cause of the fall.
- Mid360's new upside-down mounting is real in the URDF, but the torso cost builds its orientation reference with FK of the same named frame at x_ref; it does not directly request identity orientation for the lidar. Therefore a simple "MPC tries to flip the lidar upright" explanation is unsupported. Preserve the real transform; optionally choose torso_link as the torso task frame in a separately validated change.
- Keep URDF, MJCF, estimator model and CppAD cache paired. `scene_ramp.xml` still references the old MJCF. Changing URDF alone while reusing an old generated cache remains unsafe.

Artifacts remain in the container: each prefix has `.json`, `.csv`, `.log`, `.yaml`; wrapper runs also have `_joints.csv`, `_model.xml`, `_scene.xml`. `/tmp/g1_model_audit.py` wraps the repository harness to use temporary model files; `/tmp/g1_audit_bin/xacro` applies gain overrides only when prepended to that test's PATH. `/tmp/g1_static_audit.py` and `/tmp/g1_joint_audit.py` provide model and joint statistics. Nothing in the normal launch uses these wrappers. All launched test processes were stopped.

## Latest Checkpoint (2026-10-23, uncommitted): gait parameters + closest-to-hardware G1 model

**User builds and tests; no agent build/sim was run after the model files were added.**

### Gait settings are ROS parameters (built and smoke-tested)

- `config/g1/gait.yaml` and `gait_wb_id.yaml` are removed, along with the `gaitLibraryFile` parameter and launch arg. Each controller block of `ros2_controllers.yaml` now holds everything under `ocs2.gait`:
  - `templates.names` plus one `{modeSequence, switchingTimes}` per name;
  - `transitions` (`minDwellTime`, `switchDelayHorizonFraction`, `ladder`, plus one `{linearVelocity, yawRate, measuredMargin}` per rung);
  - `swingTrajectory` (moved from `ocs2.swingTrajectory`).
- Values are identical to the old files, checked by script. Builders: `buildGaitMap` / `buildGaitTransitionConfig` in `config_builder_utils.hpp`.
- Smoke runs, flat, the same before and after: WB at 0.5 m/s survived, peak tilt 0.037, 87 Hz; centroidal at 0.25 m/s survived. The WB contract test checks templates and ladder.
- The ladder overlaps are deliberate hysteresis; keep `next.min < prev.max`. Up needs command > max AND measured > max - margin. Down needs command < min AND measured < min + margin, on all axes.
- Known quirks, unchanged:
  - measured speed is world-axis and unfiltered;
  - the centroidal "measured yaw" reads `h_ang,x/m`;
  - the centroidal `skip` template has an out-of-order 0.08.

### G1 model: `g1_29dof.xml` / `g1_29dof.urdf` are now rev_1_0 (replaced in place)

- **Single model for everything:** `description/mujoco/robots/g1/g1_29dof.xml` (all scenes include it) and `description/urdf/robots/g1/g1_29dof.urdf` (the launch default `urdfFile`, read by the MPC, both estimators and robot_state_publisher). The temporary `*_correct` files are gone. `g1_29dof_real.xml` is the superseded old-mass variant and nothing includes it.
- **Previously Unitree's deprecated `g1_29dof`** (`mode_machine` 2, 35.11 kg, hip roll 88 Nm). Now `g1_29dof_rev_1_0` (`mode_machine` 5, the current standard 29-DoF). Changes:
  - torso 6.78 vs 8.56 kg, 33.34 kg total, no waist_support_link;
  - waist joint offsets and shoulder/sensor offsets changed;
  - hip roll 139 Nm / 20 rad/s;
  - Mid-360 mounted upside down (`rpy pi 0.0511 0`).
  - Check the robot's `mode_machine` in the Unitree app (Device -> Data -> Robot -> Machine Type). Newer units are `g1_29dof_mode_11..18`.
- **MJCF joint dynamics:**
  - per-motor armature from unitree_rl_lab (hip pitch/yaw + waist yaw 0.010178, hip roll + knee 0.025102, ankles + waist roll/pitch 0.00722, shoulders/elbow/wrist roll 0.00361, wrist pitch/yaw 0.00425);
  - damping 0.05 and frictionloss 0.2 (0.1 on wrist pitch/yaw) from unitree_mujoco, replacing the old damping 10;
  - `implicitfast`.
  - Kept local: contact solref/solimp, free-joint damping 0, `imu-pelvis-quat` sensor, `mid360` site at the rev_1_0 pose.
- **URDF:** rev_1_0 plus local package:// meshes, ankle/waist roll+pitch limits 50 Nm / 37 rad/s (rev_1_0 MJCF and unitree_rl_lab; Unitree's URDF says 35/30), and the four `*_est` foot points. Pinocchio loads it: 33.341 kg, frames present.
- **Launch:** `g1.ros2_control.xacro` takes `robot_urdf_file`, and `g1.launch.py` passes `urdfFile` to it, so robot_state_publisher matches the MPC.
- **CppAD cache:** the default `libFolder` is again `auto_generated/g1`.
  - The libraries generated from the old 35.1 kg URDF were renamed `wb_id_mpc_g1_old_35kg` / `centroidal_mpc_g1_old_35kg`, not deleted. The rev_1_0 WB libraries moved in as `wb_id_mpc_g1`.
  - With `recompileLibrariesCppAd: false` OCS2 never checks that cached libraries match the URDF, so any future model change needs the same swap or a fresh folder.
  - The centroidal rev_1_0 libraries don't exist yet; the first centroidal launch regenerates them, which takes long.
- Hip-roll torque limits are 139 in both controller blocks: WB-ID uses `ocs2.constraints.jointTorqueLimits.limits` and centroidal uses `control.torqueLimit`; the ros2_control command limit in the xacro is 139 too.
- **Expected risks:**
  - Losing damping 10 removes a lot of free passive stability, so the controller may need retuning.
  - Armature should fix the ankle-chatter instability (kd*dt/I about 12 -> about 0.7), though that's unmeasured.
  - The MPC model has no armature, so expect a small torque mismatch.
- **References (raw GitHub):**
  - unitree_ros `robots/g1_description/{g1_29dof_rev_1_0.xml, g1_29dof_rev_1_0.urdf, g1_29dof.urdf, README.md}`;
  - unitree_mujoco `unitree_robots/g1/g1_29dof.xml`;
  - unitree_rl_lab `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`;
  - unitree_rl_gym `resources/robots/g1_description/g1_29dof.xml`;
  - mujoco_menagerie `unitree_g1/g1.xml`.

## Earlier Checkpoint: User Will Build and Test

**2026-10-22: stop automated builds/simulations here. The standalone parameter cleanup below has NOT been compiled or walking-tested.**
The earlier rate-limiter build completed, but subsequent schema/controller/builder changes require a fresh build.
The user requested committing and pushing this unbuilt checkpoint for their own build/test.

- WB-ID now has one generated parameter schema and one listener; no centroidal Params type, builder call, or nested `wholeBody.*` parameter tree.
- A single `ocs2.costs.q/qFinal` contains `basePose`, `baseVelocity`, `jointPositions`, `jointVelocities`.
- A single `ocs2.costs.r` contains `contactWrenches`, `baseAccelerations`, `jointAccelerations`, `jointTorques`.
- Values were moved without retuning Q/R. Runtime scales remain qScaling=1, rScaling=0.001, terminalCostScaling=4.
- Torque-window setting is `ocs2.constraints.inverseDynamics.torqueConstraintNodes`; barrier settings are `ocs2.constraints.jointTorqueLimits` and `jointVelocityLimits`; feedforward is `control.feedforward`.
- Removed WB-only unused centroidal momentum/model settings, inertia-weighted momentum toggle, centroidal input joint-velocity weights, unused controller PD arrays, unused gradient switches/ICP settings, diagnostics CSV parameters and future frame-relation parameters. The separate centroidal controller was not removed or retuned.
- WB capture-foot placement is disabled in its builder and removed from its schema/YAML. The common terrain planner remains a separate feature. Earlier passing runs below used capture placement and do NOT validate this checkpoint; a previous capture-disabled experiment fell.
- Controller-side rate limits are `ocs2.reference.velocityCommandRateLimits=[0.1,0.08,0.05,0.1]` (moved from `target.*` on 2026-10-23) for `[ax,ay,height_rate,yaw_acceleration]` in physical units. Centroidal defaults disable this new limiter. Solver time governs increments, capped at 50 ms per update; reset starts limited twist axes at zero.
- WB GUI ranges are now vx +/-0.25 m/s, vy +/-0.15 m/s, yaw +/-0.15 rad/s. Launch reads GUI scales from the selected YAML, no longer hardcodes 2.4/1.2/1.0. A full normalized topic command now means these smaller limits.
- `tests/wb_id_parameter_contract_test.py` passes locally: duplicate keys, declared runtime keys, unified Q/Qf/R layout, joint-array lengths, and direct builder schema accesses. This does not substitute for compilation or closed-loop testing.

### New Reproduction Evidence Before This Cleanup

Both tests used the old capture-enabled controller with the default linear KF, flat scene, 100 Hz MPC / 1000 Hz MRT, height 0.6 m, and all GUIs disabled.
`/tmp/wbid_step_baseline_vx`: a 0.4 m/s forward command reached in 0.02 s caused a fall at 6.82 s, about 1.8 s after the command, after five seconds of stable MPC stance.
`/tmp/wbid_step_baseline_yaw`: a 0.15 rad/s yaw step survived 30 s including an abrupt stop; achieved 2.04 rad versus 3.0 commanded, average yaw rate 0.098 rad/s, max translation drift 0.328 m.
Thus abrupt commands and speed/gait selection expose failures hidden by the earlier gentle-ramp tests; neither lowering GUI limits nor removing capture placement has yet been demonstrated to fix this.

### Paper Audit

The local Molnar 2025 PDF, section IV-A/B, and `wb-mpc-locoman/optimization/ocp_whole_body_rnea.py` use state `[q,v]`, quadratic state/input tracking, RNEA equalities, contact/swing constraints, and joint limits; horizontal swing placement is optimized rather than capture-point prescribed.
The local Khazoom 2024 PDF, section II-B, uses pose/velocity/contact-force tracking plus first-step torque-change regularization, contact height/no-slip constraints, joint limits and collision constraints/CBFs.
Current WB-ID has the common whole-body state/input structure but is not numerically identical to either paper: Translation + SphericalZYX Euler-rate state versus spatial twist; acceleration-level stabilized contact constraints; uniform OCS2 SQP grid and soft inequality barriers; 6D flat-foot wrenches versus point contacts; extra foot/torso and external-leg-load costs; no first-step torque-change cost or first-node collision CBF.
The retained extra costs are active G1 adaptations, not dead centroidal parameters. Joint-torque zero regularization is not the MIT torque-change regularizer.
Do not claim an SE(3) state-model conversion or paper-equivalent hard constraints based on this parameter cleanup.

Everything below is historical experiment context, not validation of the latest checkpoint.

## Scope and Status (2026-10-22)

Work is on `feat/wb_id_mpc`, starting from `1fbd2e2`.
Only whole-body inverse-dynamics MPC is being tuned; do not copy these weights into centroidal MPC.
The user requests flat-ground, separate forward and lateral walking tests in Parallels Docker `ros_jazzy`, with MuJoCo, RViz and the command GUI all headless/disabled.
Experiments below are simulation results, not hardware or terrain validation.
At that earlier stage, implementation fixes were built in `ros_jazzy`; the latest checkpoint above supersedes this build status.
No robot/test processes are intentionally left running.
GT-driven forward/lateral tests survived 41 s; with the nominal-posture correction the default linear-KF forward test also survived 41 s, but failed the strict heading check (0.552 rad maximum yaw drift).
This is improved survival, not a claim that tracking, all estimator paths, or stairs are solved.

## Reproduction

SSH to `parallels@10.211.55.17`, enter container `ros_jazzy`, source `/opt/ros/jazzy/setup.bash` and `/root/ocs2_ros2_ws/install/setup.bash`, and work in `/root/ocs2_ros2_ws`.
Sources are shared with the Mac checkout.

```bash
python3 src/legged_robot_mpc_controller/tests/wb_id_walk_test.py \
  --axis vx --speed 0.25 --mpc-rate 100 --prefix /tmp/wbid_vx
```

For lateral tests use `--axis vy --speed 0.15`.
The runner selects `scene_flat.xml`, `rviz:=false`, `mujoco_headless:=true`, `baseCommandGui:=false`, and `mrtFreq:=1000`.
It uses ground truth by default to separate controller faults from estimator faults; use `--source state_estimator --estimator humanoid_inekf` or `humanoid_linear_kf` for closed-loop estimation tests.
Each run saves its exact YAML, launch log, GT odometry CSV and JSON verdict at the prefix inside the container.
Commands are specified in physical m/s and normalized using the tested YAML limits.
Phases use simulation time: settle 5 s, ramp in 3 s, hold 25 s, ramp out 3 s, rest 5 s (41 s total).
Fall detection is height below 0.38 m or absolute roll/pitch above 0.8 rad for 0.15 s.
An upright run is not automatically a pass: projected travel must exceed half of the command integral.
The updated runner also requires heading error below 0.5 rad and cross-axis travel below 35% of the intended travel; the early experiments below predate those extra checks.

## Results Before Implementation Fixes

All tests below used GT feedback, 100 Hz requested MPC, 1000 Hz control, and 0.6 m requested pelvis height.
Artifacts are `/tmp/wbid_<name>.{yaml,log,csv,json}` in `ros_jazzy`.

| Name | Change | Result |
| --- | --- | --- |
| `baseline_vx` | Original YAML, vx = 0.25 m/s | Fell at 14.80 s; peak pitch 1.128 rad. |
| `baseline_vy` | Original YAML, vy = 0.15 m/s | Upright 41 s, but only 0.141 m lateral travel and -1.737 m unwanted x travel; not tracking. |
| `sqp2_vx` | Two SQP iterations | Fell at 9.70 s; actual solver rate about 51 Hz instead of roughly 90-98 Hz. |
| `weights_vx` | Weight set A | Upright 41 s; 3.10 / 7.0 m forward travel, insufficient speed tracking. |
| `weights_vy` | Weight set A | Upright 41 s; 1.59 / 4.2 m lateral travel and -1.78 rad final yaw; not acceptable tracking. |
| `freefoot_vx` | Weight set B, capture-foot-placement cost disabled | Fell at 10.52 s; retain this cost for now. |
| `weights2_vx` | Weight set B, capture cost retained | Upright 41 s; 5.22 / 7.0 m forward travel, peak roll 0.069 rad, pitch 0.056 rad. Final y 0.40 m, yaw 0.152 rad. |

Weight set A changes both running and terminal base-pose weights to `[0, 0, 1000, 0, 200, 200]`, and both base-velocity weights to `[100, 100, 30, 10, 10, 10]`.
Weight set B keeps those pose weights and uses base-velocity weights `[300, 300, 30, 100, 10, 10]`.
Base-pose order is `[x, y, z, yaw, pitch, roll]`; base-velocity coordinates are `[vx, vy, vz, yaw_dot, pitch_dot, roll_dot]`, not body angular velocity components.
At this stage these weights were temporary `--set` overrides, not changes saved to `config/g1/ros2_controllers.yaml`.
Set B still has forward speed error (about 0.19 m/s achieved for 0.25 m/s) and walking height error (about 0.65 m for 0.6 m requested), despite good upright stability.

## Implementation Findings

1. Torque-window interpolation accepted invalid knots.
   The torque equality is active for the first three knots at 0, 0.02, 0.04 s; duration is 0.05 s.
   Evaluating at 0.045 s previously passed the time-window test but blended the valid 0.04 s torque with the unconstrained 0.06 s torque.
   `WbIdTorqueWindow::containsInterpolatedInput()` now checks every contributing knot using OCS2's interpolation indexing and the active policy's own start time.
   Beyond that window the existing RNEA feedforward fallback is used.
   The standalone `tests/wb_id_torque_window_test.cpp` passed, including event-duplicate times, old policy/new solver horizons, singleton trajectories and expired plans.
2. Active policy ownership crossed threads.
   The solver worker called `MRT_BASE::updatePolicy()` while the control loop called `evaluatePolicy()` and `getPolicy()` without a lock.
   OCS2 protects its buffer, not reads of the active trajectory; these operations must belong to the same consumer thread.
   Move the policy swap into `update_and_write_commands()` and latch successful first activation there.
   A missing/newly locked buffer is normal, not a solver failure.
   This is a correctness fix; its quantitative contribution to falling has not been isolated experimentally.

The first torque-window rebuild passed (warnings only).
The policy-ownership rebuild also passed (warnings only).

### Tests With Both Implementation Fixes

| Artifact suffix | Weights | Result |
| --- | --- | --- |
| `fixed_weights2_vy` | B | Fell at 10.28 s during the first lateral steps. Forward-only tuning cannot be promoted. |
| `weights3_vy` | Pose `[0,0,1000,0,1000,1000]`, velocity `[300,100,30,300,30,30]`, running and terminal | Upright 41 s, but 1.29 / 4.2 m travel and -0.76 rad yaw. No tracking pass. |
| `weights4_vy` | Same pose; velocity `[300,300,30,1000,30,30]` | Fell at 10.40 s. |
| `weights5_vy` | Set 3 with footCenterOffset = 0 | Upright, but slow and 0.57 rad yaw drift. |
| `weights6_vy` | Set 3 with vy weight 150, yaw-rate weight 1000, footCenterOffset = 0 | Fell at 10.42 s. |
| `shortgait_vy` | Set 3, slow_walk single support 0.40 s / double support 0.15 s | Upright 41 s; 2.17 m lateral travel, peak yaw error 0.054 rad, peak roll 0.036 rad. Mean vy 0.081 m/s for 0.15 commanded. |
| `shortgait_w7_vy` | Set 3 with vy weight 150, shorter gait | Upright 41 s; 2.72 m lateral travel, peak yaw error 0.184 rad. |
| `shortgait_w7_vx` | Same set 7 | Fell at 19.91 s after yaw divergence. |
| `shortgait_w8_vx` | Set 7, both contact-moment mz input weights 100 instead of 0.1 (before rScaling = 0.001) | Upright 41 s, 5.26 m forward, peak roll/pitch 0.036/0.040 rad, peak yaw error 0.422 rad. |
| `shortgait_w8_vy` | Same set 8 | Upright 41 s, 2.46 m lateral, peak roll/pitch 0.046/0.016 rad, peak yaw error 0.081 rad. |
| `saved_gt50_vx` | Set 8 saved in normal YAML, default WB-ID gait, 50 Hz | Upright 41 s, 4.93 m forward, but peak yaw error 0.770 rad fails the heading check. |
| `yaw1000_gt50_vx` | Set 8 with yaw-rate weight 1000 instead of 300 | Upright 41 s, 5.21 m forward, peak yaw error 0.546 rad still fails heading check. Not promoted. |
| `saved_kf100_vx` | Set 8 with original near-straight nominal joint posture; default linear KF | Fell 1.24 s after entering locomotion, before the walking command. Bring-up stood at 0.78 m while the requested MPC height was 0.60 m. |
| `crouch_kf100_vx` | Set 8 with nominal hip/ankle pitch -0.8 rad, knee 1.6 rad on both legs | Upright 41 s; 5.07 m forward, peak roll/pitch 0.035/0.038 rad, minimum height 0.599 m. Mean vx 0.197 m/s for 0.25 commanded. Peak yaw error 0.552 rad fails heading check. |

Higher tilt weights stabilized this lateral case; yaw-rate and speed tracking still need improvement without restoring absolute xy/yaw pose penalties.
Shortening single support improved lateral walking more than simply escalating the motion weights.
The new `gait_wb_id.yaml` changes only `slow_walk`; the launch selects it for WB-ID, leaving centroidal on `gait.yaml` and respecting an explicit `gaitLibraryFile` override.
(Since 2026-10-23 both libraries are the `ocs2.gait.templates` parameters of their controller blocks; the gait files and `gaitLibraryFile` are removed.)
It also corrects the inherited `skip` template's non-monotonic 0.08 s entry to 0.8 s; that gait is not tested.
Set 8 is now saved in `ros2_controllers.yaml`, only in the WB-ID block.
Increasing contact yaw-moment regularization helped forward survival; this is still a soft cost and does not supply the missing torsional wrench-feasibility constraint.
The crouched nominal joint posture is also saved in the WB-ID block; it is used by both reference generation and bring-up and matches the default 0.60 m target without changing estimator parameters or using GT feedback.
The last parameter-only change needs no C++ rebuild with the current shared symlink install.
The final combined configuration has not yet been rerun on GT lateral or on estimator-driven lateral; the preceding GT lateral pass used the same weights/gait with the old nominal joint posture.

## Model and Tuning Interpretation

The state is `[q, v]`, including floating-base position/Euler-ZYX coordinates and joint positions; input is two six-axis contact wrenches, generalized acceleration, and joint torque.
Base RNEA rows are enforced throughout the horizon; joint torque rows only within the initial torque window.
Stance constraints regulate contact acceleration with position/velocity feedback; this is not exactly the papers' discrete no-slip velocity constraint.
The default base-height weight 10 and velocity weights 3 were weak relative to competing posture, torque and foot costs.
Increasing height/tilt and commanded-velocity weights substantially improved tracking/stability in the tests above without removing local xy/yaw invariance.
Increasing SQP iterations alone was counterproductive due to additional delay.
Nonlinear equality residuals spike during contact transitions even in upright runs; log them alongside motion and solve time, not as a stand-alone success criterion.

The low-level `effort_pd` path uses hardware joint gains (typically Kp 1200, Kd 10), not the generic `control.mpcJointKp` field.
The MuJoCo model has joint damping 10; no corresponding passive damping term was identified in WB RNEA, but its effect has not been isolated.
Do not change shared damping/contact physics to hide a WB-only issue or inadvertently regress centroidal MPC.
The capture-foot-placement implementation approximates CoM velocity using base linear velocity for WB; disabling it outright failed in the tested configuration.
The launch default estimator is `humanoid_linear_kf`; explicitly selecting InEKF tests a different configuration.

## Next Verification

1. User is taking over testing; do not start another simulation concurrently.
2. Recheck final saved configuration on both axes, GT and linear KF, at 50/100 Hz; InEKF has not been tested in this WB-ID tuning session.
3. Address yaw drift and speed error without an absolute xy/yaw reference or hiding the error by loosening test thresholds.
4. Consider a proper torsional contact-wrench bound; the current mz penalty is not a feasibility guarantee.
5. Only slow_walk was retuned; higher-speed gait transitions, terrain, other pelvis heights, and hardware remain unverified.
6. No commits or pushes requested for this task.

## Current User Test Command

After rebuilding the latest checkpoint, run in the sourced Jazzy workspace.
The GUI is enabled here for manual testing; keep it disabled for automated commands.

```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller \
  floatingBaseSource:=state_estimator floatingBaseEstimator:=humanoid_linear_kf \
  mujocoModelFile:=scene_flat.xml refBaseHeight:=0.6 \
  mpcFreq:=100 mrtFreq:=1000 \
  rviz:=true mujoco_headless:=true baseCommandGui:=true
```

Start with vx = 0.25 m/s or vy = 0.15 m/s separately, with gentle ramps and zero yaw command.
Under the new WB-ID limits, normalized vx=1 means 0.25 m/s and normalized vy=1 means 0.15 m/s; GUI readouts use those same scales.
The fixed torque-window unit test was compiled and run manually without `NDEBUG`; it is not yet registered with CTest.
Python syntax and `git diff --check` passed.
