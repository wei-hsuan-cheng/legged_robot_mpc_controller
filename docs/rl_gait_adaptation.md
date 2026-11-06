# RL Gait Adaptation for the WB-ID MPC

An RL policy adapts the WB-ID MPC's gait parameters and its dynamics model (the paper's residual
dynamics) online, following Kamohara et al. 2025, "RL-augmented MPC". Training runs on the
lockstep multi-environment substrate (`docs/multi_env_handoff.md`). Deployment runs the
policy inside the controller, in C++, next to the ordinary asynchronous MPC.

## Design

**Actions, normalized to [-1, 1], one decision per 2 MPC solves (50 Hz):**

| action | effect | range | paper analogue |
|---|---|---|---|
| `period_scale` | step durations x (1 + 0.3 a) | +-30 % | sampling-time coefficient s |
| `swing_height` | swing apex + 0.05 a [m] | +-0.05 m | apex residual dh |
| `apex_shift` | apex at (0.5 + 0.2 a) of the swing | +-0.2 | control-point shift dcp, z part |
| `foothold_x`, `foothold_y` | swing foot's landing target + 0.15 a [m], forward / lateral in the heading frame | +-0.15 m | control-point shift dcp, xy part (where the foot lands) |
| `dynamics_residual.linear_acceleration_{x,y,z}` | base linear acceleration offset (0.5, 0.5, 1.0) a [m/s^2], heading frame | 1/4 of the paper's | residual f_x^(1) |
| `dynamics_residual.angular_acceleration_{x,y,z}` | base angular acceleration offset 0.25 a [rad/s^2] | 1/4 of the paper's | residual f_x^(2) |
| `dynamics_residual.force_gain_{x,y,z}` | total contact force's effect x (1 + 0.05 a) per heading axis | +-5 % | residual actuation diag f_u^(1) (paper 0.2 / m) |
| `dynamics_residual.moment_gain_{x,y,z}` | total contact moment's effect x (1 + 0.05 a) | +-5 % | residual actuation diag f_u^(2) (paper 0.2 / I) |

**Residual dynamics** (added 2026-11-06, as in the paper). The paper adds affine offsets to its
single-rigid-body accelerations, `a += f_x1 + diag(f_u1) sum F` and `alpha += f_x2 + diag(f_u2) sum M`.
The WB-ID MPC has no single-rigid-body model, so the same offsets enter its floating-base dynamics
(`WbIdRneaConstraintCppAd`, base rows) as an external wrench on the base, in the heading frame:

    F = m a_res + g_F o sum_c f_c,     M = diag(I) alpha_res + g_M o sum_c ((p_c - p_base) x f_c + m_c)

with m and diag(I) the total mass and the composite inertia at the neutral pose, mapped through
the root body's Jacobian. Moments are about the base origin rather than the CoM. The 16 values
are parameters of the generated library (`wb_id_rnea_base_residual_v1`), held over the horizon and
updated every decision; zero is the nominal model. The joint rows are unchanged, so the torques
stay those of the nominal robot under the planned contact wrenches.

The ranges are a quarter of the paper's (`config/h2/ros2_controllers.yaml`, `gaitAdaptation`).
Accelerations are scale-free, but on this MPC the paper's are far too strong. Flat ground, nominal
dynamics, training commands, 8 min per check:

| residual | episodes | fell | equality SSE median | forward velocity error |
|---|---|---|---|---|
| none | 28 | 0 | 54 | +0.04 m/s |
| N(0, 0.5) actions, paper's range | 44 | 32 | 27660 | +0.11 |
| constant +2 m/s^2 forward (paper's max) | 210 | 210 | 15550 | -1.14 |
| constant +1 m/s^2 forward | 44 | 18 | 121 | -0.38 |
| constant +0.5 m/s^2 forward (new max) | 31 | 7 | 71 | -0.15 |
| N(0, 0.5) actions, new range | 29 | 1 | 353 | +0.07 |

The sign is as expected: the MPC brakes against a forward push that is not there, and the robot
goes backwards.

**When an action takes effect.**
- The gait actions apply to every gait phase that has not started yet, by retiming them in
  place (`GaitSchedule::setAdaptation`). The phase in progress is never changed, and the
  contact sequence never changes.
- They are read **once per phase**, at the first solve after it starts; later changes in the
  same phase wait for the next one. Retiming the horizon at every solve moved the MPC's contact
  switches under its warm start, and one SQP iteration could not follow (see run 4 below).
- The foothold actions move the xy target of the `captureFootPlacement` foot cost (weight 50)
  for every swinging foot, **the swing in progress included**
  (`SwitchedModelReferenceManager::setFootholdOffset`). They are the only actions that act
  within the step a push lands in.
- Zero actions reproduce the hand-tuned MPC exactly.

**Decision rate.** The env holds each action for `EnvConfig.action_repeat` = 2 MPC ticks and sums
the rewards over them, so gamma = 0.99 keeps the 2 s horizon. Deployment matches it with
`gaitAdaptation.policyDecisionTicks: 2`. Policies from before the foothold actions (3 actions,
71 observations, one decision per tick) still load: the controller feeds them their old
observation and holds the footholds at zero (`rl/evaluate.py --decision-ticks 1`).

**Observation (71).** The controller computes it once and exports it as `rl.obs.*`, and the
same function feeds the deployed C++ policy, so training and deployment cannot drift apart.
It is built from the MPC's own observation, i.e. on-board signals only:
- projected gravity, base linear velocity in the base frame, base angular velocity;
- command (vx, vy, yaw rate, height offset) and base height offset;
- joint positions (minus the stand) and joint velocities;
- previous action;
- swing phase per foot and planned contact per foot.

**Rewards: robustness first, then the paper's Table III terms.** On-board signals only: the
estimate comes from the IMU and leg kinematics, foot velocities are forward kinematics of that
estimate, and contacts are the MPC's plan. The paper's reward is mostly tracking; its robustness
comes from the training conditions and from falls ending the episode. Here survival and posture
are rewarded explicitly as well:
- survival bonus (1.0 per step) and a termination penalty (-50);
- uprightness: `|projected gravity_xy|^2` from the IMU / estimate;
- roll/pitch rate and vertical velocity penalties;
- linear and yaw velocity tracking, and height tracking (above the stance feet, so relative to the terrain);
- joint velocity, action smoothness, and feet slide gated by the planned contacts.

The discount is gamma = 0.995 (a 2 s horizon at 100 Hz), so a recovery that pays off within a
couple of seconds is valued. The paper's knee-collision, leg-base-angle and step-width terms are
left out because they are specific to its HECTOR robot. The critic is symmetric: it sees the
same observation as the actor.

**Ground truth** is used only to end an episode (pelvis height above the terrain, tilt) and to
move the terrain curriculum (distance walked). It is never a reward or a network input.

**Training conditions (blind locomotion, as in the paper).** All randomization is in MuJoCo's
model only. The MPC, its Pinocchio model and the estimator keep the nominal URDF, so the policy
learns to walk under model error.
- **Terrain** (`rl/terrain.py`): one heightfield of 8 m tiles, legged_gym-style.
  - 8 difficulty rows x 9 columns: flat, rough x2, slope up/down (1-10 deg), stairs up/down
    (2-10 cm steps, 0.35 m deep), discrete steps (+-2-8 cm), stepping stones (1.0 -> 0.5 m stones,
    5 -> 20 cm gaps over 0.3 m pits).
  - Each episode spawns at a tile-centre platform. The env's level goes up after walking at least
    half the commanded distance without falling, and down after a fall.
- **Slippery ground:** sliding friction x U(f_low, 1.0) per episode, f_low from 0.6 down to 0.3
  with the disturbance level below.
- **Dynamics the MPC does not model:** every link's mass and inertia x U(0.9, 1.1); a torso
  payload of U(-2, 6) kg with a centre-of-mass shift of +-5 cm per axis.
- **Pushes during the episode:** a horizontal force on the torso for 0.1 s, in a random direction,
  every U(4, 8) s, U(75, F_max) N with F_max from 300 up to 600 N with the disturbance level.
  600 N for 0.1 s is 60 N s, about 0.8 m/s on the 75.6 kg robot.
- **Initial state:** random tile and yaw; joint position and velocity noise (+-0.03 rad,
  +-0.3 rad/s); base twist (+-0.1 m/s, +-0.2 rad/s).
- **Disturbance curriculum:** one level d in [0, 1], shared by all envs, sets f_low and F_max.
  Every 50 learning episodes it rises 0.1 if fewer than 30 % fell and drops 0.1 if more than
  50 % did. The terrain level stays per env.
- **Control group** (`train.py --baseline-envs K`): the last K envs always run the MPC alone. They
  take their terrain level from a random learning env at each reset, so both groups see the same
  distribution, and they do not move d.

**Episodes.**
- **First start:** each simulation goes through the normal bring-up once, on the flat border of
  the terrain map.
- **Snapshot:** the simulator then saves a settled standing state (sim `episode/snapshot`).
- **Reset:** every episode starts from that state, moved onto its tile and randomized as above
  (sim `episode/reset`). The MPC takes over directly: estimator re-initialized, gait reset to
  stance, warm start reset (MPC `episode.reset`).
- **Commands:** vx, vy, yaw rate and height are sampled per episode and resampled halfway.
- **End:** a fall (ground-truth tilt, or pelvis height above the terrain), an MPC fault, or the
  10 s timeout.

**Deployment validation** (`rl/evaluate.py`): one simulation through the ordinary launch
(asynchronous MPC, real time, topic commands), MPC alone vs MPC with the C++ policy. Scenarios:
- `flat`: the mixed command sequence at friction 1.0 and 0.5;
- `push`: the same sequence with random pushes (150-600 N for 0.1 s, random times and
  directions; one seed per repeat, shared by both modes), scored per push;
- `slippery`: friction 0.3;
- `rough`: the uneven-box terrain;
- `terrain:<type>:<difficulty>`: walking out from the centre of one generated tile.

## Progress

- [x] core: `GaitSchedule` adaptation and reset, swing planner apex shift and height offset, reference manager (`setGaitAdaptation`, `requestGaitReset`), motion manager reset and command getter.
- [x] WB-ID controller:
  - `gaitAdaptation.{source, policyFile, *Range}`;
  - references `gait_adaptation.*` and `episode.reset`;
  - exports `rl.obs.00-70`, `rl.reward.*` and `status.episode`;
  - the `Restart` phase;
  - the C++ `GaitAdaptationPolicy`.
- [x] mujoco_system: `episode` GPIO (snapshot, reset, base twist, friction scale; later spawn pose, noise, dynamics randomization and pushes). `mujoco_compile_model` gets an optional friction scale.
- [x] Training config and launch:
  - `config/h2/rl_gait_training.yaml`;
  - launch arg `episodeControl`, which sets xacro `use_episode_control` on H2;
  - `LockstepVecEnv.step_async` / `step_wait`.
- [x] Python:
  - `rl/h2_gait_env.py`, `rl/sac.py`, `rl/train.py`;
  - `rl/evaluate.py` for deployment validation;
  - PyTorch 2.15 CPU in the container venv `/root/claude_rlvenv` (system site packages, so ROS stays visible).
- [x] Smoke tests:
  - **Baseline:** 2 envs, episodes and resets work (snapshot, restart, locomotion), 0 sync violations, 74 transitions/s.
  - **SAC:** trains, checkpoints and exports.
  - **Deployment:** a single real-time env loads `policy.txt` in C++ and applies the adaptation at 95-97 Hz MPC.
- [x] Flat-ground policy `flat_v1`: SAC, 1.02M transitions on 8 envs, flat ground, reset twist
  only, the paper's rewards. Kept as a baseline in `/root/claude_rl_policies/flat_v1/`, with its
  checkpoint, curve and evaluations.
  - Training return 1345 +- 28 vs 1333 +- 15 for the MPC alone (the same episodes): +2 % velocity
    tracking, nothing else changed.
  - Deployment, MPC only vs MPC + flat_v1, 2 runs each:

    | case | MPC only | MPC + flat_v1 |
    |---|---|---|
    | flat, friction 1.0 / 0.5 | 4/4, vel RMSE 0.214-0.220 | 4/4, 0.217-0.233; height RMSE 10 % lower |
    | slippery, friction 0.3 | 2/2 | 1/2 (fell at 21 s, in the 0.55 m/s phase) |
    | rough boxes up to 10 cm | 2/2, roll/pitch 0.039-0.051 | 2/2, 0.032-0.041 |
    | scripted pushes (fixed directions, growing) | fell after ~300 N (0.4 m/s) | fell after ~600 N (0.8 m/s) |

    The scripted push test is one sample per mode (a fixed schedule) and overstated the effect.
    Random pushes, 8 seeds shared by both modes (`--scenario push --repeats 8`), on top of the
    mixed command sequence, 150-600 N for 0.1 s every 3-6 s:

    | | MPC only | MPC + flat_v1 |
    |---|---|---|
    | runs that fell | 5/8 | 5/8 |
    | mean time to fall (27 s if none) | 19.9 s | 22.3 s |
    | pushes recovered, 150-300 / 300-450 / 450-600 N | 13/13, 5/8, 6/8 | 12/12, 7/9, 10/13 |
    | roll/pitch RMS | 0.057 | 0.046 |

    Per seed it is mixed: the policy is better on 0, 6 and 7, worse on 2, and the same on 1, 3 and
    4. So flat_v1 is at most marginally more push-robust than the MPC alone. It was never trained
    with pushes, which is what the robust training adds.
- [x] mujoco_ros2_control:
  - the controller time is snapped to the physics-step grid. Accumulated float error had
    slipped one physics step after ~170 s per sim: 2072 sync violations in run1;
  - external wrenches clear only the bodies they set, so episode pushes survive;
  - episode control: spawn pose / yaw, joint noise, link mass and inertia, payload, pushes.
- [x] Robust run 2 (`robust_run2_1M`, stopped at 1.0M of 2M transitions): no better than the
  MPC alone. Learning envs fell in 51-59 % of episodes, the 2 control envs in 50-55 %, and the
  learners' fall rate never trended down.
- [x] Why: the MPC alone, flat ground, run 2's randomization, 360 episodes
  (`/tmp/claude_rl/fall_diag`):
  - 158 fell (44 %). With every randomization off except the commands: 0 of 120.
  - 85 falls came in the first 3 s, before any push: from the start velocity (up to 0.3 m/s per
    axis), worse on low friction. Early fall rate:

    | friction | start speed < 0.15 m/s | 0.15-0.3 | > 0.3 |
    |---|---|---|---|
    | 0.3-0.5 | 17 % | 33 % | 58 % |
    | 0.5-0.7 | 16 % | 29 % | 29 % |
    | 0.7-1.0 | 0 % | 17 % | 17 % |

  - The other 73 came within 3 s of a push. Fell within 2 s of a push: 2 % at 75-250 N, 28 % at
    250-425 N, 47 % at 425-600 N; 34 % from the side, 17-19 % front or back.
  - Payload, link mass and inertia, centre-of-mass shift, angular start velocity and the commands:
    no measurable effect (logistic |z| < 1.5; friction z = -4.5, start speed z = +3.7).
  - So half the falls were decided at the reset, which the 3 gait parameters cannot undo: noise
    in the return. Hence the smaller start velocity and the disturbance curriculum above.
- [x] Robust run 3 (`robust_run3`, stopped at 0.89M): the curriculum made the conditions
  learnable (learners fell ~31 % instead of ~58 %, terrain level ~5, disturbance level 0.5), but
  the policy still did not beat the MPC alone. From 0.54M a deterministic-policy group ran beside
  the stochastic learners and the MPC-only group, all on the same levels: 30 % (n=285), ~36 %
  (n=74) and ~36 % (n=76) of episodes fell. Training return stayed flat at 1700-1950.
- [x] Recipe review, what was structurally wrong:
  - **No authority where it counts.** The 3 actions only reshape phases that have not started.
    A push is caught (or not) by where the swing foot in progress lands, which no action
    touched. Hence the foothold actions.
  - **Most decisions were thrown away.** A gait action shapes a phase only until it starts, so of
    the ~40 decisions in a step, the last one before the boundary counted and the rest were
    overwritten. (Worse, each of them retimed the MPC's horizon: see run 4.) With gamma 0.995 at 100 Hz, a fall 1 s later reached them through 100
    bootstrap steps. Hence 50 Hz decisions (and the footholds, which act every decision).
  - **Few gradient steps.** 1 update per 12-env tick is 0.1 updates per transition, and the
    critic's Q was still climbing (82 -> 238) when run 2 stopped. Now 1 update per tick, i.e.
    2 per decision: 0.2 per transition.
  - **The comparison measured the exploration noise.** Fixed with the deterministic group.
- [x] Run 4 (`robust_run4`, stopped at 0.6M): 5 actions, 50 Hz, gamma 0.99; groups: 8 learning,
  2 deterministic, 2 MPC-only. Worse than the MPC alone. From 0.3M to 0.6M:

  | group | episodes | fell |
  |---|---|---|
  | learning (stochastic) | ~560 | 35 % |
  | deterministic policy | 127 | 38 % |
  | MPC alone | 123 | 24 % |

- [x] Why: **the actions broke the MPC.** `GaitSchedule::setAdaptation` retimed every future
  phase whenever the action changed. A continuous policy changes it every decision, so every
  solve saw the horizon's contact switches somewhere else than its warm start, and one SQP
  iteration could not follow. The MPC's equality residual (`MPC_HEALTH` equality SSE, mostly
  `rneaBaseDynamics`), per 2 s window:
  - every RL run so far (run 1, the smoke runs, run 4's learning and deterministic envs):
    median ~5e3-5e4;
  - the MPC alone: median ~50.

  A/B on flat ground, nominal dynamics, training commands, 6 envs with zero actions and 6 with
  fresh N(0, 0.5) actions every decision, 8 min (`/tmp/claude_rl/churn_{before,after}`):

  | | zero | random, before | random, after the fix |
  |---|---|---|---|
  | equality SSE median / p90 | 46 / 234 | 8159 / 28980 | 127 / 1529 |
  | falls, MPC faults | 0, 0 (42 episodes) | 2, 1 (44) | 0, 0 (42) |

  The fix reads the gait actions once per phase, at its start. The foothold actions only move a
  cost reference and never disturbed the MPC.
- [x] A second fault path: OCS2 moves the plan's first node onto a contact switch lying within its
  minimum step (~10 us) after the initial time, and the controller rejected such plans
  (`evaluation_time >= times.front()`) and latched FAULT, a "fall". 32 such episodes in run 4,
  all learning or deterministic, none MPC-only. Now 0.1 ms of tolerance.
- [x] Authority: can hand-made actions prevent falls? MPC alone vs a fixed policy, ~100 episodes
  each (`/tmp/claude_rl/authority`):

  | condition | MPC alone | hand-made | |
  |---|---|---|---|
  | blind stairs up, 5.4-10 cm risers | 45 % fell | swing apex +5 cm: 44 % | no effect |
  | flat, pushes 75-600 N, friction 0.3-1.0 | 40 % | capture-point foothold, (v - v_cmd) / omega: 32 % | z = 1.2 |

  So swing height does not help on these stairs (the falls are not toe stubs), and foot
  placement helps under pushes, modestly. That bounds what this action space can buy.
- [x] Also changed for run 5: the actor's output layer starts at zero (the policy starts as the
  MPC alone, exploration std 0.3), and warm-up noise is N(0, 0.5) around zero instead of
  uniform over [-1, 1].
- [x] Run 5 (`robust_run5`, stopped at 0.32M): run 4's recipe with the two fixes. No MPC faults,
  and the policy envs' equality SSE median was 300-600 (MPC alone 77-98; run 4 ~5e3-5e4). Still no
  gain: deterministic policy 33/121 (27.3 %) fell, MPC alone 35/125 (28.0 %), z = 0.1; learning
  envs 36-42 %. The deterministic policy led early (22 % vs 30 % at 0.2M) and fell back, as run 4
  had. About half of the learners' falls were on blind stairs and stepping stones (51-67 % fell
  there), where the authority test found the actions powerless.
- [x] Run 6 (`robust_run6_residual`, stopped at 0.76M): run 5 plus the 12 residual-dynamics
  actions (17 in all, 85 observations); rewards unchanged (the paper has no residual-specific term;
  its action smoothness and squared-action loss are already here). Groups 8 / 2 / 2. **Worse than the
  MPC alone, and getting worse:**

  | | whole run | 0.6-0.76M |
  |---|---|---|
  | deterministic policy | 162/371 fell (44 %) | 45/83 (54 %) |
  | MPC alone | 100/344 (29 %) | 20/81 (25 %) |

  z = -4.1 and -3.9. The learners fell 32-50 %, the disturbance curriculum stepped back to 0, and
  the return fell from ~1750 to ~1500 after 0.6M.
- [x] Why: **critic overestimation.** The critic's mean Q rose to ~435-442 and stayed there while
  its loss climbed (51 -> 185). With the measured tracking terms (~2.13 per tick, 2 ticks per
  decision) even a policy that never falls is worth ~426 at gamma 0.99, and these policies fell in
  a third to a half of their episodes. The deterministic action (the mean, what deployment runs) did
  worse than the stochastic one, i.e. the mean sat where the critic was most wrong; 17 action
  dimensions give it many more such directions than 5.
- [x] Run 7 (`/tmp/claude_rl/run7_critic`): run 6 with an overestimation-resistant learner
  (`train.py --critics 4 --actor-delay 2 --prior-weight 0.1`): 4 critics, target = the minimum of
  a random pair (REDQ), actor on their mean and updated every 2nd critic update (TD3), and the
  actor's objective divided by mean |Q| with 0.1 |mean action|^2 added (TD3+BC-style pull towards the
  MPC alone). Actions, observations and rewards as run 6.
  - To 0.46M (`robust_run7_prior0.1`): no longer harmful, not better. Deterministic policy 74/204
    (36 %) fell, MPC alone 87/227 (38 %), z = +0.4. The critic stayed pessimistic (Q ~217-239,
    below the ~426 ceiling; the 4 critics within ~4 of each other), the learners fell 20-40 % and the
    curricula reached their furthest yet (disturbance 0.6, terrain level 5.5). But the deterministic
    action's RMS sat at 0.034-0.036 from 0.3M on: the pull held the policy at the MPC alone.
  - From 0.46M: resumed with `--prior-weight 0.03`. Updates wait until the (emptied) replay buffer
    holds 10k transitions again.
  - Stopped at 0.97M (`/root/claude_rl_policies/robust_run7`, with `policy_overlay.yaml`): **no
    gain**. Disjoint 20-episode blocks, deterministic policy vs MPC alone: whole run 163/431 (38 %)
    vs 169/455 (37 %), z = -0.2; since the resume 89/227 (39 %) vs 82/228 (36 %), z = -0.7; last
    0.2M 34/82 (41 %) vs 25/83 (30 %), z = -1.5. Falls after a push were equal (76 vs 76 over the
    run). The learners stayed at 33-40 % (return ~1,800) for 0.8M decisions at disturbance 0.6; the
    critic stayed pessimistic (Q 220-247, 4 critics within ~4); the deterministic action's RMS rose to
    0.07 and stayed there. MPC equality SSE medians: learners 140, deterministic policy 126, MPC
    alone 75 - the residuals cost the MPC some consistency, far less than the run-4 churn (~5e3).
  - Reading: with the learner fixed, the policy is harmless but finds nothing on this mixed
    distribution. What it is not: overestimation (fixed), MPC churn (fixed), an unreachable action
    (the authority tests show the actions act). What remains open: whether 1M decisions on 12
    envs is simply too little for 17 actions, and whether the gain
    is in specific conditions the mixed average hides - which `rl/evaluate.py` measures.

User constraints (2026-11-05):
- rewards, actor and critic use on-board signals only; ground truth and contacts only for termination
  (and the terrain curriculum);
- robustness (balance, near-fall recovery) is the priority, tracking second;
- randomize dynamics in MuJoCo only; the MPC and Pinocchio keep the nominal URDF;
- pushes are random and measured as force, in training and in the deployment tests;
- do not push;
- WB-ID only.
