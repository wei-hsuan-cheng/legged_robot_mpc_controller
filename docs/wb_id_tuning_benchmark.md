# WB-ID MPC tuning benchmark

How to measure whether a WB-ID change helps, and where a problem originates: in what
the MPC plans, or in how the joint servo follows it. Last updated 2026-10-23 (round 4).

## Resume here (handoff, 2026-10-23)

**Nothing is running at handoff.** Round 7 (`walk` gait timing) finished: all three variants were worse, so the default stays (5.7). Check the container is idle before starting anything (`ps -eo cmd | grep "[r]os2 launch"`).

**The shipped config is the best measured case (`d_kn10`):**
- kp 200 / kd 20, knee kd 10;
- vz cost 10, pitch/roll-rate cost 30;
- arm velocity cost 0.06;
- `useFeedbackPolicy: false`, `sqpIteration: 1`;
- the default `walk` timing;
- MuJoCo controller time.

**Uncommitted changes:**
- `legged_robot_mpc_controller`:
  - controller diagnostics (`~/tracking`, `~/solver`) in the WB controller `.hpp` / `.cpp` / parameter schema;
  - the tuned WB block in `config/g1/ros2_controllers.yaml`;
  - `tests/tuning/`, this doc, `Codex.md`, `.gitignore` (`tuning_results/`).
- `mujoco_ros2_control` (branch `jazzy`): `controller_time_from_sim` in `mujoco_ros2_control.{hpp,cpp}`.
  - The kp/kd command interfaces it builds on are already committed (`b23ee84`).
  - Both packages must be built from these sources: the shipped WB config claims `<joint>/kp|kd`, and exact controller time needs the new `mujoco_ros2_control`.

**How to work:**
- One change per case (`make_config.py`, then `run_case.sh`).
- Read the `SUMMARY` / `DIAG` lines **and** look at `joints.png`, `base.png`, `ff_fb.png` and `solver.png` for every case: the averages have hidden spikes before, and plots of quantized data showed artifacts.
- Run-to-run spread is about 3% (`d_def` vs `c_nofb_simclk`), so treat smaller differences as noise.
- **Priorities (from the user):** no falls first, then smoothness (bounce, jitter); velocity tracking last.
- **Solver budget:** keep `sqpIteration: 1` and spend compute on MPC rate; the user accepts no less than 60 Hz.

**Next steps, most promising first:**
1. **Per-solve command steps.** Ripple at the solve period in the knee/shoulder feedback; legs `cmd_tau_jump_at_switch` 0.79 vs 0.31 on other ticks. Options: blend the new policy into the old one over a few ms in the controller, or cut solve time to raise the rate (`f_thr6_120` reached 104 Hz).
2. **Foot contact model.** The MJCF sole is four spheres spanning 6 cm laterally, so the CoP soft constraint is near or over its limit (5.5). A realistic sole box plus a matching `model.contactGeometry.rectangle` would give lateral margin. Ask the user first: it's a model decision.
3. **Lateral tracking** is 35% (`r_vy020`). The cost weight barely helps; look at gait and foot placement.
4. **Plan consistency at higher speed** (`r_vx075`): walking equality violation 28, with isolated CoP-violation cost spikes.

## 0. Current state (read first)

**Shipped WB-ID settings** (`config/g1/ros2_controllers.yaml`, WB block, each commented in place):

| setting | value | before this work |
|---|---|---|
| `control.jointEffortPd` kp / kd | 200 / 20 on all MPC joints, knee kd 10 | 1200 / 20 |
| `ocs2.costs.q/qFinal.baseVelocity` vz, pitch rate, roll rate | 10, 30, 30 | 3, 3, 3 |
| arm `ocs2.costs.q.jointVelocities` | 0.06 | 0.02 |
| `ocs2.solver.sqp.useFeedbackPolicy` | false | true |
| `ocs2.solver.sqp.sqpIteration` | 1 (kept: rate beats iterations, see 5.3) | 1 |

**Simulation clock (`mujoco_ros2_control`, `controller_time_from_sim`, default true):**
- **What it does:** the controller manager's clock is set from `mj_data->time` before every update. Its own `/clock` subscription is remapped to an unused topic, so controllers see exact 1 ms steps.
- **What it fixes:** before, the controller manager stamped updates from `/clock`, published at `mujoco_publish_rate` (100 Hz). So the controllers saw time advance in 10 ms steps: the joint command was a 100 Hz staircase, and the estimators got zero time steps 9 ticks out of 10.
- **Scope:** simulation only. On hardware the controller manager reads the system clock.

**Result, same 0.5 m/s walk.** "Before" is kp 1200/20, the old costs, the feedback policy on and the 100 Hz clock; its high-frequency columns are biased (section 1.3).

| | before | now (`d_kn10`) |
|---|---|---|
| pelvis bounce (z sd) | 14.4 mm | **11.9 mm** |
| forward speed | 0.40 m/s | 0.40 m/s |
| roll / pitch RMS | 0.54° / 0.18° | **0.40°** / 0.18° |
| pelvis 4–40 Hz vertical acceleration | 2.09 | **1.19** |
| joint tracking error, legs / arms | 0.39° / 0.11° | **0.13° / 0.02°** |
| leg torque 4–40 Hz | 5.9 Nm | **3.7 Nm** |
| arm torque 4–40 Hz | 1.7 Nm | **0.8 Nm** |
| servo touchdown peak (99.9th percentile) | about 95 Nm (knee) | **54 Nm** |
| MPC rate | 78 Hz | **97 Hz** |
| torque jump at a new solve, legs (vs other ticks) | 3.4 (0.75) | **0.79 (0.31)** |

**What remains:**
- the MPC-planned bounce (11–12 mm);
- touchdown spikes of about 50 Nm at the knee and ankle;
- a ripple at the solve rate, i.e. the command stepping at each new solution;
- the CoP running near the foot edge.

Gain tuning has reached diminishing returns (round 4); the next levers are MPC-side (section 6).

## 1. Instrumentation and recording

### 1.1 Controller topics

`humanoid_whole_body_inverse_dynamics_mpc_controller` → `diagnostics.trackingRate` [Hz]. It defaults to 0 (off, no cost). The experiments use 500.

| topic | rate | content |
|---|---|---|
| `~/tracking` (`Float64MultiArray`, names on `~/tracking_names`) | `trackingRate` | per control tick, 272 columns, listed below this table |
| `~/solver` (`Float64MultiArray`, names on `~/solver_names`) | every solve | listed below this table |

`~/tracking` columns:
- `t`, bring-up `phase`, `policy_valid`, `plan_mode`, `policy_t0`;
- per MPC joint:
  - `cmd_q`, `cmd_qd`, `cmd_tau`: what was written to the hardware; `cmd_tau` is the feedforward;
  - `plan_q`, `plan_v`, `plan_tau`: the MPC policy at the current time;
  - `meas_q`, `meas_v`, `meas_tau`: the measured state; `meas_tau` is the applied torque, i.e. feedforward plus PD;
- base pose and twist, planned and estimated;
- both contact frames' position and velocity, planned and estimated;
- the planned contact wrenches.

`~/solver` columns:
- `t`, `advance_ms`, and the SQP performance indices (merit, cost, dynamics / equality / inequality SSE, Lagrangians);
- `eq_sse/<term>`: equality violation summed over the horizon; `eq_sse_near[/<term>]`: the same over the first 3 nodes, which drive the command;
- `cost/`, `state_cost/`, `soft/`, `state_soft/<term>` at the first node.
  - These are evaluated on a private copy of the problem, so they never race the solver.
  - Soft constraints are relaxed log barriers: negative well inside, about 0 or positive near the limit.

### 1.2 Recording

`run_case.sh` records with `ros2 bag` (mcap). `bag_to_npz.py` turns the bag into `data.npz`, and the bag is then deleted. Measured rates: `/joint_states` about 930 Hz, `~/tracking` about 490 of 500 Hz, every solve.

### 1.3 Biases found, and which results they affect

1. **Python subscribers drop data in this container.** A probe subscribing directly received `/joint_states` at about 46 Hz and `~/tracking` at about 80 Hz. Fixed by recording with rosbag.
2. **Controller time was quantized to 10 ms** (the clock above). Joint-state and ground-truth header stamps still advance only at `mujoco_publish_rate`. So joint signals are taken from `~/tracking` (`meas_*`, stamped with the exact controller time); ground truth is used only for low-frequency metrics plus `acc_hf`.
3. **The `MPC_HEALTH` line in `summary.txt` is the last 2 s, after the robot has stopped.** Use `diag.json → solver` for walking values.

Consequence: in the round 1–2 tables (5.1, 5.2), every 4–40 Hz "jitter" number is unreliable; low-frequency metrics are fine. Rows marked **exact** come from the bag plus exact controller time.

## 2. Tools (`tests/tuning/`)

| file | does |
|---|---|
| `make_config.py` | writes a variant of `config/g1/ros2_controllers.yaml`: `--kp/--kd` by joint group (`hp hr hy kn ap ar wy wr wp sp sr sy el`, or `all`), `path=value` edits below the WB `ros__parameters`, `--tracking HZ` (default 500) |
| `run_case.sh` | in the `ros_jazzy` container: launches WB-ID on `scene_flat.xml`, records a bag around the probe, converts it, plots, cleans up. Env: `HEIGHT`, `LAUNCH_ARGS` (extra launch args), `RVIZ=true SHOTS=1 RVIZCFG=...`, `OUT` |
| `tune_probe.py` | drives the walk on a wall-clock schedule: 4 s stand → 3 s ramp → 20 s at `vx` → 3 s ramp down → 5 s stop. Watches for a fall and writes `meta.npz` |
| `bag_to_npz.py` | bag + meta → `data.npz` |
| `plot_tracking.py` | `data.npz` → `SUMMARY` / `DIAG` lines, `metrics.json`, `diag.json` and the plots; takes a case folder |
| `xwd2png.py` | converts RViz screenshots |

```bash
# inside the container; the source tree is the Mac repo through the shared mount
cd /root/ocs2_ros2_ws/src/legged_robot_mpc_controller/tests/tuning
python3 make_config.py /tmp/wbt_mycase.yaml --kd kn:10 'ocs2.costs.q.baseVelocity=[30.0, 15.0, 10.0, 30.0, 30.0, 30.0]'
./run_case.sh mycase /tmp/wbt_mycase.yaml 0.5
python3 plot_tracking.py ../../tuning_results/mycase     # re-plot
```

Notes:
- **Busy container:** check nothing else runs first, since two simulations share one ROS graph.
- **Repetition:** one run per case unless noted. Run-to-run spread on the default is in round 4; treat differences under about 5% as noise.
- **Read the plots, not only the numbers:** check `joints.png`, `base.png`, `ff_fb.png` and `solver.png` for every case. Averages hide touchdown spikes, and plots of quantized data can show artifacts.

## 3. Outputs and where they are

Each case writes `tuning_results/<tag>/` in the repo. That folder is git-ignored and on the Mac at `/Users/whcheng/src/mpc/legged_robot_mpc_controller/tuning_results/<tag>/`:

| file | content |
|---|---|
| `config.yaml` | the exact controllers yaml used |
| `sim.log`, `bag.log` | launch and recorder logs |
| `data.npz` | all streams, 5–7 MB |
| `summary.txt` | the printed lines |
| `metrics.json`, `diag.json` | the numbers |
| `joints.png` | 8 joints: plan vs measured position; MPC torque, feedforward, applied |
| `base.png` | pelvis z, vz, roll/pitch (plan / estimate / ground truth), foot z, planned normal force |
| `spectra.png` | feedforward vs PD torque and command vs measured position, per group |
| `ff_fb.png` | feedforward (`tau_ff`) vs feedback (PD = applied − feedforward) |
| `solver.png` | solve time, violation (all and near), cost, cost terms |

Round 1 used scratch versions of the tools; its metrics are in the container's `/tmp/claude_tune_*_metrics.json` (raw data deleted).

## 4. How to read the numbers

All metrics cover the 3rd to 20th second of the 0.5 m/s hold.

**`SUMMARY`:**
- `vx`;
- `z_sd`: pelvis height sd, trend removed;
- `vz_rms`;
- `roll`, `pitch`: sd;
- `acc_hf`: 4–40 Hz pelvis vertical acceleration;
- `arm/leg vel_hf, tau_hf`: 4–40 Hz joint velocity / applied torque RMS, mean over the group;
- `swing`: shoulder-pitch peak to peak.

**`DIAG`**, the plan-vs-measurement decision tool:

| comparison | similar → | different → |
|---|---|---|
| `plan_z_sd` vs `obs_z_sd` | the MPC **plans** it; tune the MPC | the robot departs from the plan; look at tracking |
| `cmd_q_hf` vs `meas_q_hf` | jitter is in the command (MPC) | jitter is added by the servo or contact |
| `track_err_q` | small (≤ 0.2°): the servo tracks | the servo is too soft, saturating, or contact is disturbing it |
| `cmd_tau_hf` (FF) vs `pd_tau_hf` (FB) | — | whoever is larger produces the torque jitter |
| `PD peak p99.9`, `pd_tau_max` | — | impact spikes (touchdown) |
| `cmd_tau_jump_at_switch` vs `..._otherwise` | successive solves agree | each new solve jumps; the plan is inconsistent |

**`solver`:**
- `advance_ms` against 10 ms;
- `eq_sse_mean`, and `eq_sse_near_mean`, the near-horizon value the command actually uses;
- `cost_terms_mean`: a soft constraint near 0 or positive is near its limit. `contactMomentXY` (the CoP) sits at about +1.5: often near the foot edge.

## 5. Results (flat, 0.5 m/s command, linear-KF estimator)

### 5.1 Round 1: gains and costs (scratch tools; jitter columns biased, low-frequency fine)

Baseline: kp 1200 / kd 20, the original costs, pelvis height 0.795.

| case | change | fell | z sd [mm] | roll [°] | arm swing [°] | notes |
|---|---|---|---|---|---|---|
| baseline | — | no | 14.5 | 0.54 | 16.7 | |
| crit1 | kd = 2√(kp·I) per joint | **6.4 s** | – | – | – | swing inertia ≠ stance inertia: ankle kd 6 fell |
| arms-critical | shoulder yaw 7, elbow 13.5 | no | 14.3 | 0.55 | 17.3 | arm jitter much worse: keep kd ≥ 20 on light joints |
| height 0.75 / 0.72 | commanded pelvis height | no | **17.5 / 20.5** | 0.45 | 17–18 | lower is bouncier: the bounce is chosen, not a reach limit |
| vz30 | vz cost 30 | no | **7.1** | **0.97** | 18.4 | arms and roll take the motion |
| z300 | height cost 300 | **4.6 s** | – | – | – | |
| armacc | arm acceleration cost ×10 | no | 13.8 | 0.61 | 19.5 | arm jitter much worse, solver violation 184 |
| armvel | arm velocity cost ×10 | no | 14.9 | 0.52 | **6.4** | jitter gone but swing too |
| kp400 / kp200 | servo stiffness | no | 15.1 / 14.7 | 0.52 / 0.54 | 17 | less servo torque, same tracking |
| best1 | kp 200, vz 10, rates 30, arm velocity cost 0.06 | no | **10.6** | 0.62 | 13.0 | shipped |

### 5.2 Round 2: plan vs measurement, first instrumented runs (100 Hz controller clock)

Findings that still hold:
1. **The bounce is planned:** plan z sd ≈ measured.
2. **Arm jitter is in the commanded position.**
3. **Stiff servo:** at kp 1200 the PD torque exceeded the feedforward above about 8 Hz.
4. **One SQP iteration violates `rneaBaseDynamics` every solve.**

Its jitter numbers are superseded by 5.3.

### 5.3 Round 3: bag recording; /clock 1 kHz (A) or MuJoCo controller time (B)

| case | vx | z sd | roll/pitch | acc_hf | arm vel/τ HF | swing | leg τ HF | track err legs/arms [°] | τ jump at switch (else) | solve | eq SSE all / near |
|---|---|---|---|---|---|---|---|---|---|---|---|
| best1, 100 Hz clock | 0.380 | 11.0 | 0.62/0.20 | 2.01 | 0.098/1.13 | 12.7 | 5.29 | 0.375/0.084 | 1.21 (0.56) | 11.6 ms, 85 Hz | 9.7 / 1.85 |
| best1, clock 1k | 0.387 | 10.8 | 0.56/0.21 | 1.69 | 0.104/1.11 | 13.1 | 5.22 | 0.171/0.033 | 1.12 (0.42) | 11.6 ms, 86 Hz | 7.1 / 2.54 |
| + feedback policy off (`c_nofb`) | 0.401 | 11.4 | 0.53/0.21 | 1.57 | 0.088/0.87 | 12.7 | 4.27 | 0.154/0.026 | 0.87 (0.36) | 10.0 ms, 98 Hz | 8.6 / 2.21 |
| + 2 SQP iterations, fb off | 0.403 | 12.7 | 0.54/0.20 | 1.46 | 0.085/0.86 | 13.1 | 4.29 | 0.222/0.038 | 0.83 (0.32) | 19.8 ms, **50 Hz** | 2.6 / **0.04** |
| + 2 SQP, fb off, 0.8 s horizon | 0.388 | 12.6 | 0.65/0.23 | 1.95 | 0.104/1.07 | 13.9 | 5.47 | 0.255/0.044 | 0.93 (0.33) | 16.1 ms, 62 Hz | 3.3 / 0.20 |
| + `feedforward: inverse_dynamics` | 0.374 | 10.8 | **0.46/0.09** | 1.97 | **0.388/3.26** | 16.3 | **7.00** | 0.190/0.320 | 2.20 (0.83) | 11.9 ms, 83 Hz | 18.6 / 252 |
| **fb off + MuJoCo clock (B), exact** (`c_nofb_simclk`) | 0.402 | 11.5 | 0.53/0.20 | **1.50** | 0.090/0.88 | 12.7 | **4.15** | **0.130/0.020** | **0.82 (0.32)** | 10.1 ms, 97 Hz | 8.5 / 2.12 |

Findings:
1. **The correct clock halves tracking error** and removes the 100 Hz command staircase. B (in-process) beats A (`/clock` at 1 kHz, about 10% repeated timestamps).
2. **The feedback policy off is better or equal everywhere,** and about 1.5 ms faster per solve.
3. **2 SQP iterations nearly zero the near-horizon violation but don't walk better.** Keeping above the 60 Hz floor needs a shorter horizon, which is worse. Decision (agreed with the user, consistent with the MIT humanoid paper's real-time-iteration argument): keep 1 iteration and spend the time on rate.
4. **ID feedforward** steadies the body but injects measurement noise into the arms. Rejected.

### 5.4 Round 4: touchdown impact and per-joint damping (all exact)

| case | change (from `c_nofb_simclk` settings) | z sd | roll | acc_hf | leg τ HF | PD peak p99.9 (max) | leg PD HF | notes |
|---|---|---|---|---|---|---|---|---|
| `d_def` | none (repeat) | 11.4 | 0.52 | 1.51 | 4.15 | 78.9 (L knee 83.5) | 2.33 | matches `c_nofb_simclk` within 3%: run-to-run spread is about 3% |
| **`d_kn10`** | knee kd 20 → 10 | 11.9 | **0.40** | **1.19** | **3.73** | **53.9** (R ankle pitch 58.5) | **1.86** | **shipped** |
| `d_tdh0` | touchdown height offset −0.001 → 0 | 11.9 | 0.45 | 1.52 | 4.04 | 77.9 (L knee 84.1) | 2.29 | no effect on the spike |
| `e_ank15` | + ankle pitch/roll kd 15 | 11.7 | 0.41 | 1.14 | 3.61 | 52.0 (L knee 55.6) | 1.79 | 3–4% better, within spread: not adopted |
| `e_hip15` | + hip pitch/roll kd 15 | 11.9 | 0.42 | 1.19 | 3.68 | 53.3 | 1.79 | no change |
| `e_armkp100` | + arm kp 100 | 11.9 | 0.39 | 1.19 | 3.72 | 53.7 | 1.84 | no change; arm jitter is in the feedforward (0.72) not the servo (0.26) |

Plots (`d_kn10/ff_fb.png`):
- knee servo spikes at touchdown went from −85 to −50 Nm;
- a small ripple at about the solve period (±1.5–3 Nm) remains in the knee-stance and shoulder feedback: the servo answering the command step at each new solution.

### 5.5 Round 5: robustness of the shipped settings (exact)

| case | command | achieved | fell | z sd | roll/pitch | leg PD peak p99.9 (max) | eq SSE all / near | notes |
|---|---|---|---|---|---|---|---|---|
| `r_vy020` | vy 0.2 m/s | **vy 0.07 (35%)**, heading drift 5.6° | no | 8.9 | 0.20/0.21 | 35 (L ankle pitch 47) | 0.4 / 0.8 | lateral tracking is the weak spot |
| `r_yaw050` | yaw 0.5 rad/s | 0.44 rad/s (88%) | no | 4.7 | 0.19/0.06 | 23 (R hip yaw 33) | 0.4 / 1.6 | |
| `r_stops` | 0.5 m/s, 4 s walk / 3 s stand | survived every stop | no | 11.8 | 0.30/0.34 | 48 (L knee 55) | 4.3 / 1.3 | |
| `r_vx025` | vx 0.25 | 0.22 (86%) | no | 9.2 | 0.20/0.14 | 32 (L ankle pitch 49) | 0.3 / 1.5 | |
| `r_vx075` | vx 0.75 | 0.62 (83%) | no | 13.3 | 0.36/0.25 | 61 (R knee 69) | **28 / 5.1** | isolated cost spikes: see the CoP finding below |

**CoP finding** (`r_vx075/solver.png`):
- At 0.75 m/s, isolated solves have total cost spikes up to 340. Each coincides with the stance foot's `contactMomentXY` soft constraint jumping to 3.7e5–6e5, i.e. the plan's CoP outside the foot rectangle. At 0.5 m/s the same term averages about +1.5, near its limit.
- **Cause:** a model property, not a mistuning. `constraints.contact` (rectangle x ±0.09, y ±0.03 m about a frame 3.5 cm ahead of the ankle) matches the MuJoCo foot exactly: Unitree's MJCF collides through four 5 mm spheres at y = ±0.025 to ±0.03 m. The support is only 6 cm wide, so lateral CoP margins are small.
- **Options:** widening the rectangle alone would let the MPC plan CoPs the simulated foot can't produce. A realistic sole collision model (a box matching the G1 sole) plus the matching rectangle would raise the margin. That's a model decision, left to the user.

### 5.6 Round 6: MPC-side costs, threads, lateral cost (exact; default = `d_kn10`)

| case | change | z sd | roll | acc_hf | arm vel/τ HF | leg τ HF | near eq SSE | MPC | verdict |
|---|---|---|---|---|---|---|---|---|---|
| `d_kn10` | default | 11.9 | 0.40 | 1.19 | 0.083/0.82 | 3.73 | 2.1 | 97 Hz | |
| `f_vz20` | vz cost 10 → 20 | **9.7** | **0.55** | 1.32 | 0.097/0.92 | 3.78 | 2.2 | 98 Hz | bounce −18%, roll +38%: no |
| `g_vz20r100` | vz 20 + pitch/roll rate 100 | 9.8 | 0.51 | 1.28 | 0.096/0.95 | 3.80 | 2.4 | 98 Hz | rates barely help: no |
| `g_vz10r100` | pitch/roll rate 30 → 100 | 11.9 | 0.43 | 1.21 | 0.081/0.84 | 3.72 | 1.8 | 98 Hz | no effect above 30 |
| `f_thr6_120` | `sqp.nThreads` 6, `mpcFreq` 120 | 11.8 | 0.42 | 1.22 | 0.081/0.82 | 3.83 | **1.6** | **104 Hz** | faster, more consistent plan, no motion gain; more CPU: not adopted |
| `f_vy30` (vy profile) | lateral velocity cost 15 → 30 | 9.6 | 0.18 | 1.01 | 0.077/0.96 | 3.51 | 1.0 | 97 Hz | vy 0.070 → 0.085 of 0.2; hip-roll peak 65 Nm: no |

Plots (`f_vz20/base.png`): the bounce/roll trade-off is a **roll snap at each touchdown**. The planned roll jumps about ±1° at the contact switch and the robot follows. The roll-rate cost does not govern it (`g_*`), so round 7 tests the `walk` gait timing (0.1 s double support, never tuned for WB-ID).

### 5.7 Round 7: `walk` gait timing (done)

`ocs2.gait.templates.walk` (WB block). The default is `[0.0, 0.6, 0.7, 1.3, 1.4]`: 0.6 s single support, 0.1 s double support. Cases:
- `w1`: `[0.0, 0.5, 0.65, 1.15, 1.3]` (0.5 / 0.15);
- `w2`: `[0.0, 0.6, 0.8, 1.4, 1.6]` (0.6 / 0.2);
- `w3`: `[0.0, 0.4, 0.55, 0.95, 1.1]` (0.4 / 0.15, the tuned `slow_walk` timing).

Compare against `d_kn10` (z sd 11.9, roll 0.40, acc_hf 1.19, leg τ HF 3.73, PD peak 53.9). Watch the touchdown roll snap in `base.png`.

| case | z sd | roll | acc_hf | arm vel/τ HF | leg τ HF | PD peak p99.9 (max) | vx | verdict |
|---|---|---|---|---|---|---|---|---|
| `d_kn10` (default 0.6/0.1) | **11.9** | **0.40** | **1.19** | 0.083/0.82 | **3.73** | 53.9 | 0.401 | |
| `w1` (0.5/0.15) | 12.8 | 0.45 | 1.28 | 0.099/0.99 | 4.18 | 53.3 (R ankle pitch 58) | 0.397 | worse |
| `w2` (0.6/0.2) | 14.2 | 0.62 | 1.61 | 0.079/0.84 | 4.33 | 64.2 (L knee 71) | 0.359 | worse |
| `w3` (0.4/0.15) | 12.1 | 0.44 | 1.41 | 0.138/1.37 | 5.83 | 53.4 (L ankle pitch 56) | 0.401 | worse: legs' 4–40 Hz feedforward 5.3 vs about 3.0, shoulder feedforward ±10 Nm |

All three are worse. Longer double support raises bounce and roll, and faster stepping raises feedforward jitter. **The default `walk` timing `[0.0, 0.6, 0.7, 1.3, 1.4]` stays.**

## 6. Open items (next steps, in order)

1. **Touchdown impact.** Round 4, then touchdown timing vs actual contact, and the swing touchdown velocity.
2. **Robustness:** N ≥ 3 on the final config; vy, yaw and stop sequences; 0.25 and 0.75 m/s. The probe needs those profiles added.
3. **Plan consistency within the time budget:**
   - `rneaBaseDynamics` near-horizon violation is about 2;
   - try `sqp.dt` / horizon trade-offs that keep at least 90 Hz, warm start, and `torqueConstraintNodes`.
4. **The CoP soft constraint is near its limit at node 0:** check the foot rotation margin and foot placement.
5. **Why the vz/rate/arm costs raised the walking violation** (3.1 → 8–10): attribute one factor at a time.
