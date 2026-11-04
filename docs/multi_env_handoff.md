# Multi-Environment Lockstep Training: Handoff

This is the state of the parallel-simulation substrate for an RL-augmented MPC, written for
whoever continues it. The target method is Kamohara et al. 2025, "RL-augmented MPC": a SAC
policy outputs parameters for the MPC at the MPC rate. The requirements were:
- keep ros2_control;
- train with PPO or SAC on CPU only;
- parallelize across simulations, not on a GPU;
- deploy on CPU like the current OCS2 MPC.

Status: **the substrate works and is validated. No RL code exists yet.**

| repo | branch | contents |
|---|---|---|
| `mujoco_ros2_control` | `feat/multi_env` (from `jazzy`) | new package `mujoco_ros2_control_multienv`, unpaced mode, `.mjb` compile tool, a sim-time fix |
| `legged_robot_mpc_controller` | `feat/multi_env` (from `main`, which now equals `jazzy`) | synchronous MPC, MPC exported state and command references, launch overlay, H2 training overlay, test, CppAD memory fix |

## What runs

N copies of the unchanged `launch/robot.launch.py`. Each copy is:
- one MuJoCo process;
- one controller manager;
- the estimator controller;
- the WB-ID MPC;
- a lockstep bridge.

Each copy gets its own `ROS_DOMAIN_ID` (`100 + i`), so nothing is namespaced: topic names,
TF and controller names are the single-simulation ones. The trainer steps them through
shared memory:

```
trainer (Python)                         simulation i (one process)
LockstepVecEnv ──/dev/shm/legged_env_i── lockstep_bridge ──refs──> WB-ID MPC <──chained──> estimator
   step(actions) -> obs                   (blocks the controller manager       (solves in the tick,
                                           until the trainer answers)           every 10 ms of sim time)
```

Every link is in-process memory or shared memory with semaphores; there is no topic anywhere
in the loop:
- **Estimator <-> MPC:** the existing chained interfaces.
- **MPC solve:** `mpcExecution.mode: synchronous` solves inside `update()` on the controller-time
  grid, so the simulation waits for it.
- **Bridge <-> trainer:** POSIX shm plus two process-shared semaphores.

### Timing contract (measured, holds on every step)

- **One step:** one RL step = one MPC solve = 10 ms of controller time.
- **Update order:** the controller manager runs the MPC before the bridge. The bridge detects this
  with its `order_probe` (the MPC's `status.update_time`) and exchanges **one tick before** each
  solve tick (`handshake_tick_offset = -1`).
- **Freshness:** each observation is 1 ms old when the solve its action drives starts.
- **Observation age:** `t_obs - last_solve_time` is exactly 9 ms on every step.
- **Determinism:** environments given the same commands are bit-identical. Any divergence
  between them is a bug.

## Running it

In the `ros_jazzy` container, from the workspace, because the CppAD cache path
`auto_generated/<robot>` is relative to the working directory:

```bash
cd /root/ocs2_ros2_ws && source install/setup.bash
colcon build --symlink-install --packages-select mujoco_ros2_control mujoco_ros2_control_multienv legged_robot_mpc_controller
python3 /root/src/mpc/legged_robot_mpc_controller/tests/lockstep_multi_env_test.py --num-envs 16
```

The test starts the copies 3-4 s apart and steps them all from the first physics tick. Each
one goes through the bring-up (stand-up, estimator init, MPC start) and then
stand / walk 0.1-0.4 m/s / stop / turn +-0.3 rad/s / stop, commanded through the bridge. It
checks:
- survival (ground-truth height and tilt, no FAULT);
- lockstep timing;
- exactly one solve per step;
- a constant observation age;
- identical results within each command group.

It writes `result.json` and `env_<i>.log` to `--log-dir` (default `/tmp/lockstep_multi_env`).

Measured on the 8-vCPU / 7.7 GB Parallels VM (H2, `humanoid_linear_kf`, flat scene,
28 s of controller time):

| envs | result | aggregate RTF | per-env RTF | solve [ms] | sims PSS |
|---|---|---|---|---|---|
| 1 | PASS | 0.50 | 0.50 | 20 | 173 MB |
| 4 | PASS | 1.53 | 0.38 | 21 | 545 MB |
| 8 | PASS | 2.02 | 0.25 | 27 | 1041 MB |
| 16 | PASS | 1.98 | 0.12 | 55 | 2034 MB |

Throughput saturates at about 8 environments on this VM. 16 fit in memory with 3.9 GB to
spare. "Aggregate RTF" counts start-up in the wall time, so the steady state is slightly
higher.

To start copies by hand, without the test:

```bash
ros2 run mujoco_ros2_control_multienv multi_env --num-envs 8 --shm-name legged_env_{env} -- \
  legged_robot_mpc_controller robot.launch.py robotName:=h2 rviz:=false baseCommandGui:=false \
  mujoco_headless:=true visualizationNodes:=false mujoco_real_time_factor:=0.0 \
  mujocoModelFile:=<scene.mjb> controllersOverlay:=<share>/config/h2/lockstep_training.yaml \
  lockstepBridge:=lockstep_bridge lockstepShmName:=legged_env_{env}
```

Then attach with `LockstepVecEnv([...])` (see `mujoco_ros2_control_multienv/README.md`).

## The pieces

**mujoco_ros2_control_multienv** (in the `mujoco_ros2_control` repo):
- `LockstepBridgeController`: a generic ros2_control controller with these parameters:
  - `shm_name`, `step_period`;
  - `state_interfaces` (observed, any claimable state interface);
  - `command_interfaces` (actions, typically another controller's references);
  - `command_defaults`, `order_probe`.
- `lockstep_protocol.hpp` / `lockstep.py`: the shared-memory layout, version 1. Change both
  together and bump the version.
- `MultiEnvLauncher`, `compile_model()` and the `multi_env` CLI.

**mujoco_ros2_control:**
- `mujoco_real_time_factor: 0` means unpaced.
- `mujoco_compile_model <xml> <mjb>` compiles a scene once.
- Sim time is rounded to the nanosecond instead of truncated (see below).

**legged_robot_mpc_controller:**
- **WB-ID MPC, `mpcExecution.mode` / `policyDelay`:** synchronous solves. `asynchronous`, the
  default, is the old wall-clock solver thread and is unchanged.
- **WB-ID MPC exported state interfaces:**
  - `status.update_time`, `bring_up_phase`, `policy_ready`, `solve_count`;
  - `status.last_solve_time`, `last_solve_wall_ms`, `solver_exceptions`;
  - `plan.mode`, `plan.contact.left|right`.
- **WB-ID MPC reference interfaces:** `walking_velocity_command.{linear_velocity_x,
  linear_velocity_y, desired_pelvis_height, angular_velocity_z}`, with the topic's fields and
  bounds. They are used only in chained mode and only once all four are finite; until then the
  topic drives, as before.
- **`robot.launch.py`:**
  - `controllersOverlay` is deep-merged after substitution.
  - `lockstepBridge` names the bridge, which is activated in the same switch as the MPC, so the
    MPC starts in chained mode.
  - `lockstepShmName` names the channel.
  - `visualizationNodes` controls the RViz-only static TF and environment publisher.
  - Every declared controller now gets `<name>.params_file`.
- **`config/h2/lockstep_training.yaml`:**
  - the bridge declaration;
  - synchronous MPC with `nThreads: 1`;
  - `joint_state_broadcaster` at 50 Hz;
  - 102 observations: deployable signals first (estimate, IMU, MPC status, joints), then
    privileged ones (ground truth, foot contact);
  - 4 actions.
- **CppAD terms:** the ten CppAD cost/constraint terms release their taping-only CppAD
  Pinocchio copy after `initialize()`.

## Why things are the way they are

- **Separate processes, not one process with N controller managers.** This keeps the stack
  exactly as deployed, a crash stays local, the OS schedules the cores, and no namespacing is
  needed. The cost is one `ros2 launch` and one `robot_state_publisher` per copy, about 40 MB.
- **The bridge blocks inside `update()`.** That is the only way to hold a ros2_control
  simulation at an exact controller time without changing the simulator. It is training-only:
  never load the bridge on hardware.
- **Synchronous MPC.** With the asynchronous solver thread, a loaded machine makes plans older
  in controller time: an unfixed latency. Synchronous solving turns load into slower wall time
  instead.
- **Ground truth vs. estimator.** The paper does not say which it used; it mentions only
  "projected gravity from IMU data" in IsaacLab, which suggests simulator state. Here the MPC
  runs on the estimator (deployable), the policy should observe only the deployable block, and
  ground truth is for rewards, termination and an asymmetric critic.
- **Memory decided the environment count.** One H2 environment was 1.4 GB RSS:
  - **CppAD Pinocchio copies:** each term's ~15 MB copy (nv^3 AD tensors) was duplicated into
    every solver clone, one per SQP thread. Releasing it took the default async environment
    to 455 MB.
  - **MJCF loading:** it retains ~230 MB per process (asset cache plus compiler state) against
    ~55 MB for the `.mjb`. Use `compile_model()`.

## Traps found

- **`/dev/shm`** collects zombie Fast DDS segments from killed processes: 157 MB here before
  cleaning. `fastdds shm clean` (after sourcing ROS) removes only dead ones.
- **Duplicate state-interface claims** are refused by the controller manager ("not unique").
  The bridge reuses an observed `order_probe` instead of claiming it twice.
- **`ParameterFile(..., allow_substs=True)`** deletes its substituted copy when garbage-collected;
  hold a reference while reading it.
- **Sim time truncated to ns** read 0.001 s as 999999 ns, so that control tick slipped one physics
  step and some periods were 1.5 ms. This was found by the lockstep timing check (3 irregular
  steps in 28 s) and fixed in `mujoco_ros2_control`. It also shifts the control-tick phase of
  normal runs, so old and new logs are not sample-aligned.
- **A failed controller sequence** does not take `ros2 launch` down: the simulation waits to
  start forever. The test therefore also treats "process has died" in a copy's log as dead.

## Open, in priority order

1. **RL action space.** Expose the adaptation parameters as more MPC reference interfaces, in
   the same pattern as `walking_velocity_command.*` (read in `apply_command_references`, applied
   before the solve). In order of cost:
   - swing apex height;
   - step-duration scale, spliced at the next contact event; `GaitScheduleUpdater` is safe for
     this since d15929b;
   - foothold residual on `captureFootPlacement`;
   - base-wrench dynamics residual, which needs a CppAD parameter in the RNEA constraint and a
     codegen.

   Add them to the overlay's `command_interfaces`. Zero must mean the hand-tuned MPC.
2. **Episode reset without relaunching.** A reset request through the bridge would:
   - make the simulation restore the spawn pose (it needs a hardware command interface or hook
     in `mujoco_ros2_control`);
   - make the MPC re-enter bring-up, or a shortened standing reset that re-seeds the estimator
     (new epoch) and calls `resetMpcNode`.

   Today an episode is a process lifetime, with about 10 s of bring-up.
3. **Trainer.** SAC (or PPO) in PyTorch on CPU over `LockstepVecEnv`. Rewards come from the
   privileged block; terminate on FAULT, solver exceptions or low pelvis height.
4. **Throughput.** `LockstepVecEnv` steps everyone together, so each step waits for the slowest
   simulation. Stepping each channel independently, collecting whichever is ready, suits
   off-policy SAC and should recover part of the gap to the 8-core ideal of about 4x
   single-env RTF.
5. **`policyDelay`** is 0 in the overlay. The real-time deployment solves in about 8-10 ms, so
   `0.01` would train against the latency the robot will see. That is a decision to make
   before training.
6. **Centroidal MPC** has neither a synchronous mode nor command references yet; it needs the
   same changes as the WB-ID controller.
7. **Linear KF velocity error.** It is 0.11-0.14 m/s RMS against ground truth while walking. It
   is pre-existing, but the policy will observe it; compare `humanoid_inekf`
   (`--estimator humanoid_inekf`).
