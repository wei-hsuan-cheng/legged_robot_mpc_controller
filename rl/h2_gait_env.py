# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""H2 WB-ID MPC gait-adaptation environment: N lockstep simulations, one RL step per MPC solve.

The policy adapts the MPC's gait (step duration, swing height, swing apex timing, footholds) and
its dynamics model (residual accelerations and contact-force gains) as in RL-augmented MPC
(Kamohara et al. 2025), for blind locomotion: rough ground, slopes, stairs,
discrete steps and stepping stones (rl/terrain.py, a legged_gym-style curriculum), slippery
ground, pushes, and mass / inertia / payload errors the MPC's nominal model does not know about.
Observations and rewards use only on-board signals: the controller's rl.obs.* (from the
estimator, IMU and encoders) and rl.reward.* (the same estimate, foot velocities by kinematics,
planned contacts). Ground truth only ends an episode (pelvis height above the ground, tilt) and
moves the terrain curriculum (distance walked). See docs/rl_gait_adaptation.md.
"""

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from mujoco_ros2_control_multienv import LockstepVecEnv, MultiEnvLauncher, compile_model, remove_stale

MPC = "humanoid_whole_body_inverse_dynamics_mpc_controller"
LOCOMOTION, FAULT, RESTART = 6, 7, 8
# Actions: 5 gait (period, swing height, apex, foothold x/y) and 12 residual dynamics.
OBS_DIM, ACT_DIM = 85, 17
# Policy observation layout (controller compute_policy_observation): see docs.
O_ANG_VEL, O_COMMAND, O_HEIGHT, O_JOINT_POS, O_JOINT_VEL = 6, 9, 13, 14, 39
O_PREV_ACTION, O_SWING, O_CONTACT = 64, 81, 83
O_GRAVITY = 0


@dataclass
class RewardConfig:
    """Robustness first, then tracking. The tracking and regularization terms are the paper's
    (Table III, weights adapted to H2 and these units); survival, uprightness and a large
    termination penalty are added so that not falling, and recovering from a near-fall, dominate
    the return. All on-board: the estimate (IMU + leg kinematics), encoders, the MPC's plan."""
    alive: float = 1.0
    termination: float = -50.0
    upright: float = -2.0        # x |projected gravity_xy|^2 = sin^2(tilt), from the IMU / estimate
    ang_vel_xy: float = -0.1
    lin_vel_z: float = -0.5
    lin_vel: float = 1.0
    ang_vel: float = 0.5
    height: float = 0.1          # base height above the stance feet (kinematic), so terrain-relative
    joint_vel: float = -2.5e-4
    action_rate: float = -0.015
    feet_slide: float = -0.1
    sigma_lin: float = 0.25    # [m/s]
    sigma_ang: float = 0.25    # [rad/s]
    sigma_height: float = 0.05  # [m]


@dataclass
class EnvConfig:
    num_envs: int = 8
    episode_seconds: float = 10.0
    # MPC ticks (10 ms) per policy decision: the action is held and the rewards summed over them.
    # 2 = 50 Hz, legged_gym's rate; one decision per tick left most actions overwritten before the
    # gait used them and doubled the bootstrap chain.
    action_repeat: int = 2
    command_resample_seconds: float = 5.0
    vx: tuple = (-0.3, 0.6)          # [m/s]
    vy: tuple = (-0.2, 0.2)
    yaw_rate: tuple = (-0.5, 0.5)    # [rad/s]
    height: tuple = (0.97, 1.05)     # [m]
    stand_probability: float = 0.1
    # Initial state, at each reset.
    # 0.3 m/s caused over half of all MPC-alone falls, within 2 s of the start, before any push.
    reset_linear: float = 0.1        # base velocity, +- per horizontal axis [m/s]
    reset_angular: float = 0.2       # base angular velocity, +- per axis [rad/s]
    joint_position_noise: float = 0.03   # [rad]
    joint_velocity_noise: float = 0.3    # [rad/s]
    # Dynamics the MPC does not model: friction (slippery down to 0.8 x 0.3 = 0.24), every link's
    # mass and inertia, a torso payload and its centre-of-mass offset.
    friction: tuple = (0.3, 1.0)     # sliding friction scale
    link_mass_scale: float = 0.1     # +- fraction
    payload: tuple = (-2.0, 6.0)     # [kg] on the torso
    payload_com: float = 0.05        # +- [m] per axis
    # Pushes during the episode: a horizontal force on the torso in a random direction. 600 N for
    # 0.1 s is 60 N s, i.e. 0.8 m/s on the 75.6 kg robot. rl/evaluate.py tests 150-600 N.
    push_interval: tuple = (4.0, 8.0)  # [s]
    push_force: tuple = (75.0, 600.0)  # [N]
    push_duration: float = 0.1         # [s]
    # Disturbance curriculum: one level d in [0, 1], shared by every environment, moves friction's
    # lower bound from `friction_easy` and the push force's upper bound from `push_force_easy` to
    # the ranges above. Every `disturbance_window` learning episodes, d rises a step if fewer than
    # `disturbance_up` of them fell and drops one if more than `disturbance_down` did. Off: d = 1.
    disturbance_curriculum: bool = True
    friction_easy: float = 0.6
    push_force_easy: float = 300.0     # [N]
    disturbance_window: int = 50
    disturbance_step: float = 0.1
    disturbance_up: float = 0.3
    disturbance_down: float = 0.5
    # Terrain curriculum (legged_gym): up a level after walking at least `promote_distance` of
    # the commanded distance without falling, down a level after a fall - unless the fall came
    # within `push_blame_seconds` of a push, which says nothing about the terrain.
    terrain: bool = True
    initial_max_level: int = 2
    promote_distance: float = 0.5
    push_blame_seconds: float = 2.0
    settle_seconds: float = 1.5      # standing after the first bring-up, before the snapshot
    fall_height: float = 0.6         # ground-truth pelvis height above the ground [m] (termination only)
    fall_tilt: float = math.pi / 6   # ground-truth roll / pitch [rad] (termination only; paper)
    reward: RewardConfig = field(default_factory=RewardConfig)


def tilt_from_quaternion(w, x, y, z):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return np.maximum(np.abs(roll), np.abs(pitch))


class H2GaitEnv:
    """Vectorized: reset() once, then step(actions) -> (obs, reward, terminated, truncated, active, info).

    An environment whose episode ended is reset by the simulator and the MPC in place; it is
    inactive (no transition) until the MPC has taken over again, normally the next step.
    """

    def __init__(self, config, log_dir, workspace, domain_id_base=100, stagger=3.0, seed=0,
                 estimator="humanoid_linear_kf", follower_envs=0):
        """follower_envs: the last K environments measure rather than learn (e.g. the MPC alone, or
        the deterministic policy). They take their terrain level from a random learning
        environment at each reset and do not move the disturbance level, so every group sees the
        learners' distribution."""
        from ament_index_python.packages import get_package_share_directory
        import yaml
        import sys
        self.cfg = config
        self.n = config.num_envs
        self.rng = np.random.default_rng(seed)
        self.follower = np.zeros(self.n, dtype=bool)
        if follower_envs > 0:
            self.follower[-follower_envs:] = True
        self.disturbance = 0.0 if config.disturbance_curriculum else 1.0
        self.start_levels = None  # terrain levels to start from (a resumed run), else random
        self.disturbance_outcomes = []
        share = Path(get_package_share_directory("legged_robot_mpc_controller"))
        reference = yaml.safe_load((share / "config/h2/ros2_controllers.yaml").read_text())[MPC]["ros__parameters"]["ocs2"]["reference"]
        self.scale = np.array([reference["maxDisplacementVelocityX"], reference["maxDisplacementVelocityY"],
                               reference["maxRotationVelocity"]])
        self.default_height = float(reference["defaultBaseHeight"])
        self.dt = 0.01
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        if config.terrain:
            from terrain import TerrainMap
            self.terrain = TerrainMap.training(seed=seed)
            mjb = compile_model(self.terrain.write_scene(log_dir), log_dir / "scene_terrain.mjb")
        else:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
            from mujoco_scene import write_scene
            self.terrain = None
            mjb = compile_model(write_scene(log_dir / "scene_flat.xml", "h2"), log_dir / "scene_flat.mjb")
        self.names = [f"rl_env_{i}" for i in range(self.n)]
        remove_stale(self.names)
        launch = [
            "legged_robot_mpc_controller", "robot.launch.py", "robotName:=h2", f"mpcControllerName:={MPC}",
            "rviz:=false", "baseCommandGui:=false", "mujoco_headless:=true", "visualizationNodes:=false",
            "mujoco_real_time_factor:=0.0", f"mujocoModelFile:={mjb}", "episodeControl:=true",
            "floatingBaseSource:=state_estimator", f"floatingBaseEstimator:={estimator}",
            f"controllersOverlay:={share / 'config/h2/rl_gait_training.yaml'}",
            "lockstepBridge:=lockstep_bridge", "lockstepShmName:=rl_env_{env}",
        ]
        self.launcher = MultiEnvLauncher(self.n, launch, log_dir, domain_id_base, stagger, cwd=str(workspace))
        self.log_dir = log_dir
        self.vec = None

    # ------------------------------------------------------------------ helpers
    def _env_alive(self, i):
        if not self.launcher.alive(i):
            return False
        with open(self.log_dir / f"env_{i}.log", "rb") as log:
            log.seek(0, 2)
            log.seek(max(0, log.tell() - 8192))
            return b"process has died" not in log.read()

    def _ix(self, name):
        return self.vec.index(name)

    def _sample_command(self, i):
        c = self.cfg
        if self.rng.random() < c.stand_probability:
            self.command[i] = [0.0, 0.0, 0.0, self.default_height]
        else:
            self.command[i] = [self.rng.uniform(*c.vx), self.rng.uniform(*c.vy), self.rng.uniform(*c.yaw_rate),
                               self.rng.uniform(*c.height)]

    def _write_command(self, i):
        vx, vy, wz, h = self.command[i]
        self.actions[i, self.a["walk"]] = [vx / self.scale[0], vy / self.scale[1], h, wz / self.scale[2]]

    def _a(self, name):
        return self.vec.act_names.index(name)

    def _ground(self, raw):
        if self.terrain is None:
            return np.zeros(len(raw))
        return self.terrain.height(raw[:, self.i_gt_x], raw[:, self.i_gt_y])

    def _begin_reset(self, i):
        c, a, rng = self.cfg, self.a, self.rng
        self.episode[i] += 1
        act = self.actions[i]
        act[a["mpc_episode"]] = self.episode[i]
        act[a["reset"]] = self.episode[i]
        if self.follower[i] and not self.follower.all():
            self.level[i] = self.level[rng.choice(np.flatnonzero(~self.follower))]
        if self.terrain is not None:
            self.column[i] = rng.integers(self.terrain.columns)
            x, y, z = self.terrain.spawn(int(self.level[i]), int(self.column[i]))
            act[a["position"]] = [x - self.home[i, 0], y - self.home[i, 1], z]
            self.spawn_xy[i] = [x, y]
        act[a["yaw"]] = rng.uniform(-math.pi, math.pi)
        act[a["linear_velocity"]] = [rng.uniform(-c.reset_linear, c.reset_linear),
                                     rng.uniform(-c.reset_linear, c.reset_linear), 0.0]
        act[a["angular_velocity"]] = rng.uniform(-c.reset_angular, c.reset_angular, 3)
        d = self.disturbance
        act[a["friction_scale"]] = rng.uniform(c.friction_easy + d * (c.friction[0] - c.friction_easy), c.friction[1])
        act[a["seed"]] = rng.integers(1, 2 ** 31)
        act[a["joint_position_noise"]] = c.joint_position_noise
        act[a["joint_velocity_noise"]] = c.joint_velocity_noise
        act[a["link_mass_scale"]] = c.link_mass_scale
        act[a["payload_mass"]] = rng.uniform(*c.payload)
        act[a["payload_com"]] = rng.uniform(-c.payload_com, c.payload_com, 3)
        act[a["gait"]] = 0.0
        self._sample_command(i)
        self._write_command(i)
        self.active[i] = False
        self.elapsed[i] = 0.0
        self.commanded_distance[i] = 0.0
        self.next_push[i] = rng.uniform(*c.push_interval)
        self.last_push[i] = -math.inf
        self.prev_action[i] = 0.0

    def _push(self, i):
        c, a, rng = self.cfg, self.a, self.rng
        heading = rng.uniform(-math.pi, math.pi)
        force = rng.uniform(c.push_force[0], c.push_force_easy + self.disturbance * (c.push_force[1] - c.push_force_easy))
        self.push_count[i] += 1
        self.actions[i, a["push"]] = self.push_count[i]
        self.actions[i, a["push_force"]] = [force * math.cos(heading), force * math.sin(heading), 0.0]
        self.actions[i, a["push_duration"]] = c.push_duration
        self.last_push[i] = self.elapsed[i]
        self.next_push[i] = self.elapsed[i] + rng.uniform(*c.push_interval)

    def _end_episode(self, i, fell, raw):
        """Terrain curriculum (legged_gym): returns what the episode reached, for logging."""
        walked = float(np.hypot(*(raw[i, [self.i_gt_x, self.i_gt_y]] - self.spawn_xy[i])))
        pushed = self.elapsed[i] - self.last_push[i] < self.cfg.push_blame_seconds
        record = {"level": int(self.level[i]), "terrain": None, "walked": walked, "after_push": bool(fell and pushed),
                  "disturbance": self.disturbance}
        if self.cfg.disturbance_curriculum and not self.follower[i]:
            self.disturbance_outcomes.append(fell)
            if len(self.disturbance_outcomes) >= self.cfg.disturbance_window:
                rate = float(np.mean(self.disturbance_outcomes))
                step = self.cfg.disturbance_step * ((rate < self.cfg.disturbance_up) - (rate > self.cfg.disturbance_down))
                self.disturbance = float(np.clip(self.disturbance + step, 0.0, 1.0))
                self.disturbance_outcomes = []
        if self.terrain is not None:
            record["terrain"] = self.terrain.tiles[(int(self.level[i]), int(self.column[i]))][0]
            top = self.terrain.cfg.levels - 1
            if fell:
                if not pushed:
                    self.level[i] = max(self.level[i] - 1, 0)
            elif walked >= self.cfg.promote_distance * self.commanded_distance[i]:
                # Past the top: a random level, so the hardest tiles are not all it ever sees.
                self.level[i] = self.rng.integers(top + 1) if self.level[i] >= top else self.level[i] + 1
        return record

    def _policy_obs(self, raw):
        return raw[:, self.i_obs]

    # ------------------------------------------------------------------ API
    def reset(self):
        """Start the simulations, run the bring-up, snapshot the settled stand, start every episode."""
        self.launcher.start()
        self.vec = LockstepVecEnv(self.names, timeout=900.0, alive=self._env_alive)
        ix = self._ix
        self.i_obs = np.array([ix(f"{MPC}/rl.obs.{k:02d}") for k in range(OBS_DIM)])
        r = f"{MPC}/rl.reward."
        self.i_vel_h = [ix(r + "heading_velocity.x"), ix(r + "heading_velocity.y")]
        self.i_vz, self.i_yaw = ix(r + "velocity.z"), ix(r + "yaw_rate")
        self.i_feet = [[ix(r + f"{side}_foot_velocity.{a}") for a in "xy"] for side in ("left", "right")]
        self.i_phase, self.i_episode = ix(f"{MPC}/status.bring_up_phase"), ix(f"{MPC}/status.episode")
        self.i_exceptions = ix(f"{MPC}/status.solver_exceptions")
        self.i_gt_x, self.i_gt_y, self.i_gt_z = ix("pelvis/position.x"), ix("pelvis/position.y"), ix("pelvis/position.z")
        self.i_gt_q = [ix(f"pelvis/orientation.{a}") for a in "wxyz"]
        self.i_sim_reset = ix("episode/reset")

        a = self._a
        self.a = {"walk": slice(a(f"{MPC}/walking_velocity_command.linear_velocity_x"), a(f"{MPC}/walking_velocity_command.linear_velocity_x") + 4),
                  "gait": slice(a(f"{MPC}/gait_adaptation.period_scale"), a(f"{MPC}/gait_adaptation.period_scale") + ACT_DIM),
                  "mpc_episode": a(f"{MPC}/episode.reset"), "snapshot": a("episode/snapshot"), "reset": a("episode/reset"),
                  "position": [a(f"episode/position.{k}") for k in "xyz"], "yaw": a("episode/yaw"),
                  "linear_velocity": [a(f"episode/linear_velocity.{k}") for k in "xyz"],
                  "angular_velocity": [a(f"episode/angular_velocity.{k}") for k in "xyz"],
                  "friction_scale": a("episode/friction_scale"), "seed": a("episode/seed"),
                  "joint_position_noise": a("episode/joint_position_noise"),
                  "joint_velocity_noise": a("episode/joint_velocity_noise"),
                  "link_mass_scale": a("episode/link_mass_scale"), "payload_mass": a("episode/payload_mass"),
                  "payload_com": [a(f"episode/payload_com.{k}") for k in "xyz"],
                  "push": a("episode/push"), "push_force": [a(f"episode/push_force.{k}") for k in "xyz"],
                  "push_duration": a("episode/push_duration")}
        n = self.n
        self.actions = np.zeros((n, len(self.vec.act_names)))
        self.actions[:, self.a["friction_scale"]] = 1.0
        self.command = np.zeros((n, 4))
        self.command[:, 3] = self.default_height
        for i in range(n):
            self._write_command(i)
        self.episode = np.zeros(n)
        self.active = np.zeros(n, dtype=bool)
        self.elapsed = np.zeros(n)
        self.prev_action = np.zeros((n, ACT_DIM))
        self.exceptions = np.zeros(n)
        if self.start_levels is not None and len(self.start_levels) == n:
            self.level = np.asarray(self.start_levels, dtype=int).copy()
        else:
            self.level = self.rng.integers(0, self.cfg.initial_max_level + 1, n)
        self.column = np.zeros(n, dtype=int)
        self.spawn_xy = np.zeros((n, 2))
        self.commanded_distance = np.zeros(n)
        self.next_push = np.zeros(n)
        self.last_push = np.full(n, -math.inf)
        self.push_count = np.zeros(n)

        raw = self.vec.first()
        settled = np.full(n, -1.0)
        start = time.monotonic()
        while (settled < 0).any() or (self.vec.times - settled < self.cfg.settle_seconds).any():
            phase = raw[:, self.i_phase]
            for i in range(n):
                if settled[i] < 0 and phase[i] == LOCOMOTION:
                    settled[i] = self.vec.times[i]
                if phase[i] == FAULT:
                    raise RuntimeError(f"environment {i} faulted during the bring-up; see {self.log_dir}/env_{i}.log")
            raw = self.vec.step(self.actions)
            if time.monotonic() - start > 1800:
                raise TimeoutError("bring-up did not finish")
        self.actions[:, self.a["snapshot"]] = 1.0
        raw = self.vec.step(self.actions)
        # Where the robot stood for the snapshot: spawns are offsets from it.
        self.home = raw[:, [self.i_gt_x, self.i_gt_y]].copy()
        for i in range(n):
            self._begin_reset(i)
        self.raw = raw
        return self._policy_obs(raw)

    def step(self, policy_actions, between=None):
        """policy_actions: (n, ACT_DIM) in [-1, 1], used for the environments active at the start of
        the step and held for cfg.action_repeat MPC ticks; the rewards are summed over those ticks.
        `between` runs while the simulations step, every tick (e.g. gradient updates). Returns obs,
        reward, terminated, truncated, was_active (the transitions to learn from) and an info dict.
        An environment whose episode ends inside the step returns the observation it ended at (the
        bootstrap state for a timeout) and is not taken over again before the next step."""
        n = self.n
        was_active = self.active.copy()
        live = was_active.copy()  # still in the episode the step started with
        actions = np.where(was_active[:, None], np.clip(policy_actions, -1.0, 1.0), 0.0)
        reward = np.zeros(n)
        terminated = np.zeros(n, dtype=bool)
        truncated = np.zeros(n, dtype=bool)
        ticks = np.zeros(n)
        terms_sum = {}
        episode_seconds = self.elapsed.copy()
        obs_out = None
        ended = {}
        for _ in range(self.cfg.action_repeat):
            obs, r, term, trunc, terms, seconds = self._tick(np.where(live[:, None], actions, 0.0), ended, between)
            update = live | ~was_active  # ended environments keep the observation they ended at
            obs_out = obs.copy() if obs_out is None else np.where(update[:, None], obs, obs_out)
            reward += np.where(live, r, 0.0)
            ticks += live
            for k, v in terms.items():
                terms_sum[k] = terms_sum.get(k, 0.0) + np.where(live, v, 0.0)
            done = term | trunc
            terminated |= term
            truncated |= trunc
            episode_seconds = np.where(done, seconds, episode_seconds)
            live &= ~done
        episode_seconds = np.where(terminated | truncated, episode_seconds, self.elapsed)
        terms = {k: (v / np.maximum(ticks, 1))[was_active] for k, v in terms_sum.items()}  # per tick
        info = {"terms": terms, "fell": terminated, "elapsed": episode_seconds,
                "ended": [ended[i] for i in sorted(ended)], "levels": self.level.copy(),
                "sync_violations": self.vec.sync_violations}
        return obs_out, reward, terminated, truncated, was_active, info

    def _tick(self, gait, ended, between):
        """One MPC tick with the gait actions `gait`: rewards, termination, the episode events. Fills
        `ended` (environment -> episode record) and never takes over an environment listed there."""
        c, w = self.cfg, self.cfg.reward
        was_active = self.active.copy()
        self.actions[:, self.a["gait"]] = gait
        self.vec.step_async(self.actions)
        if between is not None:
            between()
        raw = self.vec.step_wait()
        obs = self._policy_obs(raw)

        # Rewards, on-board only: robustness, then the paper's Table III terms.
        vel = raw[:, self.i_vel_h]
        lin = np.exp(-np.sum((vel - self.command[:, :2]) ** 2, axis=1) / w.sigma_lin ** 2)
        ang = np.exp(-(raw[:, self.i_yaw] - self.command[:, 2]) ** 2 / w.sigma_ang ** 2)
        height_error = obs[:, O_HEIGHT] - (self.command[:, 3] - self.default_height)
        height = np.exp(-height_error ** 2 / w.sigma_height ** 2)
        lin_z = raw[:, self.i_vz] ** 2
        ang_xy = np.sum(obs[:, O_ANG_VEL:O_ANG_VEL + 2] ** 2, axis=1)
        joint_vel = np.sum(obs[:, O_JOINT_VEL:O_JOINT_VEL + 25] ** 2, axis=1)
        action_rate = np.sum((gait - self.prev_action) ** 2, axis=1)
        slide = sum(obs[:, O_CONTACT + f] * np.sum(raw[:, self.i_feet[f]] ** 2, axis=1) for f in range(2))
        upright = np.sum(obs[:, O_GRAVITY:O_GRAVITY + 2] ** 2, axis=1)
        terms = {"alive": np.full(self.n, w.alive), "upright": w.upright * upright,
                 "lin_vel": w.lin_vel * lin, "ang_vel": w.ang_vel * ang, "height": w.height * height,
                 "lin_vel_z": w.lin_vel_z * lin_z, "ang_vel_xy": w.ang_vel_xy * ang_xy,
                 "joint_vel": w.joint_vel * joint_vel, "action_rate": w.action_rate * action_rate,
                 "feet_slide": w.feet_slide * slide}
        reward = sum(terms.values())

        # Episode ends: ground truth or the MPC's own fault, for termination only.
        tilt = tilt_from_quaternion(*(raw[:, k] for k in self.i_gt_q))
        exceptions = raw[:, self.i_exceptions]
        fell = (raw[:, self.i_gt_z] - self._ground(raw) < c.fall_height) | (tilt > c.fall_tilt) | \
            (raw[:, self.i_phase] == FAULT) | (exceptions > self.exceptions)
        self.exceptions = exceptions
        terminated = was_active & fell
        reward = np.where(terminated, reward - w.alive + w.termination, reward)
        self.elapsed += np.where(was_active, self.dt, 0.0)
        self.commanded_distance += np.where(was_active, np.hypot(self.command[:, 0], self.command[:, 1]) * self.dt, 0.0)
        truncated = was_active & ~terminated & (self.elapsed >= c.episode_seconds)
        self.prev_action = np.where(was_active[:, None], gait, self.prev_action)

        seconds = self.elapsed.copy()  # before the resets below zero it
        # Mid-episode command change and pushes, then resets and take-overs.
        for i in range(self.n):
            if was_active[i] and not (terminated[i] or truncated[i]):
                if abs(self.elapsed[i] - c.command_resample_seconds) < 0.5 * self.dt:
                    self._sample_command(i)
                    self._write_command(i)
                if self.elapsed[i] >= self.next_push[i]:
                    self._push(i)
            if terminated[i] or truncated[i]:
                ended[i] = self._end_episode(i, terminated[i], raw)
                self._begin_reset(i)
            elif not was_active[i] and i not in ended and raw[i, self.i_episode] == self.episode[i] and \
                    raw[i, self.i_phase] == LOCOMOTION and raw[i, self.i_sim_reset] == self.episode[i]:
                self.active[i] = True
        self.raw = raw
        return obs, reward, terminated, truncated, terms, seconds

    def close(self):
        if self.vec is not None:
            self.vec.close()
        self.launcher.stop()
