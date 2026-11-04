#!/usr/bin/env python3
"""N whole-body ID MPC simulations in parallel, state estimator in the loop, in lockstep.

Starts N copies of robot.launch.py through mujoco_ros2_control_multienv - one ROS domain each,
the MPC solving synchronously in controller time, a lockstep bridge on the MPC's references -
and steps all of them together from the first physics tick: bring-up (stand-up, estimator
initialization, MPC start) and then a walking sequence commanded through the bridge, with no
topic anywhere in the loop. Checks, per environment:

  - it reaches locomotion and stays up (ground-truth pelvis height and tilt, no FAULT);
  - lockstep timing: step k observed at exactly k * step_period of controller time and the
    bridge saw every answer for the step it was waiting on;
  - one MPC solve per step during locomotion, each from the same fixed observation age;
  - the estimate against ground truth.

Run from a sourced workspace (the CppAD cache folder is relative to --workspace):
  python3 tests/lockstep_multi_env_test.py --num-envs 8
"""

import argparse
import json
import math
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from mujoco_ros2_control_multienv import LockstepClosed, LockstepVecEnv, MultiEnvLauncher, compile_model, remove_stale

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mujoco_scene import write_scene  # noqa: E402

CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
LOCOMOTION, FAULT = 6, 7
# Per environment, commanded through its lockstep bridge: stand, walk forward, stop, turn, stop.
# Speeds differ across environments, so each one's travel shows its own commands reached it;
# environments with the same commands must come out identical.
SPEEDS = [0.1, 0.2, 0.3, 0.4]  # [m/s]
YAW_RATE = 0.3  # [rad/s], sign alternating


def sequence(env):
    """(seconds after the env reaches locomotion, vx [m/s], yaw rate [rad/s]); the last entry ends the run."""
    vx = SPEEDS[env % len(SPEEDS)]
    yaw = YAW_RATE if (env // len(SPEEDS)) % 2 == 0 else -YAW_RATE
    return [(0.0, 0.0, 0.0), (2.0, vx, 0.0), (8.0, 0.0, 0.0), (11.0, 0.0, yaw), (16.0, 0.0, 0.0), (19.0, None, None)]


def command_group(env):
    return env % (2 * len(SPEEDS))


def quat_tilt(w, x, y, z):
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))


def rotate(w, x, y, z, v):
    """Rotate v by the unit quaternion (w, x, y, z)."""
    q = np.array([x, y, z])
    return v + 2.0 * np.cross(q, np.cross(q, v) + w * v)


def private_memory_mb():
    """Proportional set size of every simulation process, and the machine's available memory."""
    pss = 0
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            if "mujoco_ros2_con" not in Path(f"/proc/{pid}/comm").read_text():
                continue
            match = re.search(r"^Pss:\s+(\d+)", Path(f"/proc/{pid}/smaps_rollup").read_text(), re.M)
            pss += int(match.group(1)) if match else 0
        except OSError:
            continue
    available = int(re.search(r"MemAvailable:\s+(\d+)", Path("/proc/meminfo").read_text()).group(1))
    return pss / 1024.0, available / 1024.0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--robot", default="h2", choices=["h2"])
    parser.add_argument("--estimator", default="humanoid_linear_kf", choices=["humanoid_linear_kf", "humanoid_inekf"])
    parser.add_argument("--domain-id-base", type=int, default=100)
    parser.add_argument("--stagger", type=float, default=4.0)
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/lockstep_multi_env"))
    parser.add_argument("--workspace", type=Path, default=Path.home() / "ocs2_ros2_ws")
    parser.add_argument("--timeout", type=float, default=3600.0, help="wall-clock limit [s]")
    args = parser.parse_args()

    share = Path(get_package_share_directory("legged_robot_mpc_controller"))
    config = yaml.safe_load((share / "config" / args.robot / "ros2_controllers.yaml").read_text())
    reference = config[CONTROLLER]["ros__parameters"]["ocs2"]["reference"]
    height = float(reference["defaultBaseHeight"])
    vx_scale, yaw_scale = float(reference["maxDisplacementVelocityX"]), float(reference["maxRotationVelocity"])

    args.log_dir.mkdir(parents=True, exist_ok=True)
    mjb = compile_model(write_scene(args.log_dir / "scene_flat.xml", args.robot), args.log_dir / "scene_flat.mjb")
    names = [f"legged_env_{i}" for i in range(args.num_envs)]
    remove_stale(names)
    launch = [
        "legged_robot_mpc_controller", "robot.launch.py", f"robotName:={args.robot}", f"mpcControllerName:={CONTROLLER}",
        "rviz:=false", "baseCommandGui:=false", "mujoco_headless:=true", "visualizationNodes:=false",
        "mujoco_real_time_factor:=0.0", f"mujocoModelFile:={mjb}",
        "floatingBaseSource:=state_estimator", f"floatingBaseEstimator:={args.estimator}",
        f"controllersOverlay:={share / 'config' / args.robot / 'lockstep_training.yaml'}",
        "lockstepBridge:=lockstep_bridge", "lockstepShmName:=legged_env_{env}",
    ]

    launcher = MultiEnvLauncher(args.num_envs, launch, args.log_dir, args.domain_id_base, args.stagger,
                                cwd=str(args.workspace))
    wall_start = time.monotonic()
    result = {"num_envs": args.num_envs, "estimator": args.estimator, "verdict": "ERROR"}
    def env_alive(i):
        """The launch is up and none of its processes died (a failed controller sequence does not
        take the launch down, and the simulation would otherwise wait to start forever)."""
        if not launcher.alive(i):
            return False
        with open(args.log_dir / f"env_{i}.log", "rb") as log:
            log.seek(0, os.SEEK_END)
            log.seek(max(0, log.tell() - 8192))
            return b"process has died" not in log.read()

    try:
        launcher.start()
        vec = LockstepVecEnv(names, timeout=600.0, alive=env_alive)
        print(f"{args.num_envs} environments attached after {time.monotonic() - wall_start:.1f} s; "
              f"{len(vec.obs_names)} observations, {len(vec.act_names)} actions, step {vec.step_period} s", flush=True)
        ix = {name: vec.index(name) for name in vec.obs_names}
        est = args.estimator
        n = args.num_envs
        actions = np.tile([0.0, 0.0, height, 0.0], (n, 1))  # stand: zero velocity, nominal height

        locomotion_since = np.full(n, np.nan)
        done = np.zeros(n, dtype=bool)
        failed = [""] * n
        solves_prev = np.full(n, np.nan)
        solve_gaps = [[] for _ in range(n)]          # solves per step during locomotion
        solve_age = [set() for _ in range(n)]        # observation time - latest solve's observation time
        est_err_z, est_err_v = [[] for _ in range(n)], [[] for _ in range(n)]
        travelled = np.zeros(n)
        start_xy = [None] * n
        max_tilt = np.zeros(n)
        min_height = np.full(n, np.inf)
        advance_ms = [[] for _ in range(n)]
        peak_pss, min_available = 0.0, float("inf")
        next_memory = 0.0
        sim_start = None
        steps = 0

        obs = vec.first()
        sim_start = vec.times.copy()
        while not done.all():
            if time.monotonic() - wall_start > args.timeout:
                raise TimeoutError("wall-clock limit reached")
            t = vec.times
            for i in range(n):
                o = obs[i]
                phase = int(o[ix[f"{CONTROLLER}/status.bring_up_phase"]])
                q = [o[ix[f"pelvis/orientation.{a}"]] for a in "wxyz"]
                z = o[ix["pelvis/position.z"]]
                if steps > 0:
                    max_tilt[i] = max(max_tilt[i], quat_tilt(*q))
                if phase == FAULT and not failed[i]:
                    failed[i] = f"FAULT at t={t[i]:.2f} s"
                if not math.isnan(locomotion_since[i]) and (z < 0.6 * height or quat_tilt(*q) > 0.8) and not failed[i]:
                    failed[i] = f"fell at t={t[i]:.2f} s (z={z:.2f} m, tilt={quat_tilt(*q):.2f} rad)"
                if failed[i]:
                    done[i] = True
                    continue
                if phase == LOCOMOTION and math.isnan(locomotion_since[i]):
                    locomotion_since[i] = t[i]
                    start_xy[i] = np.array([o[ix["pelvis/position.x"]], o[ix["pelvis/position.y"]]])
                if math.isnan(locomotion_since[i]):
                    continue
                min_height[i] = min(min_height[i], z)
                elapsed = t[i] - locomotion_since[i]
                segment = [s for s in sequence(i) if s[0] <= elapsed + 1e-9][-1]
                if segment[1] is None:
                    done[i] = True
                    travelled[i] = float(np.linalg.norm(
                        np.array([o[ix["pelvis/position.x"]], o[ix["pelvis/position.y"]]]) - start_xy[i]))
                    continue
                actions[i] = [segment[1] / vx_scale, 0.0, height, segment[2] / yaw_scale]
                solves = o[ix[f"{CONTROLLER}/status.solve_count"]]
                if not math.isnan(solves_prev[i]):
                    solve_gaps[i].append(solves - solves_prev[i])
                solves_prev[i] = solves
                solve_age[i].add(round(t[i] - o[ix[f"{CONTROLLER}/status.last_solve_time"]], 6))
                advance_ms[i].append(o[ix[f"{CONTROLLER}/status.last_solve_wall_ms"]])
                if o[ix[f"{est}/valid"]] > 0.5:
                    est_err_z[i].append(abs(o[ix[f"{est}/position.z"]] - z))
                    v_gt = rotate(*q, np.array([o[ix[f"pelvis/linear_velocity.{a}"]] for a in "xyz"]))
                    v_est = np.array([o[ix[f"{est}/linear_velocity_world.{a}"]] for a in "xyz"])
                    est_err_v[i].append(float(np.linalg.norm(v_est - v_gt)))
            if time.monotonic() > next_memory:
                pss, available = private_memory_mb()
                peak_pss, min_available = max(peak_pss, pss), min(min_available, available)
                next_memory = time.monotonic() + 5.0
                print(f"  wall {time.monotonic() - wall_start:6.0f} s | controller time {np.min(t):6.2f}-{np.max(t):6.2f} s"
                      f" | locomotion {int(np.sum(~np.isnan(locomotion_since)))}/{n} | done {int(done.sum())}/{n}"
                      f" | simulations {pss:.0f} MB PSS, {available:.0f} MB available", flush=True)
            obs = vec.step(actions)
            steps += 1

        wall = time.monotonic() - wall_start
        sim = vec.times - sim_start
        envs = []
        for i in range(n):
            gaps = np.array(solve_gaps[i])
            envs.append({
                "env": i,
                "vx": sequence(i)[1][1], "yaw_rate": sequence(i)[3][2],
                "result": failed[i] or "survived",
                "locomotion_at": None if math.isnan(locomotion_since[i]) else round(float(locomotion_since[i]), 3),
                "travelled_m": round(float(travelled[i]), 2),
                "max_tilt_rad": round(float(max_tilt[i]), 3),
                "min_height_m": round(float(min_height[i]), 3) if np.isfinite(min_height[i]) else None,
                "solves_per_step": sorted(set(gaps.tolist())) if gaps.size else [],
                "observation_age_s": sorted(solve_age[i]),
                "solve_ms_mean": round(float(np.nanmean(advance_ms[i])), 2) if advance_ms[i] else None,
                "estimator_z_err_mean_m": round(float(np.mean(est_err_z[i])), 4) if est_err_z[i] else None,
                "estimator_v_err_rms": round(float(np.sqrt(np.mean(np.square(est_err_v[i])))), 4) if est_err_v[i] else None,
            })
        survived = sum(e["result"] == "survived" for e in envs)
        # Same commands, same physics, same synchronous solves: everything but the wall-clock solve time must match.
        groups = {}
        for e in envs:
            groups.setdefault(command_group(e["env"]), []).append({k: v for k, v in e.items() if k not in ("env", "solve_ms_mean")})
        deterministic = all(all(member == members[0] for member in members) for members in groups.values())
        timing_ok = vec.sync_violations == 0 and vec.bridge_sync_errors == 0 and all(
            e["solves_per_step"] == [1.0] and len(e["observation_age_s"]) == 1 for e in envs if e["result"] == "survived")
        result.update({
            "verdict": "PASS" if survived == n and timing_ok and deterministic else "FAIL",
            "identical_within_command_groups": deterministic,
            "survived": survived, "steps": steps, "wall_s": round(wall, 1),
            "controller_time_s": round(float(np.mean(sim)), 2),
            "aggregate_real_time_factor": round(float(np.sum(sim)) / wall, 3),
            "per_env_real_time_factor": round(float(np.mean(sim)) / wall, 3),
            "client_sync_violations": vec.sync_violations, "bridge_sync_errors": vec.bridge_sync_errors,
            "violation_examples": vec.violation_examples,
            "handshake_tick_offset": vec.channels[0].handshake_tick_offset,
            "peak_simulation_pss_mb": round(peak_pss), "min_available_mb": round(min_available),
            "envs": envs,
        })
        vec.close()
    except (LockstepClosed, TimeoutError) as error:
        result["error"] = str(error)
        result["dead_envs"] = [i for i in range(args.num_envs) if launcher.processes and not env_alive(i)]
    finally:
        launcher.stop()
    (args.log_dir / "result.json").write_text(json.dumps(result, indent=2))
    for env in result.get("envs", []):
        print(env)
    print(json.dumps({k: v for k, v in result.items() if k != "envs"}, indent=2))
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
