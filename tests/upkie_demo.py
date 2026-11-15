#!/usr/bin/env python3
"""Upkie demos: the kneeling stand-up, and joystick-style driving with gaits on the terrain course.

  standup  kneeling idle -> planned stand-up -> MPC, then stand at the default height
  course   the same stand-up on the flat start of the upkie_course terrain (uneven fields, ramps, a 2 cm step), then a
           seeded random joystick schedule: drive / jump-while-driving / drive / jump-while-driving (3.75 s each),
           walk 10 s, trot 10 s, stop. Each segment (1.2-2.5 s) is a pure translation, a pure rotation, both, or no
           motion; vx is forward with p 0.8 and the yaw sign steers back toward the course heading with p 0.7, like
           someone driving down the course. Each drive block goes to maxBaseHeight for 1 s and back to
           defaultBaseHeight; jumps, walk and trot stay at defaultBaseHeight.
  scan     list seeds whose nominal path (commands integrated as a unicycle) reaches every course feature

The wheel KF drives the MPC in closed loop by default (--source ground_truth_state for GT); GT is only logged.
Writes PREFIX.json (verdict and metrics), PREFIX_replay.json, PREFIX_labels.json (captions for the video) and, for
the course, PREFIX_schedule.json. Exit status 0 when the run passes (no fall).

  cd ~/ocs2_ros2_ws && source install/setup.bash
  python3 src/legged_robot_mpc_controller/tests/upkie_demo.py course --prefix /tmp/upkie_course
  python3 src/legged_robot_mpc_controller/tests/upkie_demo.py standup --prefix /tmp/upkie_standup
Videos (MuJoCo 3.13, Pillow, ffmpeg; e.g. on the host after copying the PREFIX_* files):
  python3 tests/tuning/render_upkie_video.py /tmp/upkie_course
  python3 tests/tuning/render_upkie_video.py /tmp/upkie_standup --start 3.9 --end 6.2 --speed 0.25
"""
import argparse
import json
import math
from pathlib import Path
import random
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
from mujoco_scene import write_scene  # noqa: E402

SETTLE = 4.0  # [s] after the MPC takes over: the catch and the rise to the default height
BLOCKS = [("drive", 3.75), ("jump", 3.75), ("drive", 3.75), ("jump", 3.75), ("walk", 10.0), ("trot", 10.0)]
RANGES = {"drive": (0.15, 0.45, 0.30, 1.00), "jump": (0.05, 0.25, 0.20, 0.60),  # |vx| min/max, |yaw rate| min/max
          "walk": (0.05, 0.20, 0.20, 0.60), "trot": (0.05, 0.20, 0.20, 0.60)}
KIND_TEXT = {"translation": "pure translation", "rotation": "pure rotation", "combined": "translation + rotation",
             "neutral": "no motion command", "stop": "stop"}
BLOCK_TEXT = {"drive": "Drive (wheel_drive), joystick", "jump": "Jump while driving (jump)", "walk": "Walk (walk)",
              "trot": "Trot (trot)", "stop": "Stop (wheel_drive)"}


def joystick_schedule(seed, default_height, max_height):
    rng = random.Random(seed)
    gaits, commands, blocks, t, heading = [], [], [], SETTLE, 0.0
    heights = [(SETTLE, default_height)]
    for gait, length in BLOCKS:
        gaits.append([round(t, 3), "" if gait == "drive" else gait])
        blocks.append([round(t, 3), gait])
        if gait == "drive":
            heights += [(round(t, 3), max_height), (round(t + 1.0, 3), default_height)]
        end = t + length
        while t < end - 1e-9:
            duration = min(rng.uniform(1.2, 2.5), end - t)
            if end - (t + duration) < 0.8:
                duration = end - t
            kind = rng.choices(["combined", "translation", "rotation", "neutral"], [0.45, 0.3, 0.15, 0.1])[0]
            vmin, vmax, wmin, wmax = RANGES[gait]
            vx = rng.uniform(vmin, vmax) * (1 if rng.random() < 0.8 else -1)
            steer = -math.copysign(1, heading) if abs(heading) > 0.05 else rng.choice([-1, 1])
            wz = rng.uniform(wmin, wmax) * (steer if rng.random() < 0.7 else -steer)
            vx, wz = {"translation": (vx, 0.0), "rotation": (0.0, wz), "neutral": (0.0, 0.0)}.get(kind, (vx, wz))
            commands.append([round(t, 3), round(vx, 3), round(wz, 3), kind])
            heading += 0.8 * wz * duration  # yaw tracking is about 80 %
            t += duration
    gaits.append([round(t, 3), ""])
    blocks.append([round(t, 3), "stop"])
    commands.append([round(t, 3), 0.0, 0.0, "stop"])
    sequence, kinds = [], []
    for time in sorted({c[0] for c in commands} | {h[0] for h in heights}):
        command = next(c for c in reversed(commands) if c[0] <= time)
        height = next(h for h in reversed(heights) if h[0] <= time)[1]
        sequence.append([time, command[1], command[2], height])
        kinds.append(command[3])
    return {"seed": seed, "end": round(t + 3.0, 3), "command_sequence": sequence, "kinds": kinds,
            "gait_sequence": gaits, "blocks": blocks}


def nominal_path(schedule, speed_ratio=0.85, yaw_ratio=0.8):
    sequence, x, y, heading, points = schedule["command_sequence"], 0.0, 0.0, 0.0, []
    for k, (t, vx, wz, _) in enumerate(sequence):
        t1 = sequence[k + 1][0] if k + 1 < len(sequence) else schedule["end"]
        steps = max(1, int((t1 - t) / 0.05))
        for _ in range(steps):
            dt = (t1 - t) / steps
            heading += yaw_ratio * wz * dt
            x += speed_ratio * vx * math.cos(heading) * dt
            y += speed_ratio * vx * math.sin(heading) * dt
            points.append((t, x, y))
    return points


def harness_command(args, prefix, extra):
    command = [sys.executable, str(ROOT / "tests/wb_id_walk_test.py"), "--robot-name", "upkie", "--prefix", str(prefix),
               "--mpc-rate", "200", "--record-joints", "--record-tracking", "--max-yaw-error", "100",
               "--minimum-height", "0.05", "--startup-timeout", "900", "--source", args.source,
               "--set", f'bringUp.standUpTrajectoryFile="{ROOT / "config/upkie/stand_up_trajectory.yaml"}"',
               "--launch-arg", f"initialPoseFile:={ROOT / 'config/upkie/initial_pose_idle.yaml'}"]
    if args.source == "state_estimator":
        command += ["--estimator", "humanoid_linear_kf"]
    return command + extra


def stand_up_labels(result, params, replay, source):
    takeover = result["takeover_time"]
    blend = params["bringUp"]["blendSeconds"]
    handover = yaml.safe_load((ROOT / "config/upkie/stand_up_trajectory.yaml").read_text())["stand_up"]["handover_time"]
    default = params["ocs2"]["reference"]["defaultBaseHeight"]
    driver = "estimator-driven, no ground truth" if source == "state_estimator" else "ground-truth base"
    phases = [[replay[0]["time"], "1  Idle: kneeling on knees and wheels"],
              [takeover - blend - handover, "2  Planned stand-up: offline plan played back open loop"],
              [takeover - blend, f"3  Handover: blending from the playback into the MPC ({blend:g} s)"],
              [takeover, f"4  MPC in control ({driver})"]]
    standing = next((s["time"] for s in replay if s["time"] > takeover and s["position"][2] >= default - 0.005), None)
    rate = params["ocs2"]["reference"].get("takeoverHeightRate", 0.0)
    subtitles = [[replay[0]["time"], "knees on the floor behind the wheels, trunk leaning back"],
                 [takeover - blend - handover, "fold the legs, then push the knees down: the trunk pivots forward over the axles"],
                 [takeover, f"catch, then rise from the measured height to {default:.2f} m at {rate:g} m/s"]]
    if standing is not None:
        subtitles.append([standing, f"standing at {default:.2f} m"])
    return phases, subtitles


def run_standup(args):
    params = load_params()
    default = params["ocs2"]["reference"]["defaultBaseHeight"]
    extra = ["--axis", "vx", "--speed", "0", "--settle", "4", "--ramp", "0", "--hold", str(args.stand_seconds),
             "--rest", "2", "--command-sequence", json.dumps([[0.0, 0.0, 0.0, default]])]
    status, result = run_harness(args, extra)
    if "takeover_time" in result:
        replay = json.loads(Path(f"{args.prefix}_replay.json").read_text())
        phases, subtitles = stand_up_labels(result, params, replay, args.source)
        write_labels(args.prefix, "none", replay[0]["time"] + 0.2, result["takeover_time"] + 3.9, phases, subtitles,
                     camera_distance=1.25, front_marker=True)
    return status


def run_course(args):
    params = load_params()
    reference = params["ocs2"]["reference"]
    schedule = joystick_schedule(args.seed, reference["defaultBaseHeight"], reference["maxBaseHeight"])
    Path(f"{args.prefix}_schedule.json").write_text(json.dumps(schedule, indent=1) + "\n")
    scene = write_scene(Path(f"{args.prefix}_scene.xml"), "upkie", "upkie_course")
    rest = 3.0
    extra = ["--axis", "vx", "--speed", "0", "--settle", str(SETTLE), "--ramp", "0",
             "--hold", f"{schedule['end'] - SETTLE - rest:g}", "--rest", f"{rest:g}", "--scene", str(scene),
             "--command-sequence", json.dumps([c for c in schedule["command_sequence"]]),
             "--gait-sequence", json.dumps(schedule["gait_sequence"])]
    status, result = run_harness(args, extra)
    if "takeover_time" in result:
        takeover = result["takeover_time"]
        driver = "wheel KF in the loop" if args.source == "state_estimator" else "ground-truth base"
        phases = [[takeover, f"Standing after the stand-up ({driver})"]]
        phases += [[takeover + t, BLOCK_TEXT[name] + (f"  |  seed {args.seed}" if name == "drive" else "")]
                   for t, name in schedule["blocks"]]
        subtitles = [[takeover + t, f"vx {vx:+.2f} m/s   yaw rate {wz:+.2f} rad/s   height {h:.2f} m   ({KIND_TEXT[kind]})"]
                     for (t, vx, wz, h), kind in zip(schedule["command_sequence"], schedule["kinds"])]
        write_labels(args.prefix, "upkie_course", takeover + SETTLE - 1.0, takeover + schedule["end"], phases, subtitles,
                     camera_distance=1.6, camera_elevation=-14.0, front_marker=False)
    return status


def run_harness(args, extra):
    command = harness_command(args, args.prefix, extra)
    print("running:", " ".join(command[:4]), "... (log:", f"{args.prefix}.out)", flush=True)
    with open(f"{args.prefix}.out", "w") as log:
        status = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
    result_path = Path(f"{args.prefix}.json")
    result = json.loads(result_path.read_text()) if result_path.is_file() else {}
    print(f"verdict: {result.get('verdict', 'no result')}  min z {result.get('min_z', float('nan')):.3f} m  "
          f"max pitch/roll {result.get('max_pitch', float('nan')):.3f}/{result.get('max_roll', float('nan')):.3f} rad "
          "(includes the stand-up)", flush=True)
    return status, result


def write_labels(prefix, terrain, start, end, phases, subtitles, **camera):
    labels = dict(terrain=terrain, start=round(start, 3), end=round(end, 3), phases=phases, subtitles=subtitles, **camera)
    Path(f"{prefix}_labels.json").write_text(json.dumps(labels, indent=1) + "\n")


def load_params():
    config = yaml.safe_load((ROOT / "config/upkie/ros2_controllers.yaml").read_text())
    return config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]


def scan(args):
    reference = load_params()["ocs2"]["reference"]
    print("seed  x@jump1 x@walk x@trot  max x  min x  max|y|  (course: uneven 0.5-1.5, ramp 1.6-2.4, step 2.9-3.5 m)")
    for seed in range(1, args.scan + 1):
        schedule = joystick_schedule(seed, reference["defaultBaseHeight"], reference["maxBaseHeight"])
        points = nominal_path(schedule)
        at = lambda block: next(p[1] for p in points if p[0] >= dict((n, t) for t, n in reversed(schedule["blocks"]))[block] - 1e-9)
        xs, ys = [p[1] for p in points], [abs(p[2]) for p in points]
        if max(xs) >= 3.2 and min(xs) > -1.6 and max(ys) < 1.8 and 0.9 <= at("walk") <= 2.6:
            print(f"{seed:4d}  {at('jump'):6.2f} {at('walk'):6.2f} {at('trot'):6.2f} {max(xs):6.2f} {min(xs):6.2f} {max(ys):6.2f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("demo", choices=["standup", "course", "scan"])
    parser.add_argument("--prefix", default=None, help="output prefix (default /tmp/upkie_<demo>)")
    parser.add_argument("--seed", type=int, default=216, help="joystick schedule seed (course); 216 crosses every feature")
    parser.add_argument("--source", choices=["state_estimator", "ground_truth_state"], default="state_estimator")
    parser.add_argument("--stand-seconds", type=float, default=4.0, help="standing time after the settle (standup)")
    parser.add_argument("--scan", type=int, default=400, help="number of seeds to scan (scan)")
    args = parser.parse_args()
    args.prefix = args.prefix or f"/tmp/upkie_{args.demo}"
    if args.demo == "scan":
        return scan(args)
    Path(args.prefix).parent.mkdir(parents=True, exist_ok=True)
    return run_standup(args) if args.demo == "standup" else run_course(args)


if __name__ == "__main__":
    raise SystemExit(main())
