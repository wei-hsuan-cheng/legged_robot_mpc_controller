#!/usr/bin/env python3
"""Upkie demos: the kneeling stand-up, and a locomotion test on the terrain course.

  standup  kneeling idle -> planned stand-up -> MPC, then stand at the default height
  course   the same stand-up on the flat start of the upkie_course terrain, then, at the default height unless stated:
             1. height test standing still: maxBaseHeight, minBaseHeight, back to defaultBaseHeight (2 s each)
             2. ramp pass, straight along y = 0 at 0.3 m/s: the uneven field, the full-width ramp, plateau, 1 cm step
                and ramp down (both wheels), then the narrow ramp under one wheel (the other stays on the floor). The
                stand-up catch leaves the heading off by up to about 0.2 rad, so the harness steers this pass like an
                operator, holding the spawn line from the robot's own state estimate (wb_id_walk_test line hold)
             3. random joystick play, pure wheel_drive, on the floor beyond for 24 s, then stop. Each 1.2-2.5 s
                segment is a pure translation, a pure rotation, coupled motion (both) or no motion; vx is forward
                with p 0.8. The video covers 60 s from standing. (The jump, walk and trot gaits are left out.)
  scan     list seeds whose play part has at least two each of pure rotation, pure translation and coupled motion,
           turns both ways and drives in reverse at least once

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
from pathlib import Path
import random
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
from mujoco_scene import write_scene  # noqa: E402

SETTLE = 4.0  # [s] after the MPC takes over: the catch and the rise to the default height
HEIGHT_TEST = 2.0  # [s] standing still at each of maxBaseHeight, minBaseHeight and defaultBaseHeight
RAMP_PASS_SPEED, RAMP_PASS_SECONDS = 0.3, 26.0  # from about x = -0.2 m to past the one-wheel ramp (ends at x = 7.0 m)
PLAY_SECONDS = 24.0
VX_RANGE, YAW_RATE_RANGE = (0.15, 0.40), (0.30, 1.00)  # |vx| [m/s] and |yaw rate| [rad/s] of the random segments
SEED = 6  # default play seed (see scan)
KIND_TEXT = {"line": "pure translation along the course line", "translation": "pure translation",
             "rotation": "pure rotation", "coupled": "coupled motion",
             "neutral": "no motion command", "height": "height test, standing still", "stop": "stop"}
BLOCK_TEXT = {"drive": "Random joystick (wheel_drive)", "stop": "Stop (wheel_drive)"}


def course_schedule(seed, reference):
    default = reference["defaultBaseHeight"]
    commands = [[SETTLE + i * HEIGHT_TEST, 0.0, 0.0, reference[key], "height"]
                for i, key in enumerate(("maxBaseHeight", "minBaseHeight", "defaultBaseHeight"))]
    t = SETTLE + 3 * HEIGHT_TEST
    commands.append([t, RAMP_PASS_SPEED, 0.0, default, "line"])
    gaits, blocks = [[SETTLE, ""]], [[SETTLE, "height"], [t, "ramp"]]
    t += RAMP_PASS_SECONDS
    rng = random.Random(seed)
    gaits.append([round(t, 3), ""])
    blocks.append([round(t, 3), "drive"])
    end = t + PLAY_SECONDS
    while t < end - 1e-9:
        duration = min(rng.uniform(1.2, 2.5), end - t)
        if end - (t + duration) < 0.8:
            duration = end - t
        kind = rng.choices(["coupled", "translation", "rotation", "neutral"], [0.4, 0.3, 0.2, 0.1])[0]
        vx = rng.uniform(*VX_RANGE) * (1 if rng.random() < 0.8 else -1)
        wz = rng.uniform(*YAW_RATE_RANGE) * rng.choice([-1.0, 1.0])
        vx, wz = {"translation": (vx, 0.0), "rotation": (0.0, wz), "neutral": (0.0, 0.0)}.get(kind, (vx, wz))
        commands.append([round(t, 3), round(vx, 3), round(wz, 3), default, kind])
        t += duration
    gaits.append([round(t, 3), ""])
    blocks.append([round(t, 3), "stop"])
    commands.append([round(t, 3), 0.0, 0.0, default, "stop"])
    return {"seed": seed, "end": round(t + 3.0, 3), "command_sequence": [c[:4] + [1] * (c[4] == "line") for c in commands],
            "kinds": [c[4] for c in commands], "gait_sequence": gaits, "blocks": blocks}


def harness_command(args, prefix, extra):
    command = [sys.executable, str(ROOT / "tests/wb_id_walk_test.py"), "--robot-name", "upkie", "--prefix", str(prefix),
               "--mpc-rate", "200", "--record-joints", "--record-tracking", "--max-yaw-error", "100",
               "--minimum-height", "0.05", "--startup-timeout", "900", "--source", args.source]
    # The kneeling spawn and planned stand-up are Upkie's launch defaults.
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


def ramp_pass_labels(replay, begin, end):
    """Phase captions of the ramp pass from where the robot actually was (base x along the course)."""
    course = yaml.safe_load((ROOT / "config/terrains/upkie/course_params.yaml").read_text())["upkie_course"]
    ramps = {r["name"]: r["x"] for r in course["ramps"]}
    field = next(f["x"] for f in course["uneven_fields"] if f["name"] == "uneven_front")
    marks = [(field[0], "Ramp pass: uneven field"),
             (ramps["ramp_up"][0], "Ramp pass: both wheels on the ramp, plateau and 1 cm step"),
             (ramps["ramp_down"][1], "Ramp pass: flat floor"),
             (ramps["one_wheel_ramp_up"][0], "Ramp pass: one wheel on the ramp, the other on the flat floor"),
             (ramps["one_wheel_ramp_down"][1], "Ramp pass: flat floor")]
    phases = [[begin, "Ramp pass: straight ahead"]]
    for x_mark, text in marks:
        when = next((s["time"] for s in replay if begin <= s["time"] <= end and s["position"][0] >= x_mark), None)
        if when is not None:
            phases.append([when, text])
    return phases


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
                     camera_distance=1.25)
    return status


def run_course(args):
    params = load_params()
    reference = params["ocs2"]["reference"]
    schedule = course_schedule(args.seed, reference)
    Path(f"{args.prefix}_schedule.json").write_text(json.dumps(schedule, indent=1) + "\n")
    scene = write_scene(Path(f"{args.prefix}_scene.xml"), "upkie", "upkie_course")
    rest = 3.0
    extra = ["--axis", "vx", "--speed", "0", "--settle", str(SETTLE), "--ramp", "0",
             "--hold", f"{schedule['end'] - SETTLE - rest:g}", "--rest", f"{rest:g}", "--scene", str(scene),
             "--command-sequence", json.dumps(schedule["command_sequence"]),
             "--gait-sequence", json.dumps(schedule["gait_sequence"])]
    status, result = run_harness(args, extra)
    if "takeover_time" in result:
        takeover = result["takeover_time"]
        replay = json.loads(Path(f"{args.prefix}_replay.json").read_text())
        driver = "wheel KF in the loop" if args.source == "state_estimator" else "ground-truth base"
        phases = [[takeover, f"Standing after the stand-up ({driver})"]]
        for (t, name), (t_next, _) in zip(schedule["blocks"], schedule["blocks"][1:] + [[schedule["end"], ""]]):
            if name == "height":
                phases.append([takeover + t, "Height test (wheel_drive): max, min, default"])
            elif name == "ramp":
                phases += ramp_pass_labels(replay, takeover + t, takeover + t_next)
            else:
                phases.append([takeover + t, BLOCK_TEXT[name] + (f"  |  seed {args.seed}" if name != "stop" else "")])
        yaw_text = lambda wz, kind: "yaw rate: line hold" if kind == "line" else f"yaw rate {wz:+.2f} rad/s"
        subtitles = [[takeover + t, f"vx {vx:+.2f} m/s   {yaw_text(wz, kind)}   height {h:.2f} m   ({KIND_TEXT[kind]})"]
                     for (t, vx, wz, h, *_), kind in zip(schedule["command_sequence"], schedule["kinds"])]
        write_labels(args.prefix, "upkie_course", takeover + SETTLE - 1.0, takeover + SETTLE + 59.0, phases, subtitles,
                     camera_distance=1.6, camera_elevation=-14.0)
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
    print("seed  pure rotation / pure translation / coupled segments in the play part")
    for seed in range(1, args.scan + 1):
        schedule = course_schedule(seed, reference)
        play = [(c, k) for c, k in zip(schedule["command_sequence"], schedule["kinds"]) if c[0] > SETTLE + 3 * HEIGHT_TEST]
        counts = [sum(k == kind for _, k in play) for kind in ("rotation", "translation", "coupled")]
        both_turns = any(c[2] > 0 for c, _ in play) and any(c[2] < 0 for c, _ in play)
        if min(counts) >= 2 and both_turns and any(c[1] < 0 for c, _ in play):
            print(f"{seed:4d}  {counts[0]:13d} {counts[1]:16d} {counts[2]:7d}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("demo", choices=["standup", "course", "scan"])
    parser.add_argument("--prefix", default=None, help="output prefix (default /tmp/upkie_<demo>)")
    parser.add_argument("--seed", type=int, default=SEED, help=f"random play seed (course), default {SEED}")
    parser.add_argument("--source", choices=["state_estimator", "ground_truth_state"], default="state_estimator")
    parser.add_argument("--stand-seconds", type=float, default=4.0, help="standing time after the settle (standup)")
    parser.add_argument("--scan", type=int, default=200, help="number of seeds to scan (scan)")
    args = parser.parse_args()
    args.prefix = args.prefix or f"/tmp/upkie_{args.demo}"
    if args.demo == "scan":
        return scan(args)
    Path(args.prefix).parent.mkdir(parents=True, exist_ok=True)
    return run_standup(args) if args.demo == "standup" else run_course(args)


if __name__ == "__main__":
    raise SystemExit(main())
