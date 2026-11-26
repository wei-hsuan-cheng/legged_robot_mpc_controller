#!/usr/bin/env python3
"""Pineapple robustness suite: graded pushes, terrain and agile commands, each a wb_id_walk_test.py run.

Every case starts from the seated spawn and the planned stand-up with the wheel-aware linear KF driving the MPC (the
default bring-up). Cases run in parallel, one ROS domain each, and the summary (verdict, worst tilt, lowest base,
distance) is written to OUT/TAG/summary.md and summary.json.

  python3 tests/tuning/pineapple_suite.py --tag baseline                       # every case
  python3 tests/tuning/pineapple_suite.py --tag t1 --cases 'push_*' --set ocs2.costs.q.basePose='[0,0,3000,0,800,2000]'
  python3 tests/tuning/pineapple_suite.py --list

Run it in the container from a sourced workspace (the harness launches robot.launch.py).
"""

import argparse
import concurrent.futures
import csv
import fnmatch
import json
import math
import os
import queue
import re
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
HARNESS = ROOT / "tests/wb_id_walk_test.py"
PUSHES = ROOT / "config/tests/pineapple"
TERRAINS = ROOT / "config/terrains/pineapple"
H = 0.36  # default height


def stand(push, hold):
    """Standing still at the default height, the pushes from 4 s after the takeover."""
    return dict(args=["--command-sequence", json.dumps([[0, 0, 0, H]]), "--hold", str(hold),
                      "--wrench-sequence", str(PUSHES / push), "--wrench-delay", "4"])


def drive(push, hold, speed=0.5):
    """Driving straight at `speed` (holding the spawn line), the pushes from 4 s after the takeover."""
    return dict(args=["--command-sequence", json.dumps([[1, speed, 0, H, 1]]), "--hold", str(hold),
                      "--wrench-sequence", str(PUSHES / push), "--wrench-delay", "4"])


def terrain(params, speed, until, past=None):
    """Drive along the spawn line at `speed` from 2 s to `until` s, then stop; `past`: the forward distance [m] that
    clears the obstacle (shorter is STUCK)."""
    commands = json.dumps([[2, speed, 0, H, 1], [until, 0, 0, H]])
    return dict(terrain=params, past=past, args=["--command-sequence", commands, "--hold", str(until + 1)])


def gait(name, speed=0.0, start=4, stop=12, hold=14, params=None, scene=None, past=None):
    """Request the gait template `name` from `start` to `stop` s (standing, or rolling at `speed` along the spawn line)."""
    commands = json.dumps([[1, speed, 0, H, 1]] if speed else [[0, 0, 0, H]])
    case = dict(past=past, args=["--command-sequence", commands, "--gait-sequence", json.dumps([[start, name], [stop, ""]]),
                                 "--hold", str(hold)])
    if params:
        case["terrain"] = params
    if scene:
        case["scene"] = scene
    return case


def lateral(points, hold):
    """Sideways (and combined) commands: [time, vx, vy, yaw rate] from the takeover, at the default height."""
    return dict(args=["--command-sequence", json.dumps([{"time": t, "vx": vx, "vy": vy, "yaw_rate": wz, "height": H}
                                                        for t, vx, vy, wz in points]), "--hold", str(hold)])


CASES = {
    "push_x60": stand("push_x60.yaml", 10), "push_x100": stand("push_x100.yaml", 10),
    "push_x140": stand("push_x140.yaml", 10),
    "push_y30": stand("push_y30.yaml", 10), "push_y50": stand("push_y50.yaml", 10),
    "push_y80": stand("push_y80.yaml", 10),
    "push_torque": stand("push_torque.yaml", 23),
    "drive_push_x100": drive("push_x100.yaml", 10), "drive_push_y50": drive("push_y50.yaml", 10),
    "drive_push_mixed": drive("push_mixed.yaml", 17),
    "step20": terrain("step20_params.yaml", 0.4, 14, past=2.0), "step30": terrain("step30_params.yaml", 0.4, 14, past=2.0),
    "step40": terrain("step40_params.yaml", 0.4, 14, past=2.0), "step30_fast": terrain("step30_params.yaml", 0.8, 8, past=2.0),
    "slope12": terrain("slope12_params.yaml", 0.4, 16),
    "extreme_course": terrain("extreme_course_params.yaml", 0.4, 25, past=6.0),
    "extreme_course_fast": terrain("extreme_course_params.yaml", 0.7, 15, past=6.0),
    "upkie_course_fast": dict(scene="scene_course.xml", args=[
        "--command-sequence", json.dumps([[2, 0.8, 0, H, 1], [13, 0, 0, H]]), "--hold", "14"]),
    # Full-speed drive, reversals, spins and height swings while moving (vx, yaw rate at the command limits).
    "agile": dict(args=["--command-sequence", json.dumps([
        [1, 1.0, 0, H], [4, -1.0, 0, H], [7, 0, 3.14, H], [9, 0, -3.14, 0.28], [11, 0.8, 2.0, 0.44],
        [14, 1.0, 0, 0.25], [17, -0.8, -1.5, 0.42], [20, 0.6, 3.0, 0.30], [22, 0, 0, H]]), "--hold", "24"]),
    # Gaits (ocs2.gait.templates), requested on /humanoid/gait_command like the joystick does.
    "jump_place": gait("jump"), "jump_drive": gait("jump", 0.3),
    "walk_place": gait("walk"), "walk_drive": gait("walk", 0.3),
    "trot_place": gait("trot"), "trot_drive": gait("trot", 0.3),
    # The 40 mm step wheel_drive cannot climb slowly (docs/pineapple.md), walked and jumped onto at 0.3 m/s.
    "walk_step40": gait("walk", 0.3, start=2, stop=14, hold=15, params="step40_params.yaml",
                        past=2.0),
    "jump_step40": gait("jump", 0.3, start=2, stop=14, hold=15, params="step40_params.yaml",
                        past=2.0),
    # Sideways: vy selects the stepping gait (ocs2.gait.transitions), then vy with vx and with turning.
    # Gaits under pushes, on the rough course, while turning and switched while driving.
    "trot_push": dict(args=["--command-sequence", json.dumps([[0, 0, 0, H]]), "--gait-sequence",
                            json.dumps([[2, "trot"], [11, ""]]), "--hold", "12",
                            "--wrench-sequence", str(PUSHES / "push_y50.yaml"), "--wrench-delay", "4"]),
    "walk_push": dict(args=["--command-sequence", json.dumps([[0, 0, 0, H]]), "--gait-sequence",
                            json.dumps([[2, "walk"], [11, ""]]), "--hold", "12",
                            "--wrench-sequence", str(PUSHES / "push_x60.yaml"), "--wrench-delay", "4"]),
    "trot_upkie_course": gait("trot", 0.3, start=1, stop=26, hold=27, scene="scene_course.xml"),
    "walk_upkie_course": gait("walk", 0.3, start=1, stop=26, hold=27, scene="scene_course.xml"),
    "trot_course": gait("trot", 0.3, start=1, stop=24, hold=25, params="extreme_course_params.yaml",
                        past=6.0),
    "walk_course": gait("walk", 0.3, start=1, stop=24, hold=25, params="extreme_course_params.yaml",
                        past=6.0),
    "jump_turn": dict(args=["--command-sequence", json.dumps([[0, 0, 1.0, H]]), "--gait-sequence",
                            json.dumps([[3, "jump"], [10, ""]]), "--hold", "12"]),
    "gait_switch": dict(args=["--command-sequence", json.dumps([[1, 0.5, 0, H, 1], [14, 0, 0, H]]), "--gait-sequence",
                              json.dumps([[3, "walk"], [5, "trot"], [7, "jump"], [9, ""], [10, "trot"], [12, ""]]),
                              "--hold", "15"]),
    "vy_left": lateral([[1, 0, 0.2, 0], [7, 0, 0, 0]], 10), "vy_right": lateral([[1, 0, -0.2, 0], [7, 0, 0, 0]], 10),
    "vy_mixed": lateral([[1, 0.2, 0.15, 0], [5, 0.0, -0.25, 0], [9, -0.2, 0.15, 0.5], [13, 0, 0, 0]], 16),
}
COMMON = ["--robot-name", "pineapple", "--mpc-rate", "200", "--source", "state_estimator",
          "--estimator", "humanoid_linear_kf", "--minimum-height", "0.15", "--startup-timeout", "900",
          "--max-yaw-error", "100", "--axis", "vx", "--speed", "0", "--settle", "0", "--ramp", "0", "--rest", "2"]


def scene_for(params, out, name):
    """MuJoCo scene with the course `params` (generate_upkie_course.py format), generated into `out` for case `name`
    (one file per run: concurrent repeats must not rewrite a scene another launch is reading)."""
    mjcf = out / f"{name}_terrain.xml"
    subprocess.run([sys.executable, str(ROOT / "description/mujoco/terrains/generate_upkie_course.py"),
                    "--params", str(TERRAINS / params), "--mjcf", str(mjcf)], check=True, capture_output=True)
    sys.path.insert(0, str(ROOT / "tests"))
    from mujoco_scene import write_scene
    scene = write_scene(out / f"{name}_scene.xml", "pineapple", "upkie_course")
    scene.write_text(scene.read_text().replace(str(ROOT / "description/mujoco/terrains/upkie_course.xml"), str(mjcf)))
    return scene


def after_takeover(prefix, takeover, skip=1.5):
    """Worst roll, pitch, lowest base (world z) and top body speed from `skip` s after the MPC takeover, so the
    stand-up does not mask the test."""
    worst = dict(max_roll=0.0, max_pitch=0.0, min_z=0.0, max_speed=0.0, min_mpc_hz=0.0, forward=0.0, lateral=0.0)
    if takeover is None:
        return worst
    try:
        rows = [r for r in csv.DictReader(open(str(prefix) + ".csv")) if float(r["time"]) > takeover + skip]
    except OSError:
        return worst
    try:  # the lowest MPC rate the controller logged (every 2 s): a starved machine shows here
        rates = [float(v) for v in re.findall(r"MPC_TIMING\] target=[\d.]+ Hz, actual=([\d.]+) Hz",
                                              Path(str(prefix) + ".log").read_text(errors="replace"))]
        worst["min_mpc_hz"] = min(rates) if rates else 0.0
    except OSError:
        pass
    if rows:
        # Displacement in the heading the robot had once settled (the stand-up leaves it turned in the world).
        first, last = rows[0], rows[-1]
        yaw, dx, dy = float(first["yaw"]), float(last["x"]) - float(first["x"]), float(last["y"]) - float(first["y"])
        worst.update(forward=dx * math.cos(yaw) + dy * math.sin(yaw), lateral=-dx * math.sin(yaw) + dy * math.cos(yaw))
        worst.update(max_roll=max(abs(float(r["roll"])) for r in rows), max_pitch=max(abs(float(r["pitch"])) for r in rows),
                     min_z=min(float(r["z"]) for r in rows),
                     max_speed=max(abs(float(r["vx_body"])) + abs(float(r["vy_body"])) for r in rows))
    return worst


def run(name, case, out, domain, extra):
    prefix = out / name
    command = [sys.executable, str(HARNESS), *COMMON, *case["args"], "--prefix", str(prefix), *extra]
    if "terrain" in case:
        command += ["--scene", str(scene_for(case["terrain"], out, name))]
    elif "scene" in case:
        command += ["--scene", case["scene"]]
    began = time.monotonic()
    with open(str(prefix) + ".out", "w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
                       timeout=2400)
    try:
        result = json.loads(Path(str(prefix) + ".json").read_text())
    except (OSError, json.JSONDecodeError):
        result = {"verdict": "NO_RESULT"}
    final = result.get("final_state") or [0.0] * 7
    row = dict(case=name, verdict=result.get("verdict"), x=final[1], y=final[2], sim_seconds=result.get("sim_seconds"),
               wall_seconds=round(time.monotonic() - began), **after_takeover(prefix, result.get("takeover_time")))
    judge(row, case)
    if "push_final_recovery" in result:
        row["push_recovered"] = result["push_final_recovery"]
    print(f"{name:22s} {row['verdict']:14s} roll {row['max_roll']:.3f} pitch {row['max_pitch']:.3f} "
          f"min z {row['min_z']:.3f} speed {row['max_speed']:.2f} fwd {row['forward']:+.2f} lat {row['lateral']:+.2f} MPC {row['min_mpc_hz']:.0f} Hz ({row['wall_seconds']} s)", flush=True)
    return row


def busy_domains():
    """ROS domains some running process already uses (from /proc): a leftover node there, e.g. a teleop publishing
    height commands, would drive the test robot. Linux only; empty elsewhere."""
    busy = set()
    for environ in Path("/proc").glob("[0-9]*/environ"):
        try:
            found = re.search(rb"(?:^|\0)ROS_DOMAIN_ID=(\d+)", environ.read_bytes())
        except OSError:
            continue
        if found:
            busy.add(int(found.group(1)))
    return busy


def judge(row, case):
    """STUCK: upright to the end, but short of the far side of the case's obstacle."""
    if row["verdict"] == "SUCCESS" and case.get("past") and row["forward"] < case["past"]:
        row["verdict"] = "STUCK"
    return row


def summarize_only(name, out):
    result = json.loads((out / f"{name}.json").read_text())
    final = result.get("final_state") or [0.0] * 7
    row = dict(case=name, verdict=result.get("verdict"), x=final[1], y=final[2], sim_seconds=result.get("sim_seconds"),
               wall_seconds=0, **after_takeover(out / name, result.get("takeover_time")))
    return judge(row, CASES[re.sub(r"_r\d+$", "", name)])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tag", default="run")
    parser.add_argument("--out", type=Path, default=Path("/tmp/pineapple_suite"))
    parser.add_argument("--cases", nargs="+", default=["*"], help="case names or glob patterns")
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--repeat", type=int, default=1, help="runs per case (CASE, CASE_r2, ...): results near a limit vary")
    parser.add_argument("--domain", type=int, default=171,
                        help="first ROS domain; each job takes the next one no other process uses")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--summarize", action="store_true", help="only rebuild the summary of an existing tag")
    args, extra = parser.parse_known_args()  # the rest (--set, --config, --gait-sequence ...) goes to the harness
    if args.list:
        print("\n".join(CASES))
        return 0
    names = [n for n in CASES if any(fnmatch.fnmatch(n, p) for p in args.cases)]
    names = [n if k == 0 else f"{n}_r{k + 1}" for n in names for k in range(args.repeat)]
    out = (args.out / args.tag).resolve()
    if args.summarize:
        names = [n for n in names if (out / f"{n}.json").is_file()]
    out.mkdir(parents=True, exist_ok=True)
    if not args.summarize:
        (out / "extra_args.json").write_text(json.dumps(extra))
    domains = queue.Queue()  # a free ROS domain per running case: concurrent launches must not share one
    busy, candidate = busy_domains(), args.domain
    for _ in range(args.jobs):
        while candidate in busy:
            print(f"ROS domain {candidate} is in use by another process: skipped", flush=True)
            candidate += 1
        domains.put(candidate)
        candidate += 1

    def job(name):
        if args.summarize:
            return summarize_only(name, out)
        domain = domains.get()
        try:
            return run(name, CASES[re.sub(r"_r\d+$", "", name)], out, domain, extra)
        finally:
            domains.put(domain)

    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(job, names))
    (out / "summary.json").write_text(json.dumps(rows, indent=1) + "\n")
    passed = sum(r["verdict"] == "SUCCESS" for r in rows)
    lines = [f"# Pineapple suite `{args.tag}`: {passed}/{len(rows)} SUCCESS", "", f"harness extra: `{' '.join(extra)}`", "",
             "Worst values from 1.5 s after the MPC takeover (the stand-up excluded); min z is the world height of the base.",
             "STUCK: upright to the end, but short of the far side of the case's step or course.", "",
             "", "Forward and lateral: displacement in the robot's settled heading (the stand-up leaves it turned in the world).",
             "", "| case | verdict | max roll | max pitch | min z | max speed | forward | lateral | min MPC Hz |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    lines += [f"| {r['case']} | {r['verdict']} | {r['max_roll']:.3f} | {r['max_pitch']:.3f} | {r['min_z']:.3f} | "
              f"{r['max_speed']:.2f} | {r['forward']:+.2f} | {r['lateral']:+.2f} | {r['min_mpc_hz']:.0f} |" for r in rows]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if passed == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
