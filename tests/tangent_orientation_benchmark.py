#!/usr/bin/env python3
"""Serial Euler/tangent A/B runs using the existing headless WB-ID walking harness.

Run in a sourced workspace. Only the orientation representation changes between
paired runs; controller/estimator gains and command timing stay identical.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys
import yaml

from mujoco_scene import write_scene


CASES = {
    "vx": ("vx", 0.5, 25.0),
    "vy": ("vy", 0.25, 25.0),
    "yaw": ("yaw", 0.6, 30.0),
    "run": ("vx", 1.0, 30.0),
    "rough": ("vx", 0.2, 20.0),
    "default_scene": ("vx", 0.2, 20.0),
    "push": ("vx", 0.3, 25.0),
}


def summarize(prefix, result):
    log = prefix.with_suffix(".log").read_text()
    row = {key: result.get(key) for key in (
        "verdict", "sim_seconds", "min_z", "max_roll", "max_pitch", "max_observed_yaw_error",
        "walk_mean_vx", "walk_mean_vy", "walk_mean_wz", "walk_pose_mean_vx_world",
        "walk_pose_mean_vy_world", "displacement", "expected_displacement")}
    for name, pattern in {
        "solve_ms_mean": r"average advance=([\d.]+) ms",
        "actual_hz_mean": r"actual=([\d.]+) Hz",
    }.items():
        values = [float(value) for value in re.findall(pattern, log)]
        row[name] = statistics.mean(values) if values else None
    values = [float(value) for value in re.findall(r"max advance=([\d.]+) ms", log)]
    row["solve_ms_max"] = max(values) if values else None
    values = [float(value) for value in re.findall(r"equality SSE mean=([\deE.+-]+)", log)]
    row["equality_sse_mean"] = statistics.mean(values) if values else None
    values = [float(value) for value in re.findall(r"dynamics SSE max=([\deE.+-]+)", log)]
    row["dynamics_sse_max"] = max(values) if values else None
    row["solver_exceptions"] = sum(int(value) for value in re.findall(r"solver exceptions=(\d+)", log))
    row["torque_window_overruns"] = max([int(value) for value in re.findall(r"torque-window overruns=(\d+)", log)], default=0)
    row["requested_mpc_hz"] = result.get("mpc_rate")
    for name, pattern in {
        "lq_ms": r"LQ Approximation\s*:\s*([\d.eE+-]+) \[ms\]",
        "qp_ms": r"Solve QP\s*:\s*([\d.eE+-]+) \[ms\]",
    }.items():
        values = re.findall(pattern, log)
        row[name] = float(values[-1]) if values else None
    with prefix.with_suffix(".csv").open() as file:
        samples = [sample for sample in csv.DictReader(file) if sample["phase"] == "walk"]
    totals = [0.0, 0.0, 0.0, 0.0]
    for a, b in zip(samples, samples[1:]):
        dt = float(b["time"]) - float(a["time"])
        if dt <= 0:
            continue
        dyaw = math.remainder(float(b["yaw"]) - float(a["yaw"]), 2 * math.pi)
        heading = float(a["yaw"]) + dyaw / 2
        dx, dy = float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])
        totals = [totals[0] + dx * math.cos(heading) + dy * math.sin(heading),
                  totals[1] - dx * math.sin(heading) + dy * math.cos(heading), totals[2] + dyaw, totals[3] + dt]
    for key, value in zip(("pose_body_vx", "pose_body_vy", "pose_yaw_rate"), totals):
        row[key] = value / totals[3] if totals[3] > 0 else None
    row["prefix"] = str(prefix)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", choices=("h2", "g1"), default="h2")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=["vx", "vy", "yaw", "run", "rough"])
    parser.add_argument("--representations", nargs="+", choices=("euler_zyx", "tangent_space"), default=["euler_zyx", "tangent_space"])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--mpc-rate", type=int, default=80)
    parser.add_argument("--source", choices=("state_estimator", "ground_truth_state"), default="state_estimator")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=JSON", help="Apply the same tuning override to both modes")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resume", action="store_true", help="Keep completed rows and add missing trials")
    parser.add_argument("--summarize-only", action="store_true", help="Refresh existing metrics without launching a simulator")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    harness = Path(__file__).with_name("wb_id_walk_test.py")
    comparison = args.output / "comparison.json"
    rows = json.loads(comparison.read_text()) if args.resume and comparison.exists() else []
    for row in rows:
        prefix = args.output.resolve() / Path(row["prefix"]).name
        row.update(summarize(prefix, json.loads(prefix.with_suffix(".json").read_text())))
    if args.summarize_only:
        rows = json.loads(comparison.read_text())
        for row in rows:
            prefix = args.output.resolve() / Path(row["prefix"]).name
            row.update(summarize(prefix, json.loads(prefix.with_suffix(".json").read_text())))
        comparison.write_text(json.dumps(rows, indent=2) + "\n")
        with (args.output / "comparison.csv").open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        return
    completed_keys = {(row["case"], row["representation"], row["repeat"]) for row in rows}
    for case in args.cases:
        axis, speed, hold = CASES[case]
        for repeat in range(args.repeats):
            order = args.representations if repeat % 2 == 0 else list(reversed(args.representations))
            for representation in order:
                if (case, representation, repeat + 1) in completed_keys:
                    continue
                prefix = args.output.resolve() / f"{args.robot_name}_{case}_{representation}_{repeat + 1}"
                command = [sys.executable, str(harness), "--robot-name", args.robot_name,
                           "--axis", axis, "--speed", str(speed), "--hold", str(hold),
                           "--mpc-rate", str(args.mpc_rate), "--source", args.source,
                           "--estimator", "humanoid_inekf", "--record-joints", "--record-tracking",
                           "--set", "ocs2.model.orientationRepresentation=" + json.dumps(representation),
                           "--prefix", str(prefix)]
                for override in args.set:
                    command += ["--set", override]
                if case == "rough":
                    scene = write_scene(Path(str(prefix) + "_scene.xml"), args.robot_name, "rough")
                    command += ["--scene", str(scene)]
                elif case == "default_scene":
                    command += ["--scene", "scene.xml"]
                elif case == "push":
                    # Reuse the first (pelvis, 50 N lateral, 0.5 s) existing pulse.
                    sequence = yaml.safe_load((harness.parents[1] / "config/tests/push_recovery.yaml").read_text())
                    push = Path(str(prefix) + "_push.yaml")
                    push.write_text(yaml.safe_dump(sequence[:1]))
                    command += ["--wrench-sequence", str(push)]
                print("RUN", " ".join(command), flush=True)
                with Path(str(prefix) + "_harness.log").open("w") as output:
                    completed = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT)
                result = json.loads(prefix.with_suffix(".json").read_text())
                row = dict(case=case, representation=representation, repeat=repeat + 1,
                           returncode=completed.returncode, **summarize(prefix, result))
                rows.append(row)
                comparison.write_text(json.dumps(rows, indent=2) + "\n")
                with (args.output / "comparison.csv").open("w", newline="") as output:
                    writer = csv.DictWriter(output, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
                print(json.dumps(row), flush=True)
                if result["verdict"] in ("STARTUP_ERROR", "TIMEOUT"):
                    raise RuntimeError("Fix startup/timeout before continuing the benchmark: " + str(prefix))


if __name__ == "__main__":
    main()
