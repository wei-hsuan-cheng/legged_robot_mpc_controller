#!/usr/bin/env python3
"""MPC robustness sweep: the same seeded commands under each setting of one sweep YAML, any legged robot.

    python3 tests/robustness_sweep.py config/robustness/upkie.yaml [--settings nominal ...] [--runs N] [--jobs J]

Following Khazoom et al., "Tailoring Solution Accuracy for Fast Whole-body Model Predictive Control of Legged
Robots" (arXiv 2407.10789): every run starts the robot (Upkie: kneeling, then its planned stand-up), holds one
constant command sampled uniformly within the sweep's ranges, normalized to the robot's command limits in
config/<robot>/ros2_controllers.yaml, then stops; a run succeeds if the robot does not fall (the paper's measure), and it also tracks if it
succeeds and its mean forward, lateral and yaw rates over the hold are each within half the command (plus a small
floor, TRACKING). Every setting sees the same command set (seed + run index), so settings differ only in what the setting changes.

A setting may override controller parameters (set: {dotted.key: value}), add launch arguments (launch_args) and give
plant-side modeling errors (model_error: the simulator's YAML map, see mujoco_ros2_control model_error.hpp), which each
run draws with its own seed (seed * 100000 + run index) and reports in run_NNN_model_error.json.
Each run is one tests/wb_id_walk_test.py launch on its own ROS domain. Results go to <output>/<setting>/run_NNN.* and
<output>/summary.{json,md}; finished runs are kept and skipped when the sweep is restarted.
Run from the sourced workspace: cd /root/ocs2_ros2_ws (the CppAD cache is relative to it).
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
# Verdicts in which the robot stayed up; the others are failures (FALL, TIMEOUT, STANDUP_FAULT, ...).
SURVIVED = ("SUCCESS", "NO_TRACKING", "HEIGHT_TRACKING_ERROR")
# Infrastructure failures, not the robot's: left out of the success rate and reported as excluded.
EXCLUDED = ("HARNESS_ERROR", "HARNESS_TIMEOUT", "STARTUP_ERROR", "LAUNCH_EXITED", "STALE_ODOM")
# A run that stayed up tracks if |mean - command| <= relative * |command| + floor on every commanded axis (a nonzero
# normalized range) over the hold: the harness's single-axis rule is 50 %; the floor keeps near-zero commands from
# failing on noise; an axis the sweep never commands is not checked (a wheeled base off its axle drifts sideways in a
# turn). The sweep YAML's tracking: map overrides any of these.
TRACKING = dict(relative=0.5, vx=0.1, vy=0.1, yaw_rate=0.2)


def robot_limits(robot):
    config = yaml.safe_load((ROOT / "config" / robot / "ros2_controllers.yaml").read_text())
    params = config[CONTROLLER]["ros__parameters"]
    reference = params["ocs2"]["reference"]
    return dict(vx=float(reference["maxDisplacementVelocityX"]), vy=float(reference["maxDisplacementVelocityY"]),
                yaw_rate=float(reference["maxRotationVelocity"]), height=float(reference["defaultBaseHeight"]),
                frame_yaw=float(params.get("target", {}).get("commandFrameYaw", 0.0)))


def sample_commands(sweep, limits, runs):
    """One constant command per run: uniform within the normalized ranges, scaled by the robot's limits."""
    ranges = sweep["command"]["normalized"]
    commands = []
    for run in range(runs):
        rng = np.random.default_rng([int(sweep.get("seed", 0)), run])
        command = {key: float(rng.uniform(*ranges.get(key, [0.0, 0.0]))) * limits[key] for key in ("vx", "vy", "yaw_rate")}
        commands.append({key: round(value, 4) for key, value in command.items()})
    return commands


def harness_command(sweep, setting, command, prefix, limits, run=0):
    timing = sweep["command"]
    start, hold, stop = float(timing["start"]), float(timing["hold"]), float(timing["stop"])
    sequence = [dict(time=start, **command), dict(time=start + hold)]
    cmd = [sys.executable, str(ROOT / "tests/wb_id_walk_test.py"), "--robot-name", sweep["robot"], "--prefix", str(prefix),
           "--speed", "0", "--settle", "0", "--ramp", "0", "--hold", str(start + hold + stop), "--rest", "0",
           "--command-sequence", json.dumps(sequence)]
    cmd += [str(a) for a in sweep.get("harness_args", [])] + [str(a) for a in setting.get("harness_args", [])]
    for key, value in {**sweep.get("set", {}), **setting.get("set", {})}.items():
        cmd += ["--set", f"{key}={json.dumps(value)}"]
    for argument in setting.get("launch_args", []):
        cmd += ["--launch-arg", argument]
    if setting.get("model_error"):
        # One file per setting; each run draws its own errors (seed + run index) and reports what it drew.
        error_file = Path(prefix).parent / "model_error.yaml"
        if not error_file.exists():
            error_file.write_text(yaml.safe_dump(dict(model_error=setting["model_error"]), sort_keys=False))
        cmd += ["--launch-arg", f"modelErrorFile:={error_file}",
                "--launch-arg", f"modelErrorSeed:={int(sweep.get('seed', 0)) * 100000 + run}",
                "--launch-arg", f"modelErrorReport:={prefix}_model_error.json"]
    return cmd


def run_metrics(prefix, command, sweep, limits):
    """Verdict and command tracking of one run, from its result JSON and ground-truth CSV."""
    result = json.loads(Path(f"{prefix}.json").read_text())
    metrics = dict(verdict=result.get("verdict", "MISSING"), command=command,
                   fell_before_takeover=bool(result.get("fell_before_takeover", False)), tracked=False)
    metrics["survived"] = metrics["verdict"] in SURVIVED
    log = Path(f"{prefix}.log")
    if log.exists():
        rates = [float(line.split("actual=")[1].split()[0]) for line in log.read_text(errors="replace").splitlines()
                 if "[MPC_TIMING]" in line and "actual=" in line]
        if rates:
            metrics["mpc_rate_hz"] = float(np.mean(rates))
    report = Path(f"{prefix}_model_error.json")
    if report.exists():
        realized = json.loads(report.read_text())
        if "total_mass" in realized:
            metrics["total_mass"] = realized["total_mass"][1]
    takeover = result.get("takeover_time")
    if takeover is None or not Path(f"{prefix}.csv").exists():
        return metrics
    timing = sweep["command"]
    # The hold window, after a settling margin for the command rate limits and the response.
    begin = takeover + float(timing["start"]) + float(timing.get("transient", 1.0))
    end = takeover + float(timing["start"]) + float(timing["hold"])
    c, s = math.cos(limits["frame_yaw"]), math.sin(limits["frame_yaw"])
    rows = [r for r in csv.DictReader(open(f"{prefix}.csv")) if r["phase"] != "startup"]
    t = np.array([float(r["time"]) for r in rows])
    tilt = np.array([max(abs(float(r["roll"])), abs(float(r["pitch"]))) for r in rows])
    window = (t >= begin) & (t <= end)
    # From the command onset: the stand-up handover's transient is the same in every run of a setting.
    commanded = t >= takeover + float(timing["start"])
    if commanded.any():
        metrics["max_tilt"] = float(tilt[commanded].max())
        metrics["min_z"] = float(min(float(r["z"]) for r, a in zip(rows, commanded) if a))
    if window.sum() < 10:
        return metrics
    vx = np.array([float(r["vx_body"]) for r in rows])[window]
    vy = np.array([float(r["vy_body"]) for r in rows])[window]
    wz = np.array([float(r["wz_body"]) for r in rows])[window]
    forward, lateral = c * vx + s * vy, -s * vx + c * vy
    metrics.update(mean_vx=float(forward.mean()), mean_vy=float(lateral.mean()), mean_yaw_rate=float(wz.mean()),
                   vx_error=float(abs(forward.mean() - command["vx"])), vy_error=float(abs(lateral.mean() - command["vy"])),
                   yaw_rate_error=float(abs(wz.mean() - command["yaw_rate"])))
    tolerance = dict(TRACKING, **sweep.get("tracking", {}))
    axes = [axis for axis in ("vx", "vy", "yaw_rate") if any(sweep["command"]["normalized"].get(axis, [0.0, 0.0]))]
    metrics["tracked"] = metrics["survived"] and all(
        metrics[f"{axis}_error"] <= tolerance["relative"] * abs(command[axis]) + tolerance[axis] for axis in axes)
    return metrics


def expand_sources(sweep):
    """The sweep's settings plus one generated setting per level of each source: <source>_<level>, its template with
    "$level" / "-$level" replaced by the level (so one sweep YAML covers a whole success-vs-error curve)."""
    settings = {name: dict(setting or {}, source=None, level=0.0) for name, setting in sweep.get("settings", {}).items()}

    def fill(value, level):
        if isinstance(value, dict):
            return {key: fill(item, level) for key, item in value.items()}
        if isinstance(value, list):
            return [fill(item, level) for item in value]
        if value == "$level":
            return level
        if value == "-$level":
            return -level
        if isinstance(value, str) and "$level" in value:
            return value.replace("$level", repr(level))
        return value

    for source, spec in sweep.get("sources", {}).items():
        template = {key: value for key, value in spec.items() if key not in ("levels", "label")}
        for level in spec["levels"]:
            settings[f"{source}_{level:g}"] = dict(fill(template, float(level)), source=source, level=float(level))
    return settings


def wilson(successes, n, z=1.96):
    if n == 0:
        return [0.0, 1.0]
    p = successes / n
    centre, half = (p + z * z / (2 * n)) / (1 + z * z / n), z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [max(0.0, centre - half), min(1.0, centre + half)]


def summarize(name, runs, setting=None):
    excluded = [r for r in runs if r["verdict"] in EXCLUDED]
    runs = [r for r in runs if r["verdict"] not in EXCLUDED]
    n, survived, tracked = len(runs), sum(r["survived"] for r in runs), sum(r.get("tracked", False) for r in runs)
    failures = {}
    for r in runs:
        if not r["survived"]:
            key = r["verdict"] + (" (stand-up)" if r["fell_before_takeover"] else "")
            failures[key] = failures.get(key, 0) + 1
    stats = lambda key: [float(np.percentile([r[key] for r in runs if key in r and r["survived"]], q))
                         for q in (50, 90)] if any(key in r and r["survived"] for r in runs) else None
    rates = [r["mpc_rate_hz"] for r in runs if "mpc_rate_hz" in r]
    return dict(setting=name, source=(setting or {}).get("source"), level=(setting or {}).get("level", 0.0),
                mean_mpc_rate_hz=float(np.mean(rates)) if rates else None, excluded=len(excluded),
                runs=n, survived=survived, success_rate=survived / n if n else 0.0,
                success_rate_ci95=wilson(survived, n), tracked=tracked, tracking_rate=tracked / n if n else 0.0,
                tracking_rate_ci95=wilson(tracked, n), failures=failures, vx_error_p50_p90=stats("vx_error"),
                vy_error_p50_p90=stats("vy_error"), yaw_rate_error_p50_p90=stats("yaw_rate_error"),
                max_tilt_p50_p90=stats("max_tilt"))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sweep", type=Path)
    parser.add_argument("--settings", nargs="+", help="run only these settings (default: all)")
    parser.add_argument("--runs", type=int, help="override the sweep's runs per setting")
    parser.add_argument("--jobs", type=int, default=1, help="parallel runs; each uses its own ROS domain and CPU")
    parser.add_argument("--domain-base", type=int, default=180, help="ROS_DOMAIN_ID of the first job")
    parser.add_argument("--output", type=Path, help="default: <repo>/demo_videos/robustness/<sweep file stem>")
    parser.add_argument("--timeout", type=float, default=600.0, help="wall-clock limit per run [s]")
    parser.add_argument("--dry-run", action="store_true", help="print the commands and harness calls, run nothing")
    args = parser.parse_args()

    sweep = yaml.safe_load(args.sweep.read_text())["sweep"]
    limits = robot_limits(sweep["robot"])
    runs = args.runs or int(sweep["runs"])
    commands = sample_commands(sweep, limits, runs)
    settings = expand_sources(sweep)
    if args.settings:
        unknown = set(args.settings) - set(settings)
        if unknown:
            parser.error(f"unknown settings {sorted(unknown)}; the sweep has {sorted(settings)}")
        settings = {name: settings[name] for name in args.settings}
    output = args.output or ROOT / "demo_videos" / "robustness" / args.sweep.stem
    output.mkdir(parents=True, exist_ok=True)
    (output / "commands.json").write_text(json.dumps(commands, indent=1) + "\n")

    jobs = [(name, index) for name in settings for index in range(runs)]
    if args.dry_run:
        for name, index in jobs[:3]:
            print(name, index, commands[index])
            (output / name).mkdir(parents=True, exist_ok=True)
            print(" ".join(harness_command(sweep, settings[name], commands[index], output / name / f"run_{index:03d}",
                                          limits, index)))
        print(f"... {len(jobs)} runs")
        return 0

    domains = queue.Queue()
    for job in range(args.jobs):
        domains.put(args.domain_base + job)

    progress_lock = threading.Lock()

    def execute(job):
        name, index = job
        prefix = output / name / f"run_{index:03d}"
        prefix.parent.mkdir(parents=True, exist_ok=True)
        began = time.monotonic()
        fresh = not Path(f"{prefix}.json").exists()
        if fresh:
            domain = domains.get()
            try:
                with open(f"{prefix}.out", "w") as log:
                    subprocess.run(harness_command(sweep, settings[name], commands[index], prefix, limits, index),
                                   stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout,
                                   env={**os.environ, "ROS_DOMAIN_ID": str(domain)})
            except subprocess.TimeoutExpired:
                Path(f"{prefix}.json").write_text(json.dumps(dict(verdict="HARNESS_TIMEOUT")))
            finally:
                domains.put(domain)
            if not Path(f"{prefix}.json").exists():  # the harness failed before writing a result (see run_NNN.out)
                tail = Path(f"{prefix}.out").read_text(errors="replace").splitlines()[-3:]
                Path(f"{prefix}.json").write_text(json.dumps(dict(verdict="HARNESS_ERROR", error=tail)))
            Path(f"{prefix}.csv.gz").unlink(missing_ok=True)
        metrics = run_metrics(prefix, commands[index], sweep, limits)
        line = (f"{time.strftime('%H:%M:%S')} {name} run {index:3d} {metrics['verdict']:12s} "
                f"mpc {metrics.get('mpc_rate_hz', float('nan')):5.0f} Hz {time.monotonic() - began:5.0f} s"
                f"{'' if fresh else ' (kept)'} {json.dumps(commands[index])}")
        with progress_lock:
            with open(output / "progress.log", "a") as log:
                log.write(line + "\n")
        print(line, flush=True)
        return name, index, metrics

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        finished = list(pool.map(execute, jobs))

    per_setting = {name: [m for n, _, m in sorted(finished) if n == name] for name in settings}
    summary = [summarize(name, per_setting[name], settings[name]) for name in settings]
    labels = {source: spec.get("label", source) for source, spec in sweep.get("sources", {}).items()}
    (output / "summary.json").write_text(json.dumps(dict(sweep=str(args.sweep), labels=labels, commands=commands,
                                                         summary=summary,
                                                         runs={n: r for n, r in per_setting.items()}), indent=1) + "\n")
    fmt = lambda pair: "-" if pair is None else f"{pair[0]:.3f} / {pair[1]:.3f}"
    lines = ["| setting | success | 95% CI | tracked | failures | vx error p50/p90 | yaw-rate error p50/p90 | max tilt p50/p90 "
             "| MPC Hz |", "| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |"]
    for s in summary:
        rate = "-" if s["mean_mpc_rate_hz"] is None else f"{s['mean_mpc_rate_hz']:.0f}"
        failures = ", ".join(f"{k} {v}" for k, v in s["failures"].items()) or "-"
        if s["excluded"]:
            failures += f" (+{s['excluded']} excluded: harness)"
        lines.append(f"| {s['setting']} | {s['survived']}/{s['runs']} | {s['success_rate_ci95'][0]:.2f}-"
                     f"{s['success_rate_ci95'][1]:.2f} | {s['tracked']}/{s['runs']} | {failures} | {fmt(s['vx_error_p50_p90'])} | "
                     f"{fmt(s['yaw_rate_error_p50_p90'])} | {fmt(s['max_tilt_p50_p90'])} | {rate} |")
    (output / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    if any(s["source"] for s in summary):
        subprocess.run([sys.executable, str(ROOT / "tests/tuning/plot_robustness.py"), str(output / "summary.json")])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
