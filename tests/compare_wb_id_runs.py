#!/usr/bin/env python3
"""Compare two wb_id_walk_test prefixes without changing their configurations."""

import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics

import yaml


CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


def load(prefix):
    result = json.loads(Path(str(prefix) + ".json").read_text())
    config = yaml.safe_load(Path(str(prefix) + ".yaml").read_text())[CONTROLLER]
    log = Path(str(prefix) + ".log").read_text(errors="replace")
    rates = [float(value) for value in re.findall(r"MPC_TIMING\].*?actual=([0-9.]+) Hz", log)]
    health = {name: max([int(value) for value in re.findall(name + r"=(\d+)", log)], default=0)
              for name in ("solver exceptions", "torque-window overruns")}
    # The first timing windows include solver initialization and policy handover.
    steady_rates = rates[2:] or rates
    return result, config, statistics.median(steady_rates) if steady_rates else None, health


def compare(before_prefix, after_prefix):
    before, before_config, before_hz, before_health = load(before_prefix)
    after, after_config, after_hz, after_health = load(after_prefix)
    checks = {"same_controller_settings": before_config == after_config,
              "both_runs_completed": before["verdict"] == after["verdict"] == "SUCCESS",
              "both_runs_balanced": before.get("balance_survived") and after.get("balance_survived")}
    for key in ("robot_name", "source", "estimator", "mpc_rate", "axis", "speed", "hold",
                "ramp", "settle", "rest", "height", "command_sequence"):
        checks["same_" + key] = before.get(key) == after.get(key)
    for key in ("max_roll", "max_pitch"):
        checks[key] = after[key] <= before[key] + 0.05
    checks["minimum_height"] = after["min_z"] >= before["min_z"] - 0.03
    for key in ("walk_mean_vx", "walk_mean_vy", "walk_mean_wz"):
        checks[key] = abs(after[key] - before[key]) <= max(0.05, 0.15 * abs(before[key]))
    checks["solve_rate"] = (before_hz is not None and after_hz is not None
                            and after_hz >= 0.85 * before_hz)
    for name in before_health:
        checks[name] = after_health[name] <= before_health[name]
    metrics = ("walk_mean_vx", "walk_mean_vy", "walk_mean_wz", "walk_mean_z", "min_z",
               "max_roll", "max_pitch", "displacement")
    return dict(passed=all(checks.values()), checks=checks,
                before={**{key: before[key] for key in metrics}, "solve_hz": before_hz, **before_health},
                after={**{key: after[key] for key in metrics}, "solve_hz": after_hz, **after_health})


def plot(before_prefix, after_prefix, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for prefix, label, color in ((before_prefix, "Before", "#2166ac"),
                                  (after_prefix, "After", "#c33b36")):
        result = json.loads(Path(str(prefix) + ".json").read_text())
        with Path(str(prefix) + ".csv").open() as stream:
            samples = [row for row in csv.DictReader(stream) if row["phase"] != "startup"]
        times = [float(row["time"]) - result["takeover_time"] for row in samples]
        curves = ([float(row["z"]) for row in samples],
                  [math.degrees(float(row["roll"])) for row in samples],
                  [math.degrees(float(row["pitch"])) for row in samples],
                  [float(row["vx_body"]) for row in samples])
        for axis, values in zip(axes, curves):
            axis.plot(times, values, label=label, color=color, linewidth=1, alpha=0.8)
    for axis, label in zip(axes, ("Base height [m]", "Roll [deg]", "Pitch [deg]", "Body vx [m/s]")):
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    axes[0].legend()
    axes[0].set_title(result["robot_name"] + ": matched WB-ID regression")
    axes[-1].set_xlabel("Time after MPC takeover [s]")
    figure.tight_layout()
    figure.savefig(output, dpi=130)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    result = compare(args.before, args.after)
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + "\n")
    if args.plot:
        plot(args.before, args.after, args.plot)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
