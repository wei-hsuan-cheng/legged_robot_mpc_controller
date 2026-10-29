#!/usr/bin/env python3
"""Plot raw Euler/tangent benchmark traces without requiring ROS."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory
    cases = json.loads((directory / "comparison.json").read_text())
    for case in dict.fromkeys(row["case"] for row in cases):
        selected = [row for row in cases if row["case"] == case]
        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        for result in selected:
            prefix = directory / Path(result["prefix"]).name
            if not prefix.with_suffix(".csv").exists():
                print("Skipping missing trace:", prefix.with_suffix(".csv"))
                continue
            with prefix.with_suffix(".csv").open() as file:
                records = [row for row in csv.DictReader(file) if row["phase"] != "startup"]
            if not records:
                continue
            read = lambda name: np.array([float(row[name]) for row in records])
            t = read("time") - float(records[0]["time"])
            color = "tab:blue" if result["representation"] == "euler_zyx" else "tab:red"
            label = f"{result['representation']} #{result['repeat']} ({result['verdict']})"
            axes[0].plot(t, read("z"), color=color, label=label)
            axes[1].plot(t, np.rad2deg(np.hypot(read("roll"), read("pitch"))), color=color, label=label)
            yaw = np.unwrap(read("yaw"))
            if case == "yaw":
                speed = np.gradient(yaw, t)
            else:
                vx, vy = np.gradient(read("x"), t), np.gradient(read("y"), t)
                speed = -vx * np.sin(yaw) + vy * np.cos(yaw) if case == "vy" else vx * np.cos(yaw) + vy * np.sin(yaw)
            axes[2].plot(t, speed, color=color, label=label)
            axes[3].plot(t, np.rad2deg(yaw), color=color, label=label)
        speed_label = "Pose yaw rate [rad/s]" if case == "yaw" else "Pose-derived body speed [m/s]"
        for axis, label in zip(axes, ("Base height [m]", "RP magnitude [deg]", speed_label, "Unwrapped yaw [deg]")):
            axis.set_ylabel(label)
            axis.grid(alpha=0.25)
            if axis.lines:
                axis.legend(fontsize=8)
        axes[-1].set_xlabel("Simulation time after MPC takeover [s]")
        fig.suptitle(case + ": identical gains, estimator and command profile")
        fig.tight_layout()
        fig.savefig(directory / (case + "_comparison.png"), dpi=140)
        plt.close(fig)


if __name__ == "__main__":
    main()
