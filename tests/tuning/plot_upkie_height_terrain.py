#!/usr/bin/env python3
"""Plot logged height commands and measured support/attitude, without ROS."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_wheeled_biped import read_csv


def height_rows(prefix):
    with open(prefix + "_height.csv") as stream:
        rows = list(csv.DictReader(stream))
    return {name: np.array([float(row[name]) for row in rows]) for name in rows[0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height-prefix", required=True)
    parser.add_argument("--ramp-prefix", required=True)
    parser.add_argument("--rough-prefix", required=True)
    parser.add_argument("--split-prefix", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    height = height_rows(args.height_prefix)
    axes[0, 0].plot(height["elapsed"], height["command_height"], "--", label="command")
    axes[0, 0].plot(height["elapsed"], height["measured_height"], label="measured")
    axes[0, 0].set(title="Flat-ground height steps", ylabel="Base height [m]")
    axes[0, 0].legend()

    split, phases = read_csv(args.split_prefix + "_tracking.csv")
    mask = phases != "startup"
    t = split["t"][mask] - split["t"][mask][0]
    # On the horizontal strip/floor only: center height minus radius estimates tread level.
    for side in ("left", "right"):
        axes[0, 1].plot(t, 1000 * (split[f"obs_foot/{side}_tire_center/z"][mask] - .055), label=side)
    axes[0, 1].set(title="Split-height trial: measured wheel support levels", ylabel="Center z - radius [mm]")
    axes[0, 1].legend()
    for label, prefix in (("ramp", args.ramp_prefix), ("uneven", args.rough_prefix), ("split + height steps", args.split_prefix)):
        data, phases = read_csv(prefix + "_tracking.csv")
        mask = phases != "startup"
        t = data["t"][mask] - data["t"][mask][0]
        for col, angle in enumerate(("pitch", "roll")):
            axes[1, col].plot(t, np.degrees(data["obs_base/" + angle][mask]), label=label)
            axes[1, col].set(title=f"Terrain-trial base {angle}", ylabel="Angle [deg]")
    for ax in axes.flat:
        ax.set_xlabel("Time after takeover [s]")
        ax.grid(alpha=.25)
    for ax in axes[1]:
        ax.legend()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
