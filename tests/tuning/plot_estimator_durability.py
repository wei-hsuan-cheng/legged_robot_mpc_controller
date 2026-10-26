#!/usr/bin/env python3
"""Comparison plots for estimator_durability_test.py runs.

matrix: per-minute estimate error and control outcome of several runs on shared axes, from the
        <prefix>_durability.json written by analyze_estimator_durability.py (run here if missing).
        InEKF runs are solid, linear KF runs dashed, and a run that fell ends in an x.
stop:   attitude, true vs estimated, around the commanded stop (start of ramp_out), one column per
        run, from the raw <prefix>.csv. A tilt error hidden during a long turn shows up here.

  plot_estimator_durability.py matrix DIR/claude_dur_g1_* --out DIR/g1_matrix_summary.png
  plot_estimator_durability.py stop DIR/claude_dur_g1_walk_inekf DIR/claude_dur_h2_walk_inekf --out DIR/inekf_stop_tilt.png
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_estimator_durability import analyze, load  # noqa: E402


def run_label(summary):
    estimator = (summary.get("estimator") or "").replace("humanoid_", "")
    command = f"vx {summary.get('vx_cmd', 0):.2f} wz {summary.get('yaw_cmd', 0):.2f}"
    return f"{summary.get('robot')} {estimator}, {command}: {summary.get('verdict')}"


def plot_matrix(prefixes, out):
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    for i, prefix in enumerate(prefixes):
        path = Path(str(prefix) + "_durability.json")
        if not path.exists():
            analyze(prefix, plot=False)
        report = json.loads(path.read_text())
        summary, minutes = report["summary"], report["minutes"]
        if not minutes:
            print(f"{Path(prefix).name}: no full hold minute, skipped")
            continue
        t = np.array([m["t_min"] for m in minutes]) + 0.5
        col = lambda k: np.array([m[k] for m in minutes])
        style = dict(color=f"C{i % 10}", ls="-" if "inekf" in (summary.get("estimator") or "") else "--")
        series = (np.degrees(np.hypot(col("roll_err"), col("pitch_err"))), 100 * col("z_err"),
                  np.degrees(col("gt_tilt_rms")), col("gt_z"))
        for axis, values in zip(axes, series):
            axis.plot(t, values, label=run_label(summary), **style)
            if summary.get("verdict") == "FALL":
                axis.plot(t[-1], values[-1], "x", color=style["color"], ms=9, mew=2)
    axes[0].set_ylabel("tilt estimate error [deg]\n(1 min mean of |roll, pitch|)")
    axes[1].set_ylabel("height estimate error [cm]\n(1 min mean)")
    axes[2].set_ylabel("true tilt RMS [deg]")
    axes[3].set_ylabel("true base height [m]")
    axes[-1].set_xlabel("hold time [min]")
    axes[0].legend(fontsize=7, ncol=2)
    for axis in axes:
        axis.grid(alpha=0.3)
    fig.suptitle("Estimator durability: estimate error and control outcome per hold minute (InEKF solid, linear KF dashed, x = fall)")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_stop(prefixes, out, before, after):
    fig, axes = plt.subplots(3, len(prefixes), figsize=(5.8 * len(prefixes), 8), sharex="col", squeeze=False)
    for j, prefix in enumerate(prefixes):
        phase, d = load(prefix)
        if not (phase == "ramp_out").any():
            print(f"{Path(prefix).name}: never reached the stop, skipped")
            continue
        # True yaw rate from the yaw angle: the simulator's odometry angular z reads ~0 while turning.
        wz = np.convolve(np.gradient(np.unwrap(d["gt_yaw"]), d["t"]), np.ones(11) / 11, mode="same")
        t0 = d["t"][np.argmax(phase == "ramp_out")]
        m = (d["t"] > t0 - before) & (d["t"] < t0 + after)
        t = d["t"][m] - t0
        deg = lambda k: np.degrees(d[k][m])
        axis = axes[0, j]
        axis.plot(t, deg("gt_pitch"), label="true pitch")
        axis.plot(t, deg("est_pitch"), label="estimated pitch")
        axis.plot(t, deg("gt_roll"), alpha=0.6, label="true roll")
        axis.plot(t, deg("est_roll"), alpha=0.6, label="estimated roll")
        axis.set_ylim(-8, 4)
        axis.set_ylabel("attitude [deg]")
        axis.set_title(Path(prefix).name.replace("claude_dur_", ""))
        axis = axes[1, j]
        axis.plot(t, deg("est_pitch") - deg("gt_pitch"), label="pitch error")
        axis.plot(t, deg("est_roll") - deg("gt_roll"), label="roll error")
        axis.set_ylabel("estimate - true [deg]")
        axis = axes[2, j]
        axis.plot(t, wz[m], label="true yaw rate [rad/s]")
        axis.plot(t, d["gt_z"][m], label="true base height [m]")
        axis.set_xlabel("time from start of the stop ramp [s]")
        stop_ramp = d["t"][phase == "ramp_out"]
        for axis in axes[:, j]:
            axis.axvspan(0, stop_ramp[-1] - t0, color="0.9")
            axis.legend(fontsize=8)
            axis.grid(alpha=0.3)
    fig.suptitle("Attitude around the commanded stop (grey: stop ramp). The controller holds the estimate, so a tilt error moves the true body.")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kind", choices=("matrix", "stop"))
    parser.add_argument("prefixes", nargs="+", help="run prefixes (paths without extension); globbed suffixes are stripped")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--before", type=float, default=10.0, help="stop plot: seconds before the stop ramp")
    parser.add_argument("--after", type=float, default=45.0, help="stop plot: seconds after the start of the stop ramp")
    args = parser.parse_args()
    # Accept a shell glob such as claude_dur_g1_*: keep one prefix per run, from its <prefix>.json.
    prefixes = sorted({p for p in args.prefixes if Path(p + ".json").exists()} |
                      {p[:-5] for p in args.prefixes if p.endswith(".json") and not p.endswith("_durability.json")})
    if args.kind == "matrix":
        plot_matrix(prefixes, args.out)
    else:
        plot_stop(prefixes, args.out, args.before, args.after)
    print(f"wrote {args.out}")
