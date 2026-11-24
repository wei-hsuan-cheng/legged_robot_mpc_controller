#!/usr/bin/env python3
"""Success rate vs modeling error, one panel per error source, from tests/robustness_sweep.py summaries.

    python3 tests/tuning/plot_robustness.py demo_videos/robustness/upkie/summary.json [--output robustness.png]

Each source of the sweep (sources: in its YAML) is a curve over its levels, with the nominal setting at level 0; the
success rate (no fall, the paper's measure) carries its Wilson 95% interval, and its label counts the falls during the
planned stand-up (falls, and handovers rejected because the robot is not on its wheels), before the MPC runs; the dashed curve is the share of runs that also tracked the command (each axis
within half the command, robustness_sweep.py TRACKING). The lower row shows the median forward-speed tracking error and
the median maximum tilt from the command onset of the runs that stayed up.
(pip install numpy matplotlib)
"""

import argparse
import json
import math
from pathlib import Path
import textwrap

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--output", type=Path, help="default: next to the summary, robustness.png")
    args = parser.parse_args()
    data = json.loads(args.summary.read_text())
    summary, labels = data["summary"], data.get("labels", {})
    nominal = next((s for s in summary if s["setting"] == "nominal"), None)
    present = {s["source"] for s in summary if s["source"]}
    sources = [source for source in labels if source in present] + sorted(present - set(labels))  # the sweep's order
    if not sources:
        parser.error("the summary has no sources (levels of an error); nothing to plot")
    # Up to four sources per row of blocks; each block is the success panel over the error panel.
    columns = min(4, len(sources))
    blocks = math.ceil(len(sources) / columns)
    fig, axes = plt.subplots(2 * blocks, columns, figsize=(4.4 * columns, 6.0 * blocks), squeeze=False)
    for spare in range(len(sources), blocks * columns):
        axes[2 * (spare // columns)][spare % columns].axis("off")
        axes[2 * (spare // columns) + 1][spare % columns].axis("off")
    for index, source in enumerate(sources):
        row, column = 2 * (index // columns), index % columns
        own = [s for s in summary if s["source"] == source]
        # A source with its own level-0 setting (e.g. ground patches without friction error) replaces the nominal.
        reference = [] if any(s["level"] == 0.0 for s in own) or not nominal else [nominal]
        points = sorted(reference + own, key=lambda s: s["level"])
        levels = [s["level"] for s in points]
        rate = [100 * s["success_rate"] for s in points]
        low = [100 * (s["success_rate"] - s["success_rate_ci95"][0]) for s in points]
        high = [100 * (s["success_rate_ci95"][1] - s["success_rate"]) for s in points]
        top, bottom = axes[row][column], axes[row + 1][column]
        line = top.errorbar(levels, rate, yerr=[low, high], marker="o", capsize=3, label="success (no fall)")[0]
        if all("tracking_rate" in s for s in points):
            top.plot(levels, [100 * s["tracking_rate"] for s in points], linestyle="--", marker="^",
                     color=line.get_color(), alpha=0.7, label="success and tracking")
        for level, value, s in zip(levels, rate, points):
            if s["survived"] == s["runs"]:
                continue  # label only the points with failures
            standup = sum(count for key, count in s["failures"].items() if "stand-up" in key or "STANDUP" in key)
            top.annotate(f"{s['survived']}/{s['runs']}" + (f"\n({standup} stand-up)" if standup else ""), (level, value), textcoords="offset points", xytext=(0, 6),
                         ha="center", fontsize=8)
        top.set_title("\n".join(textwrap.wrap(labels.get(source, source), 46)), fontsize=9)
        top.set_ylabel("success rate [%] (95% CI)", fontsize=8)
        bottom.set_ylabel("median, runs that stayed up", fontsize=8)
        top.set_ylim(-5, 110)
        top.grid(alpha=0.3)
        median = lambda s, key: s[key][0] if s.get(key) else float("nan")
        bottom.plot(levels, [median(s, "vx_error_p50_p90") for s in points], marker="o", label="forward speed error [m/s]")
        bottom.plot(levels, [median(s, "max_tilt_p50_p90") for s in points], marker="s",
                    label="max tilt from command onset [rad]")
        bottom.set_xlabel("error level (0 = nominal)")
        bottom.grid(alpha=0.3)
    axes[0][0].legend(fontsize=8, loc="lower left")
    axes[1][0].legend(fontsize=8)
    fig.suptitle(f"{Path(data['sweep']).stem}: success rate vs modeling error ({points[0]['runs']} runs per point, "
                 "same commands)", fontsize=11)
    fig.tight_layout()
    output = args.output or args.summary.with_name("robustness.png")
    fig.savefig(output, dpi=130)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
