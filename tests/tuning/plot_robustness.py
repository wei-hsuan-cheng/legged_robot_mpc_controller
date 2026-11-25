#!/usr/bin/env python3
"""Success rate vs modeling error, one panel per error source, from tests/robustness_sweep.py summaries.

    python3 tests/tuning/plot_robustness.py demo_videos/robustness/upkie/summary.json [--output robustness.png]
    python3 tests/tuning/plot_robustness.py <variant>/summary.json ... --output comparison.png  # variants overlaid

Each source of the sweep (sources: in its YAML) is a curve over its levels, with the nominal setting at level 0; the
success rate (no fall, the paper's measure) carries its Wilson 95% interval, and its label counts the falls during the
planned stand-up (falls, and handovers rejected because the robot is not on its wheels), before the MPC runs; the
dashed curve is the share of runs that also tracked the command (each axis within half the command, robustness_sweep.py
TRACKING). The lower row shows the median forward-speed tracking error and the median maximum tilt from the command
onset of the runs that stayed up. With several summaries (the variants of one sweep) each is one colour, named with
its mean achieved MPC rate, and the lower row shows the forward-speed error only.
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


def curve(summary, source):
    """The points of one source, the nominal setting as level 0 unless the source has its own level-0 setting (e.g.
    ground patches without friction error)."""
    nominal = next((s for s in summary if s["setting"] == "nominal"), None)
    own = [s for s in summary if s["source"] == source]
    reference = [] if any(s["level"] == 0.0 for s in own) or not nominal else [nominal]
    return sorted(reference + own, key=lambda s: s["level"])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("summaries", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, help="default: next to the (first) summary, robustness.png")
    args = parser.parse_args()
    datasets = [json.loads(path.read_text()) for path in args.summaries]
    compare = len(datasets) > 1
    labels = {key: value for data in datasets for key, value in data.get("labels", {}).items()}
    present = {s["source"] for data in datasets for s in data["summary"] if s["source"]}
    sources = [source for source in labels if source in present] + sorted(present - set(labels))  # the sweep's order
    if not sources:
        parser.error("the summaries have no sources (levels of an error); nothing to plot")
    # Up to four sources per row of blocks; each block is the success panel over the error panel.
    columns = min(4, len(sources))
    blocks = math.ceil(len(sources) / columns)
    fig, axes = plt.subplots(2 * blocks, columns, figsize=(4.4 * columns, 6.0 * blocks), squeeze=False)
    for spare in range(len(sources), blocks * columns):
        axes[2 * (spare // columns)][spare % columns].axis("off")
        axes[2 * (spare // columns) + 1][spare % columns].axis("off")
    for color, data in enumerate(datasets):
        summary = data["summary"]
        rates = [s["mean_mpc_rate_hz"] for s in summary if s.get("mean_mpc_rate_hz")]
        name = data.get("name") or Path(data["sweep"]).stem
        if rates:
            name += f" ({sum(rates) / len(rates):.0f} Hz)"
        style = dict(color=f"C{color}") if compare else {}
        for index, source in enumerate(sources):
            row, column = 2 * (index // columns), index % columns
            top, bottom = axes[row][column], axes[row + 1][column]
            points = curve(summary, source)
            if not points:
                continue
            levels = [s["level"] for s in points]
            rate = [100 * s["success_rate"] for s in points]
            low = [100 * (s["success_rate"] - s["success_rate_ci95"][0]) for s in points]
            high = [100 * (s["success_rate_ci95"][1] - s["success_rate"]) for s in points]
            line = top.errorbar(levels, rate, yerr=[low, high], marker="o", capsize=3, alpha=0.8 if compare else 1.0,
                                label=name if compare else "success (no fall)", **style)[0]
            if all("tracking_rate" in s for s in points):
                top.plot(levels, [100 * s["tracking_rate"] for s in points], linestyle="--", marker="^",
                         color=line.get_color(), alpha=0.6, label=None if compare else "success and tracking")
            for level, value, s in zip(levels, rate, points):
                if compare or s["survived"] == s["runs"]:
                    continue  # label only the points with failures, and none when variants overlap
                standup = sum(count for key, count in s["failures"].items() if "stand-up" in key or "STANDUP" in key)
                top.annotate(f"{s['survived']}/{s['runs']}" + (f"\n({standup} stand-up)" if standup else ""),
                             (level, value), textcoords="offset points", xytext=(0, 6), ha="center", fontsize=8)
            median = lambda s, key: s[key][0] if s.get(key) else float("nan")
            bottom.plot(levels, [median(s, "vx_error_p50_p90") for s in points], marker="o",
                        label=name if compare else "forward speed error [m/s]", **style)
            if not compare:
                bottom.plot(levels, [median(s, "max_tilt_p50_p90") for s in points], marker="s",
                            label="max tilt from command onset [rad]")
            top.set_title("\n".join(textwrap.wrap(labels.get(source, source), 46)), fontsize=9)
            top.set_ylabel("success rate [%] (95% CI)", fontsize=8)
            bottom.set_ylabel("forward speed error [m/s], median" if compare else "median, runs that stayed up",
                              fontsize=8)
            top.set_ylim(-5, 110)
            top.grid(alpha=0.3)
            bottom.set_xlabel("error level (0 = nominal)")
            bottom.grid(alpha=0.3)
    axes[0][0].legend(fontsize=7 if compare else 8, loc="lower left")
    if not compare:
        axes[1][0].legend(fontsize=8)
    counts = sorted({s["runs"] for data in datasets for s in data["summary"]})
    per_point = f"{counts[0]}" if len(counts) == 1 else f"{counts[0]}-{counts[-1]}"
    title = Path(datasets[0]["sweep"]).stem if compare else (datasets[0].get("name") or Path(datasets[0]["sweep"]).stem)
    fig.suptitle(f"{title}: success rate vs modeling error ({per_point} runs per point, same commands"
                 f"{'; dashed: success and tracking' if compare else ''})", fontsize=11)
    fig.tight_layout()
    output = args.output or args.summaries[0].with_name("robustness.png")
    fig.savefig(output, dpi=130)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
