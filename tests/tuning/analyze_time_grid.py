#!/usr/bin/env python3
"""Compare solve latency, mesh resolution and measured motion from walking-test prefixes."""

import argparse
import csv
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def table(path):
    with path.open() as file:
        return list(csv.DictReader(file))


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {}
    return dict(mean=float(np.mean(values)), p95=float(np.quantile(values, .95)),
                p99=float(np.quantile(values, .99)), max=float(np.max(values)))


def analyze(prefix):
    result = json.loads(prefix.with_suffix(".json").read_text())
    base = table(prefix.with_suffix(".csv"))
    base = [row for row in base if row["phase"] != "startup"]
    if not base:
        return dict(prefix=prefix.name, verdict=result["verdict"])
    t = np.array([float(row["time"]) for row in base])
    t -= t[0]
    column = lambda name: np.array([float(row[name]) for row in base])
    walking = np.array([row["phase"] == "walk" for row in base])
    summary = dict(prefix=prefix.name, verdict=result["verdict"], min_z=result.get("min_z"),
                   balance_survived=result.get("balance_survived", result["verdict"] in ("SUCCESS", "NO_TRACKING")),
                   max_tilt_rad=float(np.max(np.hypot(column("roll"), column("pitch")))),
                   walk_vx=stats(column("vx_body")[walking]), walk_vy=stats(column("vy_body")[walking]),
                   walk_z_sd=float(np.std(column("z")[walking])) if walking.any() else None,
                   max_x=float(np.max(column("x"))), sim_seconds=result.get("sim_seconds"))
    logs = prefix.with_suffix(".log").read_text(errors="replace")
    rates = [float(x) for x in re.findall(r"\[MPC_TIMING\].*?actual=([\d.]+) Hz", logs)]
    summary["reported_actual_hz"] = stats(rates[2:-1] if len(rates) > 4 else rates)
    summary["solver_exceptions"] = max([int(x) for x in re.findall(r"solver exceptions=(\d+)", logs)], default=0)
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    axes = axes.ravel()
    axes[0].plot(t, column("z")); axes[0].set_ylabel("Base height [m]")
    for name in ("roll", "pitch"):
        axes[1].plot(t, np.degrees(column(name)), label=name)
    axes[1].set_ylabel("Attitude [deg]"); axes[1].legend()
    for name in ("vx_body", "vy_body"):
        axes[2].plot(t, column(name), label=name)
    axes[2].set_ylabel("Measured speed [m/s]"); axes[2].legend()
    solver_file = Path(str(prefix) + "_solver.csv")
    if solver_file.exists():
        solver = table(solver_file)
        st = np.array([float(row["t"]) for row in solver])
        st -= float(base[0]["time"])
        hold = [row for row in solver if row["test_phase"] == "walk"]
        for name in ("advance_ms", "grid_nodes", "grid_min_dt", "grid_max_dt", "grid_fine_max_dt", "eq_sse_near", "eq_sse", "dyn_sse"):
            if hold and name in hold[0]:
                summary[name] = stats([float(row[name]) for row in hold])
        if hold:
            latency = np.array([float(row["advance_ms"]) for row in hold])
            summary["advance_over_8ms_fraction"] = float(np.mean(latency > 8))
            summary["advance_over_10ms_fraction"] = float(np.mean(latency > 10))
        axes[3].plot(st, [float(row["advance_ms"]) for row in solver])
        axes[3].axhline(8, color="red", ls="--", label="125 Hz budget")
        axes[3].axhline(10, color="black", ls=":", label="100 Hz budget")
        axes[3].set_ylabel("MPC advance [ms]"); axes[3].legend()
        for name in ("eq_sse", "eq_sse_near"):
            axes[4].semilogy(st, [max(1e-15, float(row[name])) for row in solver], label=name)
        axes[4].set_ylabel("Equality residual SSE"); axes[4].legend()
        if solver and "grid_nodes" in solver[0]:
            axes[5].plot(st, [float(row["grid_nodes"]) for row in solver])
            axes[5].set_ylabel("Prediction nodes")
    for axis in axes:
        axis.grid(alpha=.2)
        axis.set_xlabel("Time after takeover [s]")
    fig.suptitle(f"{prefix.name}: {result['verdict']}")
    fig.tight_layout()
    fig.savefig(str(prefix) + "_time_grid.png", dpi=140)
    plt.close(fig)
    Path(str(prefix) + "_time_grid.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefixes", nargs="+", type=Path)
    args = parser.parse_args()
    for prefix in args.prefixes:
        print(json.dumps(analyze(prefix)))
