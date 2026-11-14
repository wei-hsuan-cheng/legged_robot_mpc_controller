#!/usr/bin/env python3
"""Wheeled-biped (Upkie) run analysis: the signals the first acceptance test asks for.

    python3 analyze_wheeled_biped.py PREFIX [--plot]

Reads PREFIX_tracking.csv and PREFIX_solver.csv written by tests/wb_id_walk_test.py with
--record-tracking, and reports per test phase:
  - base pitch and forward velocity (heading frame) against the command,
  - commanded feed-forward and applied effort for all six joints, and how often the applied effort
    sits at its limit,
  - each wheel's measured normal force and contact flag (MuJoCo contact sensors) and its tread slip
    and gap at the contact point (obs_wheel columns),
  - the solver's dynamics and per-term equality residuals and solve time.
Writes PREFIX_wheeled_summary.json, and PREFIX_wheeled.png with --plot.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

EFFORT_LIMITS = {"hip": 16.0, "knee": 16.0, "wheel": 1.7}


def read_csv(path):
    with open(path) as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = list(reader)
    phases = np.array([row[0] for row in rows])
    data = np.array([[float(v) if v not in ("", "nan") else math.nan for v in row[1:]] for row in rows]) if rows else np.zeros((0, len(header) - 1))
    return {name: data[:, i] for i, name in enumerate(header[1:])}, phases


def stats(values):
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return dict(mean=float(np.mean(values)), min=float(np.min(values)), max=float(np.max(values)),
                rms=float(np.sqrt(np.mean(values ** 2))), max_abs=float(np.max(np.abs(values))))


def limit_of(joint):
    for kind, limit in EFFORT_LIMITS.items():
        if joint.endswith(kind):
            return limit
    return math.nan


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefix")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--saturation", type=float, default=0.98, help="Fraction of the limit counted as saturated")
    args = parser.parse_args()

    tracking, phases = read_csv(args.prefix + "_tracking.csv")
    solver, solver_phases = read_csv(args.prefix + "_solver.csv") if Path(args.prefix + "_solver.csv").exists() else ({}, np.array([]))
    joints = [name.split("/", 1)[1] for name in tracking if name.startswith("cmd_tau/")]
    wheels = sorted({name.split("/")[1] for name in tracking if name.startswith("obs_wheel/")})
    sensors = sorted({name.split("/")[1] for name in tracking if name.startswith("meas_contact/")})

    t = tracking["t"]
    yaw = tracking["obs_base/yaw"]
    v_forward = np.cos(yaw) * tracking["obs_base/vx"] + np.sin(yaw) * tracking["obs_base/vy"]
    v_lateral = -np.sin(yaw) * tracking["obs_base/vx"] + np.cos(yaw) * tracking["obs_base/vy"]
    plan_forward = np.cos(yaw) * tracking["plan_base/vx"] + np.sin(yaw) * tracking["plan_base/vy"]

    summary = dict(prefix=args.prefix, samples=int(t.size), joints=joints, wheels=wheels, contact_sensors=sensors, phases={})
    for phase in dict.fromkeys(phases):
        mask = phases == phase
        entry = dict(duration=float(t[mask][-1] - t[mask][0]) if mask.sum() > 1 else 0.0,
                     pitch=stats(tracking["obs_base/pitch"][mask]),
                     roll=stats(tracking["obs_base/roll"][mask]),
                     height=stats(tracking["obs_base/z"][mask]),
                     v_forward=stats(v_forward[mask]),
                     v_lateral=stats(v_lateral[mask]),
                     plan_v_forward=stats(plan_forward[mask]))
        effort = {}
        for joint in joints:
            limit = limit_of(joint)
            applied = tracking["meas_tau/" + joint][mask]
            finite = applied[np.isfinite(applied)]
            effort[joint] = dict(feedforward=stats(tracking["cmd_tau/" + joint][mask]),
                                 applied=stats(applied),
                                 limit=limit,
                                 saturated_fraction=float(np.mean(np.abs(finite) >= args.saturation * limit)) if finite.size else None)
        entry["effort"] = effort
        entry["wheels"] = {wheel: dict(gap=stats(tracking[f"obs_wheel/{wheel}/gap"][mask]),
                                       slip_lon=stats(tracking[f"obs_wheel/{wheel}/slip_lon"][mask]),
                                       slip_lat=stats(tracking[f"obs_wheel/{wheel}/slip_lat"][mask]),
                                       plan_fz=stats(tracking[f"plan_wrench/{wheel}/fz"][mask]))
                           for wheel in wheels}
        entry["contact"] = {sensor: dict(normal_force=stats(tracking[f"meas_contact/{sensor}/normal_force"][mask]),
                                         in_contact_fraction=float(np.nanmean(tracking[f"meas_contact/{sensor}/contact"][mask] > 0.5)))
                            for sensor in sensors}
        if solver:
            smask = solver_phases == phase
            entry["solver"] = {name: stats(solver[name][smask]) for name in solver
                               if name in ("advance_ms", "dyn_sse", "eq_sse") or name.startswith("eq_sse/")}
        summary["phases"][phase] = entry

    out = Path(args.prefix + "_wheeled_summary.json")
    out.write_text(json.dumps(summary, indent=2) + "\n")

    for phase, entry in summary["phases"].items():
        pitch, vf = entry["pitch"], entry["v_forward"]
        line = f"{phase:9s} {entry['duration']:6.2f} s  pitch max|.| {pitch['max_abs']:.3f} rad  v_fwd mean {vf['mean']:+.3f} m/s"
        for sensor, contact in entry["contact"].items():
            nf = contact["normal_force"]
            line += f"  {sensor}: Fn {nf['min']:.1f}-{nf['max']:.1f} N, contact {contact['in_contact_fraction']:.3f}"
        print(line)
        for wheel, values in entry["wheels"].items():
            print(f"          {wheel}: slip lon rms {values['slip_lon']['rms']:.4f} max {values['slip_lon']['max_abs']:.4f}, "
                  f"lat rms {values['slip_lat']['rms']:.4f}, gap max|.| {values['gap']['max_abs']:.4f} m")
        sat = {joint: e["saturated_fraction"] for joint, e in entry["effort"].items()}
        print("          saturated fraction:", ", ".join(f"{j} {v:.3f}" for j, v in sat.items() if v is not None))
        if "solver" in entry and entry["solver"].get("advance_ms"):
            print(f"          solve {entry['solver']['advance_ms']['mean']:.2f} ms (max {entry['solver']['advance_ms']['max']:.2f}), "
                  f"eq_sse mean {entry['solver']['eq_sse']['mean']:.2e}")
    print("wrote", out)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(6, 1, figsize=(12, 18), sharex=True)
        axes[0].plot(t, tracking["obs_base/pitch"], label="pitch")
        axes[0].plot(t, tracking["obs_base/roll"], label="roll")
        axes[0].set_ylabel("rad"); axes[0].legend(); axes[0].set_title("base attitude")
        axes[1].plot(t, v_forward, label="measured forward")
        axes[1].plot(t, plan_forward, label="planned forward", alpha=0.7)
        axes[1].plot(t, v_lateral, label="measured lateral", alpha=0.7)
        axes[1].set_ylabel("m/s"); axes[1].legend(); axes[1].set_title("base velocity (heading frame)")
        for sensor in sensors:
            axes[2].plot(t, tracking[f"meas_contact/{sensor}/normal_force"], label=f"{sensor} measured")
        for wheel in wheels:
            axes[2].plot(t, tracking[f"plan_wrench/{wheel}/fz"], "--", label=f"{wheel} planned fz")
        axes[2].set_ylabel("N"); axes[2].legend(); axes[2].set_title("wheel normal force")
        for wheel in wheels:
            axes[3].plot(t, tracking[f"obs_wheel/{wheel}/slip_lon"], label=f"{wheel} lon")
            axes[3].plot(t, tracking[f"obs_wheel/{wheel}/slip_lat"], label=f"{wheel} lat", alpha=0.7)
        axes[3].set_ylabel("m/s"); axes[3].legend(); axes[3].set_title("tread slip at the contact point")
        for joint in joints:
            axes[4 if not joint.endswith("wheel") else 5].plot(t, tracking["meas_tau/" + joint], label=f"{joint} applied")
            axes[4 if not joint.endswith("wheel") else 5].plot(t, tracking["cmd_tau/" + joint], "--", label=f"{joint} ff", alpha=0.6)
        for ax, limit in ((axes[4], 16.0), (axes[5], 1.7)):
            ax.axhline(limit, color="k", lw=0.8); ax.axhline(-limit, color="k", lw=0.8)
            ax.set_ylabel("N m"); ax.legend(ncol=4, fontsize=7)
        axes[4].set_title("hip/knee effort"); axes[5].set_title("wheel effort")
        axes[5].set_xlabel("time [s]")
        # Phase boundaries
        for ax in axes:
            for i in range(1, len(phases)):
                if phases[i] != phases[i - 1]:
                    ax.axvline(t[i], color="grey", lw=0.6, ls=":")
        fig.tight_layout()
        fig.savefig(args.prefix + "_wheeled.png", dpi=90)
        print("wrote", args.prefix + "_wheeled.png")


if __name__ == "__main__":
    main()
