#!/usr/bin/env python3
"""Plot solver feasibility, actuation and initial-state consistency from a walk-test prefix."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import yaml


def load(path):
    with path.open() as file:
        rows = list(csv.DictReader(file, skipinitialspace=True))
    if not rows:
        raise ValueError(f"No samples in {path}")
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0] if key != "test_phase" and key != "phase"}


def analyze(prefix, start=5.0, end=12.0):
    with prefix.with_suffix(".csv").open() as file:
        gt_rows = [r for r in csv.DictReader(file) if r["phase"] != "startup"]
    if not gt_rows:
        raise ValueError(f"No post-startup samples in {prefix}")
    t0 = float(gt_rows[0]["time"])
    gt = {key: np.array([float(r[key]) for r in gt_rows]) for key in gt_rows[0] if key != "phase"}
    tr = load(Path(str(prefix) + "_tracking.csv"))
    so = load(Path(str(prefix) + "_solver.csv"))
    st = so["t"] - t0
    mask = (st >= start) & (st <= end)
    if not mask.any():
        raise ValueError(f"No solver samples between {start} and {end} seconds in {prefix}")
    nx = len([k for k in so if k.startswith("x0/")])
    config = yaml.safe_load(prefix.with_suffix(".yaml").read_text())
    tangent = config["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]["ocs2"]["model"].get(
        "orientationRepresentation", "euler_zyx") == "tangent_space"
    initial_orientation_error = np.full(len(st), np.nan)
    initial_joint_error = np.full(len(st), np.nan)
    if nx:
        joint_names = [key.split("/", 1)[1] for key in tr if key.startswith("meas_q/")]
        for i, time in enumerate(so["t"]):
            rotation = np.array([so[f"x0/{j}"][i] for j in (3, 4, 5)])
            if tangent:
                q0 = np.array([so[f"x0/{j}"][i] for j in (nx - 1, nx - 4, nx - 3, nx - 2)])
                mujoco.mju_quatIntegrate(q0, rotation, 1.0)
            else:
                q0 = np.zeros(4)
                mujoco.mju_euler2Quat(q0, rotation, "zyx")
            angles = np.array([np.interp(time, tr["t"], tr["obs_base/" + axis]) for axis in ("yaw", "pitch", "roll")])
            observed = np.zeros(4)
            mujoco.mju_euler2Quat(observed, angles, "zyx")
            error = np.zeros(3)
            mujoco.mju_subQuat(error, q0, observed)
            initial_orientation_error[i] = np.linalg.norm(error)
            initial_joint_error[i] = max(abs(so[f"x0/{6 + j}"][i] - np.interp(time, tr["t"], tr["meas_q/" + name]))
                                         for j, name in enumerate(joint_names))
    summary = {"prefix": str(prefix), "window": [start, end], "samples": int(mask.sum())}
    for key in so:
        if key.startswith("plan/"):
            summary[key] = {"min": float(np.nanmin(so[key][mask])), "max": float(np.nanmax(so[key][mask]))}
    if nx:
        summary["initial_orientation_error_max_rad"] = float(np.nanmax(initial_orientation_error[mask]))
        summary["initial_joint_error_max_rad"] = float(np.nanmax(initial_joint_error[mask]))
    Path(str(prefix) + "_numerics.json").write_text(json.dumps(summary, indent=2) + "\n")
    fig, axes = plt.subplots(4, 2, figsize=(14, 13), sharex=True)
    ax = axes.ravel()
    tt, gg = tr["t"] - t0, gt["time"] - t0
    tm, gm = (tt >= start) & (tt <= end), (gg >= start) & (gg <= end)
    for name in ("obs_base/z", "plan_base/z"):
        ax[0].plot(tt[tm], tr[name][tm], label=name)
    ax[0].plot(gg[gm], gt["z"][gm], label="GT z")
    ax[0].set_ylabel("Base height [m]")
    ax[1].plot(gg[gm], np.rad2deg(np.hypot(gt["roll"][gm], gt["pitch"][gm])), label="GT tilt")
    ax[1].plot(tt[tm], np.rad2deg(np.hypot(tr["obs_base/roll"][tm], tr["obs_base/pitch"][tm])), label="estimated tilt")
    ax[1].set_ylabel("RP magnitude [deg]")
    for name in ("obs_base/vx", "plan_base/vx"):
        ax[2].plot(tt[tm], tr[name][tm], label=name)
    if np.all(np.diff(gt["time"]) > 0):
        gt_vx = np.gradient(gt["x"], gt["time"])
        ax[2].plot(gg[gm], gt_vx[gm], label="GT pose-derived vx", alpha=0.6)
    ax[2].set_ylabel("World forward velocity [m/s]")
    for name in (k for k in tr if k.startswith("plan_wrench/") and k.endswith("/fz")):
        ax[3].plot(tt[tm], tr[name][tm], label=name)
    ax[3].set_ylabel("Planned normal force [N]")
    if "plan/near_torque_ratio_max" in so:
        ax[4].plot(st[mask], so["plan/near_torque_ratio_max"][mask], label="near planned torque / limit")
    ax[4].axhline(1, color="black", linestyle="--", label="limit")
    ax[4].set_ylabel("Torque ratio")
    for name in ("plan/near_acceleration_max", "plan/acceleration_max"):
        if name in so:
            ax[5].plot(st[mask], so[name][mask], label=name)
    ax[5].set_yscale("symlog", linthresh=10)
    ax[5].set_ylabel("Max generalized acceleration")
    for name in ("eq_sse", "eq_sse_near", "dyn_sse"):
        ax[6].plot(st[mask], so[name][mask], label=name)
    ax[6].set_yscale("symlog", linthresh=0.01)
    ax[6].set_ylabel("Logged constraint SSE")
    ax[7].plot(st[mask], np.rad2deg(initial_orientation_error[mask]), label="initial attitude error")
    ax[7].plot(st[mask], np.rad2deg(initial_joint_error[mask]), label="max initial joint error")
    ax[7].set_ylabel("Plan initial state vs observation [deg]")
    for a in ax:
        a.set_xlim(start, min(end, gg[-1]))
        a.grid(alpha=0.3)
        a.legend(fontsize=7)
        a.set_xlabel("Time after MPC takeover [s]")
    fig.suptitle(prefix.name)
    fig.tight_layout()
    fig.savefig(str(prefix) + "_numerics.png", dpi=130)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path, nargs="+")
    parser.add_argument("--start", type=float, default=5.0, help="Window start after MPC takeover [s]")
    parser.add_argument("--end", type=float, default=12.0, help="Window end after MPC takeover [s]")
    args = parser.parse_args()
    if args.start >= args.end:
        parser.error("start must precede end")
    for prefix in args.prefix:
        analyze(prefix, args.start, args.end)
