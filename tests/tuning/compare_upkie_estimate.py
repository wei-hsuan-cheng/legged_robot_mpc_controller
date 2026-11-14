#!/usr/bin/env python3
"""Floating-base estimate against simulator ground truth for one wb_id_walk_test run (--record-joints).

usage: compare_upkie_estimate.py PREFIX [--plot]     (pip install numpy matplotlib)

Reads PREFIX_replay.json (GT pose with the latest estimate odometry attached) and PREFIX.csv (GT body twist).
Reports per-phase RMS/max errors of the observable states (height, roll, pitch, body velocity, yaw rate)
and the drift of the gauge states (x, y, yaw), and writes PREFIX_estimate.json and, with --plot, PREFIX_estimate.png.
"""
import csv
import json
import math
import sys

import numpy as np

prefix = sys.argv[1]
samples = [s for s in json.load(open(prefix + "_replay.json")) if s.get("estimate")]
if not samples:
    sys.exit("no estimate in the replay: run with --record-joints and an estimator")
gt_rows = list(csv.DictReader(open(prefix + ".csv")))
gt_t = np.array([float(r["time"]) for r in gt_rows])


def rpy(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


t = np.array([s["time"] for s in samples])
phase = np.array([s["phase"] for s in samples])
gt_p = np.array([s["position"] for s in samples])
gt_rpy = np.array([rpy(s["quaternion_wxyz"]) for s in samples])
idx = np.clip(np.searchsorted(gt_t, t), 0, len(gt_rows) - 1)
gt_v = np.array([[float(gt_rows[i][k]) for k in ("vx_body", "vy_body", "vz_body", "wz_body")] for i in idx])
est_p = np.array([s["estimate"].get("position", [np.nan] * 3) for s in samples])
est_rpy = np.array([s["estimate"]["rpy"] for s in samples])
est_v = np.array([s["estimate"]["linear"] + [s["estimate"]["angular"][2]] for s in samples])
age = t - np.array([s["estimate"]["time"] for s in samples])

wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
err = {
    "height [m]": est_p[:, 2] - gt_p[:, 2],
    "roll [rad]": wrap(est_rpy[:, 0] - gt_rpy[:, 0]),
    "pitch [rad]": wrap(est_rpy[:, 1] - gt_rpy[:, 1]),
    "vx body [m/s]": est_v[:, 0] - gt_v[:, 0],
    "vy body [m/s]": est_v[:, 1] - gt_v[:, 1],
    "vz body [m/s]": est_v[:, 2] - gt_v[:, 2],
    "yaw rate [rad/s]": est_v[:, 3] - gt_v[:, 3],
}
report = {"samples": int(t.size), "max estimate age [s]": float(np.max(age))}
for name in ["all", *dict.fromkeys(phase)]:
    mask = np.ones_like(t, bool) if name == "all" else phase == name
    if mask.sum() < 2:
        continue
    report[name] = {k: {"rms": float(np.sqrt(np.nanmean(e[mask] ** 2))), "max": float(np.nanmax(np.abs(e[mask])))}
                    for k, e in err.items()}
# Gauge states: drift from the first sample.
d = lambda a: a - a[0]
report["drift over the run"] = {
    "x [m]": float(d(est_p[:, 0] - gt_p[:, 0])[-1]), "y [m]": float(d(est_p[:, 1] - gt_p[:, 1])[-1]),
    "yaw [rad]": float(wrap(d(est_rpy[:, 2] - gt_rpy[:, 2]))[-1]),
    "distance travelled [m]": float(np.sum(np.linalg.norm(np.diff(gt_p[:, :2], axis=0), axis=1))),
}
json.dump(report, open(prefix + "_estimate.json", "w"), indent=1)
a = report["all"]
print(f"{prefix}: height rms {1000 * a['height [m]']['rms']:.1f} mm max {1000 * a['height [m]']['max']:.1f} | "
      f"roll/pitch rms {a['roll [rad]']['rms']:.4f}/{a['pitch [rad]']['rms']:.4f} rad | "
      f"vx rms {a['vx body [m/s]']['rms']:.3f} max {a['vx body [m/s]']['max']:.3f} m/s | "
      f"yaw rate rms {a['yaw rate [rad/s]']['rms']:.3f} | drift x {report['drift over the run']['x [m]']:+.3f} m "
      f"yaw {report['drift over the run']['yaw [rad]']:+.3f} rad over {report['drift over the run']['distance travelled [m]']:.2f} m")

if "--plot" in sys.argv:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = t - t[0]
    fig, ax = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    ax[0].plot(t0, gt_p[:, 2], label="GT"); ax[0].plot(t0, est_p[:, 2], "--", label="estimate"); ax[0].set_ylabel("base z [m]")
    for k, (i, n) in enumerate(((1, "pitch"), (0, "roll"))):
        ax[1].plot(t0, gt_rpy[:, i], label=n + " GT"); ax[1].plot(t0, est_rpy[:, i], "--", label=n + " estimate")
    ax[1].set_ylabel("rad")
    ax[2].plot(t0, gt_v[:, 0], label="GT"); ax[2].plot(t0, est_v[:, 0], "--", label="estimate"); ax[2].set_ylabel("vx body [m/s]")
    ax[3].plot(t0, gt_v[:, 1], label="vy GT"); ax[3].plot(t0, est_v[:, 1], "--", label="vy estimate")
    ax[3].plot(t0, gt_v[:, 2], label="vz GT"); ax[3].plot(t0, est_v[:, 2], "--", label="vz estimate"); ax[3].set_ylabel("m/s")
    ax[4].plot(t0, gt_v[:, 3], label="GT"); ax[4].plot(t0, est_v[:, 3], "--", label="estimate"); ax[4].set_ylabel("yaw rate [rad/s]")
    ax[5].plot(t0, est_p[:, 0] - gt_p[:, 0] - (est_p[0, 0] - gt_p[0, 0]), label="x drift [m]")
    ax[5].plot(t0, est_p[:, 1] - gt_p[:, 1] - (est_p[0, 1] - gt_p[0, 1]), label="y drift [m]")
    ax[5].plot(t0, wrap(est_rpy[:, 2] - gt_rpy[:, 2] - (est_rpy[0, 2] - gt_rpy[0, 2])), label="yaw drift [rad]")
    ax[5].set_xlabel("time [s]")
    changes = [i for i in range(1, len(phase)) if phase[i] != phase[i - 1]]
    for a_ in ax:
        for i in changes:
            a_.axvline(t0[i], color="0.8", lw=0.6)
        a_.legend(loc="upper left", fontsize=7); a_.grid(alpha=0.3)
    ax[0].set_title(prefix.split("/")[-1] + ": estimate vs GT")
    fig.tight_layout(); fig.savefig(prefix + "_estimate.png", dpi=80)
    print("wrote", prefix + "_estimate.png")
