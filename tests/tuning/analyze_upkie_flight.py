#!/usr/bin/env python3
"""Planned and physical contact phases of an Upkie run (hops, wheel lifts): wb_id_walk_test --record-tracking.

usage: analyze_upkie_flight.py PREFIX [--plot]     (pip install numpy matplotlib)

A wheel counts as physically off the ground when its simulated contact sensor reads below 0.5 N. For every planned
non-STANCE interval it reports the measured off-ground time of each wheel, the tire-centre clearance, the base apex,
the peak landing force and the base attitude; also the share of samples at the joint torque limits.
Writes PREFIX_flight.json and, with --plot, PREFIX_flight.png.
"""
import csv
import json
import sys

import numpy as np

TIRE_RADIUS = 0.08  # [m] the enlarged wheel (upkie.xml tire_collision); the original Upkie wheel is 0.055

prefix = sys.argv[1]
rows = list(csv.DictReader(open(prefix + "_tracking.csv")))
col = lambda k: np.array([float(r[k]) if r.get(k) not in (None, "", "nan") else np.nan for r in rows])
t = col("t") - col("t")[0]
mode = col("plan_mode")
force = {s: col(f"meas_contact/{s}_wheel_contact/normal_force") for s in ("left", "right")}
off = {s: force[s] < 0.5 for s in force}
tire_z = {s: col(f"obs_foot/{s}_tire_center/z") for s in force}
z, pitch, roll = col("obs_base/z"), col("obs_base/pitch"), col("obs_base/roll")
limits = {"hip": 16.0, "knee": 16.0, "wheel": 1.7}
tau = {f"{s}_{j}": col(f"cmd_tau/{s}_{j}") for s in force for j in limits}
saturated = {k: float(np.mean(np.abs(v) >= 0.99 * limits[k.split("_")[1]])) for k, v in tau.items()}

# Mode numbers: FLY 0, RF 1 (right in contact), LF 2 (left in contact), STANCE 3.
planned_air = {"left": np.isin(mode, (0, 1)), "right": np.isin(mode, (0, 2))}
intervals = []
nonstance = np.where(np.nan_to_num(mode, nan=3) != 3)[0]
if nonstance.size:
    splits = np.split(nonstance, np.where(np.diff(nonstance) > 1)[0] + 1)
    for k, idx in enumerate(splits):
        a, b = idx[0], idx[-1]
        # 0.2 s before and up to 0.6 s after (100 Hz), stopping short of the next planned interval.
        end = min(b + 60, len(t), splits[k + 1][0] - 1 if k + 1 < len(splits) else len(t))
        window = slice(max(a - 20, 0), end)
        entry = {"mode": int(mode[a]), "planned [s]": [round(t[a], 3), round(t[b] + (t[1] - t[0]), 3)]}
        for s in force:
            w = off[s][window]
            entry[f"{s} off ground [s]"] = round(float(w.sum() * np.median(np.diff(t))), 3)
            entry[f"{s} tire clearance [mm]"] = round(float(1000 * (np.nanmax(tire_z[s][window]) - TIRE_RADIUS)), 1)
            entry[f"{s} peak landing force [N]"] = round(float(np.nanmax(force[s][b:window.stop])), 1)
        entry["base apex rise [mm]"] = round(float(1000 * (np.nanmax(z[window]) - z[max(a - 20, 0)])), 1)
        entry["max |pitch| / |roll| [rad]"] = [round(float(np.nanmax(np.abs(pitch[window]))), 3),
                                               round(float(np.nanmax(np.abs(roll[window]))), 3)]
        intervals.append(entry)
report = {"planned non-stance intervals": intervals,
          "samples with both wheels off": int(np.sum(off["left"] & off["right"])),
          "max |pitch| / |roll| over the run [rad]": [float(np.nanmax(np.abs(pitch))), float(np.nanmax(np.abs(roll)))],
          "torque at limit (share of samples)": saturated}
json.dump(report, open(prefix + "_flight.json", "w"), indent=1)
print(f"{prefix}: {len(intervals)} planned non-stance intervals, {report['samples with both wheels off']} samples "
      f"with both wheels off; max pitch/roll {report['max |pitch| / |roll| over the run [rad]'][0]:.3f}/"
      f"{report['max |pitch| / |roll| over the run [rad]'][1]:.3f}")
for e in intervals:
    print("  ", e)

if "--plot" in sys.argv:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    ax[0].plot(t, mode, label="planned mode (0 FLY, 1 RF, 2 LF, 3 STANCE)"); ax[0].set_ylabel("mode")
    for s in force:
        ax[1].plot(t, force[s], label=f"{s} wheel normal force")
        ax[2].plot(t, 1000 * (tire_z[s] - TIRE_RADIUS), label=f"{s} tire clearance")
    ax[1].set_ylabel("N"); ax[2].set_ylabel("mm")
    ax[3].plot(t, z, label="base z GT"); ax[3].plot(t, col("plan_base/z"), "--", label="base z plan"); ax[3].set_ylabel("m")
    ax[4].plot(t, pitch, label="pitch"); ax[4].plot(t, roll, label="roll"); ax[4].set_ylabel("rad"); ax[4].set_xlabel("time [s]")
    for a in ax:
        a.legend(loc="upper left", fontsize=7); a.grid(alpha=0.3)
    ax[0].set_title(prefix.split("/")[-1] + ": planned vs physical contact")
    fig.tight_layout(); fig.savefig(prefix + "_flight.png", dpi=80)
    print("wrote", prefix + "_flight.png")
