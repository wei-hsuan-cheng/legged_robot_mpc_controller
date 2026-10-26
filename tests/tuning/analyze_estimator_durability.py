#!/usr/bin/env python3
"""Analyze estimator_durability_test.py runs: estimator drift over time and its effect on control.

The estimate frame is aligned to ground truth once, from the first two seconds of the settle phase
(yaw and horizontal position are unobservable gauge states, so only their drift from that alignment
means anything). Height, roll, pitch and velocities are compared directly.
Per run it writes <prefix>_durability.png and <prefix>_durability.json and prints one summary line;
with several prefixes it also prints a matrix table.
"""

import argparse
import csv
import json
import re
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def load(prefix):
    with open(str(prefix) + ".csv") as file:
        rows = list(csv.reader(file))
    header, rows = rows[0], rows[1:]
    phase = np.array([r[0] for r in rows])
    data = np.array([[float(x) for x in r[1:]] for r in rows])
    return phase, {name: data[:, i] for i, name in enumerate(header[1:])}


def log_health(prefix):
    text = Path(str(prefix) + ".log").read_text(errors="replace") if Path(str(prefix) + ".log").exists() else ""
    stamp = lambda line: float(re.search(r"\[(\d+\.\d+)\]", line).group(1))
    timing = [(stamp(l), float(re.search(r"actual=([\d.]+) Hz", l).group(1))) for l in text.splitlines() if "MPC_TIMING]" in l]
    health = [(stamp(l), float(re.search(r"equality SSE mean=([\d.e+-]+)", l).group(1)),
               int(re.search(r"torque-window overruns=(\d+)", l).group(1)), int(re.search(r"solver exceptions=(\d+)", l).group(1)))
              for l in text.splitlines() if "MPC_HEALTH] equality SSE" in l]
    return np.array(timing), np.array(health)


def windows(t, mask, width):
    edges = np.arange(t[mask][0], t[mask][-1] + 1e-9, width) if mask.any() else []
    return [(a, mask & (t >= a) & (t < a + width)) for a in edges]


def growth(series, t, lags=(60.0, 300.0, 900.0, 1800.0)):
    """Mean squared change of a 1 Hz error series over each lag, and what its growth implies.

    Bounded (mean-reverting) error saturates; a random walk grows like the lag; a steady drift
    grows like the lag squared. The class compares the largest available lag with the 60 s lag.
    """
    seconds = np.floor(t).astype(int)
    keep = np.isfinite(series)
    values = np.bincount(seconds[keep], weights=series[keep]) / np.maximum(np.bincount(seconds[keep]), 1)
    values = values[np.bincount(seconds[keep]) > 0]
    out = {}
    for lag in lags:
        n = int(lag)
        if len(values) > 2 * n:
            out[int(lag)] = float(np.mean((values[n:] - values[:-n]) ** 2))
    if len(out) >= 2 and out[min(out)] > 0:
        lag_ratio = max(out) / min(out)
        ratio = out[max(out)] / out[min(out)]
        kind = ("bounded" if ratio < 3 else "steady drift" if ratio > 0.5 * lag_ratio ** 2
                else "random walk" if ratio > 0.3 * lag_ratio else "slowly growing")
        return dict(msd=out, growth_ratio=float(ratio), lag_ratio=float(lag_ratio), kind=kind)
    return dict(msd=out, kind="too short")


def analyze(prefix, plot=True):
    result = json.loads(Path(str(prefix) + ".json").read_text())
    phase, d = load(prefix)
    t = d["t"] - d["t"][0]
    # The simulator's ground-truth odometry reports ~0 angular z even while the robot turns, so the
    # true yaw rate comes from the yaw angle itself (0.5 s centred mean of its derivative).
    d["gt_wz_field"] = d["gt_wz"]
    rate = np.gradient(np.unwrap(d["gt_yaw"]), d["t"])
    d["gt_wz"] = np.convolve(rate, np.ones(11) / 11, mode="same")
    ok = np.isfinite(d["est_x"])
    first = ok & (phase == "settle") & (t < t[ok][0] + 2.0)
    yaw0 = float(np.angle(np.mean(np.exp(1j * (d["est_yaw"][first] - d["gt_yaw"][first])))))
    c, s = np.cos(yaw0), np.sin(yaw0)
    gt_xy = np.column_stack([c * d["gt_x"] - s * d["gt_y"], s * d["gt_x"] + c * d["gt_y"]])
    offset = np.mean(np.column_stack([d["est_x"], d["est_y"]])[first] - gt_xy[first], axis=0)
    err = dict(
        xy=np.linalg.norm(np.column_stack([d["est_x"], d["est_y"]]) - gt_xy - offset, axis=1),
        # Unwrapped: a yaw estimate drifting through whole turns must not fold back into [-pi, pi).
        yaw=np.unwrap(np.where(ok, d["est_yaw"], 0.0)) - np.unwrap(d["gt_yaw"]) - yaw0, z=d["est_z"] - d["gt_z"],
        roll=d["est_roll"] - d["gt_roll"], pitch=d["est_pitch"] - d["gt_pitch"],
        vx=d["est_vx"] - d["gt_vx"], vy=d["est_vy"] - d["gt_vy"], vz=d["est_vz"] - d["gt_vz"], wz=d["est_wz"] - d["gt_wz"])
    hold = ok & (phase == "hold")
    minutes = []
    for start, m in windows(t, hold, 60.0):
        if m.sum() < 10:
            continue
        minutes.append(dict(
            t_min=float((start - t[hold][0]) / 60), xy_drift=float(err["xy"][m][-1]), yaw_drift=float(err["yaw"][m][-1]),
            z_err=float(err["z"][m].mean()), roll_err=float(err["roll"][m].mean()), pitch_err=float(err["pitch"][m].mean()),
            **{f"{k}_bias": float(err[k][m].mean()) for k in ("vx", "vy", "vz", "wz")},
            **{f"{k}_rms": float(np.sqrt(np.mean(err[k][m] ** 2))) for k in ("vx", "vy", "vz", "wz")},
            gt_vx=float(d["gt_vx"][m].mean()), gt_wz=float(d["gt_wz"][m].mean()), gt_z=float(d["gt_z"][m].mean()),
            gt_z_sd=float(d["gt_z"][m].std()), gt_tilt_rms=float(np.sqrt(np.mean(d["gt_roll"][m] ** 2 + d["gt_pitch"][m] ** 2)))))
    timing, health = log_health(prefix)
    summary = dict(prefix=Path(prefix).name, verdict=result.get("verdict"), robot=result.get("robot_name"),
                   gt_wz_field_mean=float(np.nanmean(d["gt_wz_field"][hold])) if hold.any() else None,
                   estimator=result.get("estimator"), vx_cmd=result.get("vx"), yaw_cmd=result.get("yaw_rate"),
                   sim_seconds=result.get("sim_seconds"), fall_time=result.get("fall_time"),
                   hold_minutes=len(minutes), yaw0=yaw0)
    if minutes:
        mt = np.array([m["t_min"] for m in minutes])
        col = lambda k: np.array([m[k] for m in minutes])
        slope = lambda k: float(np.polyfit(mt / 60, col(k), 1)[0]) if len(mt) > 2 else None   # per hour
        early, late = mt < 10, mt >= mt[-1] - 10
        summary.update(
            final=dict(xy_drift=minutes[-1]["xy_drift"], yaw_drift=minutes[-1]["yaw_drift"]),
            drift_per_hour={k: slope(k) for k in ("xy_drift", "yaw_drift", "z_err", "roll_err", "pitch_err", "vx_bias", "vy_bias", "wz_bias")},
            first10_vs_last10={k: [float(col(k)[early].mean()), float(col(k)[late].mean())]
                               for k in ("z_err", "roll_err", "pitch_err", "vx_bias", "vy_bias", "wz_bias", "vx_rms", "wz_rms",
                                         "gt_vx", "gt_wz", "gt_z", "gt_z_sd", "gt_tilt_rms")},
            worst_minute={k: float(np.max(np.abs(col(k)))) for k in ("z_err", "roll_err", "pitch_err", "vx_rms", "vy_rms", "vz_rms", "wz_rms")})
    if len(timing):
        summary["mpc_rate"] = dict(mean=float(timing[:, 1].mean()), min=float(timing[:, 1].min()),
                                   first10=float(timing[timing[:, 0] < timing[0, 0] + 600, 1].mean()),
                                   last10=float(timing[timing[:, 0] > timing[-1, 0] - 600, 1].mean()))
    if len(health):
        summary["solver"] = dict(equality_sse_mean=float(health[:, 1].mean()), torque_window_overruns=int(health[:, 2].max()),
                                 exceptions=int(health[:, 3].max()))
    if hold.sum() > 600:
        th = t[hold] - t[hold][0]
        behaviour = {}
        # Below these sizes an error cannot matter to control, whatever its trend.
        negligible = dict(z=0.002, roll=0.00175, pitch=0.00175, vx=0.01, vy=0.01, vz=0.01, wz=0.01, yaw=0.0175, xy=0.05)
        for key in ("z", "roll", "pitch", "vx", "vy", "vz", "wz", "yaw", "xy"):
            g = growth(err[key][hold], th)
            g["max_abs"] = float(np.nanmax(np.abs(err[key][hold])))
            if key not in ("vx", "vy", "vz", "wz") and g["max_abs"] < negligible[key]:
                g["kind"] = "negligible (" + g["kind"] + ")"
            if minutes and key in ("z", "roll", "pitch", "vx", "vy", "vz", "wz"):
                means = np.array([m[{"z": "z_err", "roll": "roll_err", "pitch": "pitch_err"}.get(key, key + "_bias")] for m in minutes])
                g.update(positive_minute_fraction=float(np.mean(means > 0)),
                         sign_changes=int(np.sum(np.diff(np.sign(means)) != 0)), minutes=len(means))
            behaviour[key] = g
        summary["error_behaviour"] = behaviour
    rss = [m["rss_mb"] for m in result.get("minutes", []) if m.get("rss_mb")]
    if rss:
        summary["controller_rss_mb"] = [rss[0], rss[-1]]
    Path(str(prefix) + "_durability.json").write_text(json.dumps(dict(summary=summary, minutes=minutes), indent=2) + "\n")

    if plot and minutes:
        mt = np.array([m["t_min"] for m in minutes])
        col = lambda k: np.array([m[k] for m in minutes])
        fig, axes = plt.subplots(5, 1, figsize=(11, 14), sharex=True)
        axes[0].plot(mt, col("xy_drift"), label="xy position drift [m]")
        axes[0].plot(mt, np.degrees(col("yaw_drift")) / 100, label="yaw drift [deg / 100]")
        axes[0].set_ylabel("Gauge drift")
        axes[1].plot(mt, 100 * col("z_err"), label="height error [cm]")
        axes[1].plot(mt, np.degrees(col("roll_err")), label="roll error [deg]")
        axes[1].plot(mt, np.degrees(col("pitch_err")), label="pitch error [deg]")
        axes[1].set_ylabel("Observable error (1 min mean)")
        for k in ("vx", "vy", "vz"):
            axes[2].plot(mt, col(k + "_bias"), label=f"{k} bias [m/s]")
        axes[2].plot(mt, col("wz_bias"), label="wz bias [rad/s]")
        axes[2].plot(mt, col("vx_rms"), ":", label="vx error RMS [m/s]")
        axes[2].set_ylabel("Velocity error")
        axes[3].plot(mt, col("gt_vx"), label="true forward speed [m/s]")
        axes[3].plot(mt, col("gt_wz"), label="true yaw rate [rad/s]")
        axes[3].axhline(result.get("vx", 0), color="C0", ls="--", lw=0.8)
        axes[3].axhline(result.get("yaw_rate", 0), color="C1", ls="--", lw=0.8)
        axes[3].plot(mt, col("gt_z"), label="true base height [m]")
        axes[3].plot(mt, np.degrees(col("gt_tilt_rms")) / 10, label="true tilt RMS [deg / 10]")
        axes[3].set_ylabel("Control outcome")
        if len(timing):
            axes[4].plot((timing[:, 0] - timing[0, 0]) / 60, timing[:, 1], lw=0.6, label="MPC rate [Hz]")
        if rss:
            rt = [m["t"] / 60 for m in result["minutes"] if m.get("rss_mb")]
            axes[4].plot(rt, np.array(rss) / 20, label="controller RSS [MB / 20]")
        axes[4].set_ylabel("Runtime")
        axes[-1].set_xlabel("Hold time [min]")
        for axis in axes:
            axis.legend(fontsize=8, loc="upper left")
            axis.grid(alpha=0.3)
        fig.suptitle(f"{Path(prefix).name}: {summary['verdict']}")
        fig.tight_layout()
        fig.savefig(str(prefix) + "_durability.png", dpi=110)
        plt.close(fig)
    return summary


def behaviour_line(s):
    b = s.get("error_behaviour", {})
    return "  error behaviour: " + ", ".join(
        f"{k} {v['kind']}" + (f" (x{v['growth_ratio']:.1f} over x{v['lag_ratio']:.0f} lag"
                              + (f", +{100*v['positive_minute_fraction']:.0f}% min, {v['sign_changes']} flips" if "sign_changes" in v else "") + ")"
                              if "growth_ratio" in v else "")
        for k, v in b.items())


def line(s):
    if "first10_vs_last10" not in s:
        return f"{s['prefix']}: {s['verdict']} (no hold data) fall_time={s.get('fall_time')}"
    f = s["first10_vs_last10"]
    return (f"{s['prefix']}: {s['verdict']} {s['hold_minutes']} min | drift xy {s['final']['xy_drift']:.2f} m yaw "
            f"{np.degrees(s['final']['yaw_drift']):+.1f} deg | z err {100*f['z_err'][0]:+.2f}->{100*f['z_err'][1]:+.2f} cm "
            f"pitch err {np.degrees(f['pitch_err'][0]):+.2f}->{np.degrees(f['pitch_err'][1]):+.2f} deg "
            f"vx bias {f['vx_bias'][0]:+.3f}->{f['vx_bias'][1]:+.3f} | true vx {f['gt_vx'][0]:.3f}->{f['gt_vx'][1]:.3f} "
            f"wz {f['gt_wz'][0]:.3f}->{f['gt_wz'][1]:.3f} z {f['gt_z'][0]:.3f}->{f['gt_z'][1]:.3f} "
            f"tilt {np.degrees(f['gt_tilt_rms'][0]):.2f}->{np.degrees(f['gt_tilt_rms'][1]):.2f} deg | "
            f"mpc {s.get('mpc_rate', {}).get('first10', float('nan')):.1f}->{s.get('mpc_rate', {}).get('last10', float('nan')):.1f} Hz "
            f"rss {s.get('controller_rss_mb', ['?', '?'])}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefixes", nargs="+", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    for prefix in args.prefixes:
        summary = analyze(prefix, not args.no_plot)
        print(line(summary))
        if "error_behaviour" in summary:
            print(behaviour_line(summary))
