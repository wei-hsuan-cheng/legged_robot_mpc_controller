#!/usr/bin/env python3
"""Plot the Upkie ledge-drop runs of tests/tuning/upkie_ledge_drop.sh (docs/results/upkie_ledge_drop.md).

usage: plot_upkie_ledge_drop.py DIR [--out OUT_DIR] [--name upkie_ledge]     (pip install numpy matplotlib pyyaml)

Reads DIR/NAME_{05,10}_{est,gt}{.json,_replay.json,_tracking.csv}: the wheel KF driving the MPC (est) and GT driving
with the KF logged alongside (gt). Times are aligned on the landing impact (peak measured normal force past the edge).
"True height above the ground" is the GT base z minus the surface under the wheels: the terrain until the impact, the
floor after it, undefined while the wheels roll off the edge. Writes upkie_ledge_{05,10}cm.png, upkie_ledge_overview.png
and upkie_ledge_summary.json.
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
EDGE = 5.0
COLOR = {"est": "tab:blue", "gt": "tab:orange"}
LABEL = {"est": "wheel KF drives", "gt": "GT drives (KF logged)"}
JOINTS = ("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel")


def layout(h):
    course = yaml.safe_load((ROOT / f"config/terrains/upkie/ledge_{h}_params.yaml").read_text())["upkie_course"]
    ramp = course["ramps"][0]
    return ramp["x"], ramp["z"][1]


def terrain(x, h):
    (x0, x1), top = layout(h)
    return np.where(x < EDGE, np.clip((x - x0) / (x1 - x0), 0, 1) * top, 0.0)


def load(prefix, h):
    result = json.loads(Path(f"{prefix}.json").read_text())
    t0 = result["takeover_time"]
    rows = list(csv.DictReader(open(f"{prefix}_tracking.csv")))
    col = lambda key: np.array([float(r[key]) for r in rows])  # noqa: E731
    tr = dict(t=col("t") - t0, obs_z=col("obs_base/z"), tau={j: col(f"cmd_tau/{j}") for j in JOINTS},
              fz_plan=col("plan_wrench/left_tire_center/fz") + col("plan_wrench/right_tire_center/fz"),
              fn=col("meas_contact/left_wheel_contact/normal_force") + col("meas_contact/right_wheel_contact/normal_force"),
              cl=col("meas_contact/left_wheel_contact/contact"), cr=col("meas_contact/right_wheel_contact/contact"))
    replay = [s for s in json.loads(Path(f"{prefix}_replay.json").read_text()) if s["time"] >= t0]
    pitch = lambda q: np.arcsin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1))  # noqa: E731
    gt = dict(t=np.array([s["time"] for s in replay]) - t0, p=np.array([s["position"] for s in replay]),
              pitch=np.array([pitch(s["quaternion_wxyz"]) for s in replay]),
              kf_z=np.array([s["estimate"]["position"][2] if s.get("estimate") else np.nan for s in replay]))
    x = np.interp(tr["t"], gt["t"], gt["p"][:, 0])
    past = x > EDGE - 0.1
    impact = tr["t"][past][np.argmax(tr["fn"][past])]
    # The base leads the wheels over the edge, so the platform stays under the wheels until the impact.
    gt["ground"] = np.where(gt["t"] < impact, terrain(np.minimum(gt["p"][:, 0], EDGE - 0.01), h), 0.0)
    gt["h_local"] = gt["p"][:, 2] - gt["ground"]
    lost = tr["t"][(tr["t"] < impact) & (x > EDGE - 0.2) & ((tr["cl"] < .5) | (tr["cr"] < .5))]
    gt["unloaded"] = (lost.min() - impact) if len(lost) else 0.0
    gt["defined"] = ~((gt["t"] - impact > gt["unloaded"]) & (gt["t"] < impact))
    return result, tr, gt, impact


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dir", type=Path)
    parser.add_argument("--out", type=Path, help="default: DIR")
    parser.add_argument("--name", default="upkie_ledge")
    args = parser.parse_args()
    out = args.out or args.dir
    out.mkdir(parents=True, exist_ok=True)
    runs = {(h, s): load(args.dir / f"{args.name}_{h}_{s}", h) for h in ("05", "10") for s in ("est", "gt")}
    summary = {}
    for h in ("05", "10"):
        fig, ax = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
        for s in ("est", "gt"):
            result, tr, gt, impact = runs[(h, s)]
            tt, tg = tr["t"] - impact, gt["t"] - impact
            if s == "est":
                ax[0].plot(tg, gt["p"][:, 2], color="k", lw=1.6, label="GT world z (wheel KF drives)")
                ax[0].plot(tg, gt["ground"], color="0.5", lw=1.2, label="ground under the wheels")
            else:
                ax[0].plot(tg, gt["p"][:, 2], color="tab:red", lw=1.6, label="GT world z (GT drives)")
            ax[0].plot(tt, tr["obs_z"], color=COLOR[s], lw=1.3, ls="-" if s == "est" else "--",
                       label=f"MPC observed z: {LABEL[s]}")
            # The KF's height error: driving the MPC (est), or logged while GT drives (gt).
            kf = np.interp(tg, tt, tr["obs_z"]) if s == "est" else gt["kf_z"]
            error = np.where(gt["defined"], kf - gt["h_local"], np.nan)
            ax[1].plot(tg, error * 1000, color=COLOR[s] if s == "est" else "tab:green", lw=1.3 if s == "est" else 1.0,
                       label=f"KF z ({'driving' if s == 'est' else 'logged, GT drives'}) - true height above the ground")
            ax[2].plot(tg, gt["pitch"], color=COLOR[s], label=LABEL[s])
            ax[3].plot(tt, tr["tau"]["left_wheel"] + tr["tau"]["right_wheel"], color=COLOR[s], label=f"wheel torque L+R ({LABEL[s]})")
            ax[4].plot(tt, tr["tau"]["left_knee"] + tr["tau"]["right_knee"], color=COLOR[s], label=f"knee torque L+R ({LABEL[s]})")
            ax[4].plot(tt, tr["tau"]["left_hip"] + tr["tau"]["right_hip"], color=COLOR[s], ls=":", label=f"hip torque L+R ({LABEL[s]})")
            ax[5].plot(tt, tr["fn"], color=COLOR[s], label=f"measured normal force L+R ({LABEL[s]})")
            ax[5].plot(tt, tr["fz_plan"], color=COLOR[s], ls="--", lw=1, label=f"MPC planned fz L+R ({LABEL[s]})")
            after, around = (tg > 0) & (tg < 3), (tt > -0.3) & (tt < 3)
            e = error[after]
            settle = tg[after][np.where(abs(e) > 0.005)[0][-1]] if (abs(e) > 0.005).any() else 0.0
            summary[f"{h}_{s}"] = dict(
                verdict=result.get("verdict"), wheels_unloaded_before_impact_s=round(float(-gt["unloaded"]), 3),
                height_on_platform_m=round(float(gt["h_local"][(tg > -2) & (tg < -0.3)].mean()), 3),
                kf_error_after_impact_mm=[round(float(np.nanmin(e) * 1000), 1), round(float(np.nanmax(e) * 1000), 1)],
                kf_within_5mm_after_s=round(float(settle), 2), pitch_max_after_impact=round(float(abs(gt["pitch"][after]).max()), 3),
                world_z_drop_m=round(float(gt["p"][(tg > -2) & (tg < -0.3), 2].mean() - gt["p"][after, 2].min()), 3),
                wheel_torque_max=round(float(max(abs(tr["tau"]["left_wheel"][around]).max(), abs(tr["tau"]["right_wheel"][around]).max())), 2),
                knee_torque_max=round(float(max(abs(tr["tau"]["left_knee"][around]).max(), abs(tr["tau"]["right_knee"][around]).max())), 2),
                normal_force_peak=round(float(tr["fn"][around].max()), 1))
        unloaded = min(runs[(h, "est")][2]["unloaded"], runs[(h, "gt")][2]["unloaded"])
        for a, label in zip(ax, ["z [m]", "height error [mm]", "GT pitch [rad]", "[N m]", "[N m]", "[N]"]):
            a.set_ylabel(label)
            a.axvline(0, color="k", ls=":", lw=.7)
            a.axvspan(unloaded, 0, color="0.85")
            a.grid(alpha=.3)
            a.legend(fontsize=7, loc="upper right")
        ax[0].set_ylim(-0.02, 0.45)
        ax[1].set_ylim(-25, 25)
        ax[2].set_ylim(-0.08, 0.08)
        ax[1].set_title(f"blank while the wheels roll off the edge (no ground under them); the GT-driven MPC observes "
                        f"+{int(h) * 10} mm on the platform (off scale)", fontsize=8)
        ax[-1].set_xlim(-1.5, 3.0)
        ax[-1].set_xlabel("time since the landing impact [s] (grey: a wheel unloaded, rolling off the edge)")
        fig.suptitle(f"Upkie rolls off a {int(h)} cm ledge at 0.3 m/s: wheel KF driving vs GT driving")
        fig.tight_layout()
        fig.savefig(out / f"upkie_ledge_{h}cm.png", dpi=90)
        plt.close(fig)
    fig, ax = plt.subplots(1, 1, figsize=(10, 3.6))
    for h, ls in (("05", "-"), ("10", "--")):
        for s in ("est", "gt"):
            gt = runs[(h, s)][2]
            ax.plot(gt["p"][:, 0], gt["h_local"], color=COLOR[s], ls=ls, lw=1.2, label=f"{int(h)} cm, {LABEL[s]}")
    ax.axvline(EDGE, color="k", ls=":", lw=.7)
    ax.set_xlim(-1, 8)
    ax.set_ylim(0.17, 0.33)
    ax.grid(alpha=.3)
    ax.set_xlabel(f"GT base x [m] (ramp up to the platform at 3.0 m, edge at {EDGE} m)")
    ax.set_ylabel("true height above ground [m]")
    ax.legend(fontsize=8, ncol=2, loc="lower left")
    ax.set_title("Height above the local ground over the whole pass")
    fig.tight_layout()
    fig.savefig(out / "upkie_ledge_overview.png", dpi=90)
    (out / "upkie_ledge_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
