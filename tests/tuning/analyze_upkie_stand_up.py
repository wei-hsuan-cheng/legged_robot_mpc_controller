#!/usr/bin/env python3
"""Compare stand-up contact clearance, CoM motion and wheel effort from recorded trials.

python3 tests/tuning/analyze_upkie_stand_up.py PREFIX [PREFIX ...] --output report
Reads PREFIX_replay.json, PREFIX_tracking.csv and PREFIX.json from wb_id_walk_test.
Requires MuJoCo, NumPy and Matplotlib; ground truth is used for analysis only.
"""

import argparse
import csv
import json
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def analyze(prefix):
    prefix = Path(prefix)
    result = json.loads(Path(str(prefix) + ".json").read_text())
    replay = json.loads(Path(str(prefix) + "_replay.json").read_text())
    with Path(str(prefix) + "_tracking.csv").open() as stream:
        tracking = list(csv.DictReader(stream))
    active = [row for row in tracking if int(float(row["phase"])) == 1]
    if not active or "takeover_time" not in result:
        raise ValueError(f"{prefix}: no completed stand-up")
    start = float(active[0]["t"])
    # The harness can observe the log transition with queued old odometry.
    # Use the controller's timestamped phase telemetry for alignment instead.
    takeover = next(
        float(row["t"]) for row in tracking if int(float(row["phase"])) == 6
    )
    end = takeover + 3.0
    rows = [row for row in replay if start <= row["time"] <= end]
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "description/mujoco/robots/upkie/scene.xml")
    )
    data = mujoco.MjData(model)
    wheels = [model.geom(f"{side}_foot_collision").id for side in ("left", "right")]
    knees = [
        model.geom(f"{side}_femur_collision{i}").id
        for side in ("left", "right")
        for i in (1, 4)
    ]
    floor = model.geom("floor").id
    femurs = {i for i in range(model.ngeom) if "femur_collision" in model.geom(i).name}
    trunk = [i for i in range(model.ngeom) if "trunk_collision" in model.geom(i).name]
    self_penetration = 0.0
    trunk_clearance = float("inf")
    distances = np.zeros(6)
    values = []
    for row in rows:
        data.qpos[:3] = row["position"]
        data.qpos[3:7] = row["quaternion_wxyz"]
        for name, q in zip(row["names"], row["q"]):
            data.qpos[model.joint(name).qposadr[0]] = q
        mujoco.mj_forward(model, data)
        gap = lambda g: mujoco.mj_geomDistance(model, data, floor, g, 1.0, distances)
        trunk_clearance = min(trunk_clearance, *(gap(g) for g in trunk))
        for contact in data.contact:
            if (contact.geom1 in wheels and contact.geom2 in femurs) or (
                contact.geom2 in wheels and contact.geom1 in femurs
            ):
                self_penetration = max(self_penetration, -contact.dist)
        center = np.mean(data.geom_xpos[wheels], axis=0)
        q = np.array(row["quaternion_wxyz"])
        pitch = np.arcsin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0))
        values.append(
            [
                row["time"] - start,
                row["position"][0],
                row["position"][2],
                pitch,
                data.subtree_com[1, 0] - center[0],
                center[0],
                min(gap(g) for g in knees),
                max(gap(g) for g in wheels),
            ]
        )
    values = np.array(values)
    selected = [r for r in tracking if start <= float(r["t"]) <= end]
    t = np.array([float(r["t"]) - start for r in selected])
    speed = np.array(
        [
            [float(r[f"meas_v/{side}_wheel"]) for side in ("left", "right")]
            for r in selected
        ]
    )
    torque = np.array(
        [
            [float(r[f"meas_tau/{side}_wheel"]) for side in ("left", "right")]
            for r in selected
        ]
    )
    hz = []
    log = Path(str(prefix) + ".log")
    if log.exists():
        hz = [float(x) for x in re.findall(r"actual=([\d.]+) Hz", log.read_text())]
    metrics = dict(
        verdict=result["verdict"],
        playback_seconds=takeover - start,
        peak_wheel_speed_rad_s=float(np.max(np.abs(speed))),
        wheel_torque_rms_Nm=float(np.sqrt(np.mean(torque**2))),
        wheel_saturation_fraction=float(np.mean(np.abs(torque) >= 1.69)),
        wheel_center_range_m=float(np.ptp(values[:, 5])),
        peak_pitch_rad=float(np.max(np.abs(values[:, 3]))),
        max_wheel_clearance_m=float(np.max(values[:, 7])),
        mean_mpc_rate_hz=float(np.mean(hz)) if hz else None,
        max_tire_femur_penetration_m=float(self_penetration),
        minimum_trunk_ground_clearance_m=float(trunk_clearance),
    )
    handover = np.argmin(abs(values[:, 0] - (takeover - start)))
    metrics["knee_clearance_at_locomotion_m"] = float(values[handover, 6])
    return metrics, values, t, speed, torque


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", nargs="+")
    parser.add_argument(
        "--labels", nargs="+", help="Display labels, one per recorded trial"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.labels and len(args.labels) != len(args.prefix):
        parser.error("--labels needs one entry per prefix")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(5, 1, figsize=(10, 11), sharex=True)
    metrics = {}
    for index, prefix in enumerate(args.prefix):
        name = Path(prefix).name
        summary, v, t, speed, torque = analyze(prefix)
        metrics[name] = summary
        label = args.labels[index] if args.labels else name
        (line,) = axes[0].plot(v[:, 0], v[:, 2], label=label)
        color = line.get_color()
        axes[1].plot(v[:, 0], v[:, 3], color=color)
        axes[2].plot(v[:, 0], v[:, 4], color=color)
        axes[2].plot(v[:, 0], v[:, 6], color=color, ls="--", alpha=0.6)
        axes[3].plot(t, np.max(np.abs(speed), axis=1), color=color)
        axes[4].plot(t, np.max(np.abs(torque), axis=1), color=color)
    for ax, label in zip(
        axes,
        (
            "Base height [m]",
            "Trunk pitch [rad]",
            "CoM offset / knee gap [m]",
            "Peak wheel speed [rad/s]",
            "Peak wheel effort [Nm]",
        ),
    ):
        ax.set_ylabel(label)
        ax.grid(alpha=0.25)
    axes[0].legend()
    axes[4].axhline(1.7, color="gray", ls=":")
    axes[-1].set_xlabel("Seconds after stand-up playback begins")
    fig.tight_layout()
    fig.savefig(str(args.output) + ".png", dpi=140)
    Path(str(args.output) + ".json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
