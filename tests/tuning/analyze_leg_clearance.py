#!/usr/bin/env python3
"""Replay measured base/joint poses to inspect H2 foot crossing offline.

Joint and odometry messages are latest-sample paired, not hardware synchronized.
Replay contacts indicate geometric overlap, not measured contact forces.
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


def analyze(prefix, scene, render):
    samples = json.loads(Path(str(prefix) + "_replay.json").read_text())
    samples = [s for s in samples if s["phase"] != "startup"]
    if not samples:
        raise ValueError(f"No post-startup replay samples in {prefix}")
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    joints = {model.joint(i).name: model.jnt_qposadr[i] for i in range(model.njnt)}
    feet = [model.body(side + "_ankle_pitch_link").id for side in ("left", "right")]
    offset = np.array([[.035, 0, -.058], [.098, 0, -.058], [-.028, 0, -.058]])
    rows = []

    def pose(sample):
        data.qpos[:3] = sample["position"]
        data.qpos[3:7] = sample["quaternion_wxyz"]
        for name, value in zip(sample["names"], sample["q"]):
            if name in joints:
                data.qpos[joints[name]] = value
        mujoco.mj_forward(model, data)

    for sample in samples:
        pose(sample)
        points = [data.xpos[i] + offset @ data.xmat[i].reshape(3, 3).T for i in feet]
        w, x, y, z = sample["quaternion_wxyz"]
        yaw = np.arctan2(2 * (w*z+x*y), 1-2*(y*y+z*z))
        lateral = np.array([-np.sin(yaw), np.cos(yaw), 0])
        width = (points[0][0] - points[1][0]) @ lateral
        clearance = np.linalg.norm(points[0][:, None] - points[1][None, :], axis=2).min()
        overlap = 0
        for contact in data.contact:
            names = [model.body(model.geom_bodyid[g]).name for g in (contact.geom1, contact.geom2)]
            legs = [n.split("_")[0] if any(part in n for part in ("hip", "knee", "ankle")) else "" for n in names]
            overlap += set(legs) == {"left", "right"} and contact.dist < 0
        rows.append([sample["time"]-samples[0]["time"], width, clearance, overlap,
                     sample["position"][2], points[0][0, 2], points[1][0, 2],
                     *sample["position"][:2], yaw])
    values = np.array(rows)
    np.savetxt(str(prefix) + "_clearance.csv", values, delimiter=",",
               header="time,signed_width,point_distance,opposite_leg_overlaps,base_z,left_sole_z,right_sole_z,base_x,base_y,yaw", comments="")
    summary = dict(min_signed_width=float(values[:, 1].min()),
                   min_point_distance=float(values[:, 2].min()),
                   crossing_samples=int(np.sum(values[:, 1] < 0)),
                   opposite_leg_overlap_samples=int(np.sum(values[:, 3] > 0)),
                   samples=len(samples))
    Path(str(prefix) + "_clearance.json").write_text(json.dumps(summary, indent=2) + "\n")
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(values[:, 0], values[:, 1], label="Signed left-right width")
    axes[0].plot(values[:, 0], values[:, 2], label="Minimum 3D foot-point distance")
    axes[0].axhline(0, color="black", lw=.6)
    axes[0].set_ylabel("Distance [m]")
    axes[0].legend()
    axes[1].plot(values[:, 0], values[:, 4], label="Base z")
    axes[1].plot(values[:, 0], values[:, 5], label="Left sole z")
    axes[1].plot(values[:, 0], values[:, 6], label="Right sole z")
    axes[1].set_ylabel("Height [m]")
    axes[1].legend()
    axes[2].plot(values[:, 0], values[:, 3])
    axes[2].set_ylabel("Leg mesh overlaps")
    velocity = np.column_stack([np.gradient(values[:, i], values[:, 0]) for i in (7, 8)])
    axes[3].plot(values[:, 0], velocity[:, 0]*np.cos(values[:, 9]) + velocity[:, 1]*np.sin(values[:, 9]), label="Forward")
    axes[3].plot(values[:, 0], -velocity[:, 0]*np.sin(values[:, 9]) + velocity[:, 1]*np.cos(values[:, 9]), label="Lateral")
    axes[3].set_ylabel("Pose-derived speed [m/s]")
    axes[3].set_xlabel("Time after takeover [s]")
    axes[3].legend()
    fig.tight_layout()
    fig.savefig(str(prefix) + "_clearance.png", dpi=150)
    plt.close(fig)
    joint_names = samples[0]["names"]
    if all(len(s["effort"]) == len(joint_names) for s in samples):
        positions = np.array([s["q"] for s in samples])
        efforts = np.array([s["effort"] for s in samples])
        effort_summary = {}
        moving = np.array([s["phase"] != "settle" for s in samples])
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for side in ("left", "right"):
            for kind in ("hip_roll", "knee", "ankle_roll"):
                name = side + "_" + kind + "_joint"
                index = joint_names.index(name)
                joint_id = model.joint(name).id
                actuator = np.flatnonzero(model.actuator_trnid[:, 0] == joint_id)[0]
                limit = float(np.abs(model.actuator_ctrlrange[actuator]).max())
                torque = efforts[:, index]
                effort_summary[name] = dict(max_abs_effort=float(np.abs(torque).max()), limit=limit,
                                           moving_fraction_near_limit=float(np.mean(np.abs(torque[moving]) >= .98*limit)))
                if kind == "ankle_roll":
                    axes[0].plot(values[:, 0], torque, label=side)
                    axes[0].axhline(limit, color="gray", lw=.5)
                    axes[0].axhline(-limit, color="gray", lw=.5)
                if kind == "knee":
                    axes[1].plot(values[:, 0], positions[:, index], label=side)
                if kind != "ankle_roll":
                    axes[2].plot(values[:, 0], torque / limit, label=side + " " + kind)
        axes[0].set_ylabel("Ankle roll torque [Nm]")
        axes[1].set_ylabel("Knee position [rad]")
        axes[2].set_ylabel("Torque / actuator limit")
        axes[2].set_xlabel("Time after takeover [s]")
        for axis in axes:
            axis.legend()
        fig.tight_layout()
        fig.savefig(str(prefix) + "_support.png", dpi=150)
        plt.close(fig)
        Path(str(prefix) + "_effort.json").write_text(json.dumps(effort_summary, indent=2) + "\n")
    if render:
        from PIL import Image
        model.vis.global_.offwidth = 1000
        model.vis.global_.offheight = 850
        with mujoco.Renderer(model, height=850, width=1000) as renderer:
            for label, index in (("narrowest", int(values[:, 1].argmin())),
                                 ("before_failure", max(0, len(samples)-15))):
                pose(samples[index])
                camera = mujoco.MjvCamera()
                camera.lookat[:] = data.qpos[:3] + [0, 0, -.3]
                camera.distance = 3.3
                camera.azimuth = 170
                camera.elevation = -20
                renderer.update_scene(data, camera=camera)
                Image.fromarray(renderer.render()).save(str(prefix) + "_" + label + ".png")
    print(prefix.name, json.dumps(summary))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefixes", nargs="+", type=Path)
    parser.add_argument("--scene", type=Path, default=Path(__file__).resolve().parents[2] /
                        "description/mujoco/robots/h2/scene_flat.xml")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    for prefix in args.prefixes:
        analyze(prefix, args.scene, args.render)
