#!/usr/bin/env python3
"""Plot an entire base_twist ramp test; optionally render recorded physical states.

Usage: plot_ramp.py <case directory> [--render]
Replay renders use recorded simulator poses and encoders, not estimated poses.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    data = np.load(args.case / "data.npz")
    gt, tracking = data["gt"], data["track"]
    names = list(data["track_names"])
    t = tracking[:, names.index("t")]
    sample = lambda name: tracking[:, names.index(name)]
    seconds = gt[:, 0] - gt[0, 0]
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    axes[0].plot(seconds, gt[:, 2], label="physical x")
    axes[0].axhline(1.5, color="k", ls="--", label="ramp starts")
    axes[0].axhline(5.49, color="gray", ls="--", label="ramp ends")
    axes[0].set_ylabel("x [m]")
    for axis, name, column in ((axes[1], "z", 4), (axes[2], "vx", 5)):
        axis.plot(seconds, gt[:, column], label="physical")
        axis.plot(t - gt[0, 0], sample("obs_base/" + name), label="observation")
        axis.plot(t - gt[0, 0], sample("plan_base/" + name), alpha=0.6, label="policy")
        axis.set_ylabel(name + (" [m]" if name == "z" else " [m/s]"))
    settle, ramp_time, hold, stop, speed = data["phases"]
    command = np.interp(gt[:, 1], [0, settle, settle + ramp_time,
                                  settle + ramp_time + hold, settle + 2*ramp_time + hold],
                        [0, 0, speed, speed, 0])
    axes[2].plot(seconds, command, "k--", label="command")
    axes[3].plot(seconds, np.rad2deg(gt[:, 8]), label="physical roll")
    axes[3].plot(seconds, np.rad2deg(gt[:, 9]), label="physical pitch")
    axes[3].set_ylabel("angle [deg]")
    sn = list(data["solver_names"])
    solver = data["solver"]
    axes[4].semilogy(solver[:, sn.index("t")] - gt[0, 0],
                     np.maximum(solver[:, sn.index("eq_sse")], 1e-12), label="equality SSE")
    axes[4].set_ylabel("solver residual")
    axes[-1].set_xlabel("simulation time from recording start [s]")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc="upper left")
    fig.suptitle(args.case.name)
    fig.tight_layout()
    fig.savefig(args.case / "ramp.png", dpi=140)
    observed_z = np.interp(gt[:, 0], t, sample("obs_base/z"))
    ramp = (gt[:, 2] > 1.7) & (gt[:, 2] < 5.2) & (gt[:, 4] > 0.6)
    metrics = {"max_x_m": float(gt[:, 2].max()), "final_x_m": float(gt[-1, 2]),
               "fell_wall_s": None if np.isnan(data["fell"]) else float(data["fell"]),
               "ramp_height_bias_m": float(np.mean(observed_z[ramp] - gt[ramp, 4])) if ramp.any() else None}
    (args.case / "ramp_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(metrics)
    if args.render:
        render(args.case, data)


def render(directory, data):
    import mujoco
    from PIL import Image
    scene = Path(__file__).resolve().parents[2] / "description/mujoco/robots/g1/scene.xml"
    model = mujoco.MjModel.from_xml_path(str(scene))
    state = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=720, width=1100)
    gt = data["gt"]
    for number, target_x in enumerate((1.4, 2.5, 3.6, 4.8, None)):
        index = int(np.argmin(abs(gt[:, 2] - target_x))) if target_x is not None else len(gt) - 1
        row = gt[index]
        # GT quaternions are converted from the recorded ZYX orientation.
        r, p, y = row[8:11] / 2
        state.qpos[:3] = row[2:5]
        state.qpos[3:7] = [np.cos(r)*np.cos(p)*np.cos(y)+np.sin(r)*np.sin(p)*np.sin(y),
                           np.sin(r)*np.cos(p)*np.cos(y)-np.cos(r)*np.sin(p)*np.sin(y),
                           np.cos(r)*np.sin(p)*np.cos(y)+np.sin(r)*np.cos(p)*np.sin(y),
                           np.cos(r)*np.cos(p)*np.sin(y)-np.sin(r)*np.sin(p)*np.cos(y)]
        for j, name in enumerate(data["js_names"]):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
            if joint >= 0:
                state.qpos[model.jnt_qposadr[joint]] = np.interp(row[0], data["js_t"], data["js_q"][:, j])
        mujoco.mj_forward(model, state)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [row[2], row[3], 0.5]
        camera.distance, camera.azimuth, camera.elevation = 2.7, 135, -18
        renderer.update_scene(state, camera)
        Image.fromarray(renderer.render()).save(directory / f"replay{number}.png")
    renderer.close()


if __name__ == "__main__":
    main()
