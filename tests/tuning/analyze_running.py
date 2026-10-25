#!/usr/bin/env python3
"""Check scheduled flight and sampled physical clearance in flat-ground replays.

New replays carry joint timestamps and pair nearby messages. Older replays use
latest-sample pairing. Neither is a measured contact-force trace or continuous-time proof.
"""

import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mujoco_scene import temporary_scene


def intervals(times, mask):
    if len(times) < 2:
        return []
    step = float(np.median(np.diff(times)))
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    ends = np.flatnonzero(mask & ~np.r_[mask[1:], False])
    return [dict(start=float(times[a]), end=float(times[b] + step),
                 duration=float(times[b] - times[a] + step)) for a, b in zip(starts, ends)]


def lowest_point(model, data, geom):
    kind, size = model.geom_type[geom], model.geom_size[geom]
    z = data.geom_xpos[geom, 2]
    axis = data.geom_xmat[geom].reshape(3, 3)[2]
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return z - size[0]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return z - np.abs(axis) @ size
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return z - abs(axis[2]) * size[1] - size[0]
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return z - abs(axis[2]) * size[1] - size[0] * np.sqrt(max(0, 1-axis[2]**2))
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        begin, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        return z + np.min(model.mesh_vert[begin:begin+count] @ axis)
    raise ValueError(f"Unsupported foot collision geometry {kind}")


def analyze(prefix, robot, render=False):
    root = Path(__file__).resolve().parents[2]
    with temporary_scene(robot) as scene:
        model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    config = yaml.safe_load(Path(str(prefix) + ".yaml").read_text())["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
    samples = json.loads(Path(str(prefix) + "_replay.json").read_text())
    samples = [s for s in samples if s["phase"] != "startup"]
    t0 = samples[0]["time"]
    joints = {model.joint(i).name: model.jnt_qposadr[i] for i in range(model.njnt)}
    foot_bodies = [model.jnt_bodyid[model.joint(name).id] for name in config["ocs2"]["model"]["contactParentJointNames"]]
    foot_geoms = [[i for i in range(model.ngeom) if model.geom_bodyid[i] == body
                   and (model.geom_contype[i] or model.geom_conaffinity[i])] for body in foot_bodies]
    assert all(foot_geoms), "No active foot collision geometry"
    rows = []
    self_contacts = []
    for sample in samples:
        data.qpos[:3] = sample["position"]
        data.qpos[3:7] = sample["quaternion_wxyz"]
        for name, value in zip(sample["names"], sample["q"]):
            if name in joints:
                data.qpos[joints[name]] = value
        mujoco.mj_forward(model, data)
        floor_contacts = sum(contact.dist <= 0.0 and any(model.geom_bodyid[g] == 0 for g in (contact.geom1, contact.geom2))
                             for contact in data.contact)
        for contact in data.contact:
            bodies = [int(model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
            if sample["phase"] == "walk" and contact.dist < 0 and all(bodies) and any(b in foot_bodies for b in bodies):
                self_contacts.append(dict(time=sample["time"] - t0,
                                          bodies=[model.body(b).name for b in bodies], depth=-float(contact.dist)))
        low = [min(lowest_point(model, data, geom) for geom in group) for group in foot_geoms]
        rows.append([sample["time"] - t0, *low, data.subtree_com[model.body("pelvis").id, 2],
                     sample["position"][2], floor_contacts])
    values = np.array(rows)
    hold = np.array([s["phase"] == "walk" for s in samples])
    paired = all("joint_time" in s for s in samples)
    pair_error = np.array([abs(s["joint_time"]-s["time"]) for s in samples]) if paired else np.zeros(len(samples))
    airborne = (values[:, 1] > .002) & (values[:, 2] > .002) & (values[:, 5] == 0) & hold & (pair_error <= .002)
    bouts = intervals(values[:, 0], airborne)
    summary = dict(sampled_airborne_fraction=float(np.mean(airborne[hold])) if hold.any() else 0.0,
                   sampled_flight_bouts=len(bouts),
                   sampled_max_flight_duration=max((b["duration"] for b in bouts), default=0),
                   sampled_total_flight_duration=sum(b["duration"] for b in bouts),
                   sampled_flight_intervals=bouts,
                   sampled_foot_self_contacts=len(self_contacts),
                   foot_self_contact_details=self_contacts)
    summary.update(joint_timestamps_available=paired,
                   max_pair_time_error=float(pair_error.max()) if paired else None,
                   hold_fraction_paired_within_2ms=float(np.mean(pair_error[hold] <= .002)) if paired and hold.any() else None)
    np.savetxt(str(prefix) + "_running.csv", values, delimiter=",",
               header="time,left_collision_lowest_z,right_collision_lowest_z,com_z,base_z,floor_contacts", comments="")
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(values[:, 0], values[:, 1], label="Left foot collision low point")
    axes[0].plot(values[:, 0], values[:, 2], label="Right foot collision low point")
    axes[0].axhline(.002, color="gray", ls="--", lw=.7)
    axes[0].set_ylabel("Foot height [m]")
    axes[1].plot(values[:, 0], values[:, 3], label="Measured-pose CoM z")
    axes[1].plot(values[:, 0], values[:, 4], label="Base z")
    axes[1].set_ylabel("Height [m]")
    trace_path = Path(str(prefix) + "_tracking.csv")
    if trace_path.exists():
        with trace_path.open() as file:
            trace = list(csv.DictReader(file))
        times = np.array([float(row["t"]) - t0 for row in trace])
        mode = np.array([float(row["plan_mode"]) for row in trace])
        moving = np.array([row["test_phase"] == "walk" for row in trace])
        flight = (mode == 0) & moving
        summary.update(planned_flight_samples=int(flight.sum()),
                       planned_flight_fraction=float(np.mean(flight[moving])) if moving.any() else 0.0,
                       planned_flight_intervals=intervals(times, flight))
        force_keys = [key for key in trace[0] if key.startswith("plan_wrench/") and key.rsplit("/", 1)[-1] in ("fx", "fy", "fz")]
        if flight.any():
            forces = np.array([[float(row[key]) for key in force_keys] for row in trace])
            summary["max_abs_planned_flight_force"] = float(np.nanmax(np.abs(forces[flight])))
        axes[2].step(times, mode, where="post", label="Planned mode (0=FLY, 1=RF, 2=LF, 3=STANCE)")
        axes[2].set_ylabel("Contact mode")
        for foot in config["ocs2"]["model"]["contactNames6DoF"]:
            key = "plan_wrench/" + foot + "/fz"
            axes[3].plot(times, [float(row[key]) for row in trace], label=foot + " planned Fz")
        axes[3].set_ylabel("Planned force [N]")
    for bout in bouts:
        for axis in axes:
            axis.axvspan(bout["start"], bout["end"], alpha=.12, color="green")
    for axis in axes:
        if axis.get_legend_handles_labels()[0]:
            axis.legend(loc="upper right")
    axes[-1].set_xlabel("Time after takeover [s]; green = sampled physical flight")
    fig.tight_layout()
    fig.savefig(str(prefix) + "_running.png", dpi=150)
    plt.close(fig)
    if render:
        from PIL import Image, ImageDraw
        # Show a running half-cycle from the side and front, not just one selected pose.
        indices = np.flatnonzero(airborne)
        start = values[indices[len(indices)//2], 0] if len(indices) else values[np.flatnonzero(hold)[0], 0] + 1
        chosen = [int(np.abs(values[:, 0] - start - offset).argmin()) for offset in (0, .08, .16, .24)]
        model.vis.global_.offwidth = 600
        model.vis.global_.offheight = 600
        sheet = Image.new("RGB", (2400, 1200))
        with mujoco.Renderer(model, height=600, width=600) as renderer:
            for column, index in enumerate(chosen):
                sample = samples[index]
                data.qpos[:3] = sample["position"]
                data.qpos[3:7] = sample["quaternion_wxyz"]
                for name, value in zip(sample["names"], sample["q"]):
                    if name in joints:
                        data.qpos[joints[name]] = value
                mujoco.mj_forward(model, data)
                rotation = data.xmat[model.body("pelvis").id].reshape(3, 3)
                heading = np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
                for row, azimuth in enumerate((heading + 90, heading)):
                    camera = mujoco.MjvCamera()
                    camera.lookat[:] = data.qpos[:3] + [0, 0, -.25]
                    camera.distance = 2.7 if robot == "g1" else 3.4
                    camera.azimuth = azimuth
                    camera.elevation = -10
                    renderer.update_scene(data, camera=camera)
                    frame = Image.fromarray(renderer.render())
                    ImageDraw.Draw(frame).text((15, 15), f"t={values[index, 0]:.2f}s", fill="white")
                    sheet.paste(frame, (600*column, 600*row))
        sheet.save(str(prefix) + "_running_frames.png")
    Path(str(prefix) + "_running.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: value for key, value in summary.items() if not key.endswith(("intervals", "details"))}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--robot-name", choices=["g1", "h2"], required=True)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    analyze(args.prefix.resolve(), args.robot_name, args.render)
