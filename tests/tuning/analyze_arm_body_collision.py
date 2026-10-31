#!/usr/bin/env python3
"""Inspect configured sphere clearances from nominal posture or recorded joint poses.

This is offline kinematic replay, not proof of mesh separation or measured contact.
URDF joint origins are used, including fixed wrists and head offsets.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
ROOT = Path(__file__).resolve().parents[2]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def vector(text):
    return np.fromstring(text or "0 0 0", sep=" ")


def joint_origin(joint):
    transform = np.eye(4)
    origin = joint.find("origin")
    if origin is not None:
        roll, pitch, yaw = vector(origin.get("rpy"))
        transform[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
        transform[:3, 3] = vector(origin.get("xyz"))
    return transform


class SphereReplay:
    def __init__(self, robot_name, params):
        dofs = 31 if robot_name == "h2" else 29
        robot = ET.parse(ROOT / f"description/urdf/robots/{robot_name}/{robot_name}_{dofs}dof.urdf").getroot()
        children = {j.find("child").get("link") for j in robot.findall("joint")}
        roots = [link.get("name") for link in robot.findall("link") if link.get("name") not in children]
        self.roots = roots
        self.joints = []
        pending = list(robot.findall("joint"))
        known = set(roots)
        while pending:
            previous = len(pending)
            for joint in pending[:]:
                parent = joint.find("parent").get("link")
                if parent not in known:
                    continue
                child = joint.find("child").get("link")
                axis = joint.find("axis")
                self.joints.append((joint.get("name"), joint.get("type"), parent, child,
                                    joint_origin(joint), vector(axis.get("xyz")) if axis is not None else np.array([1., 0., 0.])))
                known.add(child)
                pending.remove(joint)
            if previous == len(pending):
                raise ValueError("Disconnected or cyclic URDF")
        config = params["ocs2"]["constraints"]["armSelfCollision"]
        self.config = config
        self.arms = [dict(name=name, **config["arms"][name]) for name in config["arms"]["names"]]
        self.bodies = [dict(name=name, **config["bodies"][name]) for name in config["bodies"]["names"]]
        self.nominal = dict(zip(params["robot"]["jointNames"], params["ocs2"]["reference"]["defaultJointState"], strict=True))
        for sphere in self.arms + self.bodies:
            if sphere["linkName"] not in known:
                raise ValueError(f"Unknown sphere frame: {sphere['linkName']}")

    def poses(self, positions):
        poses = {root: np.eye(4) for root in self.roots}
        for name, kind, parent, child, origin, axis in self.joints:
            motion = np.eye(4)
            if kind in ("revolute", "continuous"):
                motion[:3, :3] = rotation(axis, positions.get(name, 0.0))
            elif kind == "prismatic":
                motion[:3, 3] = axis * positions.get(name, 0.0)
            elif kind != "fixed":
                raise ValueError(f"Unsupported joint type: {kind}")
            poses[child] = poses[parent] @ origin @ motion
        return poses

    def evaluate(self, positions):
        poses = self.poses(positions)
        spheres = self.arms + self.bodies
        centers = np.array([(poses[s["linkName"]] @ np.r_[s["center"], 1.])[:3] for s in spheres])
        n = len(self.arms)
        radii = np.array([s["radius"] for s in spheres])
        gaps = np.linalg.norm(centers[:n, None] - centers[None, n:], axis=2) - radii[:n, None] - radii[None, n:]
        i, j = np.unravel_index(np.argmin(gaps), gaps.shape)
        return float(gaps[i, j]), self.arms[i]["name"] + "/" + self.bodies[j]["name"], centers


def analyze(robot_name, prefix=None, config_path=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    snapshot = Path(str(prefix) + ".yaml") if prefix else None
    config_path = config_path or (snapshot if snapshot and snapshot.exists() else ROOT / f"config/{robot_name}/ros2_controllers.yaml")
    params = yaml.safe_load(config_path.read_text())[CONTROLLER]["ros__parameters"]
    replay = SphereReplay(robot_name, params)
    nominal_gap, nominal_pair, centers = replay.evaluate(replay.nominal)
    summary = dict(robot=robot_name, config=str(config_path), pair_count=len(replay.arms) * len(replay.bodies),
                   activate=replay.config["activate"], minimumDistance=replay.config["minimumDistance"],
                   nominal_surface_gap_m=nominal_gap, nominal_closest_pair=nominal_pair)
    if prefix is None:
        print(json.dumps(summary, indent=2))
        return summary
    samples = json.loads(Path(str(prefix) + "_replay.json").read_text())
    samples = [sample for sample in samples if sample["phase"] != "startup"]
    if not samples:
        raise ValueError("No recorded post-takeover joint poses")
    rows, pairs, all_centers = [], [], []
    for sample in samples:
        positions = replay.nominal | dict(zip(sample["names"], sample["q"], strict=True))
        gap, pair, centers = replay.evaluate(positions)
        rows.append([sample["time"] - samples[0]["time"], gap, gap - replay.config["minimumDistance"], sample["position"][2]])
        pairs.append(pair)
        all_centers.append(centers)
    values = np.asarray(rows)
    minimum = int(np.argmin(values[:, 1]))
    summary.update(samples=len(rows), minimum_surface_gap_m=float(values[minimum, 1]), closest_pair=pairs[minimum],
                   worst_time_after_takeover_s=float(values[minimum, 0]),
                   overlap_samples=int(np.sum(values[:, 1] < 0)),
                   safety_margin_violation_samples=int(np.sum(values[:, 2] < 0)))
    np.savetxt(str(prefix) + "_arm_clearance.csv", values, delimiter=",",
               header="time,min_surface_gap,min_constraint_margin,base_z", comments="")
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(values[:, 0], values[:, 1], label="Minimum arm-body sphere gap")
    axes[0].axhline(replay.config["minimumDistance"], color="orange", linestyle="--", label="Requested margin")
    axes[0].axhline(0, color="red", linestyle=":", label="Sphere overlap boundary")
    axes[0].set_ylabel("m")
    axes[0].legend()
    axes[1].plot(values[:, 0], values[:, 3])
    axes[1].set_ylabel("Base height [m]")
    axes[1].set_xlabel("Time after takeover [s]")
    for ax in axes:
        ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(str(prefix) + "_arm_clearance.png", dpi=140)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(10, 6))
    for ax, dims, title in ((axes[0], (1, 2), "Front"), (axes[1], (0, 2), "Side")):
        for i, s in enumerate(replay.arms + replay.bodies):
            color = "#d05b45" if i < len(replay.arms) else "#4b88b4"
            ax.add_patch(plt.Circle(all_centers[minimum][i, list(dims)], s["radius"], color=color, alpha=.25))
        ax.set_aspect("equal")
        ax.autoscale_view()
        ax.grid(alpha=.25)
        ax.set_title(title)
    fig.suptitle("Sphere approximation at closest approach; not a full mesh check")
    fig.tight_layout()
    fig.savefig(str(prefix) + "_arm_spheres.png", dpi=140)
    plt.close(fig)
    Path(str(prefix) + "_arm_clearance.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", nargs="?", type=Path)
    parser.add_argument("--robot-name", choices=["g1", "h2"], default="h2")
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    analyze(args.robot_name, args.prefix, args.config)
