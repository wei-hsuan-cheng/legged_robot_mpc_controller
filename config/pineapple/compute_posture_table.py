#!/usr/bin/env python3
"""Balanced symmetric Pineapple postures as a function of base height; writes the posture table YAML that
ocs2.reference.postureTableFile points at (default config/pineapple/posture_table.yaml).

    pip install mujoco numpy pyyaml   # offline tool; not needed at run time
    python3 config/pineapple/compute_posture_table.py [--min 0.20] [--max 0.46] [--step 0.01] [--output FILE]

The tire radius comes from config/pineapple/geometry.yaml (checked against pineapple.xml's collision tire) and is
recorded in the table, so the controller can refuse a table generated for other geometry.
For each base height h the trunk is level at (0, 0, h), the hips are not abducted (wheels upright), both tires touch
the floor (tire centre at the radius) and the tire contact sits under the whole-robot centre of mass, the arm stowed
at zero as the MPC holds it (ocs2.model.fixedJointNames), so the posture is a static balance point of the wheeled
inverted pendulum. The thigh and calf axes are the same on both sides, so the right joints equal the left ones.
Solved by Newton iteration on (thigh, calf) from the previous height, starting at a mid-range crouch.
The maximum height keeps a knee-bend margin: near a straight knee the leg is singular and the servo chatters.
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MJCF = ROOT / "description/mujoco/robots/pineapple/pineapple.xml"
GEOMETRY = yaml.safe_load((HERE / "geometry.yaml").read_text())["geometry"]
RADIUS = GEOMETRY["wheel"]["radius"]
JOINTS = ("L_hip_joint", "L_thigh_joint", "L_calf_joint", "L_wheel_joint",
          "R_hip_joint", "R_thigh_joint", "R_calf_joint", "R_wheel_joint")
START = (0.34, 0.75, -1.5)  # (height, thigh, calf) Newton start


class Model:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(MJCF))
        self.d = mujoco.MjData(self.m)
        self.qadr = {n: self.m.joint(n).qposadr[0] for n in JOINTS}
        self.tires = [self.m.geom(name).id for name in ("left_tire_collision", "right_tire_collision")]
        self.check_tire()

    def check_tire(self):
        """The file's radius and half width must be the MJCF collision prism's (within 1 um)."""
        for tire in self.tires:
            mesh = self.m.geom_dataid[tire]
            start, count = self.m.mesh_vertadr[mesh], self.m.mesh_vertnum[mesh]
            v = self.m.mesh_vert[start:start + count]
            # MuJoCo stores the vertices in the mesh's inertial frame, axes reordered: the axle is the shortest one.
            axle = int(np.argmin(np.abs(v).max(axis=0)))
            radial = [i for i in range(3) if i != axle]
            radius, half = np.hypot(v[:, radial[0]], v[:, radial[1]]).max(), np.abs(v[:, axle]).max()
            wheel = GEOMETRY["wheel"]
            if abs(radius - wheel["radius"]) > 1e-6 or abs(half - wheel["half_width"]) > 1e-6:
                raise SystemExit(f"geometry.yaml wheel {wheel} differs from the MJCF tire ({radius:.6f}, {half:.6f})")
            # The tire is centred on its wheel body (wheel center_offset is zero).
            if np.abs(self.m.geom_pos[tire] - np.asarray(wheel["center_offset"])).max() > 1e-9:
                raise SystemExit("geometry.yaml wheel.center_offset differs from the MJCF tire position")

    def residual(self, height, thigh, calf):
        d = self.d
        d.qpos[:] = 0.0
        d.qpos[:3] = (0.0, 0.0, height)
        d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        for side in "LR":
            d.qpos[self.qadr[side + "_thigh_joint"]] = thigh
            d.qpos[self.qadr[side + "_calf_joint"]] = calf
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        tire = np.mean([d.geom_xpos[t] for t in self.tires], axis=0)
        com = d.subtree_com[1]
        return np.array([tire[2] - RADIUS, tire[0] - com[0]])

    def solve(self, height, thigh, calf):
        x = np.array([thigh, calf])
        for _ in range(50):
            r = self.residual(height, *x)
            if np.linalg.norm(r) < 1e-10:
                return x
            J = np.empty((2, 2))
            for i in range(2):
                dx = np.zeros(2)
                dx[i] = 1e-7
                J[:, i] = (self.residual(height, *(x + dx)) - r) / 1e-7
            x = x - np.linalg.solve(J, r)
        raise RuntimeError(f"no balanced posture at height {height:.3f} m")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--min", type=float, default=0.20)
    parser.add_argument("--max", type=float, default=0.46)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--output", type=Path, default=HERE / "posture_table.yaml")
    args = parser.parse_args()

    model = Model()
    start = model.solve(*START)
    heights = np.round(np.arange(args.min, args.max + 1e-9, args.step), 4)
    solutions = {}
    for direction in (heights[heights >= START[0]], heights[heights < START[0]][::-1]):
        guess = start
        for h in direction:
            guess = model.solve(h, *guess)
            solutions[h] = guess
    ranges = {n: model.m.joint(n).range for n in JOINTS}
    for h, (thigh, calf) in solutions.items():
        for name, value in (("L_thigh_joint", thigh), ("L_calf_joint", calf)):
            lower, upper = ranges[name]
            if not lower <= value <= upper:
                raise SystemExit(f"{h:.3f} m: {name} {value:.3f} outside [{lower}, {upper}]")
    lines = [
        "# Balanced symmetric Pineapple postures by base height: level trunk, hips not abducted, both tires on the",
        "# floor, tire contact under the whole-robot CoM, arm stowed at zero. The MPC interpolates its joint posture",
        "# reference at the target base height and clamps targets to [first, last] height;",
        "# ocs2.reference.minBaseHeight/maxBaseHeight must lie inside. Generated by",
        "# config/pineapple/compute_posture_table.py; do not edit by hand.",
        "posture_table:",
        f"  wheel_radius: {RADIUS:g}  # [m] config/pineapple/geometry.yaml when generated; the controller checks it",
        "  joint_names: [" + ", ".join(JOINTS) + "]",
        "  heights: [" + ", ".join(f"{h:.3f}" for h in heights) + "]",
        "  joint_states:",
    ]
    for h in heights:
        thigh, calf = solutions[h]
        lines.append(f"    - [0.0, {thigh:.6f}, {calf:.6f}, 0.0, 0.0, {thigh:.6f}, {calf:.6f}, 0.0]"
                     f"  # {h:.3f} m, knee bend {abs(calf):.3f} rad")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(f"wrote {args.output}: {len(heights)} heights {heights[0]:.3f}-{heights[-1]:.3f} m, tire radius {RADIUS:.4f} m")


if __name__ == "__main__":
    main()
