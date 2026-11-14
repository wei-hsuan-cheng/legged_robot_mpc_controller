#!/usr/bin/env python3
"""Balanced symmetric Upkie postures as a function of base height; prints the YAML for
ocs2.reference.postureHeights / postureJointStates.

    pip install mujoco numpy          # offline tool; not needed at run time
    python3 config/upkie/compute_posture_table.py [--min 0.20] [--max 0.33] [--step 0.01]

For each base height h the trunk is level at (0, 0, h), both tires touch the floor (tire centre at the
radius) and the tire contact sits under the whole-robot centre of mass, so the posture is a static
balance point of the wheeled inverted pendulum. Right joints mirror the left ones. Solved by Newton
iteration on (hip, knee) from the previous height, starting at the 0.30 m defaultJointState.
The maximum height is chosen with a knee-bend margin: at a straight knee (about 0.34 m) the leg is
singular and the knee servo chatters.
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
MJCF = HERE / ".." / ".." / "description" / "mujoco" / "robots" / "upkie" / "upkie.xml"
RADIUS = 0.055
NOMINAL = (0.30, -0.6021396734839873, -0.9655142016056263)


class Model:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(MJCF))
        self.d = mujoco.MjData(self.m)
        self.qadr = {n: self.m.joint(n).qposadr[0] for n in ("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel")}
        self.tire = self.m.geom("left_foot_collision").id

    def residual(self, height, hip, knee):
        d = self.d
        d.qpos[:] = 0.0
        d.qpos[:3] = (0.0, 0.0, height)
        d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        d.qpos[self.qadr["left_hip"]], d.qpos[self.qadr["left_knee"]] = hip, knee
        d.qpos[self.qadr["right_hip"]], d.qpos[self.qadr["right_knee"]] = -hip, -knee
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        tire = d.geom_xpos[self.tire]
        com = d.subtree_com[1]
        return np.array([tire[2] - RADIUS, tire[0] - com[0]])

    def solve(self, height, hip, knee):
        x = np.array([hip, knee])
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
    parser.add_argument("--max", type=float, default=0.33)
    parser.add_argument("--step", type=float, default=0.01)
    args = parser.parse_args()

    model = Model()
    check = model.solve(*NOMINAL)
    print(f"# check at {NOMINAL[0]:.2f} m: hip {check[0]:+.6f} knee {check[1]:+.6f} "
          f"(defaultJointState {NOMINAL[1]:+.6f} {NOMINAL[2]:+.6f})")
    heights = np.round(np.arange(args.min, args.max + 1e-9, args.step), 4)
    solutions = {}
    for direction in (heights[heights >= NOMINAL[0]], heights[heights < NOMINAL[0]][::-1]):
        guess = check
        for h in direction:
            guess = model.solve(h, *guess)
            solutions[h] = guess
    print("        postureHeights:")
    print("          [" + ", ".join(f"{h:.3f}" for h in heights) + "]")
    print("        postureJointStates:")
    rows = []
    for h in heights:
        hip, knee = solutions[h]
        rows.append(f"[{hip:.6f}, {knee:.6f}, 0.0, {-hip:.6f}, {-knee:.6f}, 0.0]")
    print("          " + ("\n           ".join(r.strip("[]") + "," for r in rows)).rstrip(",").join(("[", "]")))
    for h in heights:
        print(f"# {h:.3f} m: knee bend {abs(solutions[h][1]):.3f} rad")


if __name__ == "__main__":
    main()
