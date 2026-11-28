"""MuJoCo helpers for the Pineapple stand-up: the seated idle spawn, the servo model and the plan's state layout."""

import math
from pathlib import Path

import mujoco
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ROBOT = "pineapple_manipulator"
SCENE = ROOT / "description/mujoco/robots" / ROBOT / "scene.xml"
GEOMETRY = ROOT / "config" / ROBOT / "geometry.yaml"
FRAMES = ["wheel_l", "wheel_r", "base_link", "thigh_l", "thigh_r", "calf_l", "calf_r", "upper_arm", "fore_arm"]


def configure(robot):
    """Select the matching model, geometry and clearance frames before constructing any helpers."""
    global ROBOT, SCENE, GEOMETRY, FRAMES
    if robot not in ("pineapple", "pineapple_manipulator"):
        raise ValueError(f"unsupported Pineapple variant: {robot}")
    ROBOT = robot
    SCENE = ROOT / "description/mujoco/robots" / robot / "scene.xml"
    GEOMETRY = ROOT / "config" / robot / "geometry.yaml"
    FRAMES = ["wheel_l", "wheel_r", "base_link", "thigh_l", "thigh_r", "calf_l", "calf_r"]
    if robot == "pineapple_manipulator":
        FRAMES += ["upper_arm", "fore_arm"]

# Robot joints in robot.jointNames order (the MPC's), then the arm, held at zero by the hardware PD.
JOINTS = ["L_hip_joint", "L_thigh_joint", "L_calf_joint", "L_wheel_joint",
          "R_hip_joint", "R_thigh_joint", "R_calf_joint", "R_wheel_joint"]
ARM = ["arm_joint", "arm_base_joint", "upper_arm_joint", "fore_arm_joint", "5dof_joint", "gripper_case_joint"]
ARM_KP = np.array([60.0, 60.0, 60.0, 20.0, 20.0, 20.0])  # pineapple_manipulator.ros2_control.xacro
ARM_KD = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5])
# Plan playback servo (exported as the plan's kp/kd): legs stiffer than the MPC's 30/1 during the seated phase.
KP = np.array([40.0, 40.0, 40.0, 0.0, 40.0, 40.0, 40.0, 0.0])
KD = np.array([1.0, 1.0, 1.0, 0.1, 1.0, 1.0, 1.0, 0.1])
# Seated idle (hip, thigh, calf per leg): the deepest fold. Spawned 1 cm up it settles on the base box's back bottom
# edge and both tires, pitched back about 0.06 rad, the calf on its -3.229 rad stop.
IDLE = np.array([0.0, 1.65, -3.15, 0.0, 0.0, 1.65, -3.15, 0.0])


def load_geometry(path=None):
    return yaml.safe_load(Path(path or GEOMETRY).read_text())["geometry"]


def euler_rates_matrix(pitch, roll):
    """E with body angular velocity = E [yaw rate, pitch rate, roll rate] (ZYX Euler angles)."""
    sp, cp, sr, cr = math.sin(pitch), math.cos(pitch), math.sin(roll), math.cos(roll)
    return np.array([[-sp, 0.0, 1.0], [sr * cp, cr, 0.0], [cr * cp, -sr, 0.0]])


class Rollout:
    def __init__(self, scene=None):
        self.m = mujoco.MjModel.from_xml_path(str(scene or SCENE))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.arm = [n for n in ARM if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0]
        names = JOINTS + self.arm
        self.arm_kp = ARM_KP[[ARM.index(n) for n in self.arm]]
        self.arm_kd = ARM_KD[[ARM.index(n) for n in self.arm]]
        self.qa = np.array([m.joint(n).qposadr[0] for n in names])
        self.va = np.array([m.joint(n).dofadr[0] for n in names])
        self.act = np.array([next(a for a in range(m.nu) if m.actuator_trnid[a][0] == m.joint(n).id) for n in names])
        self.limits = np.array([m.actuator_ctrlrange[a][1] for a in self.act])
        self.floor = m.geom("floor").id
        self.base = m.body("base_link").id

    def joints(self):
        return self.d.qpos[self.qa[:8]].copy(), self.d.qvel[self.va[:8]].copy()

    def apply(self, q_des, v_des, feedforward, kp=KP, kd=KD):
        q, v = self.d.qpos[self.qa], self.d.qvel[self.va]
        target = np.r_[q_des, np.zeros(len(self.arm))]
        tau = np.r_[kp, self.arm_kp] * (target - q) + np.r_[kd, self.arm_kd] * (
            np.r_[v_des, np.zeros(len(self.arm))] - v)
        tau[:8] += feedforward
        tau = np.clip(tau, -self.limits, self.limits)
        self.d.ctrl[self.act] = tau
        mujoco.mj_step(self.m, self.d)
        return tau[:8]

    def spawn_idle(self, settle=3.0):
        """Spawn upright with the idle joints 1 cm above the floor; the robot settles onto its seat and tires."""
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[self.qa[:8]] = IDLE
        mujoco.mj_forward(m, d)
        lowest = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom) if g != self.floor and m.geom_contype[g])
        d.qpos[2] -= lowest - 0.01
        spawn_height = float(d.qpos[2])
        while d.time < settle:
            self.apply(IDLE, np.zeros(8), np.zeros(8))
        return spawn_height

    def place(self, state):
        """Set the robot to a plan state [p, yaw, pitch, roll, q, rates...] (arm at zero) and settle the contacts."""
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        yaw, pitch, roll = state[3:6]
        cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        d.qpos[:3] = state[:3]
        d.qpos[3:7] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                       cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        d.qpos[self.qa[:8]] = state[6:14]
        mujoco.mj_forward(m, d)

    def euler(self):
        w, x, y, z = self.d.qpos[3:7]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return yaw, pitch, roll

    def tilt(self):
        w, x, y, z = self.d.qpos[3:7]
        return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))

    def state(self):
        """The plan's (and the MPC's) state [p, yaw, pitch, roll, q, p_dot (world), Euler rates, q_dot]."""
        yaw, pitch, roll = self.euler()
        q, v = self.joints()
        rates = np.linalg.solve(euler_rates_matrix(pitch, roll), self.d.qvel[3:6])
        return np.r_[self.d.qpos[:3], yaw, pitch, roll, q, self.d.qvel[:3], rates, v]

    def ground_contacts(self):
        """Bodies touching the floor, with their normal force."""
        out = {}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if self.floor in (c.geom1, c.geom2):
                f = np.zeros(6)
                mujoco.mj_contactForce(self.m, self.d, i, f)
                body = self.m.body(self.m.geom_bodyid[c.geom1 if c.geom2 == self.floor else c.geom2]).name
                out[body] = out.get(body, 0.0) + f[0]
        return out
