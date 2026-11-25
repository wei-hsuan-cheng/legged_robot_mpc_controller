#!/usr/bin/env python3
"""Generate the Pineapple MPC URDF from the project MJCF, so both describe the same rigid bodies.

    python3 generate_pineapple_urdf.py        # writes pineapple.urdf next to this script

The MJCF (description/mujoco/robots/pineapple/pineapple.xml) is the simulation reference. Every link frame, joint
axis, mass, centre of mass and inertia tensor is copied from it unchanged, so the MPC model and MuJoCo agree by
construction rather than by tuning. Mesh visuals keep the MJCF placements, for RViz. The URDF has no floating-base
joint: the MPC and the estimators add the base in code, as for G1/H2/Upkie.

Wheels stay scalar revolute joints (nq == nv for the MPC's Translation + SphericalZYX base), with a large symmetric
placeholder bound the MPC never uses (wheel angles are masked out of the position limits and posture costs). The
two gripper fingers are unactuated slide joints in the MJCF; they are fixed here (their mass is lumped into the
gripper case). The six arm joints stay revolute: the controller locks them (ocs2.model.fixedJointNames) and holds
them at zero, the stowed pose.

Joint armature, damping and friction loss are MuJoCo joint properties with no URDF field; the MPC takes the armature
from ocs2.model.jointArmature and smoothed friction from ocs2.model.jointFriction.
"""

import math
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
MJCF = os.path.join(HERE, "..", "..", "..", "mujoco", "robots", "pineapple", "pineapple.xml")
OUT = os.path.join(HERE, "pineapple.urdf")
MESH_URI = "package://legged_robot_mpc_controller/description/meshes/robots/pineapple/"

# Effort limits: the MJCF actuator ctrlrange of each joint. Velocity limits are not in the vendor model: placeholders
# (leg 20 rad/s, wheel 60 rad/s = 5.5 m/s at the tire, arm 10 rad/s) until the motor data is known.
VELOCITY = {"leg": 20.0, "wheel": 60.0, "arm": 10.0}
WHEEL_PLACEHOLDER_BOUND = 1.0e4  # [rad], never active: wheel angles are masked in the MPC
FIXED_JOINTS = ("gripper_left_joint", "gripper_right_joint")


def floats(text, n=None):
    values = [float(v) for v in text.split()]
    if n is not None:
        assert len(values) == n, text
    return values


def quat_to_rpy(q):
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + w * z)
    cos_pitch = math.hypot(r00, r10)
    pitch = math.atan2(2 * (w * y - z * x), cos_pitch)
    if cos_pitch > 1.0e-12:
        roll = math.atan2(2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(2 * (w * x - y * z), 1 - 2 * (x * x + z * z))
        yaw = 0.0
    return roll, pitch, yaw


def fmt(values):
    return " ".join(f"{v:.9g}" for v in values)


def origin(parent, pos, quat):
    ET.SubElement(parent, "origin", xyz=fmt(pos), rpy=fmt(quat_to_rpy(quat)))


def add_inertial(link, inertial):
    node = ET.SubElement(link, "inertial")
    pos = floats(inertial.get("pos"), 3)
    if inertial.get("fullinertia"):
        # MJCF fullinertia order: ixx iyy izz ixy ixz iyz, in the body frame at the inertial pos.
        ET.SubElement(node, "origin", xyz=fmt(pos), rpy="0 0 0")
        ixx, iyy, izz, ixy, ixz, iyz = floats(inertial.get("fullinertia"), 6)
    else:
        # Principal inertia along the inertial quat's axes.
        origin(node, pos, floats(inertial.get("quat", "1 0 0 0"), 4))
        ixx, iyy, izz = floats(inertial.get("diaginertia"), 3)
        ixy = ixz = iyz = 0.0
    ET.SubElement(node, "mass", value=f"{float(inertial.get('mass')):.9g}")
    ET.SubElement(node, "inertia", ixx=f"{ixx:.9g}", ixy=f"{ixy:.9g}", ixz=f"{ixz:.9g}",
                  iyy=f"{iyy:.9g}", iyz=f"{iyz:.9g}", izz=f"{izz:.9g}")


def prism_size(vertex_text):
    """Circumradius and half-length of an inline-vertex prism about its local z axis (the tire collision)."""
    v = floats(vertex_text)
    points = [v[i:i + 3] for i in range(0, len(v), 3)]
    return round(max(math.hypot(x, y) for x, y, _ in points), 5), round(max(abs(z) for _, _, z in points), 5)


def mesh_file(name, files):
    return MESH_URI + files.get(name, name + ".obj")


def add_geoms(link, body, materials, prisms, files):
    for geom in body.findall("geom"):
        pos = floats(geom.get("pos", "0 0 0"), 3)
        quat = floats(geom.get("quat", "1 0 0 0"), 4)
        kind = geom.get("class")
        if geom.get("mesh") and geom.get("mesh") not in prisms and kind != "collision":
            visual = ET.SubElement(link, "visual")
            origin(visual, pos, quat)
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "mesh", filename=mesh_file(geom.get("mesh"), files))
            material = geom.get("material")
            rgba = materials.get(material) or geom.get("rgba")
            if rgba:
                node = ET.SubElement(visual, "material", name=material or f"{body.get('name')}_material")
                ET.SubElement(node, "color", rgba=rgba)
        elif kind in ("collision", "wheel"):
            collision = ET.SubElement(link, "collision", name=geom.get("name", ""))
            origin(collision, pos, quat)
            geometry = ET.SubElement(collision, "geometry")
            size = floats(geom.get("size")) if geom.get("size") else []
            if geom.get("type") == "box":
                ET.SubElement(geometry, "box", size=fmt([2 * s for s in size]))
            elif geom.get("type") == "cylinder":
                ET.SubElement(geometry, "cylinder", radius=f"{size[0]:.9g}", length=f"{2 * size[1]:.9g}")
            elif geom.get("type") == "mesh" and geom.get("mesh") in prisms:
                radius, half_length = prisms[geom.get("mesh")]
                ET.SubElement(geometry, "cylinder", radius=f"{radius:.9g}", length=f"{2 * half_length:.9g}")
            else:
                raise ValueError(f"unsupported collision geom type {geom.get('type')}")


def joint_kind(name):
    if name.endswith("wheel_joint"):
        return "wheel"
    if name[:2] in ("L_", "R_"):
        return "leg"
    return "arm"


def main():
    root = ET.parse(MJCF).getroot()
    materials = {m.get("name"): m.get("rgba") for m in root.iter("material") if m.get("rgba")}
    prisms = {m.get("name"): prism_size(m.get("vertex")) for m in root.iter("mesh") if m.get("vertex")}
    files = {m.get("name"): m.get("file") for m in root.iter("mesh") if m.get("file") and m.get("name")}
    effort = {}
    classes = {d.get("class"): d.find("motor") for d in root.iter("default") if d.find("motor") is not None}
    for motor in root.find("actuator").findall("motor"):
        effort[motor.get("joint")] = floats(classes[motor.get("class")].get("ctrlrange"), 2)[1]
    base = root.find("worldbody/body")
    assert base.get("name") == "base_link"

    robot = ET.Element("robot", name="pineapple")
    robot.append(ET.Comment(" Generated by generate_pineapple_urdf.py from description/mujoco/robots/pineapple/"
                            "pineapple.xml; do not edit. "))

    def add_link(body):
        link = ET.SubElement(robot, "link", name=body.get("name"))
        add_inertial(link, body.find("inertial"))
        add_geoms(link, body, materials, prisms, files)

    def recurse(parent_body):
        for body in parent_body.findall("body"):
            add_link(body)
            joints = body.findall("joint")
            assert len(joints) == 1, body.get("name")
            mj_joint = joints[0]
            name = mj_joint.get("name")
            assert mj_joint.get("pos", "0 0 0") == "0 0 0", name
            fixed = name in FIXED_JOINTS
            joint = ET.SubElement(robot, "joint", name=name, type="fixed" if fixed else "revolute")
            origin(joint, floats(body.get("pos", "0 0 0"), 3), floats(body.get("quat", "1 0 0 0"), 4))
            ET.SubElement(joint, "parent", link=parent_body.get("name"))
            ET.SubElement(joint, "child", link=body.get("name"))
            if not fixed:
                assert mj_joint.get("type", "hinge") == "hinge", name
                ET.SubElement(joint, "axis", xyz=fmt(floats(mj_joint.get("axis"), 3)))
                kind = joint_kind(name)
                if kind == "wheel":
                    lower, upper = -WHEEL_PLACEHOLDER_BOUND, WHEEL_PLACEHOLDER_BOUND
                else:
                    lower, upper = floats(mj_joint.get("range"), 2)
                ET.SubElement(joint, "limit", effort=f"{effort[name]:g}", velocity=f"{VELOCITY[kind]:g}",
                              lower=f"{lower:.9g}", upper=f"{upper:.9g}")
            recurse(body)

    add_link(base)
    recurse(base)

    # IMU frame at the MJCF imu site.
    imu = base.find("site[@name='imu']")
    ET.SubElement(robot, "link", name="imu")
    joint = ET.SubElement(robot, "joint", name="imu_joint", type="fixed")
    origin(joint, floats(imu.get("pos", "0 0 0"), 3), floats(imu.get("quat", "1 0 0 0"), 4))
    ET.SubElement(joint, "parent", link="base_link")
    ET.SubElement(joint, "child", link="imu")

    ET.indent(robot, space="  ")
    with open(OUT, "w") as stream:
        stream.write('<?xml version="1.0"?>\n')
        stream.write(ET.tostring(robot, encoding="unicode"))
        stream.write("\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
