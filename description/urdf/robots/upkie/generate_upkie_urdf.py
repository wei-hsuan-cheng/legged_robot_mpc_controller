#!/usr/bin/env python3
"""Generate the Upkie MPC URDF from the project MJCF, so both describe the same rigid bodies.

    python3 generate_upkie_urdf.py            # writes upkie.urdf next to this script

The MJCF (description/mujoco/robots/upkie/upkie.xml) is the simulation reference. Every link
frame, joint axis, mass, centre of mass and inertia tensor is copied from it unchanged, so the
MPC model and MuJoCo agree by construction rather than by tuning. Mesh visuals keep the MJCF
placements, for RViz. The URDF has no floating-base joint: the MPC and the estimators add the
base in code, as for G1/H2.

Wheels stay scalar revolute joints (nq == nv for the MPC's Translation + SphericalZYX base).
URDF `continuous` would make Pinocchio use a (cos, sin) pair, nq = 14 for nv = 12. Their
position bounds are a large symmetric placeholder; the MPC masks wheel angles out of the
position limits and posture costs explicitly, so the value never acts as a bound.

Joint armature and friction loss are MuJoCo joint properties with no URDF field; the MPC takes
the armature from ocs2.model.jointArmature and smoothed friction from ocs2.model.jointFriction.
"""

import math
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
MJCF = os.path.join(HERE, "..", "..", "..", "mujoco", "robots", "upkie", "upkie.xml")
OUT = os.path.join(HERE, "upkie.urdf")
MESH_URI = "package://legged_robot_mpc_controller/description/meshes/robots/upkie/"

# Joint limits not in the MJCF: effort and velocity from the upstream Upkie URDF
# (upkie_description 94735fb); positions from the MJCF ranges.
EFFORT = {"hip": 16.0, "knee": 16.0, "wheel": 1.7}
VELOCITY = {"hip": 28.8, "knee": 28.8, "wheel": 111.0}
WHEEL_PLACEHOLDER_BOUND = 1.0e4  # [rad], never active: wheel angles are masked in the MPC


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
        # At gimbal lock only a roll/yaw combination is determined; preserve it with yaw = 0.
        roll = math.atan2(2 * (w * x - y * z), 1 - 2 * (x * x + z * z))
        yaw = 0.0
    return roll, pitch, yaw


def fmt(values):
    return " ".join(f"{v:.9g}" for v in values)


def origin(parent, pos, quat):
    ET.SubElement(parent, "origin", xyz=fmt(pos), rpy=fmt(quat_to_rpy(quat)))


def body_pose(body):
    pos = floats(body.get("pos", "0 0 0"), 3)
    quat = floats(body.get("quat", "1 0 0 0"), 4)
    return pos, quat


def add_inertial(link, inertial):
    node = ET.SubElement(link, "inertial")
    ET.SubElement(node, "origin", xyz=fmt(floats(inertial.get("pos"), 3)), rpy="0 0 0")
    ET.SubElement(node, "mass", value=f"{float(inertial.get('mass')):.9g}")
    # MJCF fullinertia order: ixx iyy izz ixy ixz iyz, in the body frame at the inertial pos.
    ixx, iyy, izz, ixy, ixz, iyz = floats(inertial.get("fullinertia"), 6)
    ET.SubElement(node, "inertia", ixx=f"{ixx:.9g}", ixy=f"{ixy:.9g}", ixz=f"{ixz:.9g}",
                  iyy=f"{iyy:.9g}", iyz=f"{iyz:.9g}", izz=f"{izz:.9g}")


def prism_size(vertex_text):
    """Circumradius and half-length of an inline-vertex prism about its local z axis (the tire collision)."""
    v = floats(vertex_text)
    points = [v[i:i + 3] for i in range(0, len(v), 3)]
    # Rounded to 0.01 mm: the vertices are written to 1e-6 m, which perturbs their radii by up to about 6e-7 m.
    return round(max(math.hypot(x, y) for x, y, _ in points), 5), round(max(abs(z) for _, _, z in points), 5)


def add_geoms(link, body, materials, prisms):
    for geom in body.findall("geom"):
        pos = floats(geom.get("pos", "0 0 0"), 3)
        quat = floats(geom.get("quat", "1 0 0 0"), 4)
        if geom.get("class") == "visual" and geom.get("mesh"):
            visual = ET.SubElement(link, "visual")
            origin(visual, pos, quat)
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "mesh", filename=MESH_URI + geom.get("mesh") + ".stl")
            material = geom.get("material")
            if material in materials:
                node = ET.SubElement(visual, "material", name=material)
                ET.SubElement(node, "color", rgba=materials[material])
        elif geom.get("class") == "collision":
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
    for kind in ("hip", "knee", "wheel"):
        if name.endswith(kind):
            return kind
    raise ValueError(name)


def main():
    root = ET.parse(MJCF).getroot()
    materials = {m.get("name"): m.get("rgba") for m in root.iter("material") if m.get("rgba")}
    prisms = {m.get("name"): prism_size(m.get("vertex")) for m in root.iter("mesh") if m.get("vertex")}
    base = root.find("worldbody/body")
    assert base.get("name") == "base"

    robot = ET.Element("robot", name="upkie")
    robot.append(ET.Comment(" Generated by generate_upkie_urdf.py from description/mujoco/robots/upkie/upkie.xml; do not edit. "))

    def add_link(body):
        link = ET.SubElement(robot, "link", name=body.get("name"))
        add_inertial(link, body.find("inertial"))
        add_geoms(link, body, materials, prisms)
        return link

    def recurse(parent_body):
        for body in parent_body.findall("body"):
            add_link(body)
            joints = body.findall("joint")
            assert len(joints) == 1, body.get("name")
            mj_joint = joints[0]
            name = mj_joint.get("name")
            kind = joint_kind(name)
            assert mj_joint.get("pos", "0 0 0") == "0 0 0" and mj_joint.get("axis", "0 0 1") == "0 0 1", name
            joint = ET.SubElement(robot, "joint", name=name, type="revolute")
            pos, quat = body_pose(body)
            origin(joint, pos, quat)
            ET.SubElement(joint, "parent", link=parent_body.get("name"))
            ET.SubElement(joint, "child", link=body.get("name"))
            ET.SubElement(joint, "axis", xyz="0 0 1")
            if kind == "wheel":
                lower, upper = -WHEEL_PLACEHOLDER_BOUND, WHEEL_PLACEHOLDER_BOUND
            else:
                lower, upper = floats(mj_joint.get("range"), 2)
            ET.SubElement(joint, "limit", effort=f"{EFFORT[kind]:g}", velocity=f"{VELOCITY[kind]:g}",
                          lower=f"{lower:.9g}", upper=f"{upper:.9g}")
            recurse(body)

    add_link(base)
    recurse(base)

    # IMU frame at the MJCF imu site.
    imu = base.find("site[@name='imu']")
    ET.SubElement(robot, "link", name="imu")
    joint = ET.SubElement(robot, "joint", name="imu_joint", type="fixed")
    origin(joint, floats(imu.get("pos"), 3), floats(imu.get("quat"), 4))
    ET.SubElement(joint, "parent", link="base")
    ET.SubElement(joint, "child", link="imu")

    ET.indent(robot, space="  ")
    with open(OUT, "w") as stream:
        stream.write('<?xml version="1.0"?>\n')
        stream.write(ET.tostring(robot, encoding="unicode"))
        stream.write("\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
