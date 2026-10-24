"""Export the current scenes' static box tops to a simulation-only MPC terrain model.

Supports horizontal planes and box tops with rectangular XY projections (including
the repository's pitch-only ramp). Dynamic bodies are not foothold surfaces.
Unsupported static geometry fails explicitly rather than inventing flat ground.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def _pose(node, degrees):
    position = np.fromstring(node.get("pos", "0 0 0"), sep=" ")
    if position.size != 3 or not np.all(np.isfinite(position)):
        raise ValueError("Invalid MJCF position")
    if any(key in node.attrib for key in ("axisangle", "xyaxes", "zaxis", "fromto")):
        raise ValueError("Terrain export supports quat/euler orientations only")
    if "euler" in node.attrib:
        angles = np.fromstring(node.get("euler"), sep=" ")
        if angles.size != 3 or not np.all(np.isfinite(angles)):
            raise ValueError("Invalid MJCF Euler angles")
        if degrees:
            angles = np.deg2rad(angles)
        rotation = np.eye(3)
        for axis, angle in zip(np.eye(3), angles):
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            rotation = rotation @ (np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew)
        return position, rotation
    quat = np.fromstring(node.get("quat", "1 0 0 0"), sep=" ")
    if quat.size != 4 or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-9:
        raise ValueError("Invalid MJCF quaternion")
    w, x, y, z = quat / np.linalg.norm(quat)
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return position, rotation


def scene_regions(scene_file):
    regions, floors = [], []
    root = ET.parse(scene_file).getroot()
    compiler_settings = {"angle": "degree", "eulerseq": "xyz"}

    def read_compilers(node, directory, stack):
        if node.tag == "include":
            path = (directory / node.attrib["file"]).resolve()
            if path in stack:
                raise ValueError("Recursive MJCF include: " + str(path))
            read_compilers(ET.parse(path).getroot(), path.parent, stack + [path])
        elif node.tag == "compiler":
            compiler_settings.update(node.attrib)
        else:
            for child in node:
                read_compilers(child, directory, stack)

    scene_file = Path(scene_file).resolve()
    read_compilers(root, scene_file.parent, [scene_file])
    degrees = compiler_settings["angle"] == "degree"
    if compiler_settings["eulerseq"] != "xyz":
        raise ValueError("Terrain export requires compiler eulerseq='xyz'")

    def visit(node, directory, parent_position, parent_rotation, stack):
        if node.tag == "include":
            path = (directory / node.attrib["file"]).resolve()
            if path in stack:
                raise ValueError("Recursive MJCF include: " + str(path))
            visit(ET.parse(path).getroot(), path.parent, parent_position, parent_rotation, stack + [path])
            return
        if node.tag == "body":
            if node.find("freejoint") is not None or node.find("joint") is not None or node.get("mocap") == "true":
                return
            p, r = _pose(node, degrees)
            parent_position, parent_rotation = parent_position + parent_rotation @ p, parent_rotation @ r
        elif node.tag == "geom":
            if node.get("contype") == "0" and node.get("conaffinity") == "0":
                return
            p, r = _pose(node, degrees)
            p, r = parent_position + parent_rotation @ p, parent_rotation @ r
            kind = node.get("type", "sphere")
            if kind == "plane":
                if not np.allclose(r[:, 2], [0, 0, 1], atol=1e-7):
                    raise ValueError("Tilted infinite terrain planes are not supported")
                floors.append(float(p[2]))
            elif kind == "box":
                size = np.fromstring(node.get("size", ""), sep=" ")
                if size.size != 3 or np.any(size <= 0) or not np.all(np.isfinite(size)):
                    raise ValueError("Invalid box size: " + node.get("name", ""))
                normal = r[:, 2]
                if normal[2] < 0.2 or abs(float(r[:2, 0] @ r[:2, 1])) > 1e-7:
                    raise ValueError("Box top does not have a supported upward rectangular XY footprint: " + node.get("name", ""))
                top = p + size[2] * normal
                regions.append({
                    "name": node.get("name", "box"), "center": top[:2].tolist(), "height": float(top[2]),
                    "yaw": math.atan2(r[1, 0], r[0, 0]),
                    "half_extents": [float(size[i] * np.linalg.norm(r[:2, i])) for i in (0, 1)],
                    "normal": normal.tolist(),
                })
            else:
                raise ValueError("Unsupported static terrain geom: " + kind + " / " + node.get("name", ""))
            return
        elif node.tag not in ("mujoco", "mujocoinclude", "worldbody"):
            return
        for child in node:
            visit(child, directory, parent_position, parent_rotation, stack)

    scene_file = Path(scene_file).resolve()
    visit(root, scene_file.parent, np.zeros(3), np.eye(3), [scene_file])
    if not floors or max(floors) - min(floors) > 1e-7:
        raise ValueError("Terrain export needs one common horizontal background floor")
    return {"ground_height": floors[0], "regions": regions}


def write_terrain_config(scene_file, template_file, output_file):
    config = yaml.safe_load(Path(template_file).read_text())
    settings = config.setdefault("terrain_walk", {})
    settings.pop("stairs", None)
    settings.update(scene_regions(scene_file))
    Path(output_file).write_text(yaml.safe_dump(config, sort_keys=False))
    return len(settings["regions"])
