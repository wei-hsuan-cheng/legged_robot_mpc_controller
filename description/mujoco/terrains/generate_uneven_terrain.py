#!/usr/bin/env python3
"""Generate seeded uneven terrain and matching RViz geometry from shared YAML."""

import argparse
import json
import math
from pathlib import Path
import random
import statistics
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate(config):
    for key, count in (("size", 2), ("grid", 2), ("friction", 3)):
        if len(config[key]) != count or not all(finite(v) for v in config[key]):
            raise ValueError(f"{key} must contain {count} finite numbers")
    if not all(v > 0 for v in config["size"]):
        raise ValueError("size must be positive")
    if not all(isinstance(n, int) and n >= 2 for n in config["grid"]):
        raise ValueError("grid must contain two integer cell counts >= 2")
    if math.prod(config["grid"]) > 10000:
        raise ValueError("grid exceeds the 10000-cell safety limit")
    if not isinstance(config["seed"], int) or isinstance(config["seed"], bool):
        raise ValueError("seed must be an integer")
    for values, count, name in ((config["base"]["pos"], 3, "base.pos"),
                                 (config["profile"]["wavelength"], 2, "profile.wavelength"),
                                 (config["clear_patch"]["center"], 2, "clear_patch.center"),
                                 (config["clear_patch"]["size"], 2, "clear_patch.size")):
        if len(values) != count or not all(finite(v) for v in values):
            raise ValueError(f"{name} must contain {count} finite numbers")
    height, profile = config["height"], config["profile"]
    step = height["adjacent_difference"]
    scalars = [config["base"]["yaw"], config["depth"], config["gap"], config["color_gray"],
               height["min"], height["max"], height["jitter"], step["min"], step["max"], step["mode"],
               profile["amplitude"], profile["strength"]]
    if not all(finite(v) for v in scalars):
        raise ValueError("height, profile and physical parameters must be finite")
    if not 0 < step["min"] <= step["max"]:
        raise ValueError("adjacent_difference requires 0 < min <= max")
    if step["distribution"] not in ("uniform", "triangular"):
        raise ValueError("adjacent_difference.distribution must be uniform or triangular")
    if not step["min"] <= step["mode"] <= step["max"]:
        raise ValueError("adjacent_difference.mode must lie between min and max")
    jitter = height["jitter"]
    if jitter < 0 or 4 * jitter > step["max"] - step["min"] + 1e-12:
        raise ValueError("jitter must be in [0, (adjacent max - min) / 4]")
    axis_span = (height["max"] - height["min"]) / 2 - jitter
    if axis_span < 2 * (step["min"] + 2 * jitter) - 1e-12:
        raise ValueError("height envelope is too narrow for the adjacent minimum and jitter; widen min/max or reduce jitter")
    if config["base"]["pos"][2] + height["min"] < 0:
        raise ValueError("tile tops must not be below the background floor at world z=0")
    if config["depth"] <= 0 or config["depth"] + height["min"] <= 0:
        raise ValueError("depth must be positive and below all tile tops")
    if not 0 <= config["gap"] < min(s / n for s, n in zip(config["size"], config["grid"])):
        raise ValueError("gap must be nonnegative and smaller than both cell pitches")
    if any(v < 0 for v in config["friction"]) or not 0 <= config["color_gray"] <= 1:
        raise ValueError("friction must be nonnegative and color_gray must be in [0, 1]")
    if not 0 <= profile["strength"] <= 1 or not 0 <= profile["amplitude"] <= axis_span / 2:
        raise ValueError("profile strength must be in [0, 1] and amplitude in [0, half the per-axis height span]")
    if any(v <= 0 for v in profile["wavelength"]):
        raise ValueError("profile wavelengths must be positive")
    patch = config["clear_patch"]
    if not (all(v == 0 for v in patch["size"]) or all(v > 0 for v in patch["size"])):
        raise ValueError("clear_patch.size must be [0, 0] or two positive lengths")
    if any(abs(c) + p / 2 > s / 2 + 1e-12 for c, p, s in zip(patch["center"], patch["size"], config["size"])):
        raise ValueError("clear_patch must fit inside the field")


def axis_profile(config, axis, rng):
    """Bounded random walk biased toward a broad sinusoidal height tendency."""
    height, trend = config["height"], config["profile"]
    step = height["adjacent_difference"]
    jitter = height["jitter"]
    span = (height["max"] - height["min"]) / 2 - jitter
    minimum, maximum = step["min"] + 2 * jitter, step["max"] - 2 * jitter
    length, count = config["size"][axis], config["grid"][axis]
    phase = rng.uniform(-math.pi, math.pi)
    values = [span / 2]
    for index in range(1, count):
        x = -length / 2 + (index + 0.5) * length / count
        target = span / 2 + trend["amplitude"] * math.sin(2 * math.pi * x / trend["wavelength"][axis] + phase)
        previous = values[-1]
        signs = [sign for sign in (-1, 1) if (span - previous if sign > 0 else previous) >= minimum - 1e-12]
        preferred = 1 if target > previous else -1
        sign = preferred if preferred in signs and rng.random() < trend["strength"] else rng.choice(signs)
        upper = min(maximum, span - previous if sign > 0 else previous)
        upper = max(minimum, upper)  # Absorb roundoff at a feasible envelope boundary.
        if step["distribution"] == "uniform":
            distance = rng.uniform(minimum, upper)
        else:
            mode = min(upper, max(minimum, step["mode"]))
            distance = rng.triangular(minimum, upper, mode)
        values.append(min(span, max(0.0, previous + sign * distance)))
    return values


def generate_heights(config):
    validate(config)
    rng = random.Random(config["seed"])
    x_values, y_values = (axis_profile(config, axis, rng) for axis in (0, 1))
    jitter = config["height"]["jitter"]
    # Reserving 2*jitter in every axis increment keeps all final edge differences in bounds.
    heights = [[config["height"]["min"] + jitter + x + y + rng.uniform(-jitter, jitter)
                for y in y_values] for x in x_values]
    differences = adjacent_differences(heights)
    bounds = config["height"]["adjacent_difference"]
    if any(not bounds["min"] - 1e-10 <= d <= bounds["max"] + 1e-10 for d in differences):
        raise ValueError("generated terrain violates adjacent height bounds")
    return heights


def adjacent_differences(heights):
    nx, ny = len(heights), len(heights[0])
    return [abs(heights[i][j] - heights[i + di][j + dj])
            for i in range(nx) for j in range(ny) for di, dj in ((1, 0), (0, 1))
            if i + di < nx and j + dj < ny]


def outside_patch(rectangle, patch):
    """Clip a cell to an exact rectangular opening, without shrinking the surrounding field."""
    x0, x1, y0, y1 = rectangle
    if patch["size"] == [0, 0]:
        return [rectangle]
    cx, cy = patch["center"]
    sx, sy = patch["size"]
    a, b = max(x0, cx - sx / 2), min(x1, cx + sx / 2)
    c, d = max(y0, cy - sy / 2), min(y1, cy + sy / 2)
    if a >= b or c >= d:
        return [rectangle]
    return [r for r in ((x0, a, y0, y1), (b, x1, y0, y1), (a, b, y0, c), (a, b, d, y1))
            if r[1] - r[0] > 1e-10 and r[3] - r[2] > 1e-10]


def generate_blocks(config, heights):
    nx, ny = config["grid"]
    lx, ly = config["size"]
    gap, bottom = config["gap"], -config["depth"]
    blocks = []
    for i in range(nx):
        for j in range(ny):
            rectangle = (-lx / 2 + i * lx / nx + gap / 2, -lx / 2 + (i + 1) * lx / nx - gap / 2,
                         -ly / 2 + j * ly / ny + gap / 2, -ly / 2 + (j + 1) * ly / ny - gap / 2)
            for part, (x0, x1, y0, y1) in enumerate(outside_patch(rectangle, config["clear_patch"])):
                top = heights[i][j]
                blocks.append({"name": f"terrain_{i:03d}_{j:03d}_{part}", "cell": [i, j],
                               "pos": [(x0 + x1) / 2, (y0 + y1) / 2, (top + bottom) / 2],
                               "size": [x1 - x0, y1 - y0, top - bottom]})
    if not blocks:
        raise ValueError("clear_patch removes every terrain block")
    return blocks


def fmt(values):
    return " ".join(f"{v:.12g}" for v in values)


def build_mjcf(config, blocks):
    root = ET.Element("mujoco", model="uneven_terrain")
    root.append(ET.Comment(" Generated by generate_uneven_terrain.py; edit config/terrains/uneven_terrain_params.yaml. "))
    world = ET.SubElement(root, "worldbody")
    yaw = config["base"]["yaw"]
    body = ET.SubElement(world, "body", name="uneven_terrain", pos=fmt(config["base"]["pos"]),
                         quat=fmt([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))
    for block in blocks:
        ET.SubElement(body, "geom", name=block["name"], type="box", pos=fmt(block["pos"]),
                      size=fmt([v / 2 for v in block["size"]]), material="studio_terrain", rgba="1 1 1 1",
                      friction=fmt(config["friction"]), contype="1", conaffinity="1")
    return root


def build_urdf(config, blocks):
    root = ET.Element("robot", name="uneven_terrain")
    root.append(ET.Comment(" Generated alongside the MuJoCo terrain from the same YAML. "))
    ET.SubElement(root, "link", name="world")
    link = ET.SubElement(root, "link", name="uneven_terrain")
    for block in blocks:
        for kind in ("visual", "collision"):
            element = ET.SubElement(link, kind, name=block["name"])
            ET.SubElement(element, "origin", xyz=fmt(block["pos"]), rpy="0 0 0")
            ET.SubElement(ET.SubElement(element, "geometry"), "box", size=fmt(block["size"]))
            if kind == "visual":
                material = ET.SubElement(element, "material", name=block["name"] + "_material")
                ET.SubElement(material, "color", rgba=fmt([config["color_gray"]] * 3 + [1]))
    joint = ET.SubElement(root, "joint", name="uneven_terrain_joint", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="uneven_terrain")
    ET.SubElement(joint, "origin", xyz=fmt(config["base"]["pos"]), rpy=fmt([0, 0, config["base"]["yaw"]]))
    return root


def xml_text(root):
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=ROOT / "config/terrains/uneven_terrain_params.yaml")
    parser.add_argument("--mjcf", type=Path, default=ROOT / "description/mujoco/terrains/uneven_terrain.xml")
    parser.add_argument("--urdf", type=Path, default=ROOT / "description/urdf/terrains/uneven_terrain.urdf")
    parser.add_argument("--check", action="store_true", help="verify generated files without writing")
    args = parser.parse_args()
    try:
        config = yaml.safe_load(args.params.read_text())["uneven_terrain"]
        heights = generate_heights(config)
        blocks = generate_blocks(config, heights)
        for path, content in ((args.mjcf, xml_text(build_mjcf(config, blocks))),
                              (args.urdf, xml_text(build_urdf(config, blocks)))):
            if args.check:
                if not path.is_file() or path.read_text() != content:
                    raise ValueError(f"stale generated file: {path}")
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
        differences = adjacent_differences(heights)
        print(json.dumps({"grid": config["grid"], "blocks": len(blocks),
                          "tile_pitch_m": [s / n for s, n in zip(config["size"], config["grid"])],
                          "height_m": [min(map(min, heights)), max(map(max, heights))],
                          "adjacent_difference_m": {"min": min(differences), "max": max(differences),
                                                     "mean": statistics.mean(differences),
                                                     "std": statistics.pstdev(differences)}}, indent=2))
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
