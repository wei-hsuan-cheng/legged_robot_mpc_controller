"""Generate isolated scene fixtures without shipping extra robot scene files."""

import argparse
from contextlib import contextmanager
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODULES = {"none": None, "stairs": "terrains/stairs.xml", "ramp": "terrains/ramp.xml",
           "rough": "terrains/uneven_terrain.xml", "slope": "terrains/slope.xml", "boxes": "objects/boxes.xml"}


def write_scene(output, robot="g1", terrain="none", package=ROOT):
    scenes = (Path(package) / "description/mujoco").resolve()
    robot_dir = scenes / "robots" / robot
    root = ET.parse(robot_dir / "scene.xml").getroot()
    robot_compiler = None
    for element in list(root):
        if element.tag != "include":
            continue
        path = (robot_dir / element.get("file")).resolve()
        if path.parent in (scenes / "terrains", scenes / "objects"):
            root.remove(element)
            continue
        if path.parent == robot_dir:
            robot_compiler = ET.parse(path).find("compiler")
        if path.name.startswith("background_"):
            path = scenes / "common/backgrounds" / ("background_day.xml" if terrain == "none" else "background_night.xml")
        element.set("file", str(path))
    if robot_compiler is None:
        raise ValueError("scene.xml must include its robot model with compiler directories")
    # MuJoCo resolves compiler directories from the entry file, not the robot include.
    compiler = dict(robot_compiler.attrib)
    for key in ("meshdir", "texturedir"):
        if key in compiler:
            compiler[key] = str((robot_dir / compiler[key]).resolve())
    ET.SubElement(root, "compiler", compiler)
    if MODULES[terrain] is not None:
        ET.SubElement(root, "include", file=str(scenes / MODULES[terrain]))
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="unicode")
    return output


@contextmanager
def temporary_scene(robot="g1", terrain="none", package=ROOT):
    with tempfile.TemporaryDirectory(prefix="mpc_scene_") as directory:
        yield write_scene(Path(directory) / "scene.xml", robot, terrain, package)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", choices=("g1", "h2"), default="g1")
    parser.add_argument("--terrain", choices=MODULES, default="none")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(write_scene(args.output, args.robot_name, args.terrain))
