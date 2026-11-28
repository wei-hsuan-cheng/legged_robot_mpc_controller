"""Generate isolated scene fixtures without shipping extra robot scene files."""

import argparse
from contextlib import contextmanager
from pathlib import Path
import tempfile
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "description/mujoco"))
from mujoco_scene_creator import load_scene
MODULES = {"none": None, "stairs": "terrains/stairs.xml", "ramp": "terrains/ramp.xml",
           "rough": "terrains/uneven_terrain.xml", "slope": "terrains/slope.xml", "boxes": "objects/boxes.xml",
           "upkie_ramp": "terrains/upkie_ramp.xml", "upkie_rough": "terrains/upkie_uneven_terrain.xml",
           "upkie_course": "terrains/upkie_course.xml",
           "pineapple_gauntlet": "terrains/pineapple_gauntlet.xml",
           "split_height": None}


def write_scene(output, robot="g1", terrain="none", package=ROOT, split_height=0.01):
    scenes = (Path(package) / "description/mujoco").resolve()
    robot_dir = scenes / "robots" / robot
    root = load_scene(robot_dir / "scene.xml", str(scenes))
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
            background = {"upkie": "background_night_upkie.xml",
                          "pineapple": "background_night_pineapple.xml",
                          "pineapple_manipulator": "background_night_pineapple.xml"}.get(robot, "background_night.xml")
            path = scenes / "common/backgrounds" / ("background_day.xml" if terrain == "none" else background)
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
    if terrain == "split_height":
        if not 0.0 < split_height <= 0.05:
            raise ValueError("split height must be in (0, 0.05] m")
        # Left tire traverses the raised strip while the right remains on the floor.
        world = ET.SubElement(root, "worldbody")
        ET.SubElement(world, "geom", name="split_height_strip", type="box",
                      pos=f"2.0 0.2 {split_height / 2}", size=f"1.4 0.14 {split_height / 2}",
                      material="studio_terrain", friction="0.8 0.005 0.0001", priority="1")
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
    parser.add_argument("--robot-name", choices=("g1", "h2", "upkie", "pineapple", "pineapple_manipulator"), default="g1")
    parser.add_argument("--terrain", choices=MODULES, default="none")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split-height", type=float, default=0.01, help="Raised left-wheel strip [m]")
    args = parser.parse_args()
    print(write_scene(args.output, args.robot_name, args.terrain, split_height=args.split_height))
