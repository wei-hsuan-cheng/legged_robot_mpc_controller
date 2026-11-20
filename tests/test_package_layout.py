"""Check the WB-ID-only package surface and mirrored source/header modules."""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = "legged_robot_mpc_controller"
CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


class PackageLayoutTest(unittest.TestCase):
    def test_public_headers_have_one_package_root(self):
        self.assertEqual({p.name for p in (ROOT / "include").iterdir() if p.is_dir()}, {PACKAGE})
        modules = {"common", "controller", "mpc", "visualization"}
        self.assertEqual({p.name for p in (ROOT / "include" / PACKAGE).iterdir() if p.is_dir()}, modules)
        self.assertEqual({p.name for p in (ROOT / "src").iterdir() if p.is_dir()}, modules)
        for directory in (ROOT / "src/mpc", ROOT / "include" / PACKAGE / "mpc"):
            self.assertEqual({p.name for p in directory.iterdir() if p.is_dir()}, {"common", "wb_id"})

    def test_cpp_local_includes_resolve(self):
        generated = f"{PACKAGE}/{CONTROLLER}_parameters.hpp"
        for directory in (ROOT / "src", ROOT / "include", ROOT / "tests"):
            for path in directory.rglob("*"):
                if path.suffix not in (".h", ".hpp", ".cpp"):
                    continue
                for header in re.findall(r'#include\s*[<"](' + PACKAGE + r'/[^>"]+)[>"]', path.read_text()):
                    if header != generated:
                        self.assertTrue((ROOT / "include" / header).is_file(), f"{path}: {header}")

    def test_only_wb_id_mpc_is_registered(self):
        plugin = ET.parse(ROOT / f"{PACKAGE}_plugin.xml").getroot()
        self.assertEqual([entry.attrib["name"] for entry in plugin.findall("class")],
                         [f"{PACKAGE}/HumanoidWholeBodyInverseDynamicsMpcController"])
        for robot in ("g1", "h2", "upkie"):
            config = yaml.safe_load((ROOT / f"config/{robot}/ros2_controllers.yaml").read_text())
            registrations = config["controller_manager"]["ros__parameters"]
            mpc_names = {name for name, value in registrations.items()
                         if isinstance(value, dict) and value.get("type", "").startswith(PACKAGE + "/")}
            self.assertEqual(mpc_names, {CONTROLLER})
            self.assertEqual({name for name in config if name.endswith("_mpc_controller")}, {CONTROLLER})


if __name__ == "__main__":
    unittest.main()
