"""Per-box YAML entries drive matching MuJoCo and RViz geometry."""

import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

try:
    import xacro
except ImportError:
    xacro = None


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(xacro is None, "xacro Python package is not installed")
class BoxesConfigTest(unittest.TestCase):
    def expand(self, boxes):
        with tempfile.TemporaryDirectory() as directory:
            params = Path(directory) / "boxes.yaml"
            params.write_text(yaml.safe_dump({"boxes": boxes}, sort_keys=False))
            return [ET.fromstring(xacro.process_file(str(ROOT / relative),
                                                    mappings={"boxes_params_file": str(params)}).toxml())
                    for relative in ("description/mujoco/objects/boxes.xml.xacro",
                                     "description/urdf/objects/boxes.urdf.xacro")]

    def test_named_entries_preserve_current_box_values(self):
        boxes = yaml.safe_load((ROOT / "config/objects/boxes_params.yaml").read_text())["boxes"]
        self.assertEqual(list(boxes), ["box_1", "box_2"])
        mjcf, urdf = self.expand(boxes)
        for entry, body in zip(boxes.values(), mjcf.findall("./worldbody/body")):
            name = entry["name"]
            self.assertEqual(body.get("name"), name)
            self.assertEqual(list(map(float, body.get("pos").split())), entry["position"])
            self.assertEqual(float(body.find("inertial").get("mass")), entry["mass"])
            self.assertEqual(list(map(float, body.find("inertial").get("diaginertia").split())), entry["inertia"])
            self.assertEqual(list(map(float, body.find("geom").get("friction").split())), entry["friction"])
            visual = urdf.find(f"./link[@name='{name}']/visual")
            self.assertEqual(list(map(float, visual.find("./geometry/box").get("size").split())), entry["size"])
            self.assertEqual(list(map(float, visual.find("./material/color").get("rgba").split())), entry["color"])

    def test_count_and_attributes_are_independent_without_parallel_lists(self):
        original = yaml.safe_load((ROOT / "config/objects/boxes_params.yaml").read_text())["boxes"]
        boxes = copy.deepcopy(original)
        boxes["box_3"] = dict(position=[2., -1., .2], yaw=.4, size=[.2, .3, .4], mass=2.,
                              inertia=[.2, .3, .4], color=[.1, .2, .3, 1.], friction=[.4, .02, .003])
        mjcf, urdf = self.expand(boxes)
        body = mjcf.find("./worldbody/body[@name='box_3']")
        self.assertIsNotNone(body)
        self.assertIsNotNone(urdf.find("./link[@name='box_3']"))
        self.assertEqual(list(map(float, body.find("geom").get("friction").split())), [.4, .02, .003])
        self.assertEqual(len(mjcf.findall("./worldbody/body")), 3)
        del boxes["box_1"]
        mjcf, urdf = self.expand(boxes)
        self.assertEqual([b.get("name") for b in mjcf.findall("./worldbody/body")], ["orange_box", "box_3"])
        mjcf, urdf = self.expand({})
        self.assertEqual(len(mjcf.findall("./worldbody/body")), 0)
        self.assertEqual([link.get("name") for link in urdf.findall("./link")], ["world"])


if __name__ == "__main__":
    unittest.main()
