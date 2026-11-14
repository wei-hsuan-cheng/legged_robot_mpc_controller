"""Exercise GUI height mapping and publishing without ROS or a display."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock


SOURCE = Path(__file__).resolve().parents[1] / "launch/base_command_gui.py"
TREE = ast.parse(SOURCE.read_text())
NAMESPACE = dict(math=math, tk=SimpleNamespace(Canvas=MagicMock()),
                 KNOB_IDLE_COLOR="blue", KNOB_ACTIVE_COLOR="blue", WalkingVelocityCommand=SimpleNamespace)
SELECTED = [node for node in TREE.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))
            and node.name in ("height_limits", "HeightBar")]
PUBLISHER = next(node for node in TREE.body if isinstance(node, ast.ClassDef) and node.name == "PublisherNode")
SELECTED += [node for node in PUBLISHER.body if isinstance(node, ast.FunctionDef) and node.name == "publish_command"]
exec(compile(ast.Module(body=SELECTED, type_ignores=[]), str(SOURCE), "exec"), NAMESPACE)


class BaseCommandGuiTest(unittest.TestCase):
    def test_robot_height_mapping_and_reset(self):
        for center, minimum, maximum in ((.30, .22, .33), (.795, .395, 1.195), (1.02, .62, 1.42)):
            with self.subTest(center=center):
                bar = NAMESPACE["HeightBar"](None, MagicMock(), center, minimum, maximum)
                self.assertAlmostEqual(bar._y_to_value(bar._value_to_y(center)), center)
                self.assertAlmostEqual(bar._y_to_value(bar._top), maximum)
                self.assertAlmostEqual(bar._y_to_value(bar._bottom), minimum)
                bar.set(10)
                self.assertAlmostEqual(bar.get(), maximum)
                bar.set(-10)
                self.assertAlmostEqual(bar.get(), minimum)
                bar.set(center)
                self.assertEqual(bar.get(), center)

    def test_publication_uses_the_same_range(self):
        publisher = MagicMock()
        node = SimpleNamespace(_publisher=publisher, _minimum_height=.22, _maximum_height=.33)
        for requested, expected in ((.3, .3), (0, .22), (2, .33)):
            NAMESPACE["publish_command"](node, 0, 0, requested, 0)
            self.assertAlmostEqual(publisher.publish.call_args.args[0].desired_pelvis_height, expected)
        node._minimum_height, node._maximum_height = .62, 1.42
        NAMESPACE["publish_command"](node, 0, 0, 1.2, 0)
        self.assertEqual(publisher.publish.call_args.args[0].desired_pelvis_height, 1.2)

    def test_invalid_ranges_are_rejected(self):
        for center, minimum, maximum in ((0, 0, .1), (.3, .33, .22), (.3, -.1, .4), (math.nan, .2, .4),
                                         (.3, .2, math.inf), (.4, .22, .33), (.2, .22, .33)):
            with self.assertRaises(ValueError):
                NAMESPACE["height_limits"](center, minimum, maximum)

    def test_empty_range_holds_nominal_height(self):
        bar = NAMESPACE["HeightBar"](None, MagicMock(), .30, .30, .30)
        self.assertAlmostEqual(bar._value_to_y(.30), (bar._top + bar._bottom) / 2)
        bar.set(2)
        self.assertEqual(bar.get(), .30)
        self.assertEqual(bar._y_to_value(bar._top), .30)


if __name__ == "__main__":
    unittest.main()
