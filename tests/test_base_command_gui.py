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
        for center, delta in ((.30, .08), (.795, .4), (1.02, .4)):
            with self.subTest(center=center):
                bar = NAMESPACE["HeightBar"](None, MagicMock(), center, delta)
                self.assertAlmostEqual(bar._value_to_y(center), (bar._top + bar._bottom) / 2)
                self.assertAlmostEqual(bar._y_to_value(bar._top), center + delta)
                self.assertAlmostEqual(bar._y_to_value(bar._bottom), center - delta)
                bar.set(10)
                self.assertAlmostEqual(bar.get(), center + delta)
                bar.set(-10)
                self.assertAlmostEqual(bar.get(), center - delta)
                bar.set(center)
                self.assertEqual(bar.get(), center)

    def test_publication_uses_the_same_range(self):
        publisher = MagicMock()
        node = SimpleNamespace(_publisher=publisher, _minimum_height=.22, _maximum_height=.38)
        for requested, expected in ((.3, .3), (0, .22), (2, .38)):
            NAMESPACE["publish_command"](node, 0, 0, requested, 0)
            self.assertAlmostEqual(publisher.publish.call_args.args[0].desired_pelvis_height, expected)
        node._minimum_height, node._maximum_height = .62, 1.42
        NAMESPACE["publish_command"](node, 0, 0, 1.2, 0)
        self.assertEqual(publisher.publish.call_args.args[0].desired_pelvis_height, 1.2)

    def test_invalid_ranges_are_rejected(self):
        for center, delta in ((0, .1), (.3, -.1), (math.nan, .1), (.3, math.inf)):
            with self.assertRaises(ValueError):
                NAMESPACE["height_limits"](center, delta)

    def test_zero_delta_holds_nominal_height(self):
        bar = NAMESPACE["HeightBar"](None, MagicMock(), .30, 0.0)
        self.assertAlmostEqual(bar._value_to_y(.30), (bar._top + bar._bottom) / 2)
        bar.set(2)
        self.assertEqual(bar.get(), .30)
        self.assertEqual(bar._y_to_value(bar._top), .30)


if __name__ == "__main__":
    unittest.main()
