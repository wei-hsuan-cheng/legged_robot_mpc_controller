"""Wrench scheduling and zero-command checks without ROS or a simulator."""

from pathlib import Path
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "launch"))
import push_recovery_test as push


class PushRecoverySequenceTest(unittest.TestCase):
    def sequence(self):
        return push.WrenchSequence([
            dict(link="pelvis", force=[0, 80, 0], duration=0.15, wait_after=4),
            dict(link="torso_link", torque=[1, 0, 0], duration=0.2, wait_after=1),
        ])

    def test_pulses_pauses_and_end(self):
        seq = self.sequence()
        self.assertIsNone(seq.sample(-1))
        self.assertEqual(seq.sample(0)[0], 0)
        self.assertEqual(seq.sample(0.149)[0], 0)
        self.assertIsNone(seq.sample(0.15))
        self.assertIsNone(seq.sample(4.149))
        self.assertEqual(seq.sample(4.15)[0], 1)
        self.assertIsNone(seq.sample(seq.duration))
        self.assertAlmostEqual(seq.duration, 5.35)

    def test_paused_clock_does_not_advance_sequence(self):
        seq = self.sequence()
        self.assertEqual(seq.sample(0.1), seq.sample(0.1))

    def test_invalid_sequences(self):
        for entries in ([], {}, [1], [dict(link="", force=[1, 0, 0], duration=1)],
                        [dict(link="pelvis", force=[1, 0], duration=1)],
                        [dict(link="pelvis", force=[float("nan"), 0, 0], duration=1)],
                        [dict(link="pelvis", force=[1, 0, 0], duration=0)],
                        [dict(link="pelvis", force=[1, 0, 0], duration=1, wait_after=-1)],
                        [dict(link="pelvis", duration=1)]):
            with self.subTest(entries=entries), self.assertRaises(ValueError):
                push.WrenchSequence(entries)

    def test_example_sequence(self):
        root = Path(__file__).resolve().parents[1]
        seq = push.WrenchSequence.load(root / "config/tests/push_recovery.yaml")
        robot = ET.parse(root / "description/mujoco/robots/h2/h2_31dof.xml").getroot()
        mesh_links = [body.attrib["name"] for body in robot.findall(".//worldbody//body")
                      if any(geom.get("mesh") for geom in body.findall("geom"))]
        force_links = [event["link"] for event in seq.events if any(event["force"])]
        self.assertCountEqual(force_links, mesh_links)
        self.assertEqual(len(seq.events), len(mesh_links))
        for event in seq.events:
            self.assertEqual(event["force"], [0.0, 50.0, 0.0])
            self.assertEqual(event["torque"], [0.0, 0.0, 0.0])

    def test_zero_on_previous_link_and_stop(self):
        def vector():
            return types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

        def message():
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=None),
                                         wrench=types.SimpleNamespace(force=vector(), torque=vector()))

        sent = []
        publisher = types.SimpleNamespace(publish=sent.append)
        node = types.SimpleNamespace(
            create_publisher=lambda *args: publisher,
            get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None)))
        with patch.multiple(push, MujocoExternalWrench=message, QoSProfile=lambda **kwargs: None,
                            ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=0), create=True):
            sender = push.WrenchPublisher(node, self.sequence())
            sender.send(0.1)
            self.assertEqual(sent[-1].wrench.force.y, 80)
            sender.send(4.2)
            pelvis = [msg for msg in sent if msg.body_name == "pelvis"][-1]
            self.assertEqual(pelvis.wrench.force.y, 0)
            sender.stop()
            for msg in sent[-2:]:
                self.assertEqual((msg.wrench.force.y, msg.wrench.torque.x), (0, 0))


if __name__ == "__main__":
    unittest.main()
