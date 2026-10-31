#!/usr/bin/env python3
"""Verify real collision sphere markers during an owned headless flat walk."""

import argparse
from collections import Counter
import itertools
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from visualization_msgs.msg import MarkerArray
import yaml

from future_robot_ghosts_test import GhostProbe, ROOT


CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"


def expected_spheres(params):
    result = {}
    constraints = params["ocs2"]["constraints"]
    leg = constraints["legSelfCollision"]
    if leg["activate"]:
        for key in ("leftAnkleFrame", "rightAnkleFrame", "leftKneeFrame", "rightKneeFrame"):
            radius = leg["kneeCollisionSphereRadius" if "Knee" in key else "footCollisionSphereRadius"]
            result[("collision_legs", leg[key])] = dict(radius=radius, center=[0, 0, 0], linkName=leg[key])
        for side in ("l", "r"):
            for suffix in ("", "_collision_p_1", "_collision_p_2"):
                frame = f"foot_{side}_contact{suffix}"
                result[("collision_legs", frame)] = dict(radius=leg["footCollisionSphereRadius"], center=[0, 0, 0], linkName=frame)
    arm = constraints["armSelfCollision"]
    if arm["activate"]:
        for group, namespace in (("arms", "collision_arms"), ("bodies", "collision_body")):
            for name in arm[group]["names"]:
                result[(namespace, name)] = arm[group][name]
    return result


def validate(message, params):
    expected = expected_spheres(params)
    markers = {(m.ns, m.text): m for m in message.markers}
    assert len(markers) == len(message.markers) == len(expected)
    assert markers.keys() == expected.keys(), (markers.keys(), expected.keys())
    ids = set()
    for key, marker in markers.items():
        assert marker.action == marker.ADD and marker.type == marker.SPHERE
        assert marker.header.frame_id == params["visualization"]["frameId"]
        assert (marker.ns, marker.id) not in ids
        ids.add((marker.ns, marker.id))
        assert all(abs(x - 2 * expected[key]["radius"]) < 1e-8 for x in
                   (marker.scale.x, marker.scale.y, marker.scale.z))
        p, q = marker.pose.position, marker.pose.orientation
        assert all(math.isfinite(v) for v in (p.x, p.y, p.z))
        assert (q.x, q.y, q.z, q.w) == (0, 0, 0, 1)
        assert abs(marker.color.a - params["visualization"]["collisionSpheres"]["alpha"]) < 1e-6
        color = {"collision_legs": (0.15, 0.85, 0.15), "collision_arms": (1.0, 0.55, 0.15),
                 "collision_body": (0.15, 0.55, 1.0)}[marker.ns]
        assert all(abs(a - b) < 1e-6 for a, b in zip((marker.color.r, marker.color.g, marker.color.b), color))
        assert marker.lifetime.sec == 1
    # Spheres on the same rigid link must preserve their configured local separation.
    for a, b in itertools.combinations(expected, 2):
        if expected[a]["linkName"] != expected[b]["linkName"]:
            continue
        local = math.dist(expected[a]["center"], expected[b]["center"])
        pa, pb = markers[a].pose.position, markers[b].pose.position
        assert abs(math.dist((pa.x, pa.y, pa.z), (pb.x, pb.y, pb.z)) - local) < 1e-8
    return dict(marker_count=len(markers), groups=dict(Counter(m.ns for m in markers.values())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", choices=("g1", "h2"), required=True)
    parser.add_argument("--prefix", default="/tmp/collision_spheres")
    args = parser.parse_args()
    params = yaml.safe_load((ROOT / f"config/{args.robot_name}/ros2_controllers.yaml").read_text())[CONTROLLER]["ros__parameters"]
    topic = params["visualization"]["collisionSpheres"]["markerTopic"]
    command = [sys.executable, str(ROOT / "tests/wb_id_walk_test.py"),
               "--robot-name", args.robot_name, "--source", "state_estimator", "--estimator", "humanoid_inekf",
               "--axis", "vx", "--speed", "0.1", "--settle", "4", "--ramp", "1", "--hold", "6",
               "--rest", "3", "--mpc-rate", "80", "--prefix", args.prefix + "_walk"]
    rclpy.init()
    probe = GhostProbe()
    process = None
    messages = []
    try:
        with Path(args.prefix + "_driver.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 600
            ready_stamp = None
            connected = False
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.02)
                launch_log = Path(args.prefix + "_walk.log")
                ready = launch_log.exists() and "-> locomotion" in launch_log.read_text(errors="replace")
                if ready and probe.stamp is not None and ready_stamp is None:
                    ready_stamp = probe.stamp
                if ready_stamp is not None and not connected and probe.stamp - ready_stamp >= 1.0:
                    info = probe.get_publishers_info_by_topic(topic)
                    assert info and all(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in info)
                    assert all(i.qos_profile.durability == DurabilityPolicy.VOLATILE for i in info)
                    # Some RMW discovery implementations leave remote history/depth unknown.
                    assert all(i.qos_profile.depth in (0, 1) for i in info)
                    assert probe.count_subscribers(topic) == 0
                    probe.create_subscription(MarkerArray, topic, messages.append, probe.qos)
                    connected = True
                if len(messages) >= 5:
                    break
                if process.poll() is not None:
                    raise RuntimeError("Walking test exited before markers arrived; inspect driver log")
            assert len(messages) >= 5, "Timed out waiting for collision sphere markers"
            for message in messages:
                summary = validate(message, params)
            Path(args.prefix + "_markers.json").write_text(json.dumps(message_to_ordereddict(messages[0]), indent=2) + "\n")
            deadline = time.monotonic() + 180
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.05)
            assert process.poll() == 0, "Owned flat walking test failed or timed out"
            summary.update(robot=args.robot_name, qos="best_effort", checked_messages=5)
            Path(args.prefix + ".json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(summary, indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=30)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
