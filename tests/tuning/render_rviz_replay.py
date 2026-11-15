#!/usr/bin/env python3
"""Render RViz frames of a recorded run, one per video frame, for stacking with render_upkie_video.py.

usage: render_rviz_replay.py BAG LABELS.json OUT_DIR [--rviz CONFIG] [--fps 30] [--display :1] [--settle 0.12]
                             [--joints PREFIX_replay.json]

BAG is a ros2 bag of /tf, /tf_static, /robot_description and the MPC's /performance_visualization markers (record with
--use-sim-time). LABELS.json gives the time window (start, end; tests/upkie_demo.py writes it); the frames are the same
sim times render_upkie_video.py renders: start + k / fps. For each frame the script publishes /clock, the latest TF of
every transform group and the latest message of every marker topic up to that time, waits for RViz to draw, and grabs
RViz's render area with xwd into OUT_DIR/rviz_%05d.png. RViz runs on DISPLAY (a real or virtual X display) with
config/rviz/upkie_video.rviz by default, whose side camera follows the base like render_upkie_video.py --follow.
robot_state_publisher throttles the recorded link TF to 20 Hz, which lags fast leg motion; --joints instead runs a
robot_state_publisher here and feeds it the joint angles render_upkie_video.py uses (wb_id_walk_test --record-joints,
interpolated at each frame), so both rows show the same leg pose; the base pose still comes from the recorded TF.
Run in the sourced workspace (rclpy, rosbag2_py, cv2, xwininfo, xwd).
"""
import argparse
import json
import os
import tempfile
from pathlib import Path
import re
import struct
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[2]


def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id=""), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    messages = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == "/clock":
            continue
        messages.setdefault(topic, []).append((stamp * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return types, messages


def xwd_image(window, display):
    data = subprocess.run(["xwd", "-silent", "-display", display, "-id", window], capture_output=True, check=True).stdout
    header = struct.unpack(">25I", data[:100])
    size, width, height, line, colors = header[0], header[4], header[5], header[12], header[19]
    pixels = np.frombuffer(data[size + colors * 12:size + colors * 12 + line * height], dtype=np.uint8)
    return pixels.reshape(height, line)[:, :width * 4].reshape(height, width, 4)[:, :, :3]


def render_window(display, timeout=60.0):
    """RViz's 3D render area: the largest rviz2 window that is not the main window."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        tree = subprocess.run(["xwininfo", "-root", "-tree", "-display", display], capture_output=True, text=True).stdout
        found = [(int(w) * int(h), wid, int(w), int(h)) for wid, name, w, h in
                 re.findall(r'(0x[0-9a-f]+) "([^"]*)": \("rviz2" "rviz2"\)\s+(\d+)x(\d+)', tree) if " - RViz" not in name]
        if found:
            _, wid, width, height = max(found)
            if width > 200 and height > 200:
                return wid, width, height
        time.sleep(1.0)
    raise RuntimeError("no RViz render window found")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("labels", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--rviz", type=Path, default=ROOT / "config/rviz/upkie_video.rviz")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":1"))
    parser.add_argument("--settle", type=float, default=0.12, help="wait for RViz to draw each frame [s]")
    parser.add_argument("--joints", type=Path, help="replay JSON whose joint angles drive the robot model (see above)")
    args = parser.parse_args()
    labels = json.loads(args.labels.read_text())
    start, end = labels["start"], labels["end"]
    frames = int(round((end - start) * args.fps))
    args.out.mkdir(parents=True, exist_ok=True)

    types, messages = read_bag(args.bag)
    print("bag topics:", {topic: len(entries) for topic, entries in messages.items()}, flush=True)
    rclpy.init()
    node = rclpy.create_node("rviz_replay")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {topic: node.create_publisher(get_message(types[topic]), topic,
                                               latched if topic in ("/robot_description", "/tf_static") or "Trajectory" in topic
                                               else QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
                  for topic in messages}
    clock = node.create_publisher(Clock, "/clock", 10)
    # Latched with depth 1: one message each, so the static transforms (world -> odom, the IMU, ...) are merged.
    for _, message in messages.get("/robot_description", [])[-1:]:
        publishers["/robot_description"].publish(message)
    if messages.get("/tf_static"):
        static = messages["/tf_static"][0][1]
        static.transforms = [t for _, message in messages["/tf_static"] for t in message.transforms]
        publishers["/tf_static"].publish(static)
    # TF arrives in groups (the estimator's odom -> base, robot_state_publisher's links, ground truth): keep the latest
    # message of each group; markers: the latest message of each topic.
    joints = json.loads(args.joints.read_text()) if args.joints else None
    if joints:
        names, joint_times = joints[0]["names"], np.array([sample["time"] for sample in joints])
        joint_positions = np.array([sample["q"] for sample in joints])
        description = messages["/robot_description"][-1][1].data
        movable = set(re.findall(r'<joint name="[^"]+" type="(?:revolute|continuous|prismatic)">.*?<child link="([^"]+)"',
                                 description, re.S))
        joint_state = node.create_publisher(JointState, "/joint_states", 10)
    streams = {}
    for stamp, message in messages.get("/tf", []):
        key = ("/tf", tuple(sorted(t.child_frame_id for t in message.transforms)))
        if joints and set(key[1]) <= movable:
            continue  # the leg links come from the robot_state_publisher started below
        streams.setdefault(key, []).append((stamp, message))
    for topic in messages:
        if topic.startswith("/performance_visualization/"):
            streams[(topic, ())] = messages[topic]
    cursors = {key: -1 for key in streams}

    # cv2 points Qt at its own plugins; RViz must use the system Qt.
    env = {key: value for key, value in os.environ.items() if not key.startswith("QT_")}
    env["DISPLAY"] = args.display
    rviz_log = open(args.out / "rviz.log", "w")
    helpers = []
    if joints:
        urdf = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
        urdf.write(description)
        urdf.close()
        helpers.append(subprocess.Popen(
            ["ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf.name, "--ros-args",
             "-p", "use_sim_time:=true", "-p", "publish_frequency:=1000.0"], env=env, stdout=rviz_log,
            stderr=subprocess.STDOUT))
    rviz = subprocess.Popen(["rviz2", "-d", str(args.rviz), "--ros-args", "-p", "use_sim_time:=true"], env=env,
                            stdout=rviz_log, stderr=subprocess.STDOUT)
    try:
        window, width, height = render_window(args.display)
        print(f"RViz render area {width}x{height} (window {window})", flush=True)
        (args.out / "rviz_size.json").write_text(json.dumps(dict(width=width, height=height)) + "\n")
        time.sleep(5.0)  # robot model and meshes load
        for k in range(frames + 1):
            t = start + k / args.fps
            message = Clock()
            message.clock.sec, message.clock.nanosec = int(t), int((t - int(t)) * 1e9)
            clock.publish(message)
            if joints:
                state = JointState()
                state.header.stamp = message.clock
                state.name = names
                state.position = [float(np.interp(t, joint_times, joint_positions[:, j])) for j in range(len(names))]
                joint_state.publish(state)
            for key, entries in streams.items():
                i = cursors[key]
                while i + 1 < len(entries) and entries[i + 1][0] <= t:
                    i += 1
                if i != cursors[key] and i >= 0:
                    publishers[key[0]].publish(entries[i][1])
                    cursors[key] = i
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(args.settle if k else 2.0)
            cv2.imwrite(str(args.out / f"rviz_{k:05d}.png"), xwd_image(window, args.display))
            if k % 150 == 0:
                print(f"frame {k}/{frames}  t={t:.2f}", flush=True)
    finally:
        for helper in helpers:
            helper.terminate()
        rviz.terminate()
        node.destroy_node()
        rclpy.shutdown()
    print("wrote", frames + 1, "frames to", args.out)


if __name__ == "__main__":
    main()
