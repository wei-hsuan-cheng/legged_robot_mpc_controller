#!/usr/bin/env python3
"""Apply a timed wrench sequence to MuJoCo bodies, using simulation time."""

import argparse
import math
from pathlib import Path
import time

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from mujoco_ros2_control.msg import MujocoExternalWrench
except ImportError:
    rclpy = None
    Node = object


TOPIC = "/mujoco_ros2_control/external_wrench"
FEEDBACK_TOPIC = "/mujoco_ros2_control/external_wrench_visualization"


class WrenchSequence:
    """Sequential pulses with a zero-wrench pause after each pulse."""

    def __init__(self, entries):
        if not isinstance(entries, list) or not entries:
            raise ValueError("sequence must be a nonempty YAML list")
        self.events = []
        self.duration = 0.0
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError("each pulse must be a mapping")
            event = dict(entry)
            for field, default in (("link", ""), ("frame", "world")):
                event[field] = entry.get(field, default)
                if not isinstance(event[field], str) or not event[field].strip():
                    raise ValueError(f"{field} must be a nonempty string")
            for field in ("force", "torque"):
                values = entry.get(field, [0.0, 0.0, 0.0])
                if not isinstance(values, list) or len(values) != 3:
                    raise ValueError(f"{field} must contain three finite numbers")
                event[field] = [float(x) for x in values]
                if not all(math.isfinite(x) for x in event[field]):
                    raise ValueError(f"{field} must contain three finite numbers")
            for field, default in (("duration", 0.0), ("wait_after", 0.0)):
                event[field] = float(entry.get(field, default))
                if not math.isfinite(event[field]) or event[field] < 0:
                    raise ValueError(f"{field} must be finite and nonnegative")
            if event["duration"] <= 0:
                raise ValueError("duration must be positive")
            if not any(event["force"] + event["torque"]):
                raise ValueError("each pulse must have a nonzero force or torque")
            event["start"] = self.duration
            event["end"] = self.duration + event["duration"]
            self.events.append(event)
            self.duration = event["end"] + event["wait_after"]

    @classmethod
    def load(cls, path):
        return cls(yaml.safe_load(Path(path).read_text()))

    def sample(self, elapsed):
        for index, event in enumerate(self.events):
            if event["start"] <= elapsed < event["end"]:
                return index, event
        return None


class WrenchPublisher:
    def __init__(self, node, sequence, topic=TOPIC):
        self.node = node
        self.sequence = sequence
        self.publisher = node.create_publisher(
            MujocoExternalWrench, topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.touched = set()

    def send(self, elapsed):
        active = self.sequence.sample(elapsed)
        if active:
            self.touched.add(active[1]["link"])
        for link in sorted(self.touched):
            event = active[1] if active and active[1]["link"] == link else None
            msg = MujocoExternalWrench()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.body_name = link
            msg.wrench_frame = event["frame"] if event else "world"
            if event:
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = event["force"]
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = event["torque"]
            self.publisher.publish(msg)
        return active

    def stop(self):
        self.send(self.sequence.duration)


def default_sequence():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("legged_robot_mpc_controller")) / "config/tests/push_recovery.yaml"
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / "config/tests/push_recovery.yaml"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, default=default_sequence())
    parser.add_argument("--delay", type=float, default=3.0, help="Wait before first pulse [simulation s]")
    parser.add_argument("--topic", default=TOPIC)
    parser.add_argument("--dry-run", action="store_true")
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.delay) or args.delay < 0:
        parser.error("delay must be finite and nonnegative")
    try:
        sequence = WrenchSequence.load(args.sequence)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    if args.dry_run:
        for event in sequence.events:
            print(f"t={args.delay + event['start']:.2f}s: {event}")
        return 0
    if rclpy is None:
        parser.error("Source ROS 2 and the workspace containing mujoco_ros2_control first")
    rclpy.init(args=ros_args)
    node = Node("push_recovery_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
    sender = WrenchPublisher(node, sequence, args.topic)
    start = None
    previous = None
    last_sim = None
    last_progress = time.monotonic()
    startup_wall = last_progress
    result = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = node.get_clock().now().nanoseconds * 1e-9
            if last_sim is not None and now < last_sim:
                raise RuntimeError("Simulation clock reset; stopping the sequence")
            if now != last_sim:
                last_progress = time.monotonic()
                last_sim = now
            if time.monotonic() - last_progress > 30:
                raise RuntimeError("No simulation-clock progress for 30 seconds")
            if start is None:
                if now <= 0 or sender.publisher.get_subscription_count() == 0:
                    if time.monotonic() - startup_wall > 30:
                        raise RuntimeError("No simulation clock or MuJoCo wrench subscriber")
                    continue
                start = now
            if now < start:
                raise RuntimeError("Simulation clock reset; stopping the sequence")
            elapsed = now - start - args.delay
            active = sender.send(elapsed)
            index = active[0] if active else None
            if index != previous and active:
                node.get_logger().info(f"Pulse {index + 1}: {active[1]}")
            previous = index
            if elapsed >= sequence.duration:
                break
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        result = 1
    finally:
        if rclpy.ok():
            # Repeat zeros briefly because this simulator interface is best effort.
            for _ in range(5):
                sender.stop()
                rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
