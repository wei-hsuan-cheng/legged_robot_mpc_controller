#!/usr/bin/env python3
"""Check synthetic DualSense roll/pitch commands through teleop and the real wheeled-robot MPC."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Vector3
from legged_robot_teleop.profile import load_robot_profile
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
import yaml

from mujoco_scene import write_scene
from upkie_posture_cycle_test import CONTROLLER, Probe


class OrientationProbe(Probe):
    def __init__(self, config_path, reference):
        super().__init__(reference=reference)
        self.pitch_axis = 0.0
        self.roll_button = None
        self.center = False
        self.orientation = [0.0, 0.0]
        self.create_subscription(Vector3, "/humanoid/base_orientation_command",
                                 self.observe_orientation, qos_profile_sensor_data)
        profile, lateral = load_robot_profile(config_path, CONTROLLER.lstrip("/"))
        arguments = ["ros2", "run", "legged_robot_teleop", "base_twist", "--ros-args",
                     "-p", "joy_topic:=/codex/upkie/joy", "-p", "publish_rate:=100.0",
                     "-p", "stick_exponent:=2.0", "-p", "scale_forward:=1.0",
                     "-p", f"scale_lateral:={1.0 if lateral else 0.0}"]
        for key, value in profile.items():
            arguments.extend(["-p", f"{key}:={value}"])
        self.teleop = subprocess.Popen(arguments, start_new_session=True)

    def observe_orientation(self, message):
        self.orientation = [message.x, message.y]

    def observe(self, message):
        super().observe(message)
        q = message.pose.pose.orientation
        self.samples[-1].update(
            pitch=math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            roll=math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2)),
            command=list(self.orientation))

    def command(self, vx=0.0, yaw=0.0):
        message = Joy()
        message.axes = [0.0, vx, yaw, self.pitch_axis, -self.joy_pressure, 0.0]
        pressed = {self.roll_button} if self.roll_button is not None else set()
        if self.center:
            pressed.add(9)
        message.buttons = [int(i in pressed) for i in range(15)]
        self.joy_publisher.publish(message)


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-name", choices=("upkie", "pineapple"), required=True)
    parser.add_argument("--prefix", default="/tmp/codex_base_orientation")
    args = parser.parse_args()
    prefix = Path(args.prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    config_path = root / f"config/{args.robot_name}/ros2_controllers.yaml"
    parameters = yaml.safe_load(config_path.read_text())[CONTROLLER.lstrip("/")]["ros__parameters"]
    reference = parameters["ocs2"]["reference"]
    scene = write_scene(str(prefix) + ".xml", robot=args.robot_name, terrain="none")
    process = probe = None
    result = {"robot": args.robot_name, "verdict": "FAIL", "stages": {}}
    rclpy.init()
    try:
        with Path(str(prefix) + ".log").open("w") as log:
            process = subprocess.Popen([
                "ros2", "launch", "legged_robot_mpc_controller", "robot.launch.py",
                f"robot_name:={args.robot_name}", "mpc_freq:=200", "mrt_freq:=1000",
                "rviz:=false", "mujoco_headless:=true", "base_command_device:=none",
                f"mujoco_model_file:={scene}", "floating_base_source:=state_estimator",
                "floating_base_estimator:=humanoid_linear_kf"],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            probe = OrientationProbe(config_path, reference)
            probe.wait("idle", 120)
            probe.request("stand_up")
            probe.wait("locomotion", 60)
            probe.pump(3)
            probe.joy_pressure = 1.0
            probe.pump(.2)
            # Reach both pitch limits, hold on release, then combine roll and pitch and drive.
            for stage, axis, duration in (("forward", 1.0, .8), ("backward", -1.0, 1.5)):
                probe.stage = stage
                probe.pitch_axis = axis
                probe.pump(duration)
                probe.pitch_axis = 0.0
                probe.joy_pressure = 0.0
                probe.pump(6)
                check_stage(probe, stage, parameters, result)
                probe.joy_pressure = 1.0
                probe.pump(.2)
            probe.stage = "combined"
            probe.pitch_axis, probe.roll_button = 1.0, 14
            probe.pump(1.0)
            probe.pitch_axis, probe.roll_button = 0.0, None
            # Teleop squares stick deflection, then MPC scales it by maxDisplacementVelocityX.
            probe.pump(6, vx=math.sqrt(.1 / reference["maxDisplacementVelocityX"]))
            check_stage(probe, "combined", parameters, result)
            probe.stage = "center"
            probe.joy_pressure = 0.0
            probe.pump(.2)
            probe.center = True
            probe.pump(.2)
            probe.center = False
            probe.pump(6)
            check_stage(probe, "center", parameters, result)
            probe.request("sit_down")
            probe.wait("idle", 60)
            result["verdict"] = "PASS"
    except Exception as exc:
        result["error"] = str(exc)
        raise
    finally:
        if probe is not None:
            Path(str(prefix) + ".replay.json").write_text(json.dumps(probe.samples))
            stop(probe.teleop)
            probe.destroy_node()
        stop(process)
        rclpy.shutdown()
        Path(str(prefix) + ".json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)


def check_stage(probe, stage, parameters, result):
    assert probe.phase == "locomotion", f"{stage}: robot left locomotion ({probe.phase})"
    end = probe.samples[-1]["time"]
    rows = [s for s in probe.samples if s["stage"] == stage and s["time"] > end - 2]
    assert len(rows) > 20, f"{stage}: missing ground-truth samples"
    yaw = parameters.get("target", {}).get("commandFrameYaw", 0.0)
    c, s = math.cos(yaw), math.sin(yaw)
    mean = lambda key: sum(row[key] for row in rows) / len(rows)
    command = [sum(row["command"][i] for row in rows) / len(rows) for i in range(2)]
    pitch, roll = mean("pitch"), mean("roll")
    result["stages"][stage] = dict(command=command, pitch=pitch, roll=roll,
                                   height=sum(row["position"][2] for row in rows) / len(rows),
                                   max_speed=max(row["speed"] for row in rows))
    expected_roll, expected_pitch = c * command[0] - s * command[1], s * command[0] + c * command[1]
    assert abs(pitch - expected_pitch) < .04, f"{stage}: pitch {pitch:.3f} vs {expected_pitch:.3f}"
    assert abs(roll - expected_roll) < .04, f"{stage}: roll {roll:.3f} vs {expected_roll:.3f}"
    if stage in ("forward", "backward"):
        expected = parameters["ocs2"]["reference"]["maxBasePitch"] * (1 if stage == "forward" else -1)
        assert abs(command[1] - expected) < .01, f"{stage}: joystick did not reach pitch limit"
    if stage == "center":
        assert max(abs(value) for value in command) < .001, "L1 did not level the orientation command"


if __name__ == "__main__":
    main()
