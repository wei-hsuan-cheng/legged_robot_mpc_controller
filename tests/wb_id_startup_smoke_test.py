#!/usr/bin/env python3
"""Launch WB-ID on flat ground and stop its process group immediately after MPC takeover.

Run from a sourced ROS workspace; no walking commands are sent.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from mujoco_scene import write_scene


def stop_launch(process):
    for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--prefix", type=Path, default=Path("/tmp/wb_id_startup"))
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    args.prefix.parent.mkdir(parents=True, exist_ok=True)
    log_path = Path(str(args.prefix) + ".log")
    scene = write_scene(Path(str(args.prefix) + "_scene.xml"))
    command = [
        "ros2", "launch", "legged_robot_mpc_controller", "robot.launch.py", "robotName:=g1",
        "mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller",
        "mpcFreq:=100", "mrtFreq:=1000", f"mujocoModelFile:={scene}",
        "rviz:=false", "mujoco_headless:=true", "baseCommandGui:=false",
    ]
    failures = (
        "Traceback (most recent call last)", "while initializing controller",
        "Failed to load humanoid_whole_body_inverse_dynamics_mpc_controller",
        "Failed to configure humanoid_whole_body_inverse_dynamics_mpc_controller",
        "Failed to activate humanoid_whole_body_inverse_dynamics_mpc_controller",
        "Caught exception in launch", "configure failed",
    )
    result = "TIMEOUT"
    started = time.monotonic()
    with log_path.open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic() - started < args.timeout:
                output = log_path.read_text(errors="replace")
                if "-> locomotion" in output:
                    result = "MPC_RUNNING"
                    break
                if any(marker in output for marker in failures):
                    result = "STARTUP_ERROR"
                    break
                if process.poll() is not None:
                    result = "LAUNCH_EXITED"
                    break
                time.sleep(0.1)
        finally:
            takeover_elapsed = time.monotonic() - started
            stop_launch(process)
    verdict = {"result": result, "startup_seconds": takeover_elapsed, "log": str(log_path), "command": command}
    Path(str(args.prefix) + ".json").write_text(json.dumps(verdict, indent=2) + "\n")
    print(json.dumps(verdict, indent=2), flush=True)
    if result != "MPC_RUNNING":
        print(log_path.read_text(errors="replace")[-16000:], flush=True)
    return 0 if result == "MPC_RUNNING" else 1


if __name__ == "__main__":
    raise SystemExit(main())
