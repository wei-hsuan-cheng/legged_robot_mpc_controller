"""Click the real Tk posture buttons against isolated ROS services (run under xvfb-run)."""

import importlib.util
import os
from pathlib import Path
import threading
import time

import pytest


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="Needs a display or xvfb-run")
def test_gui_posture_buttons_and_controller_state():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    spec = importlib.util.spec_from_file_location("posture_gui", Path(__file__).parents[1] / "launch/base_command_gui.py")
    gui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gui)
    rclpy.init(args=["--ros-args", "-p", "posture_controller:=/gui_posture_test"])
    publisher = gui.PublisherNode()
    probe = Node("gui_posture_probe", parameter_overrides=[Parameter("use_sim_time", value=False)])
    calls = []

    def callback(action):
        def respond(request, response):
            calls.append(action)
            response.success = True
            response.message = "accepted"
            return response
        return respond

    servers = [probe.create_service(Trigger, "/gui_posture_test/" + name, callback(name))
               for name in ("stand_up", "sit_down")]
    state = probe.create_publisher(String, "/gui_posture_test/motion_state",
                                   QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    app = gui.baseCommandGui(publisher.publish_command, 1.0, 0.0, 1.0, .22, .2, .33, publisher)

    def pump(phase, duration=.3):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            state.publish(String(data=phase))
            app.update()
            time.sleep(.01)

    try:
        pump("idle", .6)
        assert app._stand_button["state"] == "normal"
        assert app._sit_button["state"] == "disabled"
        app._stand_button.invoke()
        pump("standing up")
        assert calls == ["stand_up"]
        assert app._stand_button["state"] == app._sit_button["state"] == "disabled"
        pump("locomotion")
        assert app._sit_button["state"] == "normal"
        app._sit_button.invoke()
        pump("sitting down")
        assert calls == ["stand_up", "sit_down"]
        pump("fallen")
        assert app._stand_button["state"] == app._sit_button["state"] == "disabled"
    finally:
        app.destroy()
        executor.shutdown()
        thread.join(timeout=2)
        publisher.destroy_node()
        for server in servers:
            probe.destroy_service(server)
        probe.destroy_node()
        rclpy.shutdown()
