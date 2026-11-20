"""Check that GUI and teleop launch selection is mutually exclusive."""

import importlib.util
from pathlib import Path

import pytest


@pytest.mark.parametrize('device', ['gui', 'joystick', 'none'])
def test_only_one_base_command_source(device):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node

    path = Path(__file__).parents[1] / 'launch/robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch_input_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    gui_node = next(
        a for a in description.entities
        if isinstance(a, Node) and a.node_executable == 'base_command_gui.py'
    )
    teleop_include = next(
        a for a in description.entities if isinstance(a, IncludeLaunchDescription)
    )
    context = LaunchContext()
    context.launch_configurations.update(baseCommandDevice=device)
    gui_enabled = gui_node.condition.evaluate(context)
    teleop_enabled = teleop_include.condition.evaluate(context)
    assert not (gui_enabled and teleop_enabled)
    assert teleop_enabled == (device == 'joystick')
    assert gui_enabled == (device == 'gui')
