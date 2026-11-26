"""Robot-independent MuJoCo/MPC launch; select Unitree G1, H2, the Upkie wheeled-biped or the Pineapple
wheel-legged robot with robot_name.

Robot configuration, scenes and CppAD caches are selected together. The selected
controller YAML owns the reference height used by MPC and the selected command frontend.
"""

import math
import os
import tempfile
import yaml
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

# Launch arguments before the snake_case rename. ros2 launch accepts any name:=value without complaint, so an old
# name would be ignored silently (robotName:=upkie would start G1); reject them instead.
RENAMED_ARGUMENTS = {
    "robotName": "robot_name",
    "rvizconfig": "rviz_config",
    "modelErrorFile": "model_error_file",
    "modelErrorSeed": "model_error_seed",
    "modelErrorReport": "model_error_report",
    "urdfFile": "urdf_file",
    "mpcControllerName": "mpc_controller_name",
    "libFolder": "lib_folder",
    "floatingBaseSource": "floating_base_source",
    "floatingBaseEstimator": "floating_base_estimator",
    "mpcFreq": "mpc_freq",
    "mrtFreq": "mrt_freq",
    "controllersFile": "controllers_file",
    "refBaseHeight": "ref_base_height",
    "stairClimbingFile": "stair_climbing_file",
    "terrainWalkingFile": "terrain_walking_file",
    "ros2ControlCommandInterface": "ros2_control_command_interface",
    "mujocoEffortCommandMode": "mujoco_effort_command_mode",
    "enableLidar": "enable_lidar",
    "initialPoseFile": "initial_pose_file",
    "spawnMpcController": "spawn_mpc_controller",
    "baseCommandDevice": "base_command_device",
    "mujocoModelFile": "mujoco_model_file",
}


def reject_renamed_arguments(context):
    used = [name for name in RENAMED_ARGUMENTS if name in context.launch_configurations]
    if used:
        raise ValueError("Renamed launch arguments (snake_case now): " +
                         ", ".join(f"{name}:= -> {RENAMED_ARGUMENTS[name]}:=" for name in used))
    return []


def generate_launch_description():
    mpc_share_dir = get_package_share_directory("legged_robot_mpc_controller")
    robot_name = LaunchConfiguration("robot_name")

    rviz_default = os.path.join(mpc_share_dir, "config", "rviz", "humanoid.rviz")
    initial_pose_default = PathJoinSubstitution([mpc_share_dir, "config", robot_name, "initial_pose.yaml"])
    # Never share generated libraries between models with different dimensions.
    lib_folder_default = PathJoinSubstitution(["auto_generated", robot_name])
    whole_body_controller = "humanoid_whole_body_inverse_dynamics_mpc_controller"

    # Official model filenames differ; all other robot-specific paths follow robot_name.
    urdf_name = PythonExpression([
        "{'g1': 'g1_29dof.urdf', 'h2': 'h2_31dof.urdf', 'upkie': 'upkie.urdf', 'pineapple': 'pineapple.urdf'}['", robot_name, "']"
    ])
    urdf_default = PathJoinSubstitution([mpc_share_dir, "description", "urdf", "robots", robot_name, urdf_name])

    # RViz environment, rooted at world and separate from the MuJoCo scene: comment or uncomment
    # the terrain/object includes in env.urdf.xacro to match the scene.
    environment_file = PathJoinSubstitution([mpc_share_dir, "description", "urdf", "env.urdf.xacro"])

    controllers_file_default = PathJoinSubstitution([
        mpc_share_dir, "config", robot_name,
        "ros2_controllers.yaml"
        ])
    
    stair_climbing_file_default = PathJoinSubstitution([
        mpc_share_dir, "config", "terrains", "stair_climbing",
        [robot_name, ".yaml"]
        ])

    terrain_walking_file_default = PathJoinSubstitution([
        mpc_share_dir, "config", "terrains", "terrain_walking",
        [robot_name, ".yaml"]
        ])

    ros2_control_xacro = PathJoinSubstitution([
        mpc_share_dir, "description", "urdf", "robots", robot_name, [robot_name, ".ros2_control.xacro"]
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="g1",
            choices=["g1", "h2", "upkie", "pineapple"],
            description="Robot model and configuration: Unitree G1, H2, the Upkie wheeled-biped, or the Pineapple "
                        "wheel-legged robot.",),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_default),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_mujoco_sim", default_value="true"),
        DeclareLaunchArgument("mujoco_headless", default_value="false"),
        DeclareLaunchArgument("mujoco_wait_to_start", default_value="true"),
        DeclareLaunchArgument("mujoco_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("mujoco_publish_rate", default_value="100.0"),
        # Plant-side modeling error for robustness studies (config/robustness, tests/robustness_sweep.py): a YAML
        # model_error map the simulator applies to the compiled model in memory; the controller keeps its own model.
        DeclareLaunchArgument("model_error_file", default_value="",
                              description="Model-error YAML: a name under config/model_error/ (e.g. template.yaml) "
                                          "or an absolute path; empty: nominal plant"),
        DeclareLaunchArgument("model_error_seed", default_value="-1", description="Overrides the file's seed when >= 0"),
        DeclareLaunchArgument("model_error_report", default_value="", description="JSON file of the realized errors"),
        DeclareLaunchArgument("gt_enabled", default_value="true"),
        # The MuJoCo free-joint body. Its ground-truth TF is world -> gt/pelvis while an
        # estimator owns odom -> pelvis, and world -> pelvis only with floating_base_estimator:=none
        # (pelvis can have one TF parent).
        DeclareLaunchArgument(
            "gt_body_frame",
            default_value=PythonExpression(["{'g1': 'pelvis', 'h2': 'pelvis', 'upkie': 'base', 'pineapple': 'base_link'}['", robot_name, "']"]),
        ),
        DeclareLaunchArgument("urdf_file", default_value=urdf_default),
        DeclareLaunchArgument(
            "mpc_controller_name",
            default_value=whole_body_controller,
            choices=[whole_body_controller],
            description="MPC ros2 controller to load and activate: "
                        "whole-body inverse-dynamics MPC (x = [q, v], u = [W, a, tau]).",),
        DeclareLaunchArgument(
            "lib_folder",
            default_value=lib_folder_default,
            description="Writable folder for generated or cached CppAD libraries, relative to the "
                        "launch directory; each MPC uses its own subfolder.",
        ),
        DeclareLaunchArgument(
            "floating_base_source", default_value="state_estimator",
            choices=["ground_truth_state", "state_estimator"],
            description="Floating-base feedback source for the MPC: "
                        "ground_truth_state (simulator/hardware body state) | state_estimator (the "
                        "estimator named by floating_base_estimator)."),
        DeclareLaunchArgument(
            "floating_base_estimator", default_value="humanoid_linear_kf",
            choices=["none", "humanoid_inekf", "humanoid_linear_kf"],
            description="Floating-base estimator the MPC controller runs, first in every "
                        "control tick. With floating_base_source:=state_estimator it drives "
                        "control; with ground_truth_state it runs alongside and is logged "
                        "against ground truth. none runs no estimator (ground_truth_state only)."),
        DeclareLaunchArgument("mpc_freq", default_value="100", description="MPC update frequency (should be integer)"),
        DeclareLaunchArgument("mrt_freq", default_value="1000", description="MRT update frequency (should be integer)"),
        DeclareLaunchArgument("controllers_file", default_value=controllers_file_default),
        DeclareLaunchArgument(
            "ref_base_height",
            default_value="",
            description="Optional nominal pelvis height override [m] for MPC and its command frontend, inside "
                        "ocs2.reference.minBaseHeight/maxBaseHeight. "
                        "Empty uses ocs2.reference.defaultBaseHeight from the selected controller YAML.",
        ),
        DeclareLaunchArgument("stair_climbing_file", default_value=stair_climbing_file_default),
        DeclareLaunchArgument("terrain_walking_file", default_value=terrain_walking_file_default),
        DeclareLaunchArgument("ros2_control_command_interface", default_value="effort_pd"),
        DeclareLaunchArgument(
            "mujoco_effort_command_mode",
            default_value="actuator",
            description="actuator: PD torque through MuJoCo motors (force limits apply); "
                        "qfrc_applied: unclamped direct generalized-force bypass.",
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="false",
            description="Simulate the Livox Mid-360 (PointCloud2 on /livox/lidar). "
                        "Needs mujoco_ros2_control built with lidar support.",
        ),
        DeclareLaunchArgument("initial_pose_file", default_value=initial_pose_default),
        DeclareLaunchArgument(
            "spawn_mpc_controller",
            default_value="true",
            description="Load, configure, and activate the MPC controller before MuJoCo physics starts.",
        ),
        DeclareLaunchArgument(
            "base_command_device",
            default_value="gui",
            choices=["gui", "joystick", "none"],
            description="Base velocity/height command source: GUI, DualSense joystick, "
                        "or none for external/scripted commands. Only one source is launched.",
        ),
        DeclareLaunchArgument(
            "mujoco_model_file",
            default_value="scene.xml",
            description="MuJoCo scene filename under description/mujoco/robots/<robot_name>/, "
                        "or an absolute path to any custom scene XML.",),
    ]

    use_mujoco_sim = LaunchConfiguration("use_mujoco_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim_time = ParameterValue(use_mujoco_sim, value_type=bool)
    mujoco_model_path = PathJoinSubstitution(
        [mpc_share_dir, "description", "mujoco", "robots", robot_name, LaunchConfiguration("mujoco_model_file")]
    )

    # The robot description: the selected URDF plus its <ros2_control> block. One description
    # for the controller manager and robot_state_publisher.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            ros2_control_xacro,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "use_mujoco_sim:=",
            use_mujoco_sim,
            " ",
            "initial_pose_file:=",
            LaunchConfiguration("initial_pose_file"),
            " ",
            "ros2_control_command_interface:=",
            LaunchConfiguration("ros2_control_command_interface"),
            " ",
            "mujoco_effort_command_mode:=",
            LaunchConfiguration("mujoco_effort_command_mode"),
            " ",
            "use_lidar:=",
            LaunchConfiguration("enable_lidar"),
            " ",
            "robot_urdf_file:=",
            LaunchConfiguration("urdf_file"),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    environment_description = {
        "robot_description": ParameterValue(
            Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", environment_file]),
            value_type=str,
        )
    }
    # Controller sequenced by controller_sequence.py: "none" starts MuJoCo with
    # joint_state_broadcaster only, which is only useful for environment smoke tests.
    sequence_controller_name = PythonExpression(
        [
            '"',
            LaunchConfiguration("mpc_controller_name"),
            '" if "',
            LaunchConfiguration("spawn_mpc_controller"),
            '" == "true" else "none"',
        ]
    )


    # The controller manager does not pass its own params file on to the controllers it
    # loads; each one reads only <name>.params_file. The yaml carries $(var ...)
    # substitutions, so it is the substituted copy written by launch that is passed.
    # The ParameterFile is kept alive: it deletes the substituted copy when collected.
    controllers_params = ParameterFile(LaunchConfiguration("controllers_file"), allow_substs=True)

    def resolve_controllers_params_file(context):
        path = controllers_params.evaluate(context)
        with open(path) as stream:
            config = yaml.safe_load(stream)
        controller = LaunchConfiguration("mpc_controller_name").perform(context)
        if controller not in config:
            raise ValueError(f"Controller '{controller}' is not configured in {LaunchConfiguration('controllers_file').perform(context)}")
        if robot_name.perform(context) == "h2" and LaunchConfiguration("enable_lidar").perform(context).lower() == "true":
            raise ValueError("The official H2 model does not include the G1 Mid-360 lidar.")
        reference = config[controller]["ros__parameters"]["ocs2"]["reference"]
        target = config[controller]["ros__parameters"].get("target", {})
        height_override = LaunchConfiguration("ref_base_height").perform(context)
        height = float(height_override or reference["defaultBaseHeight"])
        if not math.isfinite(height) or height <= 0.0:
            raise ValueError("Reference base height must be finite and positive.")
        minimum_height, maximum_height = float(reference["minBaseHeight"]), float(reference["maxBaseHeight"])
        if not minimum_height <= height <= maximum_height:
            raise ValueError(f"Reference base height {height} is outside ocs2.reference.minBaseHeight/maxBaseHeight "
                             f"[{minimum_height}, {maximum_height}].")
        if height_override:
            reference["defaultBaseHeight"] = height
            # allow_substs=True always creates a temporary copy; never rewrite the source YAML.
            with open(path, "w") as stream:
                yaml.safe_dump(config, stream, sort_keys=False)
        return [
            SetLaunchConfiguration("controllers_params_file", str(path)),
            SetLaunchConfiguration("gui_ref_base_height", str(height)),
            SetLaunchConfiguration("gui_min_base_height", str(minimum_height)),
            SetLaunchConfiguration("gui_max_base_height", str(maximum_height)),
            SetLaunchConfiguration("gui_max_vx", str(reference["maxDisplacementVelocityX"])),
            SetLaunchConfiguration("gui_max_vy", str(reference["maxDisplacementVelocityY"])),
            SetLaunchConfiguration("gui_max_yaw_rate", str(reference["maxRotationVelocity"])),
            SetLaunchConfiguration("posture_controller", "/" + controller if
                                   config[controller]["ros__parameters"].get("bringUp", {}).get("standUpTrajectoryFile") else ""),
            SetLaunchConfiguration("command_frame", str(target.get("commandFrame", ""))),
            SetLaunchConfiguration("command_frame_yaw", str(float(target.get("commandFrameYaw", 0.0)))),
        ]

    mpc_params_file = {
        (LaunchConfiguration("mpc_controller_name"), ".params_file"): LaunchConfiguration("controllers_params_file"),
        (LaunchConfiguration("floating_base_estimator"), ".params_file"): LaunchConfiguration("controllers_params_file"),
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            LaunchConfiguration("controllers_params_file"),
            mpc_params_file,
            {"use_sim_time": use_sim_time},
        ],
        condition=UnlessCondition(use_mujoco_sim),
    )

    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            LaunchConfiguration("controllers_params_file"),
            mpc_params_file,
            {"mujoco_model_path": mujoco_model_path},
            {"mujoco_headless": ParameterValue(LaunchConfiguration("mujoco_headless"), value_type=bool)},
            {
                "mujoco_wait_to_start": ParameterValue(
                    LaunchConfiguration("mujoco_wait_to_start"), value_type=bool
                )
            },
            {
                "mujoco_real_time_factor": ParameterValue(
                    LaunchConfiguration("mujoco_real_time_factor"), value_type=float
                )
            },
            {
                "mujoco_publish_rate": ParameterValue(
                    LaunchConfiguration("mujoco_publish_rate"), value_type=float
                )
            },
            {"model_error_file": ParameterValue(LaunchConfiguration("model_error_path"), value_type=str)},
            {"model_error_seed": ParameterValue(LaunchConfiguration("model_error_seed"), value_type=int)},
            {"model_error_report": ParameterValue(LaunchConfiguration("model_error_report"), value_type=str)},
            {"gt_enabled": ParameterValue(LaunchConfiguration("gt_enabled"), value_type=bool)},
            {"gt_publish_tf": True},
            {"gt_pub_hz": 100.0},
            {"gt_odom_topic": "/mujoco/ground_truth/odom"},
            {"gt_root_frame": "world"},
            {"gt_frame_prefix": PythonExpression(
                ['"" if "', LaunchConfiguration("floating_base_estimator"), '" == "none" else "gt/"']
            )},
            {
                # Build a YAML list string and parse it as string_array; a bare
                # [LaunchConfiguration] would collapse to a single string.
                "gt_body_frames": ParameterValue(
                    ["['", LaunchConfiguration("gt_body_frame"), "']"],
                    value_type=List[str],
                )
            },
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_mujoco_sim),
    )

    controller_sequence_script = PathJoinSubstitution(
        [FindPackageShare("legged_robot_mpc_controller"), "launch", "controller_sequence.py"]
    )
    controller_sequence = ExecuteProcess(
        cmd=[
            "python3",
            controller_sequence_script,
            "--controller-manager",
            "/controller_manager",
            "--robot-description-topic",
            "/robot_description",
            "--controller-name",
            sequence_controller_name,
            "--estimator-name",
            LaunchConfiguration("floating_base_estimator"),
            "--timeout",
            "120",
        ],
        output="screen",
        condition=UnlessCondition(use_mujoco_sim),
    )
    controller_sequence_mujoco = ExecuteProcess(
        cmd=[
            "python3",
            controller_sequence_script,
            "--controller-manager",
            "/controller_manager",
            "--robot-description-topic",
            "/robot_description",
            "--controller-name",
            sequence_controller_name,
            "--estimator-name",
            LaunchConfiguration("floating_base_estimator"),
            "--timeout",
            "120",
            "--use-mujoco-sim",
            "--use-sim-time",
        ],
        output="screen",
        condition=IfCondition(use_mujoco_sim),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # TF tree: world -> odom (static, identity until something localizes the robot)
    # -> pelvis (the estimator) -> links (robot_state_publisher); world -> stairs (the
    # environment); world -> gt/pelvis (simulator ground truth, for comparison).
    world_to_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom",
        output="screen",
        arguments=["--frame-id", "world", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    environment_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="environment",
        output="screen",
        parameters=[environment_description, {"use_sim_time": use_sim_time}],
    )

    base_command_gui_node = Node(
        package="legged_robot_mpc_controller",
        executable="base_command_gui.py",
        name="base_command_gui",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("base_command_device"), "' == 'gui'",
        ])),
        parameters=[
            {
                "max_linear_velocity_x": ParameterValue(LaunchConfiguration("gui_max_vx"), value_type=float),
                "max_linear_velocity_y": ParameterValue(LaunchConfiguration("gui_max_vy"), value_type=float),
                "max_yaw_rate": ParameterValue(LaunchConfiguration("gui_max_yaw_rate"), value_type=float),
                "reference_base_height": ParameterValue(
                    LaunchConfiguration("gui_ref_base_height"), value_type=float
                ),
                "minimum_base_height": ParameterValue(
                    LaunchConfiguration("gui_min_base_height"), value_type=float
                ),
                "maximum_base_height": ParameterValue(
                    LaunchConfiguration("gui_max_base_height"), value_type=float
                ),
                "posture_controller": ParameterValue(LaunchConfiguration("posture_controller"), value_type=str),
            }
        ],
    )

    base_command_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("legged_robot_teleop"), "launch", "teleop.launch.py",
        ])),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("base_command_device"), "' == 'joystick'",
        ])),
        launch_arguments={
            "robot_name": robot_name,
            "controllers_file": LaunchConfiguration("controllers_params_file"),
            "mpc_controller_name": LaunchConfiguration("mpc_controller_name"),
            "ref_base_height": LaunchConfiguration("gui_ref_base_height"),
            "use_sim_time": use_mujoco_sim,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", LaunchConfiguration("rviz_resolved_config")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # The control frame commands are given in (target.commandFrame, turned by target.commandFrameYaw about the base z
    # axis), as a static child of the base body, so RViz and other tools see the robot's own forward axis.
    command_frame_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="command_frame_tf",
        output="screen",
        arguments=["--frame-id", LaunchConfiguration("gt_body_frame"), "--child-frame-id", LaunchConfiguration("command_frame"),
                   "--yaw", LaunchConfiguration("command_frame_yaw")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration("command_frame"), '" != ""'])),
    )

    def resolve_rviz_config(context):
        # The Orbit views in the RViz config target 'pelvis' (G1's base); follow this robot's base body instead.
        # The rewritten copy lives in /tmp, so RViz's "Save Config" writes there: edit the source to keep a change.
        source = LaunchConfiguration("rviz_config").perform(context)
        base = LaunchConfiguration("gt_body_frame").perform(context)
        with open(source) as file:
            text = file.read()
        if base == "pelvis" or "Target Frame: pelvis" not in text:
            return [SetLaunchConfiguration("rviz_resolved_config", source)]
        name = os.path.splitext(os.path.basename(source))[0]
        path = os.path.join(tempfile.gettempdir(), f"{name}_{LaunchConfiguration('robot_name').perform(context)}_{os.getpid()}.rviz")
        with open(path, "w") as file:
            file.write(text.replace("Target Frame: pelvis", f"Target Frame: {base}"))
        return [SetLaunchConfiguration("rviz_resolved_config", path)]

    def resolve_model_error_file(context):
        # A name is a file under config/model_error/ (installed: a new file there needs a rebuild), else a path from
        # the working directory; empty is the nominal plant.
        name = LaunchConfiguration("model_error_file").perform(context)
        folder = os.path.join(mpc_share_dir, "config", "model_error")
        path = name
        if name and not os.path.isabs(name):
            path = os.path.join(folder, name) if os.path.isfile(os.path.join(folder, name)) else os.path.abspath(name)
        if path and not os.path.isfile(path):
            raise ValueError(f"Model-error file '{name}' is neither in {folder} (a new file there needs a rebuild) "
                             "nor a path to a file.")
        return [SetLaunchConfiguration("model_error_path", path)]

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=reject_renamed_arguments),
            OpaqueFunction(function=resolve_controllers_params_file),
            OpaqueFunction(function=resolve_model_error_file),
            OpaqueFunction(function=resolve_rviz_config),
            ros2_control_node,
            mujoco_ros2_control_node,
            controller_sequence,
            controller_sequence_mujoco,
            robot_state_publisher_node,
            world_to_odom_node,
            command_frame_node,
            environment_state_publisher_node,
            base_command_gui_node,
            base_command_teleop,
            rviz_node,
        ]
    )
