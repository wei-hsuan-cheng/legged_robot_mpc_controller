import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
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


def generate_launch_description():
    mpc_share = FindPackageShare("legged_robot_mpc_controller")
    mpc_share_dir = get_package_share_directory("legged_robot_mpc_controller")

    rviz_default = os.path.join(mpc_share_dir, "config", "rviz", "humanoid.rviz")
    initial_pose_default = os.path.join(mpc_share_dir, "config", "g1", "initial_pose.yaml")
    lib_folder_default = os.path.join("auto_generated", "g1")
    mpc_controller_default = "humanoid_centroidal_mpc_controller" # humanoid_centroidal_mpc_controller | humanoid_wb_mpc_controller

    urdf_default = PathJoinSubstitution([mpc_share, 
                                         "description", 
                                         "g1", 
                                         "urdf", 
                                         "g1_29dof.urdf", # g1_29dof.urdf | g1_29dof_stairs.urdf
                                         ])
    
    controllers_file_default = PathJoinSubstitution([
        mpc_share,
        "config",
        "g1",
        "ros2_controllers_legacy.yaml" # ros2_controllers.yaml | ros2_controllers_legacy.yaml
        ])
    
    gait_library_file_default = PathJoinSubstitution([
        mpc_share,
        "config",
        "g1",
        "gait.yaml"
        ])
    
    ros2_control_xacro = PathJoinSubstitution([mpc_share, "description", "g1", "urdf", "g1.ros2_control.xacro"])

    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rvizconfig", default_value=rviz_default),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_mujoco_sim", default_value="true"),
        DeclareLaunchArgument("mujoco_headless", default_value="false"),
        DeclareLaunchArgument("mujoco_wait_to_start", default_value="true"),
        DeclareLaunchArgument("mujoco_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("mujoco_publish_rate", default_value="100.0"),
        DeclareLaunchArgument("gt_enabled", default_value="true"),
        # pelvis is the URDF root and the MuJoCo free-joint body: world->pelvis from ground
        # truth composes with robot_state_publisher's pelvis-rooted tree without TF conflicts
        # (any other body would get two TF parents).
        DeclareLaunchArgument("gt_body_frame", default_value="pelvis"),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default),
        DeclareLaunchArgument(
            "libFolder",
            default_value=lib_folder_default,
            description="Writable folder for generated or cached CppAD libraries",
        ),
        DeclareLaunchArgument("mpcFreq", default_value="100", description="MPC update frequency (should be integer) (100 for centroidal, 50 for whole-body)"),
        DeclareLaunchArgument("mrtFreq", default_value="1000", description="MRT update frequency (should be integer)"),
        DeclareLaunchArgument("controllersFile", default_value=controllers_file_default),
        DeclareLaunchArgument("gaitLibraryFile", default_value=gait_library_file_default),
        DeclareLaunchArgument(
            "mpcControllerName",
            default_value=mpc_controller_default,
            description="Name of the supported MPC ros2 controllers to load and activate.",),
        DeclareLaunchArgument("ros2ControlCommandInterface", default_value="effort_pd"),
        DeclareLaunchArgument(
            "mujocoEffortCommandMode",
            default_value="actuator",
            description="actuator: PD torque through MuJoCo motors (force limits apply); "
                        "qfrc_applied: unclamped direct generalized-force bypass.",
        ),
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default),
        DeclareLaunchArgument(
            "spawnMpcController",
            default_value="true",
            description="Load, configure, and activate the MPC controller before MuJoCo physics starts.",
        ),
        DeclareLaunchArgument(
            "velocityCommandGui",
            default_value="true",
            description="Launch the pelvis velocity/height command GUI.",
        ),
        DeclareLaunchArgument("mujocoModelFile", default_value="scene.xml"),
    ]

    use_mujoco_sim = LaunchConfiguration("use_mujoco_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim_time = ParameterValue(use_mujoco_sim, value_type=bool)
    mujoco_model_path = PathJoinSubstitution(
        [mpc_share, "description", "g1", "mujoco", LaunchConfiguration("mujocoModelFile")]
    )

    # ros2_control hardware description (minimal kinematic chain + G1System block).
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
            LaunchConfiguration("initialPoseFile"),
            " ",
            "ros2_control_command_interface:=",
            LaunchConfiguration("ros2ControlCommandInterface"),
            " ",
            "mujoco_effort_command_mode:=",
            LaunchConfiguration("mujocoEffortCommandMode"),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Full URDF (visuals + collisions) for robot_state_publisher / RViz.
    display_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("urdfFile"),
        ]
    )
    display_description = {
        "robot_description": ParameterValue(display_description_content, value_type=str)
    }

    # Controller sequenced by controller_sequence.py: "none" starts MuJoCo with
    # joint_state_broadcaster only, which is only useful for environment smoke tests.
    sequence_controller_name = PythonExpression(
        [
            '"',
            LaunchConfiguration("mpcControllerName"),
            '" if "',
            LaunchConfiguration("spawnMpcController"),
            '" == "true" else "none"',
        ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ParameterFile(LaunchConfiguration("controllersFile"), allow_substs=True),
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
            ParameterFile(LaunchConfiguration("controllersFile"), allow_substs=True),
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
            {"gt_enabled": ParameterValue(LaunchConfiguration("gt_enabled"), value_type=bool)},
            {"gt_publish_tf": True},
            {"gt_pub_hz": 100.0},
            {"gt_odom_topic": "/mujoco/ground_truth/odom"},
            {"gt_root_frame": "world"},
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
        parameters=[display_description, {"use_sim_time": use_sim_time}],
    )

    velocity_command_gui_node = Node(
        package="legged_robot_mpc_controller",
        executable="base_velocity_controller_gui.py",
        name="base_velocity_controller_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("velocityCommandGui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            mujoco_ros2_control_node,
            controller_sequence,
            controller_sequence_mujoco,
            robot_state_publisher_node,
            velocity_command_gui_node,
            rviz_node,
        ]
    )
