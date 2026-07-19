#include "legged_robot_mpc_controller/common/ros2_procedural_mpc_motion_manager.hpp"

#include "legged_robot_mpc_controller/common/target/mpc_targets_parser.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace legged_robot_mpc_controller
{
namespace
{

/// Convert the ROS command into the bounded internal command consumed by the MPC.
ocs2::humanoid::WalkingVelocityCommand boundedCommandFromMessage(
  const ocs2_msgs::msg::WalkingVelocityCommand& message)
{
  ocs2::humanoid::WalkingVelocityCommand command;
  command.linear_velocity_x = std::clamp(message.linear_velocity_x, -1.0, 1.0);
  command.linear_velocity_y = std::clamp(message.linear_velocity_y, -1.0, 1.0);
  command.desired_pelvis_height = std::clamp(message.desired_pelvis_height, 0.2, 1.0);
  command.angular_velocity_z = std::clamp(message.angular_velocity_z, -1.0, 1.0);
  return command;
}

ocs2::humanoid::BasePoseCommand basePoseCommandFromMessage(
  const geometry_msgs::msg::PoseStamped& message)
{
  const auto& position = message.pose.position;
  const auto& orientation = message.pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w)) {
    throw std::invalid_argument("base pose command contains non-finite values");
  }

  ocs2::quaternion_t quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  if (quaternion.norm() <= 1.0e-12) {
    throw std::invalid_argument("base pose command quaternion has zero norm");
  }

  ocs2::humanoid::BasePoseCommand command;
  command.position << position.x, position.y, position.z;
  command.orientation = quaternion.normalized();
  return command;
}

ocs2::humanoid::ProceduralMpcMotionManager::TargetMode targetModeFromString(
  const std::string& mode)
{
  using TargetMode = ocs2::humanoid::ProceduralMpcMotionManager::TargetMode;
  if (mode == "base_twist") {
    return TargetMode::BaseTwist;
  }
  if (mode == "base_pose") {
    return TargetMode::BasePose;
  }
  throw std::invalid_argument(
    "target mode must be 'base_twist' or 'base_pose', got '" + mode + "'");
}

}  // namespace

Ros2ProceduralMpcMotionManager::Ros2ProceduralMpcMotionManager(
  ocs2::humanoid::GaitMap gait_map,
  const ocs2::humanoid::ReferenceConfig& reference_config,
  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> reference_manager,
  const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpc_robot_model,
  VelocityTargetToTargetTrajectories velocity_target_to_target_trajectories,
  BasePoseTargetToTargetTrajectories base_pose_target_to_target_trajectories,
  const std::string& default_target_mode)
: ProceduralMpcMotionManager(
    std::move(gait_map),
    reference_config,
    std::move(reference_manager),
    mpc_robot_model,
    std::move(velocity_target_to_target_trajectories),
    std::move(base_pose_target_to_target_trajectories))
{
  // Initialize the stored twist target independently of the selected mode.
  setVelocityCommand(
    ocs2::humanoid::WalkingVelocityCommand(0.0, 0.0, reference_config.defaultBaseHeight, 0.0));
  setTargetMode(targetModeFromString(default_target_mode));
}

void Ros2ProceduralMpcMotionManager::subscribe(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const rclcpp::QoS& qos,
  const std::string& walking_velocity_topic,
  const std::string& base_pose_topic,
  const std::string& target_mode_topic,
  const std::string& global_frame)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  velocity_command_subscription_ = node->create_subscription<ocs2_msgs::msg::WalkingVelocityCommand>(
    walking_velocity_topic,
    qos,
    [this](const ocs2_msgs::msg::WalkingVelocityCommand::SharedPtr message) {
      setVelocityCommand(boundedCommandFromMessage(*message));
    });

  base_pose_command_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    base_pose_topic,
    qos,
    [this, node, global_frame](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      try {
        geometry_msgs::msg::PoseStamped global_pose = *message;
        if (global_pose.header.frame_id.empty()) {
          global_pose.header.frame_id = global_frame;
        } else if (global_pose.header.frame_id != global_frame) {
          global_pose = tf_buffer_->transform(
            global_pose, global_frame, tf2::durationFromSec(0.05));
        }
        setBasePoseCommand(basePoseCommandFromMessage(global_pose));
      } catch (const std::exception& error) {
        RCLCPP_WARN(
          node->get_logger(), "Rejected base pose command: %s", error.what());
      }
    });

  target_mode_subscription_ = node->create_subscription<std_msgs::msg::String>(
    target_mode_topic,
    rclcpp::QoS(1).reliable(),
    [this, node](const std_msgs::msg::String::SharedPtr message) {
      try {
        setTargetMode(targetModeFromString(message->data));
        RCLCPP_INFO(node->get_logger(), "Humanoid MPC target mode: %s", message->data.c_str());
      } catch (const std::exception& error) {
        RCLCPP_WARN(node->get_logger(), "Rejected target mode: %s", error.what());
      }
    });
}

void Ros2ProceduralMpcMotionManager::subscribeMpcTargets(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& mpc_targets_topic,
  std::vector<std::string> tracked_arm_joint_names,
  std::vector<std::string> declared_frame_relation_source_frames,
  std::vector<std::string> declared_frame_relation_target_frames)
{
  tracked_arm_joint_names_ = std::move(tracked_arm_joint_names);
  declared_frame_relation_source_frames_ = std::move(declared_frame_relation_source_frames);
  declared_frame_relation_target_frames_ = std::move(declared_frame_relation_target_frames);

  mpc_targets_subscription_ = node->create_subscription<ocs2_msgs::msg::MpcTargets>(
    mpc_targets_topic,
    rclcpp::QoS(1).reliable(),
    [this, node](const ocs2_msgs::msg::MpcTargets::SharedPtr message) {
      try {
        target::applyMpcTargets(
          *message, tracked_arm_joint_names_, declared_frame_relation_source_frames_,
          declared_frame_relation_target_frames_, *switchedModelReferenceManagerPtr_);
        // RCLCPP_INFO(
        //   node->get_logger(), "Humanoid MPC targets command: %s", message->command_type.c_str());
      } catch (const std::exception& error) {
        RCLCPP_WARN(node->get_logger(), "Rejected MPC targets command: %s", error.what());
      }
    });
}

}  // namespace legged_robot_mpc_controller
