#include "legged_robot_mpc_controller/common/ros2_procedural_mpc_motion_manager.hpp"

#include <algorithm>
#include <utility>

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

}  // namespace

Ros2ProceduralMpcMotionManager::Ros2ProceduralMpcMotionManager(
  ocs2::humanoid::GaitMap gait_map,
  const ocs2::humanoid::ReferenceConfig& reference_config,
  std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> reference_manager,
  const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpc_robot_model,
  VelocityTargetToTargetTrajectories velocity_target_to_target_trajectories)
: ProceduralMpcMotionManager(
    std::move(gait_map),
    reference_config,
    std::move(reference_manager),
    mpc_robot_model,
    std::move(velocity_target_to_target_trajectories))
{
  // Hold the configured standing height until a command arrives.
  setVelocityCommand(
    ocs2::humanoid::WalkingVelocityCommand(0.0, 0.0, reference_config.defaultBaseHeight, 0.0));
}

void Ros2ProceduralMpcMotionManager::subscribe(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const rclcpp::QoS& qos)
{
  velocity_command_subscription_ = node->create_subscription<ocs2_msgs::msg::WalkingVelocityCommand>(
    "/humanoid/walking_velocity_command",
    qos,
    [this](const ocs2_msgs::msg::WalkingVelocityCommand::SharedPtr message) {
      setVelocityCommand(boundedCommandFromMessage(*message));
    });
}

}  // namespace legged_robot_mpc_controller
