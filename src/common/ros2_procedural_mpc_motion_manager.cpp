#include "legged_robot_mpc_controller/common/ros2_procedural_mpc_motion_manager.hpp"

#include "legged_robot_mpc_controller/common/target/walking_velocity_target.hpp"

#include <utility>

namespace legged_robot_mpc_controller
{

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
  // Keep the first generated target at the configured standing height until a command arrives.
  setAndScaleVelocityCommand(
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
      setAndScaleVelocityCommand(target::from_message(*message));
    });
}

void Ros2ProceduralMpcMotionManager::setAndScaleVelocityCommand(
  const ocs2::humanoid::WalkingVelocityCommand& command)
{
  std::lock_guard<std::mutex> lock(velocity_command_mutex_);
  ProceduralMpcMotionManager::setAndScaleVelocityCommand(command);
}

ocs2::humanoid::WalkingVelocityCommand
Ros2ProceduralMpcMotionManager::getScaledWalkingVelocityCommand()
{
  std::lock_guard<std::mutex> lock(velocity_command_mutex_);
  return ProceduralMpcMotionManager::getScaledWalkingVelocityCommand();
}

}  // namespace legged_robot_mpc_controller
