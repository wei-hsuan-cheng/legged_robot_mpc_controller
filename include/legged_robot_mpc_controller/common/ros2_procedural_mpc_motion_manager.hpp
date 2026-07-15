#ifndef LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_

#include <memory>

#include <ocs2_msgs/msg/walking_velocity_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>

namespace legged_robot_mpc_controller
{

/// ROS 2 front-end for the procedural motion manager: subscribes to the walking
/// velocity command topic and forwards the bounded command to the base manager,
/// which owns command conditioning (scale/filter) and gait scheduling.
class Ros2ProceduralMpcMotionManager final : public ocs2::humanoid::ProceduralMpcMotionManager
{
public:
  Ros2ProceduralMpcMotionManager(
    ocs2::humanoid::GaitMap gait_map,
    const ocs2::humanoid::ReferenceConfig& reference_config,
    std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> reference_manager,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpc_robot_model,
    VelocityTargetToTargetTrajectories velocity_target_to_target_trajectories);

  void subscribe(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const rclcpp::QoS& qos);

private:
  rclcpp::Subscription<ocs2_msgs::msg::WalkingVelocityCommand>::SharedPtr velocity_command_subscription_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_
