#ifndef LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/walking_velocity_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>

namespace legged_robot_mpc_controller
{

/// ROS 2 front-end for the procedural motion manager. Twist and pose callbacks
/// update independent stored targets; a separate mode command explicitly selects
/// which target drives the reference and gait scheduler.
class Ros2ProceduralMpcMotionManager final : public ocs2::humanoid::ProceduralMpcMotionManager
{
public:
  Ros2ProceduralMpcMotionManager(
    ocs2::humanoid::GaitMap gait_map,
    const ocs2::humanoid::ReferenceConfig& reference_config,
    std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> reference_manager,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpc_robot_model,
    VelocityTargetToTargetTrajectories velocity_target_to_target_trajectories,
    BasePoseTargetToTargetTrajectories base_pose_target_to_target_trajectories,
    const std::string& default_target_mode);

  void subscribe(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const rclcpp::QoS& qos,
    const std::string& walking_velocity_topic,
    const std::string& base_pose_topic,
    const std::string& target_mode_topic,
    const std::string& global_frame);

private:
  rclcpp::Subscription<ocs2_msgs::msg::WalkingVelocityCommand>::SharedPtr velocity_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_mode_subscription_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_
