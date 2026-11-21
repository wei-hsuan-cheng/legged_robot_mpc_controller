#ifndef LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_targets.hpp>
#include <ocs2_msgs/msg/walking_velocity_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <legged_robot_mpc_controller/mpc/common/reference_manager/ProceduralMpcMotionManager.h>

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
    GaitTransitionConfig gait_transition_config,
    const ocs2::humanoid::ReferenceConfig& reference_config,
    std::shared_ptr<ocs2::humanoid::SwitchedModelReferenceManager> reference_manager,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpc_robot_model,
    VelocityTargetToTargetTrajectories velocity_target_to_target_trajectories,
    BasePoseTargetToTargetTrajectories base_pose_target_to_target_trajectories,
    const std::string& default_target_mode);

  /// Rotation [rad] about the base z axis of the frame commands are given in: twist commands are in that frame
  /// and base pose commands give that frame's orientation. 0 (default): the base frame itself.
  void setCommandFrameYaw(double yaw) { command_frame_yaw_ = yaw; }

  // Outside locomotion, discard operator targets and hold a zero-twist reference.
  void setOperatorMotionEnabled(bool enabled) { operator_motion_enabled_.store(enabled); }
  void setPostureHeightOverride(double height) { walkingVelocityTarget_.setPostureHeightOverride(height); }
  void preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override;

  void subscribe(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const rclcpp::QoS& qos,
    const std::string& walking_velocity_topic,
    const std::string& base_pose_topic,
    const std::string& target_mode_topic,
    const std::string& global_frame,
    const std::string& gait_topic = "");

  /// Subscribes to joint targets and default-posture reset commands.
  void subscribeMpcTargets(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& mpc_targets_topic,
    std::vector<std::string> tracked_arm_joint_names);

private:
  rclcpp::Subscription<ocs2_msgs::msg::WalkingVelocityCommand>::SharedPtr velocity_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_subscription_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargets>::SharedPtr mpc_targets_subscription_;
  std::vector<std::string> tracked_arm_joint_names_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double command_frame_yaw_{0.0};
  std::atomic_bool operator_motion_enabled_{true};
  double hold_height_{0.0};
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__ROS2_PROCEDURAL_MPC_MOTION_MANAGER_HPP_
