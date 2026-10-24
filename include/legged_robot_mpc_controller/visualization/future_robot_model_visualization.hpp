#ifndef LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__FUTURE_ROBOT_MODEL_VISUALIZATION_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__FUTURE_ROBOT_MODEL_VISUALIZATION_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <humanoid_common_mpc/common/MpcRobotModelBase.h>

namespace legged_robot_mpc_controller::visualization
{

class FutureRobotModelVisualization
{
public:
  struct Settings
  {
    std::string frame_id{"world"};
    double interval{0.2};
  };

  FutureRobotModelVisualization(
    ocs2::PinocchioInterface pinocchioInterface,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
    rclcpp_lifecycle::LifecycleNode& node,
    Settings settings);

  bool hasSubscribers() const;
  void publish(
    const ocs2::scalar_array_t& times,
    const ocs2::vector_array_t& states,
    const ocs2::TargetTrajectories& targets);

private:
  using Message = visualization_msgs::msg::MarkerArray;
  using Publisher = rclcpp::Publisher<Message>;
  struct GhostVisual
  {
    std::size_t frame_id;
    Eigen::Isometry3d link_to_visual;
    visualization_msgs::msg::Marker marker;
  };

  void initializeVisuals();
  Message emptyMessage(const rclcpp::Time& stamp) const;
  void appendStateMarkers(
    Message& message, const rclcpp::Time& stamp, const ocs2::vector_t& state,
    const std::string& markerNamespace, const std::array<float, 3>& color, double alpha);

  ocs2::PinocchioInterface pinocchio_interface_;
  std::unique_ptr<ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>> mpc_robot_model_;
  rclcpp_lifecycle::LifecycleNode& node_;
  Settings settings_;
  std::vector<GhostVisual> visuals_;
  Publisher::SharedPtr future_publisher_;
  Publisher::SharedPtr terminal_state_publisher_;
  Publisher::SharedPtr terminal_target_publisher_;
};

}  // namespace legged_robot_mpc_controller::visualization

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__FUTURE_ROBOT_MODEL_VISUALIZATION_HPP_
