#ifndef LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__OPTIMIZED_STATE_TRAJECTORY_VISUALIZATION_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__OPTIMIZED_STATE_TRAJECTORY_VISUALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <humanoid_common_mpc/common/MpcRobotModelBase.h>

namespace legged_robot_mpc_controller::visualization
{

class OptimizedStateTrajectoryVisualization
{
public:
  struct Settings
  {
    std::string marker_topic{"/performance_visualization/optimizedStateTrajectory"};
    std::string frame_id{"world"};
    std::vector<std::string> frame_names;
    double line_width{0.01};
    double point_scale{0.025};
  };

  OptimizedStateTrajectoryVisualization(
    ocs2::PinocchioInterface pinocchioInterface,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
    rclcpp_lifecycle::LifecycleNode& node,
    Settings settings);

  void publish(const ocs2::vector_array_t& stateTrajectory);

private:
  using Message = visualization_msgs::msg::MarkerArray;

  ocs2::vector_array_t extractGeneralizedCoordinateTrajectory(
    const ocs2::vector_array_t& stateTrajectory) const;
  Message createMessage(const ocs2::vector_array_t& generalizedCoordinateTrajectory);

  ocs2::PinocchioInterface pinocchio_interface_;
  std::unique_ptr<ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>> mpc_robot_model_;
  std::vector<std::string> frame_names_;
  std::vector<pinocchio::FrameIndex> frame_ids_;
  rclcpp_lifecycle::LifecycleNode& node_;
  Settings settings_;
  rclcpp::Publisher<Message>::SharedPtr marker_publisher_;
};

}  // namespace legged_robot_mpc_controller::visualization

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__OPTIMIZED_STATE_TRAJECTORY_VISUALIZATION_HPP_
