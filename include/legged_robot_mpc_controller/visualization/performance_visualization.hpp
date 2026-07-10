#ifndef LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_box_best_effort.hpp>

#include <humanoid_common_mpc/common/MpcRobotModelBase.h>

#include "legged_robot_mpc_controller/visualization/optimized_state_trajectory_visualization.hpp"

namespace legged_robot_mpc_controller::visualization
{

class PerformanceVisualization
{
public:
  struct Settings
  {
    bool trajectory_active{true};
    double update_rate{5.0};
    OptimizedStateTrajectoryVisualization::Settings trajectory;
  };

  PerformanceVisualization(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const ocs2::PinocchioInterface& pinocchioInterface,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
    Settings settings);

  ~PerformanceVisualization();

  void update_visualization(const ocs2::vector_array_t& optimizedStateTrajectory);

private:
  void visualization_loop();

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  Settings settings_;
  std::unique_ptr<OptimizedStateTrajectoryVisualization> trajectory_visualization_;
  realtime_tools::RealtimeBoxBestEffort<ocs2::vector_array_t> latest_optimized_state_trajectory_;
  std::thread visualization_thread_;
  std::chrono::duration<double> period_{0.2};
  std::atomic_bool running_{false};
  std::atomic_bool has_optimized_state_trajectory_{false};
};

template <typename Params>
PerformanceVisualization::Settings makePerformanceVisualizationSettings(const Params& parameters)
{
  const auto& visualization = parameters.visualization;

  PerformanceVisualization::Settings settings;
  settings.trajectory_active = visualization.trajectory.activate;
  settings.update_rate = visualization.update_rate;

  settings.trajectory.frame_id = visualization.frameId;
  settings.trajectory.marker_topic = visualization.trajectory.markerTopic;
  settings.trajectory.frame_names = visualization.trajectory.frame_names;
  settings.trajectory.line_width = visualization.trajectory.lineWidth;
  settings.trajectory.point_scale = visualization.trajectory.pointScale;

  return settings;
}

}  // namespace legged_robot_mpc_controller::visualization

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_
