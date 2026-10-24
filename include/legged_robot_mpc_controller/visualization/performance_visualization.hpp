#ifndef LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

#include <humanoid_common_mpc/common/MpcRobotModelBase.h>

#include "legged_robot_mpc_controller/visualization/optimized_state_trajectory_visualization.hpp"
#include "legged_robot_mpc_controller/visualization/future_robot_model_visualization.hpp"

namespace legged_robot_mpc_controller::visualization
{

class PerformanceVisualization
{
public:
  struct Settings
  {
    bool trajectory_active{true};
    bool robot_models_active{true};
    double update_rate{5.0};
    OptimizedStateTrajectoryVisualization::Settings trajectory;
    FutureRobotModelVisualization::Settings robot_models;
  };

  PerformanceVisualization(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const ocs2::PinocchioInterface& pinocchioInterface,
    const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
    Settings settings);

  ~PerformanceVisualization();

  void update_visualization(
    const ocs2::scalar_array_t& times, const ocs2::vector_array_t& states,
    const ocs2::TargetTrajectories& targets);

private:
  void visualization_loop();

  struct Snapshot
  {
    ocs2::scalar_array_t times;
    ocs2::vector_array_t states;
    ocs2::TargetTrajectories targets{0};
  };

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  Settings settings_;
  std::unique_ptr<OptimizedStateTrajectoryVisualization> trajectory_visualization_;
  std::unique_ptr<FutureRobotModelVisualization> robot_model_visualization_;
  realtime_tools::RealtimeThreadSafeBox<Snapshot> latest_snapshot_;
  std::thread visualization_thread_;
  std::chrono::duration<double> period_{0.2};
  std::atomic_bool running_{false};
  std::atomic_bool has_optimized_state_trajectory_{false};
  std::atomic_bool has_subscribers_{false};
};

template <typename Params>
PerformanceVisualization::Settings makePerformanceVisualizationSettings(const Params& parameters)
{
  const auto& visualization = parameters.visualization;

  PerformanceVisualization::Settings settings;
  settings.trajectory_active = visualization.trajectory.activate;
  settings.update_rate = visualization.update_rate;
  settings.robot_models_active = visualization.optimizedStateRobotModelsActivate;
  settings.robot_models.frame_id = visualization.frameId;
  settings.robot_models.interval = visualization.optimizedStateRobotModelsInterval;

  settings.trajectory.frame_id = visualization.frameId;
  settings.trajectory.marker_topic = visualization.trajectory.markerTopic;
  settings.trajectory.frame_names = visualization.trajectory.frame_names;
  settings.trajectory.line_width = visualization.trajectory.lineWidth;
  settings.trajectory.point_scale = visualization.trajectory.pointScale;

  return settings;
}

}  // namespace legged_robot_mpc_controller::visualization

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_
