#include "legged_robot_mpc_controller/visualization/performance_visualization.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

namespace legged_robot_mpc_controller::visualization
{

PerformanceVisualization::PerformanceVisualization(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const ocs2::PinocchioInterface& pinocchioInterface,
  const ocs2::humanoid::MpcRobotModelBase<ocs2::scalar_t>& mpcRobotModel,
  Settings settings)
: node_(std::move(node)),
  settings_(std::move(settings))
{
  if (!node_) {
    throw std::invalid_argument("PerformanceVisualization node must not be null.");
  }

  if (settings_.trajectory_active) {
    trajectory_visualization_ = std::make_unique<OptimizedStateTrajectoryVisualization>(
      pinocchioInterface,
      mpcRobotModel,
      *node_,
      settings_.trajectory);
    RCLCPP_INFO(
      node_->get_logger(),
      "[Performance Visualization] Enable optimized state trajectory visualization");
  }

  if (!trajectory_visualization_) {
    RCLCPP_INFO(node_->get_logger(), "[Performance Visualization] no visualization features active");
    return;
  }

  period_ = std::chrono::duration<double>(1.0 / std::max(1.0, settings_.update_rate));
  running_.store(true);
  has_optimized_state_trajectory_.store(false);
  visualization_thread_ = std::thread(&PerformanceVisualization::visualization_loop, this);

  RCLCPP_INFO(
    node_->get_logger(),
    "[Performance Visualization] started | frame=%s rate=%.3f Hz",
    settings_.trajectory.frame_id.c_str(),
    1.0 / period_.count());
}

PerformanceVisualization::~PerformanceVisualization()
{
  running_.store(false);
  if (visualization_thread_.joinable()) {
    visualization_thread_.join();
  }
}

void PerformanceVisualization::update_visualization(
  const ocs2::vector_array_t& optimizedStateTrajectory)
{
  if (latest_optimized_state_trajectory_.trySet(optimizedStateTrajectory)) {
    has_optimized_state_trajectory_.store(true);
  }
}

void PerformanceVisualization::visualization_loop()
{
  while (running_.load()) {
    std::this_thread::sleep_for(period_);

    try {
      if (trajectory_visualization_ && has_optimized_state_trajectory_.load()) {
        const auto optimized_state_trajectory = latest_optimized_state_trajectory_.tryGet();
        if (optimized_state_trajectory.has_value()) {
          trajectory_visualization_->publish(*optimized_state_trajectory);
        }
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[Performance Visualization] failed to publish: %s",
        error.what());
    }
  }
}

}  // namespace legged_robot_mpc_controller::visualization
