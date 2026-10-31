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

  if (settings_.robot_models_active) {
    robot_model_visualization_ = std::make_unique<FutureRobotModelVisualization>(
      pinocchioInterface, mpcRobotModel, *node_, settings_.robot_models);
  }

  if (settings_.collision_spheres.activate && !settings_.collision_spheres.spheres.empty()) {
    collision_sphere_visualization_ = std::make_unique<CollisionSphereVisualization>(
      pinocchioInterface, mpcRobotModel, *node_, settings_.collision_spheres);
  }

  if (!trajectory_visualization_ && !robot_model_visualization_ && !collision_sphere_visualization_) {
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
  const ocs2::scalar_array_t& times, const ocs2::vector_array_t& states,
  const ocs2::TargetTrajectories& targets, const ocs2::vector_t& observedState)
{
  if (!has_subscribers_.load(std::memory_order_relaxed)) return;
  if (latest_snapshot_.try_set(Snapshot{times, states, targets, observedState})) {
    has_optimized_state_trajectory_.store(true);
  }
}

void PerformanceVisualization::visualization_loop()
{
  while (running_.load()) {
    std::this_thread::sleep_for(period_);

    try {
      const bool trajectory_subscribed = trajectory_visualization_ && trajectory_visualization_->hasSubscribers();
      const bool ghosts_subscribed = robot_model_visualization_ && robot_model_visualization_->hasSubscribers();
      const bool collision_subscribed = collision_sphere_visualization_ && collision_sphere_visualization_->hasSubscribers();
      const bool subscribed = trajectory_subscribed || ghosts_subscribed || collision_subscribed;
      has_subscribers_.store(subscribed, std::memory_order_relaxed);
      if (!subscribed || !has_optimized_state_trajectory_.exchange(false)) continue;
      const auto snapshot = latest_snapshot_.try_get();
      if (!snapshot.has_value()) {
        has_optimized_state_trajectory_.store(true);
        continue;
      }
      if (trajectory_subscribed) {
        trajectory_visualization_->publish(snapshot->states);
      }
      if (ghosts_subscribed) {
        robot_model_visualization_->publish(snapshot->times, snapshot->states, snapshot->targets);
      }
      if (collision_subscribed && snapshot->observed_state.size() > 0) {
        collision_sphere_visualization_->publish(snapshot->observed_state);
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
