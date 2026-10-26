// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_STATE_ESTIMATOR_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_STATE_ESTIMATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimator_interface.hpp"

namespace legged_robot_mpc_controller
{

/// The part of an estimator's configuration that does not depend on which filter
/// runs. Each concrete estimator fills it from its OWN parameters.
struct EstimatorCommonSettings
{
  std::string urdf_file;
  std::vector<std::string> joint_names;
  std::string base_frame;
  std::string imu_interface_name;
  std::string imu_frame;
  std::vector<std::string> contact_frames;
  std::string odom_topic;
  std::string odom_frame;
  double publish_rate{100.0};
  //! "scheduled" (the gait's contact flags), "torque" or "sensor".
  std::string contact_source{"scheduled"};
  //! "sensor" only: one per-foot contact state interface, in foot-index order.
  std::vector<std::string> contact_sensor_interfaces;
  // Readiness. The timer is the primary gate; the std-devs are divergence guards
  // on the observable states only (see the readiness parameters for why).
  double ready_min_seconds{5.0};
  double ready_roll_pitch_std_max{0.10};
  double ready_height_std_max{0.10};
  double ready_velocity_std_max{0.50};
};

/**
 * A floating-base state estimator: the filter plus its parameters and readiness
 * test. HumanoidStateEstimatorController runs one as a ros2_control controller.
 *
 * Abstract: the concrete class decides WHICH filter runs, and declares only that
 * filter's parameters, at the root of its controller's node, from that
 * controller's block of config/g1/ros2_controllers.yaml.
 */
class HumanoidStateEstimator
{
public:
  virtual ~HumanoidStateEstimator();

  /// Declare and read this estimator's parameters, then build the filter.
  bool configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, double sampling_time);
  state_estimation::FloatingBaseEstimatorInterface & filter() { return *filter_; }
  const state_estimation::FloatingBaseEstimatorInterface & filter() const { return *filter_; }

  const EstimatorCommonSettings & settings() const { return settings_; }

  /// Parameter prefix and log name, e.g. "humanoid_inekf".
  const std::string & name() const { return name_; }
  state_estimation::EstimatorType type() const { return estimatorType(); }

  /// Whether the estimate may take over from ground truth. Call once per tick,
  /// after the update. True once the filter has run for ready_min_seconds AND
  /// its observable states are not visibly diverged. Deliberately not called
  /// "converged": no covariance of this filter can show that (see readiness).
  bool evaluateReadiness(double time);
  bool ready() const { return ready_; }
  /// One line saying what the hand-off is waiting for, for the controller to log
  /// while it waits - so a guard that never passes is visible instead of silent.
  std::string readinessReport(double time) const;

  void reset();

protected:
  explicit HumanoidStateEstimator(std::string name);

  virtual state_estimation::EstimatorType estimatorType() const = 0;

  /// Declare this estimator's parameters under name() on the node.
  virtual bool declareParameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) = 0;
  /// Read them into the common settings and the filter's own settings.
  virtual bool readParameters(
    EstimatorCommonSettings & common, state_estimation::EstimatorSettings & filter) = 0;

private:
  std::string name_;
  EstimatorCommonSettings settings_;
  std::unique_ptr<state_estimation::FloatingBaseEstimatorInterface> filter_;
  double first_valid_time_{-1.0};
  bool ready_{false};
};

/// Build the estimator named by floatingBase.estimator ("humanoid_inekf" or
/// "humanoid_linear_kf"). Throws on an unknown name, so a typo fails at configure
/// time instead of quietly running a different filter.
std::unique_ptr<HumanoidStateEstimator> makeHumanoidStateEstimator(const std::string & name);

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_STATE_ESTIMATOR_HPP_
