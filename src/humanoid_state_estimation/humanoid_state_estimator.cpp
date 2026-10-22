// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_state_estimator.hpp"

#include <cstdio>
#include <stdexcept>
#include <utility>

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_inekf.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_linear_kf.hpp"

namespace legged_robot_mpc_controller
{

HumanoidStateEstimator::HumanoidStateEstimator(std::string name) : name_(std::move(name)) {}

HumanoidStateEstimator::~HumanoidStateEstimator() = default;

bool HumanoidStateEstimator::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node, double sampling_time)
{
  if (!declareParameters(node)) {
    return false;
  }
  state_estimation::EstimatorSettings filter;
  if (!readParameters(settings_, filter)) {
    return false;
  }
  filter.urdf_path = settings_.urdf_file;
  filter.controller_joint_names = settings_.joint_names;
  filter.sampling_time = sampling_time;
  filter.imu_frame = settings_.imu_frame;
  filter.contact_frames = settings_.contact_frames;
  filter.contact_source = settings_.contact_source;
  try {
    filter_ = state_estimation::makeFloatingBaseEstimator(estimatorType(), filter);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] building the %s filter failed: %s", name_.c_str(),
      state_estimation::toString(estimatorType()), e.what());
    return false;
  }
  return true;
}

bool HumanoidStateEstimator::evaluateReadiness(double time)
{
  ready_ = false;
  if (!filter_ || !filter_->initialized()) {
    first_valid_time_ = -1.0;
    return false;
  }
  if (first_valid_time_ < 0.0) {
    first_valid_time_ = time;
  }
  if (time - first_valid_time_ < settings_.ready_min_seconds) {
    return false;
  }
  const auto & d = filter_->diagnostics();
  // Observable states only. Yaw (attitude z) and absolute x/y are gauge states:
  // their covariance grows without bound by construction, so gating on them
  // would eventually block the hand-off for no fault of the filter. A NaN fails
  // every comparison, so a broken filter is never handed control.
  ready_ = d.attitude_std.head<2>().maxCoeff() < settings_.ready_roll_pitch_std_max &&
           d.position_std.z() < settings_.ready_height_std_max &&
           d.velocity_std.maxCoeff() < settings_.ready_velocity_std_max;
  return ready_;
}

std::string HumanoidStateEstimator::readinessReport(double time) const
{
  if (!filter_ || !filter_->initialized()) {
    return "filter not initialized";
  }
  const auto & d = filter_->diagnostics();
  const double elapsed = first_valid_time_ < 0.0 ? 0.0 : time - first_valid_time_;
  char line[256];
  std::snprintf(
    line, sizeof(line),
    "running %.1f/%.1f s | roll/pitch std %.4f/%.2f | height std %.4f/%.2f | velocity std %.4f/%.2f",
    elapsed, settings_.ready_min_seconds,
    d.attitude_std.head<2>().maxCoeff(), settings_.ready_roll_pitch_std_max,
    d.position_std.z(), settings_.ready_height_std_max,
    d.velocity_std.maxCoeff(), settings_.ready_velocity_std_max);
  return line;
}

void HumanoidStateEstimator::reset()
{
  if (filter_) {
    filter_->reset();
  }
  first_valid_time_ = -1.0;
  ready_ = false;
}

std::unique_ptr<HumanoidStateEstimator> makeHumanoidStateEstimator(const std::string & name)
{
  if (name == "humanoid_inekf") {
    return std::make_unique<HumanoidInEkf>();
  }
  if (name == "humanoid_linear_kf") {
    return std::make_unique<HumanoidLinearKf>();
  }
  throw std::invalid_argument(
    "unknown floatingBase.estimator '" + name + "' (expected humanoid_inekf or humanoid_linear_kf)");
}

}  // namespace legged_robot_mpc_controller
