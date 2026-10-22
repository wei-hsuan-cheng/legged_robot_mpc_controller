// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_state_estimator_controller.hpp"

#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "legged_robot_mpc_controller/humanoid_state_estimation/estimator_state_interfaces.hpp"

namespace legged_robot_mpc_controller
{

namespace
{
constexpr std::size_t kJointInterfaces = 3;  // position, velocity, effort
}  // namespace

HumanoidStateEstimatorController::HumanoidStateEstimatorController(std::string estimator_name)
: estimator_name_(std::move(estimator_name))
{
}

controller_interface::CallbackReturn HumanoidStateEstimatorController::on_init()
{
  try {
    estimator_ = makeHumanoidStateEstimator(estimator_name_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HumanoidStateEstimatorController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
HumanoidStateEstimatorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto & se = estimator_->settings();
  for (const auto & joint : se.joint_names) {
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  const std::string & imu = se.imu_interface_name;
  for (const char * axis : {"x", "y", "z"}) {
    config.names.emplace_back(imu + "/angular_velocity." + axis);
  }
  for (const char * axis : {"x", "y", "z"}) {
    config.names.emplace_back(imu + "/linear_acceleration." + axis);
  }
  for (const char * component : {"w", "x", "y", "z"}) {
    config.names.emplace_back(imu + "/orientation." + component);
  }
  return config;
}

controller_interface::CallbackReturn HumanoidStateEstimatorController::on_configure(
  const rclcpp_lifecycle::State &)
{
  // A zero rate is fatal, not defaulted: the filter integrates the IMU at
  // 1 / update_rate, and max(1, rate) would make that a one-second timestep.
  if (get_update_rate() == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "update_rate is 0; the filter integrates at 1 / update_rate");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!estimator_->configure(get_node(), 1.0 / static_cast<double>(get_update_rate()))) {
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto & se = estimator_->settings();
  const std::size_t n = se.joint_names.size();
  joint_pos_.assign(n, 0.0);
  joint_vel_.assign(n, 0.0);
  joint_eff_.assign(n, 0.0);
  foot_contacts_.assign(2, true);
  exported_values_.assign(estimator_interfaces::names().size(), 0.0);


  rt_odometry_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    get_node(), se.odom_topic, rclcpp::SystemDefaultsQoS());
  odometry_.header.frame_id = se.odom_frame;
  odometry_.child_frame_id = se.base_frame;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[%s] %s filter | imu=%s contacts=%zu joints=%d | %u Hz | contact_source=%s | %zu exported state "
    "interfaces",
    estimator_name_.c_str(), state_estimation::toString(estimator_->type()), se.imu_frame.c_str(),
    se.contact_frames.size(), estimator_->filter().numEstimatorJoints(), get_update_rate(),
    se.contact_source.c_str(), exported_values_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::SharedPtr>
HumanoidStateEstimatorController::on_export_state_interfaces_list()
{
  exported_interfaces_.clear();
  for (const auto & name : estimator_interfaces::names()) {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    info.data_type = "double";
    // Zero, not the NaN default: `valid` must read false before the first update.
    info.initial_value = "0.0";
    exported_interfaces_.push_back(std::make_shared<hardware_interface::StateInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return exported_interfaces_;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
HumanoidStateEstimatorController::on_export_reference_interfaces_list()
{
  reference_handles_.clear();
  for (const auto & name : estimator_interfaces::reference_names()) {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    info.data_type = "double";
    // Both feet in stance, epoch 0 (off), until the consumer writes them.
    info.initial_value = name == "initialize_epoch" ? "0.0" : "1.0";
    reference_handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return reference_handles_;
}

controller_interface::CallbackReturn HumanoidStateEstimatorController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Off until an epoch is written. One written before this activation is still in
  // the reference interface, so it is acted on at the first update.
  estimator_->reset();
  epoch_ = 0;
  estimate_ = state_estimation::FloatingBaseEstimate{};
  last_odometry_time_ = -1.0;
  write_exported_interfaces(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidStateEstimatorController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  estimate_ = state_estimation::FloatingBaseEstimate{};
  epoch_ = 0;
  write_exported_interfaces(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HumanoidStateEstimatorController::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

bool HumanoidStateEstimatorController::read_sensors()
{
  const std::size_t n = joint_pos_.size();
  std::size_t i = 0;
  for (std::size_t j = 0; j < n; ++j) {
    const auto p = state_interfaces_[i++].get_optional();
    const auto v = state_interfaces_[i++].get_optional();
    const auto e = state_interfaces_[i++].get_optional();
    if (!p || !v) {
      return false;
    }
    joint_pos_[j] = *p;
    joint_vel_[j] = *v;
    joint_eff_[j] = e ? *e : 0.0;
  }
  for (int k = 0; k < 3; ++k) {
    const auto g = state_interfaces_[i + k].get_optional();
    const auto a = state_interfaces_[i + 3 + k].get_optional();
    if (!g || !a) {
      return false;
    }
    gyro_(k) = *g;
    accel_(k) = *a;
  }
  i += 6;
  const auto qw = state_interfaces_[i].get_optional();
  const auto qx = state_interfaces_[i + 1].get_optional();
  const auto qy = state_interfaces_[i + 2].get_optional();
  const auto qz = state_interfaces_[i + 3].get_optional();
  imu_orientation_valid_ = false;
  if (qw && qx && qy && qz) {
    const Eigen::Quaterniond q(*qw, *qx, *qy, *qz);
    // A zero quaternion means the sensor has not produced a sample yet;
    // normalising it would yield NaN and poison the whole filter.
    if (q.coeffs().allFinite() && q.norm() > 1e-6) {
      imu_orientation_ = q.normalized();
      imu_orientation_valid_ = true;
    }
  }
  return gyro_.allFinite() && accel_.allFinite();
}

controller_interface::return_type HumanoidStateEstimatorController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  const double t = time.seconds();
  const auto epoch_reference =
    reference_handles_[estimator_interfaces::kInitializeEpoch]->get_optional();
  const std::uint32_t requested =
    epoch_reference && std::isfinite(*epoch_reference) && *epoch_reference > 0.0 ?
    static_cast<std::uint32_t>(*epoch_reference) : epoch_;
  if (requested != epoch_) {
    estimator_->reset();
    estimate_ = state_estimation::FloatingBaseEstimate{};
    epoch_ = requested;
    epoch_time_ = t;
  }
  if (epoch_ == 0 || !read_sensors()) {
    write_exported_interfaces(t);
    return controller_interface::return_type::OK;
  }

  auto & filter = estimator_->filter();
  // Initialized from the robot's own sensors - IMU attitude, joint encoders, and
  // standing on both feet, which whoever sent the epoch guarantees. No external
  // pose, on purpose: a real robot has none to give it.
  if (!filter.initialized()) {
    if (!imu_orientation_valid_) {
      write_exported_interfaces(t);
      return controller_interface::return_type::OK;
    }
    filter.initializeFromStance(imu_orientation_, joint_pos_);
  }
  if (imu_orientation_valid_) {
    filter.setBaseOrientation(imu_orientation_);
  }

  if (estimator_->settings().contact_source == "scheduled") {
    // From the consumer's reference interfaces; a missing or NaN value counts as
    // stance, the state the robot is in until the MPC drives.
    for (std::size_t foot = 0; foot < 2; ++foot) {
      const auto flag = reference_handles_[estimator_interfaces::kContactLeft + foot]->get_optional();
      foot_contacts_[foot] = !flag || !std::isfinite(*flag) || *flag > 0.5;
    }
    estimate_ = filter.update(gyro_, accel_, joint_pos_, joint_vel_, joint_eff_, foot_contacts_);
  } else {
    estimate_ = filter.update(gyro_, accel_, joint_pos_, joint_vel_, joint_eff_);
  }
  estimator_->evaluateReadiness(t);

  write_exported_interfaces(t);
  publish_odometry(time);
  return controller_interface::return_type::OK;
}

void HumanoidStateEstimatorController::write_exported_interfaces(double t)
{
  estimator_interfaces::EstimatorStatus status;
  status.ready = estimator_->ready();
  status.epoch = epoch_;
  status.running_time = epoch_ == 0 ? 0.0 : t - epoch_time_;
  estimator_interfaces::pack(estimate_, estimator_->filter().diagnostics(), status, exported_values_);
  for (std::size_t i = 0; i < exported_interfaces_.size(); ++i) {
    (void)exported_interfaces_[i]->set_value(exported_values_[i]);
  }
}

void HumanoidStateEstimatorController::publish_odometry(const rclcpp::Time & time)
{
  if (!estimate_.valid) {
    return;
  }
  const double t = time.seconds();
  const double period = 1.0 / std::max(1.0, estimator_->settings().publish_rate);
  if (last_odometry_time_ >= 0.0 && t - last_odometry_time_ < period) {
    return;
  }
  last_odometry_time_ = t;
  odometry_.header.stamp = time;
  odometry_.pose.pose.position.x = estimate_.position.x();
  odometry_.pose.pose.position.y = estimate_.position.y();
  odometry_.pose.pose.position.z = estimate_.position.z();
  odometry_.pose.pose.orientation.w = estimate_.orientation.w();
  odometry_.pose.pose.orientation.x = estimate_.orientation.x();
  odometry_.pose.pose.orientation.y = estimate_.orientation.y();
  odometry_.pose.pose.orientation.z = estimate_.orientation.z();
  // Twist in the body frame (odometry child-frame convention).
  const Eigen::Vector3d v_body = estimate_.orientation.conjugate() * estimate_.linear_velocity_world;
  odometry_.twist.twist.linear.x = v_body.x();
  odometry_.twist.twist.linear.y = v_body.y();
  odometry_.twist.twist.linear.z = v_body.z();
  odometry_.twist.twist.angular.x = estimate_.angular_velocity_local.x();
  odometry_.twist.twist.angular.y = estimate_.angular_velocity_local.y();
  odometry_.twist.twist.angular.z = estimate_.angular_velocity_local.z();
  rt_odometry_publisher_->try_publish(odometry_);
}

}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidInEkfController, controller_interface::ChainableControllerInterface)
PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidLinearKfController, controller_interface::ChainableControllerInterface)
