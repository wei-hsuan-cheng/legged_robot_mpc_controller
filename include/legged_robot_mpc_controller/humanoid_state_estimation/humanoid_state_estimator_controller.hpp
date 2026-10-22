// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_STATE_ESTIMATOR_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_STATE_ESTIMATOR_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimate.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_state_estimator.hpp"

namespace legged_robot_mpc_controller
{

/**
 * A floating-base estimator as a ros2_control controller.
 *
 * Both directions are chained interfaces this controller exports, and only the
 * consumer claims them (estimator_state_interfaces.hpp), so there is no cycle:
 *  - estimate -> consumer: the estimate, its status and its scalar diagnostics, as
 *    state interfaces;
 *  - consumer -> estimator: the contact flags and the initialization epoch, as
 *    reference interfaces the consumer writes. The filter stays off until an epoch is
 *    written, then initializes from stance, and again whenever it changes. The epoch
 *    acted on is exported, so the consumer can tell a fresh `ready` from a stale one.
 *
 * The controller manager runs the two in one order per tick, so one direction is a
 * tick late.
 *
 * Which filter runs is decided by which controller is loaded.
 */
class HumanoidStateEstimatorController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  explicit HumanoidStateEstimatorController(std::string estimator_name);

  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool read_sensors();
  void write_exported_interfaces(double t);
  void publish_odometry(const rclcpp::Time & time);

  std::string estimator_name_;
  std::unique_ptr<HumanoidStateEstimator> estimator_;

  // Claimed state interfaces, in this order: per joint position, velocity,
  // effort; then IMU gyro xyz, accelerometer xyz, orientation wxyz.
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  Eigen::Vector3d gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond imu_orientation_{Eigen::Quaterniond::Identity()};
  bool imu_orientation_valid_{false};
  std::vector<bool> foot_contacts_;

  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_handles_;
  std::uint32_t epoch_{0};
  double epoch_time_{0.0};

  state_estimation::FloatingBaseEstimate estimate_;
  std::vector<double> exported_values_;
  std::vector<hardware_interface::StateInterface::SharedPtr> exported_interfaces_;

  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odometry_publisher_;
  nav_msgs::msg::Odometry odometry_;
  double last_odometry_time_{-1.0};
};

class HumanoidInEkfController : public HumanoidStateEstimatorController
{
public:
  HumanoidInEkfController() : HumanoidStateEstimatorController("humanoid_inekf") {}
};

class HumanoidLinearKfController : public HumanoidStateEstimatorController
{
public:
  HumanoidLinearKfController() : HumanoidStateEstimatorController("humanoid_linear_kf") {}
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_STATE_ESTIMATOR_CONTROLLER_HPP_
