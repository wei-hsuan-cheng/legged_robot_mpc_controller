#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WB_MPC_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WB_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <humanoid_wb_mpc/command/WBMpcTargetTrajectoriesCalculator.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>

#include "legged_robot_mpc_controller/humanoid_wb_mpc_controller_parameters.hpp"
#include "legged_robot_mpc_controller/ros2_procedural_mpc_motion_manager.hpp"

namespace legged_robot_mpc_controller
{

class HumanoidWbMpcController : public controller_interface::ChainableControllerInterface
{
public:
  using Params = humanoid_wb_mpc_controller::Params;
  using ParamListener = humanoid_wb_mpc_controller::ParamListener;

  ~HumanoidWbMpcController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

private:
  using vector_t = ocs2::vector_t;

  controller_interface::InterfaceConfiguration make_joint_interface_configuration(
    const std::vector<std::string>& interface_names) const;
  std::vector<std::string> floating_base_state_interface_names() const;
  bool read_joint_state(vector_t& q, vector_t& v);
  bool read_floating_base_state(ocs2::SystemObservation& observation);
  std::optional<double> get_state_interface_value(
    const std::string& prefix_name,
    const std::string& interface_name) const;
  ocs2::SystemObservation build_observation(const rclcpp::Time& time);
  ocs2::TargetTrajectories current_observation_to_reset_trajectory(
    const ocs2::SystemObservation& observation);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void start_solver_thread(const ocs2::SystemObservation& initial_observation);
  void stop_solver_thread();
  void solver_worker();
  vector_t compute_weight_compensating_torque(const ocs2::SystemObservation& observation);
  vector_t compute_mpc_torque_command(const ocs2::SystemObservation& observation);
  void log_interface_order() const;
  void log_runtime_diagnostics(
    const ocs2::SystemObservation& observation,
    const vector_t& torque) const;
  void write_torque_command(const vector_t& torque);

  std::shared_ptr<ParamListener> param_listener_;
  Params parameters_;

  std::unique_ptr<ocs2::humanoid::WBMpcInterface> mpc_interface_;
  std::unique_ptr<ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>> control_model_;
  std::unique_ptr<ocs2::humanoid::WBMpcTargetTrajectoriesCalculator> target_trajectories_calculator_;
  std::shared_ptr<Ros2ProceduralMpcMotionManager> motion_manager_;
  std::unique_ptr<ocs2::MPC_BASE> mpc_solver_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_interface_;
  std::jthread solver_thread_;
  std::atomic_bool terminate_solver_thread_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  mutable std::mutex odometry_mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;

  vector_t initial_observation_state_;
  vector_t mpc_joint_kp_;
  vector_t mpc_joint_kd_;
  vector_t fixed_joint_kp_;
  vector_t fixed_joint_kd_;
  vector_t torque_limit_;
  bool diagnostics_active_{true};
  std::uint64_t diagnostics_period_ms_{1000};
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WB_MPC_CONTROLLER_HPP_
