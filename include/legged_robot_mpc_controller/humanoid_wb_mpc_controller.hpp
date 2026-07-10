#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WB_MPC_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WB_MPC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "legged_robot_mpc_controller/humanoid_wb_mpc_controller_parameters.hpp"

namespace legged_robot_mpc_controller
{

class HumanoidWbMpcController : public controller_interface::ChainableControllerInterface
{
public:
  using Params = humanoid_wb_mpc_controller::Params;
  using ParamListener = humanoid_wb_mpc_controller::ParamListener;

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
  controller_interface::InterfaceConfiguration make_joint_interface_configuration(
    const std::vector<std::string>& interface_names) const;

  std::shared_ptr<ParamListener> param_listener_;
  Params parameters_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WB_MPC_CONTROLLER_HPP_

