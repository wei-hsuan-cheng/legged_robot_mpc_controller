#include "legged_robot_mpc_controller/humanoid_wb_mpc_controller.hpp"

#include <string>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "legged_robot_mpc_controller/config/wb_mpc_config_builder.hpp"

namespace legged_robot_mpc_controller
{

controller_interface::CallbackReturn HumanoidWbMpcController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    parameters_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] init failed: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "[HumanoidWbMpcController] init success");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_configure(
  const rclcpp_lifecycle::State&)
{
  if (param_listener_->is_old(parameters_)) {
    parameters_ = param_listener_->get_params();
  }

  if (parameters_.paths.urdfFile.empty() || parameters_.paths.libFolder.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] paths.urdfFile or paths.libFolder is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController] CppAD library folder: %s | recompileLibraries=%s",
    parameters_.paths.libFolder.c_str(),
    parameters_.ocs2.model.recompileLibrariesCppAd ? "true" : "false");

  // Build the whole-body MPC problem from ROS 2 parameters. The first run generates and
  // compiles the CppAD model libraries into paths.libFolder, which can take several minutes;
  // subsequent runs load the cached libraries.
  try {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] Constructing WBMpcInterface (CppAD codegen folder: %s, "
      "recompile: %s). First-time codegen can take several minutes...",
      parameters_.paths.libFolder.c_str(),
      parameters_.ocs2.model.recompileLibrariesCppAd ? "true" : "false");
    mpc_interface_ = std::make_unique<ocs2::humanoid::WBMpcInterface>(buildWbMpcConfig(parameters_));
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] WBMpcInterface ready (state dim %zu, input dim %zu)",
      static_cast<size_t>(mpc_interface_->getMpcRobotModel().getStateDim()),
      static_cast<size_t>(mpc_interface_->getMpcRobotModel().getInputDim()));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "[HumanoidWbMpcController] Failed to build WBMpcInterface: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController] configured migration scaffold | joints=%zu solver=%s "
    "floatingBase.source=%s odom=%s baseFrame=%s",
    parameters_.robot.jointNames.size(),
    parameters_.ocs2.mpc.solverType.c_str(),
    parameters_.floatingBase.source.c_str(),
    parameters_.floatingBase.odometryTopic.c_str(),
    parameters_.floatingBase.baseFrame.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_activate(
  const rclcpp_lifecycle::State&)
{
  if (!parameters_.development.allowPlaceholderActivation) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] activation blocked: this package currently contains the "
      "copied humanoid MPC core and controller scaffold only. Set "
      "development.allowPlaceholderActivation=true only for interface smoke tests.");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_WARN(
    get_node()->get_logger(),
    "[HumanoidWbMpcController] placeholder activation enabled. No MPC torque adapter is "
    "running yet.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HumanoidWbMpcController::command_interface_configuration()
  const
{
  return make_joint_interface_configuration({parameters_.robot.commandInterface});
}

controller_interface::InterfaceConfiguration HumanoidWbMpcController::state_interface_configuration()
  const
{
  return make_joint_interface_configuration(parameters_.robot.stateInterfaces);
}

controller_interface::return_type HumanoidWbMpcController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type HumanoidWbMpcController::update_and_write_commands(
  const rclcpp::Time&,
  const rclcpp::Duration&)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
HumanoidWbMpcController::on_export_reference_interfaces()
{
  return {};
}

controller_interface::InterfaceConfiguration
HumanoidWbMpcController::make_joint_interface_configuration(
  const std::vector<std::string>& interface_names) const
{
  controller_interface::InterfaceConfiguration config;
  if (parameters_.robot.jointNames.empty() || interface_names.empty()) {
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : parameters_.robot.jointNames) {
    for (const auto& interface_name : interface_names) {
      if (!interface_name.empty()) {
        config.names.emplace_back(joint_name + "/" + interface_name);
      }
    }
  }
  return config;
}

}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidWbMpcController,
  controller_interface::ChainableControllerInterface)

