#ifndef LEGGED_ROBOT_MPC_CONTROLLER__CONFIG__WB_MPC_CONFIG_BUILDER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__CONFIG__WB_MPC_CONFIG_BUILDER_HPP_

#include <string>

#include <humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>
#include <humanoid_wb_mpc/WBMpcInterface.h>

#include "legged_robot_mpc_controller/humanoid_wb_mpc_controller_parameters.hpp"

namespace legged_robot_mpc_controller
{

/// Builds the full whole-body MPC configuration from the generated ROS 2 parameters.
/// This replaces the legacy task.info / reference.info loading of wb_humanoid_mpc.
ocs2::humanoid::WBMpcInterface::Config buildWbMpcConfig(const humanoid_wb_mpc_controller::Params& params);

/// Builds the reference / command generation configuration from the generated ROS 2 parameters.
ocs2::humanoid::ReferenceConfig buildReferenceConfig(const humanoid_wb_mpc_controller::Params& params);

/// Loads the named gait library (mode sequence templates) from a YAML file, see config/g1/gait.yaml.
/// This replaces the legacy gait.info loading of wb_humanoid_mpc.
ocs2::humanoid::GaitMap loadGaitMap(const std::string& gaitFile);

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__CONFIG__WB_MPC_CONFIG_BUILDER_HPP_
