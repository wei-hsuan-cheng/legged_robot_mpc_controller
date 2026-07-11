#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__CENTROIDAL_MPC_CONFIG_BUILDER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__CENTROIDAL_MPC_CONFIG_BUILDER_HPP_

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>

#include "legged_robot_mpc_controller/humanoid_centroidal_mpc_controller_parameters.hpp"

namespace legged_robot_mpc_controller
{

/// Builds the full centroidal MPC configuration from the generated ROS 2 parameters.
/// This replaces the legacy task.info / reference.info loading of wb_humanoid_mpc.
ocs2::humanoid::CentroidalMpcInterface::Config buildCentroidalMpcConfig(
  const humanoid_centroidal_mpc_controller::Params& params);

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__CENTROIDAL_MPC_CONFIG_BUILDER_HPP_
