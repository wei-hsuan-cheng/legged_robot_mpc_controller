#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONFIG_BUILDER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONFIG_BUILDER_HPP_

#include <humanoid_wb_id_mpc/WbIdMpcInterface.h>

#include "legged_robot_mpc_controller/humanoid_whole_body_inverse_dynamics_mpc_controller_parameters.hpp"

namespace legged_robot_mpc_controller
{

/// Builds x = [q, v], u = [W, a, tau] directly from the standalone WB-ID schema.
ocs2::humanoid::WbIdMpcInterface::Config buildWholeBodyInverseDynamicsMpcConfig(
  const humanoid_whole_body_inverse_dynamics_mpc_controller::Params& params);

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONFIG_BUILDER_HPP_
