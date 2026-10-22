#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONFIG_BUILDER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONFIG_BUILDER_HPP_

#include <humanoid_wb_id_mpc/WbIdMpcInterface.h>

#include "legged_robot_mpc_controller/humanoid_centroidal_mpc_controller_parameters.hpp"
#include "legged_robot_mpc_controller/humanoid_whole_body_inverse_dynamics_mpc_controller_parameters.hpp"

namespace legged_robot_mpc_controller
{

/// Builds the whole-body inverse-dynamics MPC configuration. Everything the two MPCs
/// share (model, contacts, gait, swing, solver, reference, constraints, foot costs) is
/// built exactly as for the centroidal MPC, from the same parameters; only the cost
/// matrices, the base-twist weights and the initial state are rebuilt in the
/// whole-body dimensions, from `wholeBody`.
ocs2::humanoid::WbIdMpcInterface::Config buildWholeBodyInverseDynamicsMpcConfig(
  const humanoid_centroidal_mpc_controller::Params& params,
  const humanoid_whole_body_inverse_dynamics_mpc_controller::Params& wholeBodyParams);

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONFIG_BUILDER_HPP_
