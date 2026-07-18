#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__MPC_TARGETS_PARSER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__MPC_TARGETS_PARSER_HPP_

#include <string>
#include <vector>

#include <ocs2_msgs/msg/mpc_targets.hpp>

#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>

namespace legged_robot_mpc_controller::target
{

/**
 * Parses an ocs2_msgs::msg::MpcTargets command and applies it to the humanoid
 * reference manager, mirroring the command_type dispatch of mpc_controllers.
 *
 * Supported command types:
 *   "joint"   - one target trajectory whose states are the tracked arm joints;
 *               msg.joint_names selects the ordering and must name exactly the
 *               tracked arm joints. Overrides the internal arm-swing reference.
 *   "default" - clears any external target, reverting to the internal
 *               posture + arm-swing reference.
 *
 * Future command types ("frame_relation" and combinations) extend this dispatch
 * alongside their cost terms.
 *
 * Throws std::invalid_argument on malformed or unsupported commands.
 */
void applyMpcTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& tracked_arm_joint_names,
  ocs2::humanoid::SwitchedModelReferenceManager& reference_manager);

}  // namespace legged_robot_mpc_controller::target

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__MPC_TARGETS_PARSER_HPP_
