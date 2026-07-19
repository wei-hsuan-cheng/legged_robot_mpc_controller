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
 *   "joint"                - one target trajectory whose states are the tracked
 *                            arm joints; msg.joint_names selects the ordering and
 *                            must name exactly the tracked arm joints. Overrides
 *                            the internal arm-swing reference.
 *   "frame_relation"       - one pose trajectory per source/target frame pair;
 *                            states are [position(3), quaternion x y z w] of the
 *                            source frame expressed in the world (target frames
 *                            must be a global frame; robot-relative targets are a
 *                            future extension). Source frames must be declared in
 *                            costs.frameRelationTracking.frameNames. Optional
 *                            frame_relation_tracking_weights: 6 values
 *                            [pos xyz, orientation xyz] per pair.
 *   "joint_frame_relation" - both: the joint trajectory first, followed by one
 *                            trajectory per frame pair (msg convention).
 *   "default"              - clears all external targets, reverting to the
 *                            internal posture + arm-swing reference.
 *
 * Each command replaces the full external-target state: channels it does not
 * name are cleared (e.g. a "joint" command clears frame-relation targets).
 *
 * Throws std::invalid_argument on malformed or unsupported commands.
 */
void applyMpcTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& tracked_arm_joint_names,
  const std::vector<std::string>& declared_frame_relation_frames,
  ocs2::humanoid::SwitchedModelReferenceManager& reference_manager);

}  // namespace legged_robot_mpc_controller::target

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__TARGET__MPC_TARGETS_PARSER_HPP_
