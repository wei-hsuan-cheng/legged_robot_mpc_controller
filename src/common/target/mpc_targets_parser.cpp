#include "legged_robot_mpc_controller/common/target/mpc_targets_parser.hpp"

#include <algorithm>
#include <stdexcept>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace legged_robot_mpc_controller::target
{
namespace
{

/// msg.joint_names -> tracked order: reorder[i] is the msg column of tracked joint i.
std::vector<size_t> buildReorderMap(
  const std::vector<std::string>& message_joint_names,
  const std::vector<std::string>& tracked_arm_joint_names)
{
  if (message_joint_names.size() != tracked_arm_joint_names.size()) {
    throw std::invalid_argument(
      "[mpc_targets_parser] joint command names " + std::to_string(message_joint_names.size()) +
      " != tracked arm joints " + std::to_string(tracked_arm_joint_names.size()));
  }
  std::vector<size_t> reorder(tracked_arm_joint_names.size());
  for (size_t i = 0; i < tracked_arm_joint_names.size(); ++i) {
    const auto it = std::find(
      message_joint_names.begin(), message_joint_names.end(), tracked_arm_joint_names[i]);
    if (it == message_joint_names.end()) {
      throw std::invalid_argument(
        "[mpc_targets_parser] tracked arm joint '" + tracked_arm_joint_names[i] +
        "' missing from joint command");
    }
    reorder[i] = static_cast<size_t>(std::distance(message_joint_names.begin(), it));
  }
  return reorder;
}

ocs2::TargetTrajectories parseJointTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& tracked_arm_joint_names)
{
  if (message.target_trajectories.size() != 1) {
    throw std::invalid_argument(
      "[mpc_targets_parser] joint command has " +
      std::to_string(message.target_trajectories.size()) + " trajectories, expected 1");
  }

  ocs2::TargetTrajectories trajectory =
    ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(message.target_trajectories[0]);
  const auto reorder = buildReorderMap(message.joint_names, tracked_arm_joint_names);

  for (auto& state : trajectory.stateTrajectory) {
    if (state.size() != static_cast<Eigen::Index>(reorder.size())) {
      throw std::invalid_argument(
        "[mpc_targets_parser] joint target state size " + std::to_string(state.size()) +
        " != tracked arm joints " + std::to_string(reorder.size()));
    }
    ocs2::vector_t reordered(state.size());
    for (Eigen::Index i = 0; i < state.size(); ++i) {
      reordered(i) = state(static_cast<Eigen::Index>(reorder[static_cast<size_t>(i)]));
    }
    state = std::move(reordered);
  }
  return trajectory;
}

}  // namespace

void applyMpcTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& tracked_arm_joint_names,
  ocs2::humanoid::SwitchedModelReferenceManager& reference_manager)
{
  const std::string& command_type = message.command_type;

  if (command_type == "joint") {
    reference_manager.setExternalJointTargets(
      parseJointTargets(message, tracked_arm_joint_names));
  } else if (command_type == "default") {
    reference_manager.setExternalJointTargets(ocs2::TargetTrajectories());
  } else {
    throw std::invalid_argument(
      "[mpc_targets_parser] unsupported command type '" + command_type +
      "' (supported: joint, default; frame_relation is a future extension)");
  }
}

}  // namespace legged_robot_mpc_controller::target
