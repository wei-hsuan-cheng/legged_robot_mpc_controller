#include "legged_robot_mpc_controller/common/target/mpc_targets_parser.hpp"

#include <algorithm>
#include <stdexcept>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace legged_robot_mpc_controller::target
{
namespace
{

using ocs2::humanoid::SwitchedModelReferenceManager;

bool isGlobalFrameName(const std::string& frame)
{
  return frame == "world" || frame == "odom" || frame == "map" || frame == "global";
}

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

/// Parses trajectory_index of the msg as the arm joint target (tracked order).
ocs2::TargetTrajectories parseJointTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& tracked_arm_joint_names,
  size_t trajectory_index)
{
  ocs2::TargetTrajectories trajectory =
    ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(message.target_trajectories[trajectory_index]);
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

/// Parses the frame-relation pairs; their trajectories start at trajectory_offset.
SwitchedModelReferenceManager::FrameRelationTargets parseFrameRelationTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& declared_frame_relation_frames,
  size_t trajectory_offset)
{
  const size_t pair_count = message.source_frames.size();
  if (pair_count == 0) {
    throw std::invalid_argument("[mpc_targets_parser] frame_relation command has no frame pairs");
  }
  if (message.target_frames.size() != pair_count) {
    throw std::invalid_argument(
      "[mpc_targets_parser] source_frames/target_frames size mismatch");
  }
  if (message.target_trajectories.size() != trajectory_offset + pair_count) {
    throw std::invalid_argument(
      "[mpc_targets_parser] frame_relation command expects " +
      std::to_string(trajectory_offset + pair_count) + " trajectories, got " +
      std::to_string(message.target_trajectories.size()));
  }
  const bool has_weights = !message.frame_relation_tracking_weights.empty();
  if (has_weights && message.frame_relation_tracking_weights.size() != pair_count) {
    throw std::invalid_argument(
      "[mpc_targets_parser] frame_relation_tracking_weights must be empty or one entry per pair");
  }

  SwitchedModelReferenceManager::FrameRelationTargets targets;
  for (size_t i = 0; i < pair_count; ++i) {
    const std::string& source_frame = message.source_frames[i];
    const std::string& target_frame = message.target_frames[i];

    if (!isGlobalFrameName(target_frame)) {
      throw std::invalid_argument(
        "[mpc_targets_parser] target frame '" + target_frame +
        "' is not a global frame (robot-relative targets are a future extension)");
    }
    if (std::find(declared_frame_relation_frames.begin(), declared_frame_relation_frames.end(),
                  source_frame) == declared_frame_relation_frames.end()) {
      throw std::invalid_argument(
        "[mpc_targets_parser] source frame '" + source_frame +
        "' is not declared in costs.frameRelationTracking.frameNames");
    }

    ocs2::TargetTrajectories trajectory = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(
      message.target_trajectories[trajectory_offset + i]);
    for (const auto& state : trajectory.stateTrajectory) {
      if (state.size() != 7) {
        throw std::invalid_argument(
          "[mpc_targets_parser] frame_relation states must be [position(3), quaternion xyzw]");
      }
      if (state.tail<4>().norm() < 1.0e-9) {
        throw std::invalid_argument(
          "[mpc_targets_parser] frame_relation quaternion has zero norm");
      }
    }

    ocs2::vector_t weights;
    if (has_weights) {
      const auto& weights_msg = message.frame_relation_tracking_weights[i].data;
      if (weights_msg.size() != 6) {
        throw std::invalid_argument(
          "[mpc_targets_parser] frame_relation_tracking_weights entries must have 6 values "
          "[pos xyz, orientation xyz]");
      }
      weights = Eigen::Map<const ocs2::vector_t>(
        weights_msg.data(), static_cast<Eigen::Index>(weights_msg.size()));
    }

    targets.sourceFrames.push_back(source_frame);
    targets.targets.push_back(std::move(trajectory));
    targets.weights.push_back(std::move(weights));
  }
  return targets;
}

}  // namespace

void applyMpcTargets(
  const ocs2_msgs::msg::MpcTargets& message,
  const std::vector<std::string>& tracked_arm_joint_names,
  const std::vector<std::string>& declared_frame_relation_frames,
  ocs2::humanoid::SwitchedModelReferenceManager& reference_manager)
{
  const std::string& command_type = message.command_type;

  // Parse fully before applying so a malformed command leaves the state untouched;
  // each command then replaces both channels (unnamed channels are cleared).
  if (command_type == "joint") {
    if (message.target_trajectories.size() != 1) {
      throw std::invalid_argument(
        "[mpc_targets_parser] joint command has " +
        std::to_string(message.target_trajectories.size()) + " trajectories, expected 1");
    }
    auto joint_targets = parseJointTargets(message, tracked_arm_joint_names, 0);
    reference_manager.setExternalJointTargets(std::move(joint_targets));
    reference_manager.setExternalFrameRelationTargets({});
  } else if (command_type == "frame_relation") {
    auto frame_targets =
      parseFrameRelationTargets(message, declared_frame_relation_frames, 0);
    reference_manager.setExternalJointTargets(ocs2::TargetTrajectories());
    reference_manager.setExternalFrameRelationTargets(std::move(frame_targets));
  } else if (command_type == "joint_frame_relation") {
    // Msg convention: the joint trajectory first, then one per frame pair.
    if (message.target_trajectories.empty()) {
      throw std::invalid_argument(
        "[mpc_targets_parser] joint_frame_relation command has no trajectories");
    }
    auto joint_targets = parseJointTargets(message, tracked_arm_joint_names, 0);
    auto frame_targets =
      parseFrameRelationTargets(message, declared_frame_relation_frames, 1);
    reference_manager.setExternalJointTargets(std::move(joint_targets));
    reference_manager.setExternalFrameRelationTargets(std::move(frame_targets));
  } else if (command_type == "default") {
    reference_manager.setExternalJointTargets(ocs2::TargetTrajectories());
    reference_manager.setExternalFrameRelationTargets({});
  } else {
    throw std::invalid_argument(
      "[mpc_targets_parser] unsupported command type '" + command_type +
      "' (supported: joint, frame_relation, joint_frame_relation, default)");
  }
}

}  // namespace legged_robot_mpc_controller::target
