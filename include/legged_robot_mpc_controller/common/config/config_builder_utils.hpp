#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__CONFIG__CONFIG_BUILDER_UTILS_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__CONFIG__CONFIG_BUILDER_UTILS_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

#include <humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>

namespace legged_robot_mpc_controller::common
{

/// Loads the named gait library (mode sequence templates) from a YAML file, see
/// config/g1/gait.yaml. This replaces the legacy gait.info loading of wb_humanoid_mpc.
ocs2::humanoid::GaitMap loadGaitMap(const std::string& gaitLibraryFile);

inline ocs2::vector_t toVector(const std::vector<double>& values)
{
  return Eigen::Map<const ocs2::vector_t>(values.data(), static_cast<Eigen::Index>(values.size()));
}

/// Assembles the diagonal state / input cost matrix from its per-block diagonals.
inline ocs2::matrix_t assembleDiagonalMatrix(
  const std::vector<std::vector<double>>& blocks, double scaling)
{
  Eigen::Index dim = 0;
  for (const auto& block : blocks) {
    dim += static_cast<Eigen::Index>(block.size());
  }
  ocs2::vector_t diagonal(dim);
  Eigen::Index offset = 0;
  for (const auto& block : blocks) {
    diagonal.segment(offset, static_cast<Eigen::Index>(block.size())) = toVector(block);
    offset += static_cast<Eigen::Index>(block.size());
  }
  return (scaling * diagonal).asDiagonal();
}

inline void checkJointArraySize(
  const std::vector<double>& values, size_t numJoints, const std::string& name)
{
  if (values.size() != numJoints) {
    throw std::invalid_argument(
      "[config_builder] " + name + " has " + std::to_string(values.size()) +
      " entries but robot.jointNames has " + std::to_string(numJoints));
  }
}

/// Builds the reference / command generation configuration from the generated ROS 2
/// parameters. Templated because every controller has its own generated Params type
/// with an identical ocs2.reference section.
template <typename Params>
ocs2::humanoid::ReferenceConfig buildReferenceConfig(const Params& params)
{
  const auto& r = params.ocs2.reference;

  ocs2::humanoid::ReferenceConfig config;
  config.targetDisplacementVelocity = r.targetDisplacementVelocity;
  config.targetRotationVelocity = r.targetRotationVelocity;
  config.maxDisplacementVelocityX = r.maxDisplacementVelocityX;
  config.maxDisplacementVelocityY = r.maxDisplacementVelocityY;
  config.maxDeltaPelvisHeight = r.maxDeltaPelvisHeight;
  config.maxRotationVelocity = r.maxRotationVelocity;
  config.defaultBaseHeight = r.defaultBaseHeight;
  config.basePosePositionTolerance = r.basePosePositionTolerance;
  config.basePoseOrientationTolerance = r.basePoseOrientationTolerance;
  config.defaultJointState = toVector(r.defaultJointState);
  return config;
}

}  // namespace legged_robot_mpc_controller::common

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__CONFIG__CONFIG_BUILDER_UTILS_HPP_
