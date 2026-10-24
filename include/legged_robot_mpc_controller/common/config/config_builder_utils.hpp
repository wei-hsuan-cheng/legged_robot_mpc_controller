#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__CONFIG__CONFIG_BUILDER_UTILS_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__CONFIG__CONFIG_BUILDER_UTILS_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

#include <humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include <humanoid_common_mpc/reference_manager/StairClimbingPlan.h>
#include <humanoid_common_mpc/reference_manager/TerrainFootholdPlanner.h>

namespace legged_robot_mpc_controller::common
{

/// Loads the semantic stair climbing parameters (staircase ground truth, gait
/// timing, foothold generation, base reference), see config/g1/stair_climbing.yaml.
ocs2::humanoid::StairClimbingConfig loadStairClimbingConfig(const std::string& stairClimbingFile);

/// Loads the ground-truth terrain geometry for terrain-aware walking from the
/// `terrain_walk.stairs` section (used to build the terrain model). Only the
/// geometry fields of StairClimbingConfig are populated.
ocs2::humanoid::StairClimbingConfig loadTerrainStairsGeometry(const std::string& terrainWalkingFile);
ocs2::humanoid::GroundTruthTerrainModel loadTerrainModel(const std::string& terrainWalkingFile);

/// Loads the online terrain-walk planner settings from the `terrain_walk`
/// section (defaults when absent).
ocs2::humanoid::TerrainFootholdPlannerSettings loadTerrainFootholdPlannerSettings(const std::string& terrainWalkingFile);

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

/// Indices (within the joint block) of the arm joints, identified by name. Wrist
/// joints are fixed and never appear in the MPC joint list, so matching
/// "shoulder"/"elbow" selects exactly the actuated arm joints.
inline std::vector<size_t> findArmJointIndices(const std::vector<std::string>& jointNames)
{
  std::vector<size_t> indices;
  for (size_t i = 0; i < jointNames.size(); ++i) {
    if (jointNames[i].find("shoulder") != std::string::npos ||
        jointNames[i].find("elbow") != std::string::npos) {
      indices.push_back(i);
    }
  }
  return indices;
}

/// Copy of the per-joint values with the given indices zeroed (moves those joints
/// out of the generic joint cost so the dedicated arm term owns them).
inline std::vector<double> zeroAtIndices(
  std::vector<double> values, const std::vector<size_t>& indices)
{
  for (size_t idx : indices) {
    values.at(idx) = 0.0;
  }
  return values;
}

/// The per-joint values at the given indices, in index order.
template <typename T>
inline std::vector<T> selectAtIndices(
  const std::vector<T>& values, const std::vector<size_t>& indices)
{
  std::vector<T> out;
  out.reserve(indices.size());
  for (size_t idx : indices) {
    out.push_back(values.at(idx));
  }
  return out;
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

/// Builds the named gait library from `ocs2.gait.templates` (one modeSequence /
/// switchingTimes pair per entry of `names`). Templated like buildReferenceConfig.
template <typename Params>
ocs2::humanoid::GaitMap buildGaitMap(const Params& params)
{
  const auto& templates = params.ocs2.gait.templates;
  if (templates.names.empty()) {
    throw std::invalid_argument("[config_builder] ocs2.gait.templates.names is empty");
  }
  ocs2::humanoid::GaitMap gaitMap;
  for (const auto& name : templates.names) {
    const auto& entry = templates.names_map.at(name);
    if (entry.switchingTimes.size() != entry.modeSequence.size() + 1) {
      throw std::invalid_argument(
        "[config_builder] ocs2.gait.templates." + name + ": N modes need N+1 switching times");
    }
    gaitMap.insert({name, ocs2::humanoid::modeSequenceTemplateFromStrings(entry.switchingTimes, entry.modeSequence)});
  }
  return gaitMap;
}

/// Builds the velocity-based gait selection from `ocs2.gait.transitions`: the ladder
/// (slowest to fastest, starting at "stance") and its switching timing, validated
/// against the templates in `gaitMap`.
template <typename Params>
ocs2::humanoid::ProceduralMpcMotionManager::GaitTransitionConfig buildGaitTransitionConfig(
  const Params& params, const ocs2::humanoid::GaitMap& gaitMap)
{
  const auto& transitions = params.ocs2.gait.transitions;
  ocs2::humanoid::ProceduralMpcMotionManager::GaitTransitionConfig config;
  config.minDwellTime = transitions.minDwellTime;
  config.switchDelayHorizonFraction = transitions.switchDelayHorizonFraction;
  for (const auto& gait : transitions.ladder) {
    const std::string where = "[config_builder] ocs2.gait.transitions." + gait;
    if (gaitMap.count(gait) == 0) {
      throw std::invalid_argument(where + ": not in ocs2.gait.templates.names");
    }
    const auto& rung = transitions.ladder_map.at(gait);
    if (rung.linearVelocity[0] > rung.linearVelocity[1] || rung.yawRate[0] > rung.yawRate[1]) {
      throw std::invalid_argument(where + ": bands need min <= max");
    }
    ocs2::humanoid::ProceduralMpcMotionManager::GaitModeStateConfig state;
    state.gaitCommand = gait;
    state.minLinVelCmd = rung.linearVelocity[0];
    state.maxLinVelCmd = rung.linearVelocity[1];
    state.minAngVelCmd = rung.yawRate[0];
    state.maxAngVelCmd = rung.yawRate[1];
    state.linVelErrorThresh = rung.measuredMargin[0];
    state.angVelErrorThresh = rung.measuredMargin[1];
    config.ladder.push_back(state);
  }
  if (config.ladder.empty() || config.ladder.front().gaitCommand != "stance") {
    throw std::invalid_argument("[config_builder] ocs2.gait.transitions.ladder must start with stance");
  }
  return config;
}

}  // namespace legged_robot_mpc_controller::common

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__CONFIG__CONFIG_BUILDER_UTILS_HPP_
