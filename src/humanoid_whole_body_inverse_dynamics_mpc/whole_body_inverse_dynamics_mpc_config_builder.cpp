#include "legged_robot_mpc_controller/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.hpp"

#include <stdexcept>
#include <vector>

#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"
#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/centroidal_mpc_config_builder.hpp"

namespace legged_robot_mpc_controller
{

namespace
{

using common::assembleDiagonalMatrix;
using common::checkJointArraySize;
using common::findArmJointIndices;
using common::zeroAtIndices;

/// An empty per-joint array means "this default for every joint".
std::vector<double> perJoint(const std::vector<double>& values, size_t numJoints, double defaultValue, const std::string& name)
{
  if (values.empty()) {
    return std::vector<double>(numJoints, defaultValue);
  }
  checkJointArraySize(values, numJoints, name);
  return values;
}

}  // namespace

ocs2::humanoid::WbIdMpcInterface::Config buildWholeBodyInverseDynamicsMpcConfig(
  const humanoid_centroidal_mpc_controller::Params& params,
  const humanoid_whole_body_inverse_dynamics_mpc_controller::Params& wholeBodyParams)
{
  // Everything shared, built and validated by the centroidal builder.
  const auto shared = buildCentroidalMpcConfig(params);
  const auto& p = params.ocs2;
  const auto& wb = wholeBodyParams.wholeBody;
  const size_t numJoints = params.robot.jointNames.size();

  ocs2::humanoid::WbIdMpcInterface::Config config;
  config.urdfFile = shared.urdfFile;
  config.verbose = shared.verbose;
  config.modelParams = shared.modelParams;
  config.ddpSettings = shared.ddpSettings;
  config.mpcSettings = shared.mpcSettings;
  config.sqpSettings = shared.sqpSettings;
  config.rolloutSettings = shared.rolloutSettings;
  config.initialModeSchedule = shared.initialModeSchedule;
  config.defaultModeSequenceTemplate = shared.defaultModeSequenceTemplate;
  config.swingTrajectoryConfig = shared.swingTrajectoryConfig;
  config.costConstraintConfig = shared.costConstraintConfig;
  config.taskSpaceFootCostWeights = shared.taskSpaceFootCostWeights;
  config.captureFootPlacement = shared.captureFootPlacement;
  config.armSwing = shared.armSwing;
  config.useTerrainHeightEstimate = shared.useTerrainHeightEstimate;
  config.maxTerrainHeightStep = shared.maxTerrainHeightStep;
  for (const auto& cost : shared.taskSpaceCosts) {
    config.taskSpaceCosts.push_back({cost.costName, cost.linkName, cost.weights});
  }

  // Acceleration-level foot constraints, from the same ocs2.model.footConstraint gains.
  const auto& foot = shared.modelParams.footConstraintConfig;
  auto& gains = config.footAccelerationGains;
  gains.positionZ = foot.positionErrorGain_z;
  gains.orientation = foot.orientationErrorGain;
  gains.linearVelocityXY = foot.linearVelocityErrorGain_xy;
  gains.linearVelocityZ = foot.linearVelocityErrorGain_z;
  gains.angularVelocity = foot.angularVelocityErrorGain;
  gains.linearAccelerationXY = foot.linearAccelerationErrorGain_xy;
  gains.linearAccelerationZ = foot.linearAccelerationErrorGain_z;
  gains.angularAcceleration = foot.angularAccelerationErrorGain;

  // Torque window: nodes 0 .. torqueConstraintNodes-1 of the SQP grid. Half a step past
  // the last of them, so a node exactly on the boundary is not decided by rounding.
  config.torqueWindowDuration = wb.torqueConstraintNodes > 0 ?
    (static_cast<double>(wb.torqueConstraintNodes) - 0.5) * shared.sqpSettings.dt : -1.0;
  if (!params.control.torqueLimit.empty()) {
    checkJointArraySize(params.control.torqueLimit, numJoints, "control.torqueLimit");
    config.torqueLimits = common::toVector(params.control.torqueLimit);
  }
  config.torqueLimitBarrierConfig.mu = wb.torqueLimitBarrier.mu;
  config.torqueLimitBarrierConfig.delta = wb.torqueLimitBarrier.delta;
  config.jointVelocityLimitScale = wb.jointVelocityLimit.scale;
  config.jointVelocityLimitBarrierConfig.mu = wb.jointVelocityLimit.mu;
  config.jointVelocityLimitBarrierConfig.delta = wb.jointVelocityLimit.delta;

  // --- costs in the whole-body dimensions ---
  // x = [base pose (6), joints (nj), base twist (6), joint velocities (nj)]
  // The base pose and twist are tracked by BaseMotionTrackingCost (gauge-aware: x, y and
  // yaw compared in the pelvis frame), so their blocks of Q are zero, as the base pose
  // block is in the centroidal MPC. The arm joints are owned by JointTrackingCost.
  const std::vector<size_t> armJointIndices = findArmJointIndices(params.robot.jointNames);
  const std::vector<double> zeros6(6, 0.0);
  const auto qJointVelocities = perJoint(wb.costs.q.jointVelocities, numJoints, 0.0, "wholeBody.costs.q.jointVelocities");
  const auto qFinalJointVelocities =
    perJoint(wb.costs.qFinal.jointVelocities, numJoints, 0.0, "wholeBody.costs.qFinal.jointVelocities");

  auto& costs = config.costConstraintConfig;
  costs.Q = assembleDiagonalMatrix(
    {zeros6, zeroAtIndices(p.costs.q.jointPositions, armJointIndices), zeros6, qJointVelocities}, p.costs.qScaling);
  costs.QFinal = assembleDiagonalMatrix(
    {zeros6, zeroAtIndices(p.costs.qFinal.jointPositions, armJointIndices), zeros6, qFinalJointVelocities}, p.costs.qScaling);
  costs.baseMotionQ = assembleDiagonalMatrix({p.costs.q.basePose, wb.costs.q.baseVelocity}, p.costs.qScaling);
  costs.baseMotionQFinal = assembleDiagonalMatrix({p.costs.qFinal.basePose, wb.costs.qFinal.baseVelocity}, p.costs.qScaling);

  // u = [W_l, W_r, a_base (6), a_joints (nj), tau (nj)]
  costs.R = assembleDiagonalMatrix(
    {p.costs.r.contactWrenches, wb.costs.r.baseAccelerations,
     perJoint(wb.costs.r.jointAccelerations, numJoints, 0.005, "wholeBody.costs.r.jointAccelerations"),
     perJoint(wb.costs.r.jointTorques, numJoints, 0.1, "wholeBody.costs.r.jointTorques")},
    p.costs.rScaling);

  // --- initial state: a construction-time placeholder, as for the centroidal MPC ---
  // The controller overwrites every element from the hardware before the first solve.
  const auto nv = static_cast<Eigen::Index>(6 + numJoints);
  config.initialState = ocs2::vector_t::Zero(2 * nv);
  config.initialState(2) = p.reference.defaultBaseHeight;
  config.initialState.segment(6, static_cast<Eigen::Index>(numJoints)) = shared.referenceJointState;

  return config;
}

}  // namespace legged_robot_mpc_controller
