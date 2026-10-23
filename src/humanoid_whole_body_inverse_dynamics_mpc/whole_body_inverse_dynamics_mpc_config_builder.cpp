#include "legged_robot_mpc_controller/humanoid_whole_body_inverse_dynamics_mpc/whole_body_inverse_dynamics_mpc_config_builder.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>

#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"

namespace legged_robot_mpc_controller
{

namespace
{

using common::assembleDiagonalMatrix;
using common::checkJointArraySize;
using common::findArmJointIndices;
using common::selectAtIndices;
using common::toVector;
using common::zeroAtIndices;
using ocs2::scalar_t;
using ocs2::vector3_t;
using ocs2::vector_t;

std::vector<double> perJoint(const std::vector<double>& values, size_t count, double defaultValue, const std::string& name)
{
  if (values.empty()) {
    return std::vector<double>(count, defaultValue);
  }
  checkJointArraySize(values, count, name);
  return values;
}

}  // namespace

ocs2::humanoid::WbIdMpcInterface::Config buildWholeBodyInverseDynamicsMpcConfig(
  const humanoid_whole_body_inverse_dynamics_mpc_controller::Params& params)
{
  const auto& p = params.ocs2;
  const size_t numJoints = params.robot.jointNames.size();

  ocs2::humanoid::WbIdMpcInterface::Config config;
  config.urdfFile = params.paths.urdfFile;
  config.verbose = p.interface.verbose;

  // --- model settings ---
  auto& model = config.modelParams;
  model.robotName = p.model.robotName;
  model.verboseCppAd = p.model.verboseCppAd;
  model.recompileLibrariesCppAd = p.model.recompileLibrariesCppAd;
  model.modelFolderCppAd = params.paths.libFolder;
  model.phaseTransitionStanceTime = p.model.phaseTransitionStanceTime;
  model.fixedJointNames = p.model.fixedJointNames;
  model.contactNames6DoF = p.model.contactNames6DoF;
  model.contactParentJointNames = p.model.contactParentJointNames;
  if (p.model.armJointNames.size() != 4) {
    throw std::invalid_argument(
      "[whole_body_inverse_dynamics_mpc_config_builder] model.armJointNames must contain exactly 4 entries "
      "[left_shoulder_y, right_shoulder_y, left_elbow_y, right_elbow_y]");
  }
  model.j_l_shoulder_y_name = p.model.armJointNames[0];
  model.j_r_shoulder_y_name = p.model.armJointNames[1];
  model.j_l_elbow_y_name = p.model.armJointNames[2];
  model.j_r_elbow_y_name = p.model.armJointNames[3];

  auto& foot = model.footConstraintConfig;
  foot.positionErrorGain_z = p.model.footConstraint.positionErrorGain_z;
  foot.orientationErrorGain = p.model.footConstraint.orientationErrorGain;
  foot.linearVelocityErrorGain_z = p.model.footConstraint.linearVelocityErrorGain_z;
  foot.linearVelocityErrorGain_xy = p.model.footConstraint.linearVelocityErrorGain_xy;
  foot.angularVelocityErrorGain = p.model.footConstraint.angularVelocityErrorGain;
  foot.linearAccelerationErrorGain_z = p.model.footConstraint.linearAccelerationErrorGain_z;
  foot.linearAccelerationErrorGain_xy = p.model.footConstraint.linearAccelerationErrorGain_xy;
  foot.angularAccelerationErrorGain = p.model.footConstraint.angularAccelerationErrorGain;

  model.contactFrameTranslation =
    vector3_t(p.constraints.contact.frameTranslation[0], p.constraints.contact.frameTranslation[1],
              p.constraints.contact.frameTranslation[2]);
  // yaml order: [x_max, x_min, y_max, y_min]
  model.contactRectangleXMax = p.constraints.contact.rectangle[0];
  model.contactRectangleXMin = p.constraints.contact.rectangle[1];
  model.contactRectangleYMax = p.constraints.contact.rectangle[2];
  model.contactRectangleYMin = p.constraints.contact.rectangle[3];

  // --- solver settings ---
  auto& sqp = config.sqpSettings;
  sqp.nThreads = static_cast<size_t>(p.sqp.nThreads);
  sqp.dt = p.sqp.dt;
  sqp.sqpIteration = static_cast<size_t>(p.sqp.sqpIteration);
  sqp.deltaTol = p.sqp.deltaTol;
  sqp.g_max = p.sqp.g_max;
  sqp.g_min = p.sqp.g_min;
  sqp.inequalityConstraintMu = p.sqp.inequalityConstraintMu;
  sqp.inequalityConstraintDelta = p.sqp.inequalityConstraintDelta;
  sqp.projectStateInputEqualityConstraints = p.sqp.projectStateInputEqualityConstraints;
  sqp.printSolverStatistics = p.sqp.printSolverStatistics;
  sqp.printSolverStatus = p.sqp.printSolverStatus;
  sqp.printLinesearch = p.sqp.printLinesearch;
  sqp.useFeedbackPolicy = p.sqp.useFeedbackPolicy;
  sqp.integratorType = ocs2::sensitivity_integrator::fromString(p.sqp.integratorType);
  sqp.threadPriority = p.sqp.threadPriority;

  auto& mpc = config.mpcSettings;
  mpc.timeHorizon_ = p.mpc.timeHorizon;
  mpc.solutionTimeWindow_ = p.mpc.solutionTimeWindow;
  mpc.coldStart_ = p.mpc.coldStart;
  mpc.debugPrint_ = p.mpc.debugPrint;
  mpc.mpcDesiredFrequency_ = static_cast<scalar_t>(p.mpc.mpcDesiredFrequency);
  mpc.mrtDesiredFrequency_ = static_cast<scalar_t>(p.mpc.mrtDesiredFrequency);

  auto& rollout = config.rolloutSettings;
  rollout.absTolODE = p.rollout.AbsTolODE;
  rollout.relTolODE = p.rollout.RelTolODE;
  rollout.timeStep = p.rollout.timeStep;
  rollout.integratorType = ocs2::integrator_type::fromString(p.rollout.integratorType);
  rollout.maxNumStepsPerSecond = static_cast<size_t>(p.rollout.maxNumStepsPerSecond);
  rollout.checkNumericalStability = p.rollout.checkNumericalStability;

  // WB-ID uses the SQP backend.

  // --- gait / mode schedule ---
  config.initialModeSchedule =
    ocs2::humanoid::modeScheduleFromStrings(p.gait.initialEventTimes, p.gait.initialModeSequence);
  config.defaultModeSequenceTemplate =
    ocs2::humanoid::modeSequenceTemplateFromStrings(p.gait.defaultSwitchingTimes, p.gait.defaultModeSequence);

  // --- swing trajectory ---
  auto& swing = config.swingTrajectoryConfig;
  swing.liftOffVelocity = p.gait.swingTrajectory.liftOffVelocity;
  swing.touchDownVelocity = p.gait.swingTrajectory.touchDownVelocity;
  swing.swingHeight = p.gait.swingTrajectory.swingHeight;
  swing.touchDownHeightOffset = p.gait.swingTrajectory.touchDownHeightOffset;
  swing.swingTimeScale = p.gait.swingTrajectory.swingTimeScale;
  swing.impactProximityFactorLiftOffVelocity = p.gait.swingTrajectory.impactProximityFactorLiftOffVelocity;
  swing.impactProximityFactorTouchDownVelocity = p.gait.swingTrajectory.impactProximityFactorTouchDownVelocity;
  swing.impactProximityFactorMidPointValue = p.gait.swingTrajectory.impactProximityFactorMidPointValue;

  // x = [q, v]. The controller replaces this construction-time state from sensors.
  checkJointArraySize(p.reference.defaultJointState, numJoints, "reference.defaultJointState");
  const auto nv = static_cast<Eigen::Index>(6 + numJoints);
  config.initialState = vector_t::Zero(2 * nv);
  config.initialState(2) = p.reference.defaultBaseHeight;
  config.initialState.segment(6, static_cast<Eigen::Index>(numJoints)) = toVector(p.reference.defaultJointState);

  // Acceleration-level stance/swing constraints in generalized coordinates.
  auto& gains = config.footAccelerationGains;
  gains.positionZ = foot.positionErrorGain_z;
  gains.orientation = foot.orientationErrorGain;
  gains.linearVelocityXY = foot.linearVelocityErrorGain_xy;
  gains.linearVelocityZ = foot.linearVelocityErrorGain_z;
  gains.angularVelocity = foot.angularVelocityErrorGain;
  gains.linearAccelerationXY = foot.linearAccelerationErrorGain_xy;
  gains.linearAccelerationZ = foot.linearAccelerationErrorGain_z;
  gains.angularAcceleration = foot.angularAccelerationErrorGain;

  config.torqueWindowDuration = p.model.torqueConstraintNodes > 0 ?
    (static_cast<double>(p.model.torqueConstraintNodes) - 0.5) * p.sqp.dt : -1.0;
  if (!params.control.torqueLimit.empty()) {
    checkJointArraySize(params.control.torqueLimit, numJoints, "control.torqueLimit");
    config.torqueLimits = toVector(params.control.torqueLimit);
  }
  config.torqueLimitBarrierConfig.mu = p.constraints.jointTorqueLimits.mu;
  config.torqueLimitBarrierConfig.delta = p.constraints.jointTorqueLimits.delta;
  config.jointVelocityLimitScale = p.constraints.jointVelocityLimits.scale;
  config.jointVelocityLimitBarrierConfig.mu = p.constraints.jointVelocityLimits.mu;
  config.jointVelocityLimitBarrierConfig.delta = p.constraints.jointVelocityLimits.delta;

  // --- costs ---
  checkJointArraySize(p.costs.q.jointPositions, numJoints, "costs.q.jointPositions");
  checkJointArraySize(p.costs.qFinal.jointPositions, numJoints, "costs.qFinal.jointPositions");

  // Arm joints (shoulder/elbow) are tracked by the dedicated JointTrackingCost, so
  // move their weights out of the generic joint-position blocks (behavior-preserving).
  const std::vector<size_t> armJointIndices = findArmJointIndices(params.robot.jointNames);
  const std::vector<double> qJointsGeneric = zeroAtIndices(p.costs.q.jointPositions, armJointIndices);
  const std::vector<double> qFinalJointsGeneric = zeroAtIndices(p.costs.qFinal.jointPositions, armJointIndices);

  // The single configured Q/Qf is split across model-aware evaluators without
  // double counting: base pose/velocity, arm posture, and the remaining state.
  const std::vector<double> zeros6(6, 0.0);
  const auto qJointVelocities = perJoint(p.costs.q.jointVelocities, numJoints, 0.0, "costs.q.jointVelocities");
  const auto qFinalJointVelocities = perJoint(p.costs.qFinal.jointVelocities, numJoints, 0.0, "costs.qFinal.jointVelocities");
  auto& costs = config.costConstraintConfig;
  costs.Q = assembleDiagonalMatrix({zeros6, qJointsGeneric, zeros6, qJointVelocities}, p.costs.qScaling);
  costs.QFinal = assembleDiagonalMatrix({zeros6, qFinalJointsGeneric, zeros6, qFinalJointVelocities}, p.costs.qScaling);
  costs.baseMotionQ = assembleDiagonalMatrix({p.costs.q.basePose, p.costs.q.baseVelocity}, p.costs.qScaling);
  costs.baseMotionQFinal = assembleDiagonalMatrix({p.costs.qFinal.basePose, p.costs.qFinal.baseVelocity}, p.costs.qScaling);
  costs.armJointIndices = armJointIndices;
  costs.armJointQ = assembleDiagonalMatrix(
    {selectAtIndices(p.costs.q.jointPositions, armJointIndices)}, p.costs.qScaling);
  costs.armJointQFinal = assembleDiagonalMatrix(
    {selectAtIndices(p.costs.qFinal.jointPositions, armJointIndices)}, p.costs.qScaling);
  costs.terminalCostScaling = p.costs.terminalCostScaling;
  // u = [W_left, W_right, a_base, a_joints, tau_joints].
  costs.R = assembleDiagonalMatrix(
    {p.costs.r.contactWrenches, p.costs.r.baseAccelerations,
     perJoint(p.costs.r.jointAccelerations, numJoints, 0.005, "costs.r.jointAccelerations"),
     perJoint(p.costs.r.jointTorques, numJoints, 0.1, "costs.r.jointTorques")}, p.costs.rScaling);

  // --- constraints ---
  costs.footCollisionConfig.leftAnkleFrame = p.constraints.footCollision.leftAnkleFrame;
  costs.footCollisionConfig.rightAnkleFrame = p.constraints.footCollision.rightAnkleFrame;
  costs.footCollisionConfig.footCollisionSphereRadius = p.constraints.footCollision.footCollisionSphereRadius;
  costs.footCollisionConfig.leftKneeFrame = p.constraints.footCollision.leftKneeFrame;
  costs.footCollisionConfig.rightKneeFrame = p.constraints.footCollision.rightKneeFrame;
  costs.footCollisionConfig.kneeCollisionSphereRadius = p.constraints.footCollision.kneeCollisionSphereRadius;
  costs.footCollisionBarrierConfig.mu = p.constraints.footCollision.mu;
  costs.footCollisionBarrierConfig.delta = p.constraints.footCollision.delta;

  costs.jointLimitsBarrierConfig.mu = p.constraints.jointLimits.mu;
  costs.jointLimitsBarrierConfig.delta = p.constraints.jointLimits.delta;

  costs.contactMomentBarrierConfig.mu = p.constraints.contactMomentXY.mu;
  costs.contactMomentBarrierConfig.delta = p.constraints.contactMomentXY.delta;

  costs.frictionCoefficient = p.constraints.frictionCone.frictionCoefficient;
  costs.frictionBarrierConfig.mu = p.constraints.frictionCone.mu;
  costs.frictionBarrierConfig.delta = p.constraints.frictionCone.delta;

  // --- optional per-leg external-load regularization (J_leg^T W) ---
  costs.leftLegTorqueCostConfig.activeJointNames = p.costs.legTorqueCost.leftActiveJointNames;
  costs.leftLegTorqueCostConfig.weights =
    p.costs.legTorqueCost.scaling * toVector(p.costs.legTorqueCost.leftWeights);
  costs.rightLegTorqueCostConfig.activeJointNames = p.costs.legTorqueCost.rightActiveJointNames;
  costs.rightLegTorqueCostConfig.weights =
    p.costs.legTorqueCost.scaling * toVector(p.costs.legTorqueCost.rightWeights);

  // --- task-space kinematics costs ---
  if (p.costs.taskSpaceFootCostWeights.size() != 12) {
    throw std::invalid_argument(
      "[whole_body_inverse_dynamics_mpc_config_builder] costs.taskSpaceFootCostWeights must have 12 entries");
  }
  config.taskSpaceFootCostWeights = ocs2::humanoid::EndEffectorKinematicsWeights::fromVector(
    Eigen::Map<const Eigen::Matrix<scalar_t, 12, 1>>(p.costs.taskSpaceFootCostWeights.data()),
    config.verbose);

  // Flat-ground footholds are optimized, not supplied by a capture-point heuristic.
  config.captureFootPlacement.enabled = false;

  config.armSwing.enabled = p.reference.armSwing.enabled;
  config.armSwing.shoulderPitchAmplitude = p.reference.armSwing.shoulderPitchAmplitude;
  config.armSwing.elbowAmplitude = p.reference.armSwing.elbowAmplitude;
  config.armSwing.phaseOffset = p.reference.armSwing.phaseOffset;
  config.armSwing.maxOffset = p.reference.armSwing.maxOffset;

  config.useTerrainHeightEstimate = p.model.useTerrainHeightEstimate;
  config.maxTerrainHeightStep = p.model.maxTerrainHeightStep;

  for (const auto& costName : p.costs.taskSpaceCosts.names) {
    if (costName.empty()) {
      continue;
    }
    const auto& entry = p.costs.taskSpaceCosts.names_map.at(costName);
    if (entry.weights.size() != 12) {
      throw std::invalid_argument(
        "[whole_body_inverse_dynamics_mpc_config_builder] taskSpaceCosts." + costName + ".weights must have 12 entries");
    }
    ocs2::humanoid::WbIdMpcInterface::TaskSpaceCostConfig taskSpaceCost;
    taskSpaceCost.costName = costName;
    taskSpaceCost.linkName = entry.linkName;
    taskSpaceCost.weights = ocs2::humanoid::EndEffectorKinematicsWeights::fromVector(
      Eigen::Map<const Eigen::Matrix<scalar_t, 12, 1>>(entry.weights.data()), config.verbose);
    config.taskSpaceCosts.push_back(std::move(taskSpaceCost));
  }

  return config;
}

}  // namespace legged_robot_mpc_controller
