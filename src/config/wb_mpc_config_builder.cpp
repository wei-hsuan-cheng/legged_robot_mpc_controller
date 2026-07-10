#include "legged_robot_mpc_controller/config/wb_mpc_config_builder.hpp"

#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>

namespace legged_robot_mpc_controller
{

namespace
{

using ocs2::matrix_t;
using ocs2::scalar_t;
using ocs2::vector3_t;
using ocs2::vector_t;

vector_t toVector(const std::vector<double>& values)
{
  return Eigen::Map<const vector_t>(values.data(), static_cast<Eigen::Index>(values.size()));
}

/// Assembles the diagonal state / input cost matrix from its per-block diagonals.
matrix_t assembleDiagonalMatrix(const std::vector<std::vector<double>>& blocks, double scaling)
{
  Eigen::Index dim = 0;
  for (const auto& block : blocks) {
    dim += static_cast<Eigen::Index>(block.size());
  }
  vector_t diagonal(dim);
  Eigen::Index offset = 0;
  for (const auto& block : blocks) {
    diagonal.segment(offset, static_cast<Eigen::Index>(block.size())) = toVector(block);
    offset += static_cast<Eigen::Index>(block.size());
  }
  return (scaling * diagonal).asDiagonal();
}

void checkJointArraySize(const std::vector<double>& values, size_t numJoints, const std::string& name)
{
  if (values.size() != numJoints) {
    throw std::invalid_argument(
      "[wb_mpc_config_builder] " + name + " has " + std::to_string(values.size()) +
      " entries but robot.jointNames has " + std::to_string(numJoints));
  }
}

}  // namespace

ocs2::humanoid::WBMpcInterface::Config buildWbMpcConfig(const humanoid_wb_mpc_controller::Params& params)
{
  const auto& p = params.ocs2;
  const size_t numJoints = params.robot.jointNames.size();

  ocs2::humanoid::WBMpcInterface::Config config;
  config.urdfFile = params.paths.urdfFile;
  config.verbose = p.interface.verbose;

  // --- model settings ---
  auto& model = config.modelParams;
  model.robotName = p.model.robotName;
  model.verboseCppAd = p.model.verboseCppAd;
  model.recompileLibrariesCppAd = p.model.recompileLibrariesCppAd;
  model.phaseTransitionStanceTime = p.model.phaseTransitionStanceTime;
  model.fixedJointNames = p.model.fixedJointNames;
  model.contactNames6DoF = p.model.contactNames6DoF;
  model.contactParentJointNames = p.model.contactParentJointNames;
  if (p.model.armJointNames.size() != 4) {
    throw std::invalid_argument(
      "[wb_mpc_config_builder] model.armJointNames must contain exactly 4 entries "
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

  // ddpSettings: kept at library defaults, the whole-body MPC runs with the SQP backend.

  // --- gait / mode schedule ---
  config.initialModeSchedule =
    ocs2::humanoid::modeScheduleFromStrings(p.gait.initialEventTimes, p.gait.initialModeSequence);
  config.defaultModeSequenceTemplate =
    ocs2::humanoid::modeSequenceTemplateFromStrings(p.gait.defaultSwitchingTimes, p.gait.defaultModeSequence);

  // --- swing trajectory ---
  auto& swing = config.swingTrajectoryConfig;
  swing.liftOffVelocity = p.swingTrajectory.liftOffVelocity;
  swing.touchDownVelocity = p.swingTrajectory.touchDownVelocity;
  swing.swingHeight = p.swingTrajectory.swingHeight;
  swing.touchDownHeightOffset = p.swingTrajectory.touchDownHeightOffset;
  swing.swingTimeScale = p.swingTrajectory.swingTimeScale;
  swing.impactProximityFactorLiftOffVelocity = p.swingTrajectory.impactProximityFactorLiftOffVelocity;
  swing.impactProximityFactorTouchDownVelocity = p.swingTrajectory.impactProximityFactorTouchDownVelocity;
  swing.impactProximityFactorMidPointValue = p.swingTrajectory.impactProximityFactorMidPointValue;

  // --- initial state: [base pose (6), joint positions, base velocity (6), joint velocities] ---
  checkJointArraySize(p.initialState.jointPositions, numJoints, "initialState.jointPositions");
  checkJointArraySize(p.initialState.jointVelocities, numJoints, "initialState.jointVelocities");
  config.initialState = vector_t::Zero(static_cast<Eigen::Index>(12 + 2 * numJoints));
  config.initialState << toVector(p.initialState.basePose), toVector(p.initialState.jointPositions),
    toVector(p.initialState.baseVelocity), toVector(p.initialState.jointVelocities);

  // --- costs ---
  checkJointArraySize(p.costs.q.jointPositions, numJoints, "costs.q.jointPositions");
  checkJointArraySize(p.costs.q.jointVelocities, numJoints, "costs.q.jointVelocities");
  checkJointArraySize(p.costs.qFinal.jointPositions, numJoints, "costs.qFinal.jointPositions");
  checkJointArraySize(p.costs.qFinal.jointVelocities, numJoints, "costs.qFinal.jointVelocities");
  checkJointArraySize(p.costs.r.jointAccelerations, numJoints, "costs.r.jointAccelerations");

  auto& costs = config.costConstraintConfig;
  costs.Q = assembleDiagonalMatrix(
    {p.costs.q.basePose, p.costs.q.jointPositions, p.costs.q.baseVelocity, p.costs.q.jointVelocities},
    p.costs.qScaling);
  costs.QFinal = assembleDiagonalMatrix(
    {p.costs.qFinal.basePose, p.costs.qFinal.jointPositions, p.costs.qFinal.baseVelocity,
     p.costs.qFinal.jointVelocities},
    p.costs.qScaling);
  costs.terminalCostScaling = p.costs.terminalCostScaling;
  costs.R = assembleDiagonalMatrix({p.costs.r.contactWrenches, p.costs.r.jointAccelerations}, p.costs.rScaling);

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

  // --- task-space foot tracking cost ---
  config.taskSpaceFootCostWeights = ocs2::humanoid::EndEffectorDynamicsWeights::fromVector(
    Eigen::Map<const Eigen::Matrix<scalar_t, 18, 1>>(p.costs.taskSpaceFootCostWeights.data()), config.verbose);

  return config;
}

ocs2::humanoid::ReferenceConfig buildReferenceConfig(const humanoid_wb_mpc_controller::Params& params)
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
  config.defaultJointState = toVector(r.defaultJointState);
  return config;
}

ocs2::humanoid::GaitMap loadGaitMap(const std::string& gaitFile)
{
  const YAML::Node root = YAML::LoadFile(gaitFile);
  if (!root["list"]) {
    throw std::runtime_error("[wb_mpc_config_builder] gait file has no 'list' key: " + gaitFile);
  }

  ocs2::humanoid::GaitMap gaitMap;
  for (const auto& gaitNameNode : root["list"]) {
    const auto gaitName = gaitNameNode.as<std::string>();
    const YAML::Node gait = root[gaitName];
    if (!gait || !gait["modeSequence"] || !gait["switchingTimes"]) {
      throw std::runtime_error("[wb_mpc_config_builder] gait '" + gaitName + "' is missing in " + gaitFile);
    }
    const auto modeSequence = gait["modeSequence"].as<std::vector<std::string>>();
    const auto switchingTimes = gait["switchingTimes"].as<std::vector<double>>();
    gaitMap.insert({gaitName, ocs2::humanoid::modeSequenceTemplateFromStrings(switchingTimes, modeSequence)});
  }
  return gaitMap;
}

}  // namespace legged_robot_mpc_controller
