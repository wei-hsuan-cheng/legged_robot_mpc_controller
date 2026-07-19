#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/centroidal_mpc_config_builder.hpp"

#include <stdexcept>
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
using ocs2::vector2_t;
using ocs2::vector3_t;
using ocs2::vector_t;

}  // namespace

ocs2::humanoid::CentroidalMpcInterface::Config buildCentroidalMpcConfig(
  const humanoid_centroidal_mpc_controller::Params& params)
{
  const auto& p = params.ocs2;
  const size_t numJoints = params.robot.jointNames.size();

  ocs2::humanoid::CentroidalMpcInterface::Config config;
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
      "[centroidal_mpc_config_builder] model.armJointNames must contain exactly 4 entries "
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

  config.centroidalModelType = static_cast<ocs2::CentroidalModelType>(p.model.centroidalModelType);

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

  // ddpSettings: kept at library defaults, the centroidal MPC runs with the SQP backend.

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

  // --- default joint state used to build the centroidal model info (legacy reference.info) ---
  checkJointArraySize(p.reference.defaultJointState, numJoints, "reference.defaultJointState");
  config.referenceJointState = toVector(p.reference.defaultJointState);

  // --- initial state: [normalized momentum (6), base pose (6), joint positions] ---
  checkJointArraySize(p.initialState.jointPositions, numJoints, "initialState.jointPositions");
  config.initialState = vector_t::Zero(static_cast<Eigen::Index>(12 + numJoints));
  config.initialState << toVector(p.initialState.centroidalMomentum), toVector(p.initialState.basePose),
    toVector(p.initialState.jointPositions);

  // --- costs ---
  checkJointArraySize(p.costs.q.jointPositions, numJoints, "costs.q.jointPositions");
  checkJointArraySize(p.costs.qFinal.jointPositions, numJoints, "costs.qFinal.jointPositions");
  checkJointArraySize(p.costs.r.jointVelocities, numJoints, "costs.r.jointVelocities");

  // Arm joints (shoulder/elbow) are tracked by the dedicated JointTrackingCost, so
  // move their weights out of the generic joint-position blocks (behavior-preserving).
  const std::vector<size_t> armJointIndices = findArmJointIndices(params.robot.jointNames);
  const std::vector<double> qJointsGeneric = zeroAtIndices(p.costs.q.jointPositions, armJointIndices);
  const std::vector<double> qFinalJointsGeneric = zeroAtIndices(p.costs.qFinal.jointPositions, armJointIndices);

  auto& costs = config.costConstraintConfig;
  costs.Q = assembleDiagonalMatrix(
    {p.costs.q.centroidalMomentum, std::vector<double>(6, 0.0), qJointsGeneric}, p.costs.qScaling);
  costs.QFinal = assembleDiagonalMatrix(
    {p.costs.qFinal.centroidalMomentum, std::vector<double>(6, 0.0), qFinalJointsGeneric},
    p.costs.qScaling);
  costs.baseMotionQ = assembleDiagonalMatrix(
    {p.costs.q.basePose, std::vector<double>(6, 0.0)}, p.costs.qScaling);
  costs.baseMotionQFinal = assembleDiagonalMatrix(
    {p.costs.qFinal.basePose, std::vector<double>(6, 0.0)}, p.costs.qScaling);
  costs.armJointIndices = armJointIndices;
  costs.armJointQ = assembleDiagonalMatrix(
    {selectAtIndices(p.costs.q.jointPositions, armJointIndices)}, p.costs.qScaling);
  costs.armJointQFinal = assembleDiagonalMatrix(
    {selectAtIndices(p.costs.qFinal.jointPositions, armJointIndices)}, p.costs.qScaling);
  costs.terminalCostScaling = p.costs.terminalCostScaling;
  costs.R = assembleDiagonalMatrix({p.costs.r.contactWrenches, p.costs.r.jointVelocities}, p.costs.rScaling);

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

  // --- leg external-torque quadratic costs (legacy left/right_leg_torque_cost) ---
  costs.leftLegTorqueCostConfig.activeJointNames = p.costs.legTorqueCost.leftActiveJointNames;
  costs.leftLegTorqueCostConfig.weights =
    p.costs.legTorqueCost.scaling * toVector(p.costs.legTorqueCost.leftWeights);
  costs.rightLegTorqueCostConfig.activeJointNames = p.costs.legTorqueCost.rightActiveJointNames;
  costs.rightLegTorqueCostConfig.weights =
    p.costs.legTorqueCost.scaling * toVector(p.costs.legTorqueCost.rightWeights);

  // --- task-space kinematics costs ---
  if (p.costs.taskSpaceFootCostWeights.size() != 12) {
    throw std::invalid_argument(
      "[centroidal_mpc_config_builder] costs.taskSpaceFootCostWeights must have 12 entries");
  }
  config.taskSpaceFootCostWeights = ocs2::humanoid::EndEffectorKinematicsWeights::fromVector(
    Eigen::Map<const Eigen::Matrix<scalar_t, 12, 1>>(p.costs.taskSpaceFootCostWeights.data()),
    config.verbose);

  config.icpCostWeights = vector2_t::Constant(p.costs.icpErrorWeight);

  for (const auto& costName : p.costs.taskSpaceCosts.names) {
    if (costName.empty()) {
      continue;
    }
    const auto& entry = p.costs.taskSpaceCosts.names_map.at(costName);
    if (entry.weights.size() != 12) {
      throw std::invalid_argument(
        "[centroidal_mpc_config_builder] taskSpaceCosts." + costName + ".weights must have 12 entries");
    }
    ocs2::humanoid::CentroidalMpcInterface::TaskSpaceCostConfig taskSpaceCost;
    taskSpaceCost.costName = costName;
    taskSpaceCost.linkName = entry.linkName;
    taskSpaceCost.weights = ocs2::humanoid::EndEffectorKinematicsWeights::fromVector(
      Eigen::Map<const Eigen::Matrix<scalar_t, 12, 1>>(entry.weights.data()), config.verbose);
    config.taskSpaceCosts.push_back(std::move(taskSpaceCost));
  }

  if (p.costs.frameRelationTracking.sourceFrames.size() !=
      p.costs.frameRelationTracking.targetFrames.size()) {
    throw std::invalid_argument(
      "[centroidal_mpc_config_builder] frameRelationTracking.sourceFrames/targetFrames must be "
      "index-aligned");
  }
  config.frameRelationSourceFrames = p.costs.frameRelationTracking.sourceFrames;
  config.frameRelationTargetFrames = p.costs.frameRelationTracking.targetFrames;
  config.frameRelationDefaultWeights = ocs2::humanoid::EndEffectorKinematicsWeights::fromVector(
    Eigen::Map<const Eigen::Matrix<scalar_t, 12, 1>>(p.costs.frameRelationTracking.weights.data()),
    config.verbose);

  return config;
}

}  // namespace legged_robot_mpc_controller
