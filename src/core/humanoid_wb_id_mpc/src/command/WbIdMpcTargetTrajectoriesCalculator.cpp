#include "humanoid_wb_id_mpc/command/WbIdMpcTargetTrajectoriesCalculator.h"

#include <array>
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

WbIdMpcTargetTrajectoriesCalculator::WbIdMpcTargetTrajectoriesCalculator(const ReferenceConfig& referenceConfig,
                                                                         const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                         scalar_t mpcHorizon)
    : TargetTrajectoriesCalculatorBase(referenceConfig, mpcRobotModel, mpcHorizon) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t WbIdMpcTargetTrajectoriesCalculator::makeState(
    const vector6_t& basePose, const vector6_t& baseVelocity, const vector_t& initial) const {
  // [base pose (6), joints (nj), base velocity (6), joint velocities (nj) = 0]
  vector_t state = vector_t::Zero(static_cast<Eigen::Index>(mpcRobotModelPtr_->getStateDim()));
  mpcRobotModelPtr_->setBasePose(state, basePose);
  mpcRobotModelPtr_->setJointAngles(state, targetJointState_);
  static_cast<const WbIdMpcRobotModel<scalar_t>&>(*mpcRobotModelPtr_).setBaseVelocity(state, baseVelocity);
  const auto& model = static_cast<const WbIdMpcRobotModel<scalar_t>&>(*mpcRobotModelPtr_);
  if (model.usesTangentSpace()) model.rebase(state, Eigen::Quaterniond(initial.tail<4>()));
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories WbIdMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories(const vector4_t& commandLineTarget,
                                                                                            scalar_t initTime,
                                                                                            const vector_t& initState) {
  const vector6_t currentPoseTarget = getCurrentBasePoseTarget(initState);
  const vector6_t targetPose = getDeltaBaseTarget(commandLineTarget, currentPoseTarget);
  const scalar_t targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPoseTarget);

  const scalar_array_t timeTrajectory{initTime, targetReachingTime};
  const vector_array_t stateTrajectory{makeState(currentPoseTarget, vector6_t::Zero(), initState), makeState(targetPose, vector6_t::Zero(), initState)};
  const vector_array_t inputTrajectory(2, vector_t::Zero(static_cast<Eigen::Index>(mpcRobotModelPtr_->getInputDim())));
  return TargetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories WbIdMpcTargetTrajectoriesCalculator::commandedBasePoseToTargetTrajectories(const vector6_t& targetBasePose,
                                                                                            scalar_t initTime,
                                                                                            const vector_t& initState) {
  const vector6_t currentPose = mpcRobotModelPtr_->getBasePose(initState);
  const scalar_t targetReachingTime = initTime + estimateTimeToBasePoseTarget(targetBasePose - currentPose);

  const scalar_array_t timeTrajectory{initTime, targetReachingTime};
  const vector_array_t stateTrajectory{makeState(currentPose, vector6_t::Zero(), initState), makeState(targetBasePose, vector6_t::Zero(), initState)};
  const vector_array_t inputTrajectory(2, vector_t::Zero(static_cast<Eigen::Index>(mpcRobotModelPtr_->getInputDim())));
  return TargetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories WbIdMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                                            scalar_t initTime,
                                                                                            const vector_t& initState) {
  // Re-anchored at the current pose every horizon and integrated from the pelvis-frame
  // command only, as in the centroidal calculator: estimator velocity noise must not
  // move the reference itself.
  vector6_t currentPoseTarget = getCurrentBasePoseTarget(initState);
  currentPoseTarget(2) = commandedVelocities(2);
  const scalar_t intermediateTargetTime = 0.7 * mpcHorizon_;
  const vector6_t intermediateTargetPose = integrateBodyTwistTargetBasePose(currentPoseTarget, commandedVelocities, intermediateTargetTime);
  const vector6_t finalTargetPose = integrateBodyTwistTargetBasePose(currentPoseTarget, commandedVelocities, mpcHorizon_);

  const scalar_array_t timeTrajectory{initTime, initTime + intermediateTargetTime, initTime + mpcHorizon_};
  const std::array<vector6_t, 3> poses{currentPoseTarget, intermediateTargetPose, finalTargetPose};
  vector_array_t stateTrajectory;
  stateTrajectory.reserve(poses.size());
  for (const auto& pose : poses) {
    const vector4_t velocityWorld = transformVelCommandToGlobal(commandedVelocities, pose(3));
    vector6_t baseVelocity;
    // Logical reference order: world linear velocity followed by commanded world
    // yaw angular velocity (also the yaw Euler rate for these level references).
    baseVelocity << velocityWorld(0), velocityWorld(1), 0.0, velocityWorld(3), 0.0, 0.0;
    stateTrajectory.push_back(makeState(pose, baseVelocity, initState));
  }

  const vector_array_t inputTrajectory(3, vector_t::Zero(static_cast<Eigen::Index>(mpcRobotModelPtr_->getInputDim())));
  return TargetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};
}

}  // namespace ocs2::humanoid
