#include "humanoid_common_mpc/target/BasePoseTarget.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {
namespace {

scalar_t unwrapAround(scalar_t angle, scalar_t reference) {
  return reference + std::remainder(angle - reference, 2.0 * M_PI);
}

}  // namespace

BasePoseTarget::BasePoseTarget(const ReferenceConfig& referenceConfig,
                               const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                               Generator generator)
    : generator_(std::move(generator)),
      mpcRobotModelPtr_(&mpcRobotModel),
      targetDisplacementVelocity_(referenceConfig.targetDisplacementVelocity),
      targetRotationVelocity_(referenceConfig.targetRotationVelocity),
      maxDisplacementVelocityX_(referenceConfig.maxDisplacementVelocityX),
      maxDisplacementVelocityY_(referenceConfig.maxDisplacementVelocityY),
      positionTolerance_(referenceConfig.basePosePositionTolerance),
      orientationTolerance_(referenceConfig.basePoseOrientationTolerance) {
  if (!generator_) {
    throw std::invalid_argument("[BasePoseTarget] target trajectory generator is empty");
  }
}

void BasePoseTarget::setCommand(const BasePoseCommand& command) {
  std::lock_guard<std::mutex> lock(commandMutex_);
  command_ = command;
  externalCommandReceived_ = true;
  currentPoseLatchRequested_ = false;
}

void BasePoseTarget::requestCurrentPoseLatch() {
  std::lock_guard<std::mutex> lock(commandMutex_);
  if (!externalCommandReceived_) {
    currentPoseLatchRequested_ = true;
  }
}

BasePoseTarget::Output BasePoseTarget::evaluate(scalar_t initTime,
                                                scalar_t finalTime,
                                                const vector_t& initState) {
  const vector6_t currentPose = mpcRobotModelPtr_->getBasePose(initState);

  BasePoseCommand command;
  vector6_t latchedTargetPose = vector6_t::Zero();
  bool externalCommandReceived = false;
  bool latchedTargetPoseValid = false;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    if (currentPoseLatchRequested_ && !externalCommandReceived_) {
      latchedTargetPose_ = currentPose;
      latchedTargetPoseValid_ = true;
      currentPoseLatchRequested_ = false;
    }
    command = command_;
    latchedTargetPose = latchedTargetPose_;
    externalCommandReceived = externalCommandReceived_;
    latchedTargetPoseValid = latchedTargetPoseValid_;
  }

  vector6_t targetPose = currentPose;
  if (externalCommandReceived) {
    const vector3_t commandEulerZyx = quaternionToEulerZYX(command.orientation.normalized());
    targetPose.head<3>() = command.position;
    for (Eigen::Index i = 0; i < 3; ++i) {
      targetPose(3 + i) = unwrapAround(commandEulerZyx(i), currentPose(3 + i));
    }
  } else if (latchedTargetPoseValid) {
    targetPose = latchedTargetPose;
  }

  const vector2_t worldPositionError = targetPose.head<2>() - currentPose.head<2>();
  const scalar_t planarDistance = worldPositionError.norm();
  vector2_t worldVelocity = vector2_t::Zero();
  if (planarDistance > positionTolerance_) {
    const scalar_t speed = std::min(targetDisplacementVelocity_, planarDistance);
    worldVelocity = speed * worldPositionError / planarDistance;
  }

  const scalar_t yaw = currentPose(3);
  const scalar_t cosYaw = std::cos(yaw);
  const scalar_t sinYaw = std::sin(yaw);

  Output output;
  output.motionCommand.setZero();
  output.motionCommand(0) = std::clamp(
      cosYaw * worldVelocity(0) + sinYaw * worldVelocity(1),
      -maxDisplacementVelocityX_, maxDisplacementVelocityX_);
  output.motionCommand(1) = std::clamp(
      -sinYaw * worldVelocity(0) + cosYaw * worldVelocity(1),
      -maxDisplacementVelocityY_, maxDisplacementVelocityY_);
  output.motionCommand(2) = targetPose(2);

  const scalar_t yawError = targetPose(3) - currentPose(3);
  if (std::abs(yawError) > orientationTolerance_) {
    output.motionCommand(3) = std::clamp(
        yawError, -targetRotationVelocity_, targetRotationVelocity_);
  }

  output.targetTrajectories = generator_(targetPose, initTime, finalTime, initState);
  return output;
}

}  // namespace ocs2::humanoid
