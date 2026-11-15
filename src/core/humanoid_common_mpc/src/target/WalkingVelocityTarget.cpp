/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_common_mpc/target/WalkingVelocityTarget.h"

#include <utility>
#include <algorithm>
#include <cmath>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WalkingVelocityTarget::WalkingVelocityTarget(const ReferenceConfig& referenceConfig, Generator generator)
    : generator_(std::move(generator)),
      maxDisplacementVelocityX_(referenceConfig.maxDisplacementVelocityX),
      maxDisplacementVelocityY_(referenceConfig.maxDisplacementVelocityY),
      maxRotationVelocity_(referenceConfig.maxRotationVelocity),
      minBaseHeight_(referenceConfig.minBaseHeight),
      maxBaseHeight_(referenceConfig.maxBaseHeight),
      commandRateLimits_(referenceConfig.commandRateLimits),
      takeoverHeightRate_(referenceConfig.takeoverHeightRate),
      previousCommand_(0.0, 0.0, referenceConfig.defaultBaseHeight, 0.0),
      velocityCommandFilter_(5, vector4_t(0.0, 0.0, referenceConfig.defaultBaseHeight, 0.0)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WalkingVelocityTarget::setCommand(const WalkingVelocityCommand& command) {
  std::lock_guard<std::mutex> lock(commandMutex_);
  command_ = command;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WalkingVelocityCommand WalkingVelocityTarget::scaleCommand(WalkingVelocityCommand command) const {
  command.linear_velocity_x *= maxDisplacementVelocityX_;
  command.linear_velocity_y *= maxDisplacementVelocityY_;
  command.angular_velocity_z *= maxRotationVelocity_;
  // The commanded height, not the conditioned one: a reset to a measured height outside the range (a robot
  // standing up from the floor) still ramps from there at the height rate limit.
  command.desired_pelvis_height = std::clamp(command.desired_pelvis_height, minBaseHeight_, maxBaseHeight_);
  return command;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WalkingVelocityTarget::Output WalkingVelocityTarget::evaluate(scalar_t initTime, scalar_t finalTime, const vector_t& initState) {
  WalkingVelocityCommand latestCommand;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    latestCommand = command_;
  }

  WalkingVelocityCommand scaledCommand = scaleCommand(latestCommand);
  if (filterResetRequested_.exchange(false, std::memory_order_acq_rel) ||
      !std::isfinite(previousTime_) || initTime < previousTime_) {
    previousCommand_ = scaledCommand.toVector();
    // Re-enter velocity mode from rest, without a step to a buffered nonzero command.
    for (const int i : {0, 1, 3}) {
      if (commandRateLimits_(i) > 0.0) previousCommand_(i) = 0.0;
    }
    const scalar_t resetHeight = resetHeight_.exchange(std::numeric_limits<scalar_t>::quiet_NaN());
    risingFromResetHeight_ = false;
    if (std::isfinite(resetHeight) && (commandRateLimits_(2) > 0.0 || takeoverHeightRate_ > 0.0)) {
      previousCommand_(2) = resetHeight;
      risingFromResetHeight_ = takeoverHeightRate_ > 0.0;
    }
    previousTime_ = initTime;
    velocityCommandFilter_.reset(previousCommand_);
  }
  vector4_t conditionedCommand = velocityCommandFilter_.getFilteredVector(scaledCommand.toVector(), initTime);
  // Do not turn a solver stall into an arbitrarily large reference step.
  const scalar_t dt = std::clamp(initTime - previousTime_, 0.0, 0.05);
  for (int i = 0; i < 4; ++i) {
    const scalar_t rateLimit = (i == 2 && risingFromResetHeight_) ? takeoverHeightRate_ : commandRateLimits_(i);
    if (rateLimit > 0.0) {
      const scalar_t increment = rateLimit * dt;
      const scalar_t unlimited = conditionedCommand(i);
      conditionedCommand(i) = std::clamp(unlimited, previousCommand_(i) - increment, previousCommand_(i) + increment);
      // The takeover rise ends once the height reference reaches the command without being limited.
      if (i == 2 && risingFromResetHeight_ && dt > 0.0 && conditionedCommand(i) == unlimited) risingFromResetHeight_ = false;
    }
  }
  previousCommand_ = conditionedCommand;
  previousTime_ = initTime;

  Output output;
  output.conditionedCommand = conditionedCommand;
  output.targetTrajectories = generator_(conditionedCommand, initTime, finalTime, initState);
  return output;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WalkingVelocityTarget::requestFilterReset(scalar_t initialHeight) {
  resetHeight_.store(initialHeight);
  filterResetRequested_.store(true, std::memory_order_release);
}

}  // namespace ocs2::humanoid
