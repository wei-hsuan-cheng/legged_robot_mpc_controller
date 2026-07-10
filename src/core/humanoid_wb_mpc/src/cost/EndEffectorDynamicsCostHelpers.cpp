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

#include "humanoid_wb_mpc/cost/EndEffectorDynamicsCostHelpers.h"

#include <iostream>

namespace ocs2::humanoid {

VECTOR18_T<scalar_t> EndEffectorDynamicsWeights::toVector() {
  VECTOR18_T<scalar_t> weightVector;
  weightVector << contactPositionErrorWeight, contactOrientationErrorWeight, contactLinearVelocityErrorWeight,
      contactAngularVelocityErrorWeight, contactLinearAccelerationErrorWeight, contactAngularAccelerationErrorWeight;
  return weightVector;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsWeights EndEffectorDynamicsWeights::fromVector(const VECTOR18_T<scalar_t>& weights, bool verbose) {
  if (verbose) {
    std::cerr << "\n #### Humanoid End Effector Foot Cost Weights: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "[pos, orientation, lin_vel, ang_vel, lin_acc, ang_acc]: " << weights.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  EndEffectorDynamicsWeights w;
  w.contactPositionErrorWeight = weights.segment<3>(0);
  w.contactOrientationErrorWeight = weights.segment<3>(3);
  w.contactLinearVelocityErrorWeight = weights.segment<3>(6);
  w.contactAngularVelocityErrorWeight = weights.segment<3>(9);
  w.contactLinearAccelerationErrorWeight = weights.segment<3>(12);
  w.contactAngularAccelerationErrorWeight = weights.segment<3>(15);

  return w;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR18_T<SCALAR_T> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<SCALAR_T>& current,
                                            const EndEffectorDynamicsCostElement<SCALAR_T>& reference) {
  const VECTOR3_T<SCALAR_T> orientationError = quaternionDistance<SCALAR_T>(current.getOrientation(), reference.getOrientation());

  VECTOR18_T<SCALAR_T> errors;
  errors << (current.getPosition() - reference.getPosition()), orientationError,
      (current.getLinearVelocity() - reference.getLinearVelocity()), (current.getAngularVelocity() - reference.getAngularVelocity()),
      (current.getLinearAcceleration() - reference.getLinearAcceleration()),
      (current.getAngularAcceleration() - reference.getAngularAcceleration());
  return errors;
}
template VECTOR18_T<scalar_t> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<scalar_t>& current,
                                                     const EndEffectorDynamicsCostElement<scalar_t>& reference);
template VECTOR18_T<ad_scalar_t> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<ad_scalar_t>& current,
                                                        const EndEffectorDynamicsCostElement<ad_scalar_t>& reference);

}  // namespace ocs2::humanoid
