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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_centroidal_mpc/cost/ICPCost.h"

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <cmath>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ICPCost::ICPCost(const SwitchedModelReferenceManager& referenceManager,
                 vector2_t weights,
                 const PinocchioInterface& pinocchioInterface,
                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                 std::string costName,
                 const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      sqrtWeights_(weights.cwiseSqrt()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelAdPtr_(mpcRobotModelAD.clone()) {
  // 5 parameters: 2 sqrt weights, the per-foot stance flags, and omega. All three
  // are resolved on the host at the real query time: the contact schedule cannot
  // be looked up from inside the AD graph, and computing omega here keeps a
  // square root and a conditional out of a graph that has to be differentiated
  // and code-generated.
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 5, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Initialized ICPCost with weights: " << weights.transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ICPCost::ICPCost(const ICPCost& other)
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeights_(other.sqrtWeights_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelAdPtr_(other.mpcRobotModelAdPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ICPCost::costVectorFunction(ad_scalar_t time,
                                        const ad_vector_t& state,
                                        const ad_vector_t& input,
                                        const ad_vector_t& parameters) const {
  const ad_vector_t sqrtWeightParams = parameters.head(2);
  // Already normalized on the host, so the support reference below is a plain
  // linear blend: no division by a variable inside the differentiated graph.
  const ad_scalar_t leftWeight = parameters[2];
  const ad_scalar_t rightWeight = parameters[3];
  // 1/omega rather than omega: the parameters are taped as independent variables,
  // and a division by one of them is a division by whatever value the tape was
  // recorded at. Multiplying by a precomputed reciprocal keeps the graph free of
  // any division by a variable.
  const ad_scalar_t inverseOmega = parameters[4];

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelAdPtr_->getGeneralizedCoordinates(state);

  pinocchio::centerOfMass(model, data, q, false);
  const ad_vector2_t com = data.com[0].head(2);

  pinocchio::updateFramePlacements(model, data);
  const auto contactPositions = getContactPositions<ad_scalar_t>(pinocchioInterfaceCppAd_, *mpcRobotModelAdPtr_);

  // Support reference: the centroid of the feet ACTUALLY IN CONTACT. Averaging
  // both feet unconditionally (as this used to) places the target half way
  // towards a foot that is in mid-air during single support, which is not
  // somewhere the CoM should be asked to go.
  //
  // The blend weights are normalized host-side, so this stays a plain linear
  // combination. Both weights are 0.5 in double support, 1/0 in single support,
  // and fall back to 0.5/0.5 during flight.
  const ad_vector2_t supportCentre =
      leftWeight * contactPositions[0].head(2) + rightWeight * contactPositions[1].head(2);

  // The capture point - the whole point of this term, and previously commented
  // out, which left a plain CoM-position cost. Without the velocity term there
  // is no notion of where the CoM is HEADING, so the cost cannot resist a
  // lateral fall; it can only re-centre a robot that is already stable.
  //
  // state[0..1] is the normalized linear momentum, which for the centroidal
  // model is exactly the CoM velocity.
  // capture point = com + com_vel / omega, with omega = sqrt(g / z_com) supplied
  // as its reciprocal by getParameters. state.head(2) is the horizontal part of
  // the normalized linear momentum, which for the centroidal model is exactly
  // the CoM velocity.
  const ad_vector2_t capturePoint = com + inverseOmega * state.head(2);
  const ad_vector_t errors = supportCentre - capturePoint;

  return errors.cwiseProduct(sqrtWeightParams);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t ICPCost::getParameters(scalar_t time, const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {
  // TODO Update this reference for non flat ground in the future
  // The stance flags are resolved here, at the real query time, because the
  // contact schedule cannot be looked up from inside the AD graph.
  const contact_flag_t contactFlags = referenceManagerPtr_->getContactFlags(time);

  // omega = sqrt(g / z_com), from the commanded pelvis height rather than a
  // hardcoded 0.7 m. omega scales the whole velocity contribution of the capture
  // point, so a fixed value mistunes the term whenever the robot is not standing
  // at the assumed height - and the height is commandable.
  constexpr scalar_t gravity = 9.81;
  constexpr scalar_t minimumHeight = 0.1;  // keeps omega finite for a degenerate reference
  scalar_t comHeight = 0.7;
  if (!targetTrajectories.empty()) {
    const vector_t desiredState = targetTrajectories.getDesiredState(time);
    // Base pose sits right after the 6 momentum entries; index 2 is its z.
    if (desiredState.size() > 8) {
      comHeight = desiredState[8];
    }
  }
  // Passed as 1/omega = sqrt(z_com / g); see costVectorFunction.
  const scalar_t inverseOmega = std::sqrt(std::max(comHeight, minimumHeight) / gravity);

  // Normalize the stance mask here so the AD graph only ever sees a linear blend.
  scalar_t leftWeight = contactFlags[0] ? 1.0 : 0.0;
  scalar_t rightWeight = contactFlags[1] ? 1.0 : 0.0;
  const scalar_t stanceCount = leftWeight + rightWeight;
  if (stanceCount > 0.0) {
    leftWeight /= stanceCount;
    rightWeight /= stanceCount;
  } else {
    leftWeight = 0.5;  // flight: no support reference, fall back to the midpoint
    rightWeight = 0.5;
  }

  vector_t parameters(5);
  parameters[0] = sqrtWeights_[0];
  parameters[1] = sqrtWeights_[1];
  parameters[2] = leftWeight;
  parameters[3] = rightWeight;
  parameters[4] = inverseOmega;

  return parameters;
}

}  // namespace ocs2::humanoid
