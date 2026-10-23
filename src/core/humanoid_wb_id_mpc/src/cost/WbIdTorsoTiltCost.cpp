#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/cost/WbIdTorsoTiltCost.h"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2::humanoid {

WbIdTorsoTiltCost::WbIdTorsoTiltCost(const Config& config,
                                   const PinocchioInterface& pinocchioInterface,
                                   const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                   const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr_(mpcRobotModelAD.clone()),
      frameId_(pinocchioInterface.getModel().getFrameId(config.linkName)),
      parameters_(7) {
  if (frameId_ >= static_cast<std::size_t>(pinocchioInterface.getModel().nframes)) {
    throw std::invalid_argument("[WbIdTorsoTiltCost] unknown link: " + config.linkName);
  }
  const scalar_t pi = std::acos(-1.0);
  if (!std::isfinite(config.freeTiltAngle) || config.freeTiltAngle <= 0.0 || config.freeTiltAngle >= pi / 2.0 ||
      std::cos(config.freeTiltAngle) >= 1.0) {
    throw std::invalid_argument("[WbIdTorsoTiltCost] freeTiltAngle must be resolvably positive and below pi/2 radians");
  }
  if (!std::isfinite(config.tiltWeight) || config.tiltWeight < 0.0 ||
      !std::isfinite(config.angularVelocityWeight) || config.angularVelocityWeight < 0.0) {
    throw std::invalid_argument("[WbIdTorsoTiltCost] weights must be finite and nonnegative");
  }
  const vector3_t gravity = pinocchioInterface.getModel().gravity.linear();
  if (!gravity.allFinite() || gravity.norm() < 1e-9) {
    throw std::invalid_argument("[WbIdTorsoTiltCost] the model needs finite, nonzero gravity");
  }
  // Keep tunable values as AD parameters so cached models do not freeze old weights or the corridor.
  parameters_ << -gravity.normalized(), std::cos(config.freeTiltAngle), config.freeTiltAngle,
      std::sqrt(config.tiltWeight), std::sqrt(config.angularVelocityWeight);
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), parameters_.size(),
             "wb_id_torso_tilt_v1_" + config.linkName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}

WbIdTorsoTiltCost::WbIdTorsoTiltCost(const WbIdTorsoTiltCost& other)
    : StateInputCostGaussNewtonAd(other),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelADPtr_(other.mpcRobotModelADPtr_->clone()),
      frameId_(other.frameId_),
      parameters_(other.parameters_) {}

vector_t WbIdTorsoTiltCost::getParameters(scalar_t /*time*/,
                                         const TargetTrajectories& /*targetTrajectories*/,
                                         const PreComputation& /*preComputation*/) const {
  return parameters_;
}

ad_vector_t WbIdTorsoTiltCost::costVectorFunction(ad_scalar_t /*time*/,
                                                 const ad_vector_t& state,
                                                 const ad_vector_t& /*input*/,
                                                 const ad_vector_t& parameters) const {
  using std::acos;
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();
  const ad_vector_t q = mpcRobotModelADPtr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelADPtr_->getGeneralizedVelocities(state);
  pinocchio::forwardKinematics(model, data, q, v);
  const auto& placement = pinocchio::updateFramePlacement(model, data, frameId_);

  const ad_vector3_t up = parameters.head<3>();
  const ad_scalar_t cosine = up.dot(placement.rotation().col(2));
  // Clamp before acos, not after: even inactive AD branches must have finite derivatives at upright.
  const ad_scalar_t corridorCosine = CppAD::CondExpLt(cosine, parameters(3), cosine, parameters(3));
  const ad_scalar_t lowerBound = ad_scalar_t(-1.0 + 1e-9);
  const ad_scalar_t safeCosine = CppAD::CondExpGt(corridorCosine, lowerBound, corridorCosine, lowerBound);
  const ad_scalar_t tiltExcess = acos(safeCosine) - parameters(4);

  const ad_vector3_t angularVelocity =
      pinocchio::getFrameVelocity(model, data, frameId_, pinocchio::LOCAL_WORLD_ALIGNED).angular();
  const ad_vector3_t transverseVelocity = angularVelocity - up * up.dot(angularVelocity);
  ad_vector_t residual(4);
  residual(0) = parameters(5) * tiltExcess;
  residual.tail<3>() = parameters(6) * transverseVelocity;
  return residual;
}

}  // namespace ocs2::humanoid
