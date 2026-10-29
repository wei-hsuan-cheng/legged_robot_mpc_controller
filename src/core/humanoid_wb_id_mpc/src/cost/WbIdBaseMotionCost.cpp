#include "humanoid_wb_id_mpc/cost/WbIdBaseMotionCost.h"

namespace ocs2::humanoid {

WbIdBaseMotionResidual::WbIdBaseMotionResidual(
    const matrix_t& gains, const SwitchedModelReferenceManager& reference,
    const WbIdMpcRobotModel<scalar_t>& model, const WbIdMpcRobotModel<ad_scalar_t>& adModel)
    : reference_(&reference), model_(model.modelSettings, true), adModel_(adModel.modelSettings, true),
      sqrtWeights_(gains.diagonal().cwiseSqrt()) {
  initialize(model.getStateDim(), model.getInputDim(), model.getStateDim() + 13, "tangent_base_motion_v3",
             model.modelSettings.modelFolderCppAd, model.modelSettings.recompileLibrariesCppAd, model.modelSettings.verboseCppAd);
}

vector_t WbIdBaseMotionResidual::getParameters(scalar_t t, const TargetTrajectories& targets, const PreComputation&) const {
  vector_t parameters(model_.getStateDim() + 13);
  parameters << targets.getDesiredState(t), sqrtWeights_, reference_->usesRelativeBaseTwistTracking() ? 1.0 : 0.0;
  return parameters;
}

ad_vector_t WbIdBaseMotionResidual::costVectorFunction(
    ad_scalar_t, const ad_vector_t& x, const ad_vector_t&, const ad_vector_t& p) const {
  const auto nx = model_.getStateDim();
  const auto nv = model_.getGenCoordinatesDim();
  const ad_vector_t desired = p.head(nx);
  const auto orientation = adModel_.getBaseQuaternion(x);
  const auto desiredOrientation = adModel_.getBaseQuaternion(desired);
  const auto R = orientation.toRotationMatrix();
  const auto Rd = desiredOrientation.toRotationMatrix();
  const ad_scalar_t relative = p(nx + 12);
  // Heading belongs to relative walking only. Absolute pose tracking uses world
  // velocities and must not extract a singular heading from a vertical attitude.
  const ad_scalar_t yaw = atan2(relative * R(1, 0), (1.0 - relative) + relative * R(0, 0));
  const ad_scalar_t yawDesired = atan2(relative * Rd(1, 0), (1.0 - relative) + relative * Rd(0, 0));
  const auto heading = [](ad_scalar_t angle) {
    ad_matrix3_t result = ad_matrix3_t::Identity();
    result(0, 0) = result(1, 1) = cos(angle);
    result(0, 1) = -sin(angle);
    result(1, 0) = sin(angle);
    return result;
  };
  const ad_matrix3_t H = heading(yaw);
  const ad_matrix3_t Hd = heading(yawDesired);
  // Remove heading from each attitude separately in proprioceptive twist mode.
  const ad_quaternion_t headingQuaternion(cos(yaw / 2.0), ad_scalar_t(0.0), ad_scalar_t(0.0), sin(yaw / 2.0));
  const ad_quaternion_t desiredHeadingQuaternion(cos(yawDesired / 2.0), ad_scalar_t(0.0), ad_scalar_t(0.0), sin(yawDesired / 2.0));
  const ad_quaternion_t fullError = desiredOrientation.conjugate() * orientation;
  const ad_quaternion_t tiltError = (desiredHeadingQuaternion.conjugate() * desiredOrientation).conjugate() *
                                   (headingQuaternion.conjugate() * orientation);
  ad_quaternion_t orientationError;
  orientationError.coeffs() = (1.0 - relative) * fullError.coeffs() + relative * tiltError.coeffs();
  const ad_vector3_t rotationError = pinocchio::quaternion::log3(orientationError);
  const ad_vector3_t positionError = Hd.transpose() * ad_vector3_t(x.head<3>() - desired.head<3>());
  const ad_vector3_t velocity = H.transpose() * ad_vector3_t(x.segment<3>(nv));
  const ad_vector3_t desiredVelocity = Hd.transpose() * ad_vector3_t(desired.segment<3>(nv));
  const ad_vector3_t angular = H.transpose() * R * ad_vector3_t(x.segment<3>(nv + 3));
  const ad_vector3_t desiredAngular = Hd.transpose() * Rd * ad_vector3_t(desired.segment<3>(nv + 3));
  ad_vector_t error(12);
  error << (1.0 - relative) * positionError(0), (1.0 - relative) * positionError(1), positionError(2),
      rotationError.reverse(), velocity - desiredVelocity, (angular - desiredAngular).reverse();
  return error.cwiseProduct(p.segment(nx, 12));
}

}  // namespace ocs2::humanoid
