#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/constraint/WbIdFootAccelerationConstraintCppAd.h"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2::humanoid {

WbIdFootAccelerationConstraintCppAd::WbIdFootAccelerationConstraintCppAd(Type type,
                                                                         const SwitchedModelReferenceManager& referenceManager,
                                                                         const PinocchioInterface& pinocchioInterface,
                                                                         const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                                                         size_t contactIndex,
                                                                         Gains gains,
                                                                         const ModelSettings& modelSettings)
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      type_(type),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr_(mpcRobotModelAD.clone()),
      contactIndex_(contactIndex),
      gains_(gains) {
  const auto& model = pinocchioInterface.getModel();
  const std::string& footName = modelSettings.contactNames.at(contactIndex);
  frameId_ = model.getFrameId(footName);
  if (frameId_ >= static_cast<size_t>(model.nframes)) {
    throw std::invalid_argument("[WbIdFootAccelerationConstraintCppAd] contact frame '" + footName + "' not in the pinocchio model");
  }
  // Without an acceleration term the constraint has no input dependence and the
  // state-input projection is singular; refuse that rather than fail inside the solver.
  if (type_ == Type::Stance &&
      (gains_.linearAccelerationXY == 0.0 || gains_.linearAccelerationZ == 0.0 || gains_.angularAcceleration == 0.0)) {
    throw std::invalid_argument(
        "[WbIdFootAccelerationConstraintCppAd] stance acceleration gains must be non-zero (footConstraint.*AccelerationErrorGain*)");
  }
  if (type_ == Type::SwingVertical && gains_.linearAccelerationZ == 0.0) {
    throw std::invalid_argument(
        "[WbIdFootAccelerationConstraintCppAd] swing vertical acceleration gain must be non-zero "
        "(footConstraint.linearAccelerationErrorGain_z)");
  }
  const std::string modelName = footName + (type_ == Type::Stance ? "_wb_id_stance_acceleration" : "_wb_id_swing_vertical_acceleration");
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), kNumParameters, modelName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}

WbIdFootAccelerationConstraintCppAd::WbIdFootAccelerationConstraintCppAd(const WbIdFootAccelerationConstraintCppAd& rhs)
    : StateInputConstraintCppAd(rhs),
      type_(rhs.type_),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(rhs.pinocchioInterfaceCppAd_),
      mpcRobotModelADPtr_(rhs.mpcRobotModelADPtr_->clone()),
      contactIndex_(rhs.contactIndex_),
      frameId_(rhs.frameId_),
      gains_(rhs.gains_) {}

bool WbIdFootAccelerationConstraintCppAd::isActive(scalar_t time) const {
  const bool inContact = referenceManagerPtr_->getContactFlags(time)[contactIndex_];
  return type_ == Type::Stance ? inContact : !inContact;
}

vector_t WbIdFootAccelerationConstraintCppAd::getParameters(scalar_t time, const PreComputation& /*preComputation*/) const {
  const auto& swing = *referenceManagerPtr_->getSwingTrajectoryPlanner();
  vector_t parameters(kNumParameters);
  parameters << swing.getZpositionConstraint(contactIndex_, time),  //
      swing.getZvelocityConstraint(contactIndex_, time),            //
      swing.getZaccelerationConstraint(contactIndex_, time),        //
      gains_.positionZ, gains_.orientation,                         //
      gains_.linearVelocityXY, gains_.linearVelocityZ, gains_.angularVelocity,  //
      gains_.linearAccelerationXY, gains_.linearAccelerationZ, gains_.angularAcceleration;
  return parameters;
}

ad_vector_t WbIdFootAccelerationConstraintCppAd::constraintFunction(ad_scalar_t /*time*/,
                                                                    const ad_vector_t& state,
                                                                    const ad_vector_t& input,
                                                                    const ad_vector_t& p) const {
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();  // const method: record on a copy

  const ad_vector_t q = mpcRobotModelADPtr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelADPtr_->getGeneralizedVelocities(state);
  const ad_vector_t a = mpcRobotModelADPtr_->getGeneralizedAccelerations(input);

  pinocchio::forwardKinematics(model, data, q, v, a);
  const auto& placement = pinocchio::updateFramePlacement(model, data, frameId_);
  const auto twist = pinocchio::getFrameVelocity(model, data, frameId_, pinocchio::LOCAL_WORLD_ALIGNED);
  const auto acceleration = pinocchio::getFrameClassicalAcceleration(model, data, frameId_, pinocchio::LOCAL_WORLD_ALIGNED);

  const ad_scalar_t& zRef = p[0];
  const ad_scalar_t& vzRef = p[1];
  const ad_scalar_t& azRef = p[2];
  const ad_scalar_t& kpZ = p[3];
  const ad_scalar_t& kpOrientation = p[4];
  const ad_scalar_t& kdXY = p[5];
  const ad_scalar_t& kdZ = p[6];
  const ad_scalar_t& kdAngular = p[7];
  const ad_scalar_t& kaXY = p[8];
  const ad_scalar_t& kaZ = p[9];
  const ad_scalar_t& kaAngular = p[10];

  if (type_ == Type::SwingVertical) {
    ad_vector_t g(1);
    g(0) = kaZ * (acceleration.linear()(2) - azRef) + kdZ * (twist.linear()(2) - vzRef) + kpZ * (placement.translation()(2) - zRef);
    return g;
  }

  const ad_vector3_t tilt =
      rotationMatrixDistanceToPlane<ad_scalar_t>(placement.rotation(), ad_vector3_t(ad_scalar_t(0.0), ad_scalar_t(0.0), ad_scalar_t(1.0)));
  ad_vector_t g(6);
  g(0) = kaXY * acceleration.linear()(0) + kdXY * twist.linear()(0);
  g(1) = kaXY * acceleration.linear()(1) + kdXY * twist.linear()(1);
  g(2) = kaZ * acceleration.linear()(2) + kdZ * twist.linear()(2) + kpZ * (placement.translation()(2) - zRef);
  g.tail<3>() = kaAngular * acceleration.angular() + kdAngular * twist.angular() + kpOrientation * tilt;
  return g;
}

}  // namespace ocs2::humanoid
