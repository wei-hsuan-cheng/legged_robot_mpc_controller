#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/constraint/WbIdRneaConstraintCppAd.h"

#include <stdexcept>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ocs2::humanoid {

WbIdRneaConstraintCppAd::WbIdRneaConstraintCppAd(Rows rows,
                                                 const PinocchioInterface& pinocchioInterface,
                                                 const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                                 const ModelSettings& modelSettings,
                                                 std::shared_ptr<const WbIdTorqueWindow> torqueWindow,
                                                 const SwitchedModelReferenceManager* referenceManager)
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      rows_(rows),
      pinocchioInterfaceCppAd_(std::make_unique<PinocchioInterfaceCppAd>(pinocchioInterface.toCppAd())),
      mpcRobotModelADPtr_(mpcRobotModelAD.clone()),
      numConstraints_(rows == Rows::Base ? 6 : mpcRobotModelAD.getJointDim()),
      torqueWindow_(std::move(torqueWindow)),
      referenceManagerPtr_(referenceManager) {
  const auto& model = pinocchioInterface.getModel();
  if (static_cast<size_t>(model.nv) != mpcRobotModelAD.getGenCoordinatesDim()) {
    throw std::invalid_argument("[WbIdRneaConstraintCppAd] pinocchio nv (" + std::to_string(model.nv) +
                                ") does not match the MPC generalized-coordinate dimension (" +
                                std::to_string(mpcRobotModelAD.getGenCoordinatesDim()) + ")");
  }
  if (rows_ == Rows::Joint && !torqueWindow_) {
    throw std::invalid_argument("[WbIdRneaConstraintCppAd] the joint rows need a torque window");
  }
  for (const auto& contactName : modelSettings.contactNames) {
    const auto frameId = model.getFrameId(contactName);
    if (frameId >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
      throw std::invalid_argument("[WbIdRneaConstraintCppAd] contact frame '" + contactName + "' not in the pinocchio model");
    }
    contactFrameIds_.push_back(frameId);
  }
  if (rows_ == Rows::Base) {
    // Scales for the residual accelerations: total mass and composite inertia at the neutral pose.
    pinocchio::Data data(model);
    pinocchio::ccrba(model, data, pinocchio::neutral(model), vector_t::Zero(model.nv));
    mass_ = data.Ig.mass();
    inertia_ = data.Ig.inertia().matrix().diagonal();
  }
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), rows_ == Rows::Base ? kNumBaseParameters : 0,
             rows_ == Rows::Base ? "wb_id_rnea_base_residual_v1" : "wb_id_rnea_joint", modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
  // Only the taped function reads it, and initialize() has taped or loaded the model: free it
  // rather than copy it into every clone (one per SQP thread).
  pinocchioInterfaceCppAd_.reset();
}

WbIdRneaConstraintCppAd::WbIdRneaConstraintCppAd(const WbIdRneaConstraintCppAd& rhs)
    : StateInputConstraintCppAd(rhs),
      rows_(rhs.rows_),
      mpcRobotModelADPtr_(rhs.mpcRobotModelADPtr_->clone()),
      contactFrameIds_(rhs.contactFrameIds_),
      numConstraints_(rhs.numConstraints_),
      torqueWindow_(rhs.torqueWindow_),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      mass_(rhs.mass_),
      inertia_(rhs.inertia_) {}

bool WbIdRneaConstraintCppAd::isActive(scalar_t time) const {
  return rows_ == Rows::Base || torqueWindow_->contains(time);
}

vector_t WbIdRneaConstraintCppAd::getParameters(scalar_t /*time*/, const PreComputation& /*preComputation*/) const {
  if (rows_ != Rows::Base) {
    return vector_t(0);
  }
  vector_t parameters = vector_t::Zero(kNumBaseParameters);
  if (referenceManagerPtr_ != nullptr) {
    parameters.head(SwitchedModelReferenceManager::kDynamicsResidualDim) = referenceManagerPtr_->getDynamicsResidual();
  }
  parameters.tail<4>() << mass_, inertia_;
  return parameters;
}

ad_vector_t WbIdRneaConstraintCppAd::constraintFunction(ad_scalar_t /*time*/,
                                                        const ad_vector_t& state,
                                                        const ad_vector_t& input,
                                                        const ad_vector_t& p) const {
  const auto& model = pinocchioInterfaceCppAd_->getModel();
  auto data = pinocchioInterfaceCppAd_->getData();  // const method: record on a copy

  const ad_vector_t q = mpcRobotModelADPtr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelADPtr_->getGeneralizedVelocities(state);
  const ad_vector_t a = mpcRobotModelADPtr_->getGeneralizedAccelerations(input);

  ad_vector_t residual = pinocchio::rnea(model, data, q, v, a);

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  Eigen::Matrix<ad_scalar_t, 6, Eigen::Dynamic> jacobian(6, model.nv);
  for (size_t c = 0; c < contactFrameIds_.size(); ++c) {
    jacobian.setZero();
    pinocchio::getFrameJacobian(model, data, contactFrameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    residual.noalias() -= jacobian.transpose() * mpcRobotModelADPtr_->getContactWrench(input, c);
  }

  if (rows_ == Rows::Base) {
    using ad_vector3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;
    using ad_matrix3_t = Eigen::Matrix<ad_scalar_t, 3, 3>;
    // Heading frame: the base's yaw only.
    const auto& base = data.oMi[1];
    const ad_scalar_t c = base.rotation()(0, 0);
    const ad_scalar_t s = base.rotation()(1, 0);
    const ad_scalar_t n = CppAD::sqrt(c * c + s * s + ad_scalar_t(1e-12));
    ad_matrix3_t heading;
    heading << c / n, -s / n, ad_scalar_t(0.0), s / n, c / n, ad_scalar_t(0.0), ad_scalar_t(0.0), ad_scalar_t(0.0), ad_scalar_t(1.0);

    ad_vector3_t forceSum = ad_vector3_t::Zero();
    ad_vector3_t momentSum = ad_vector3_t::Zero();
    for (size_t k = 0; k < contactFrameIds_.size(); ++k) {
      const auto wrench = mpcRobotModelADPtr_->getContactWrench(input, k);
      const ad_vector3_t force = wrench.head<3>();
      const ad_vector3_t arm = data.oMf[contactFrameIds_[k]].translation() - base.translation();
      forceSum += force;
      momentSum += arm.cross(force) + wrench.tail<3>();
    }
    const ad_vector3_t linearAcceleration = p.segment<3>(0);
    const ad_vector3_t angularAcceleration = p.segment<3>(3);
    const ad_vector3_t forceGain = p.segment<3>(6);
    const ad_vector3_t momentGain = p.segment<3>(9);
    const ad_scalar_t& mass = p[12];
    const ad_vector3_t inertia = p.segment<3>(13);
    const ad_vector3_t force = mass * linearAcceleration + forceGain.cwiseProduct(heading.transpose() * forceSum);
    const ad_vector3_t moment = inertia.cwiseProduct(angularAcceleration) + momentGain.cwiseProduct(heading.transpose() * momentSum);
    Eigen::Matrix<ad_scalar_t, 6, 1> wrench;
    wrench << heading * force, heading * moment;
    jacobian.setZero();
    pinocchio::getJointJacobian(model, data, 1, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    residual.noalias() -= jacobian.transpose() * wrench;
    return residual.head<6>();
  }
  const auto nj = static_cast<Eigen::Index>(mpcRobotModelADPtr_->getJointDim());
  return residual.tail(nj) - mpcRobotModelADPtr_->getJointTorques(input);
}

}  // namespace ocs2::humanoid
