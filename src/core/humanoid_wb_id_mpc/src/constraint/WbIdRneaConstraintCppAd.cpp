#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/constraint/WbIdRneaConstraintCppAd.h"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ocs2::humanoid {

WbIdRneaConstraintCppAd::WbIdRneaConstraintCppAd(Rows rows,
                                                 const PinocchioInterface& pinocchioInterface,
                                                 const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                                 const ModelSettings& modelSettings,
                                                 std::shared_ptr<const WbIdTorqueWindow> torqueWindow)
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      rows_(rows),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr_(mpcRobotModelAD.clone()),
      numConstraints_(rows == Rows::Base ? 6 : mpcRobotModelAD.getJointDim()),
      torqueWindow_(std::move(torqueWindow)) {
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
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 0,
             rows_ == Rows::Base ? "wb_id_rnea_base" : "wb_id_rnea_joint", modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}

WbIdRneaConstraintCppAd::WbIdRneaConstraintCppAd(const WbIdRneaConstraintCppAd& rhs)
    : StateInputConstraintCppAd(rhs),
      rows_(rhs.rows_),
      pinocchioInterfaceCppAd_(rhs.pinocchioInterfaceCppAd_),
      mpcRobotModelADPtr_(rhs.mpcRobotModelADPtr_->clone()),
      contactFrameIds_(rhs.contactFrameIds_),
      numConstraints_(rhs.numConstraints_),
      torqueWindow_(rhs.torqueWindow_) {}

bool WbIdRneaConstraintCppAd::isActive(scalar_t time) const {
  return rows_ == Rows::Base || torqueWindow_->contains(time);
}

ad_vector_t WbIdRneaConstraintCppAd::constraintFunction(ad_scalar_t /*time*/,
                                                        const ad_vector_t& state,
                                                        const ad_vector_t& input,
                                                        const ad_vector_t& /*parameters*/) const {
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();  // const method: record on a copy

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
    return residual.head<6>();
  }
  const auto nj = static_cast<Eigen::Index>(mpcRobotModelADPtr_->getJointDim());
  return residual.tail(nj) - mpcRobotModelADPtr_->getJointTorques(input);
}

}  // namespace ocs2::humanoid
