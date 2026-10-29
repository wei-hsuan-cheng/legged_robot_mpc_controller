#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_robotic_tools/common/So3Tangent.h>
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/** R = R_anchor Exp(phi), with body angular velocity, not Euler rates. */
class WbIdTangentDynamics final : public SystemDynamicsBaseAD {
 public:
  explicit WbIdTangentDynamics(const WbIdMpcRobotModel<scalar_t>& model)
      : nv_(model.getGenCoordinatesDim()), accelerationStart_(model.getGeneralizedAccelerationStartIndex()) {
    initialize(model.getStateDim(), model.getInputDim(), "tangent_dynamics_v2", model.modelSettings.modelFolderCppAd,
               model.modelSettings.recompileLibrariesCppAd, model.modelSettings.verboseCppAd);
  }
  WbIdTangentDynamics* clone() const override { return new WbIdTangentDynamics(*this); }

 private:
  ad_vector_t systemFlowMap(ad_scalar_t, const ad_vector_t& x, const ad_vector_t& u, const ad_vector_t&) const override {
    ad_vector_t dx = ad_vector_t::Zero(x.size());
    dx.head(nv_) = x.segment(nv_, nv_);
    const ad_vector3_t phi = x.segment<3>(3);
    const ad_matrix3_t inverseRightJacobian = so3RightJacobianInverse(phi);
    dx.segment<3>(3) = inverseRightJacobian * x.segment<3>(nv_ + 3);
    dx.segment(nv_, nv_) = u.segment(accelerationStart_, nv_);
    return dx;
  }
  size_t nv_;
  size_t accelerationStart_;
};

}  // namespace ocs2::humanoid
