#pragma once

#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * OCS2 <-> Pinocchio mapping, including the measured-anchor tangent chart.
 * The velocity block starts at nv in either mode; inputs do not enter kinematics.
 */
template <typename SCALAR_T>
class WbIdPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR_T> {
 public:
  using vector_t = typename PinocchioStateInputMapping<SCALAR_T>::vector_t;
  using matrix_t = typename PinocchioStateInputMapping<SCALAR_T>::matrix_t;

  WbIdPinocchioMappingTpl(size_t generalizedCoordinatesDim, size_t inputDim)
      : nq_(generalizedCoordinatesDim), inputDim_(inputDim) {}
  explicit WbIdPinocchioMappingTpl(const WbIdMpcRobotModel<SCALAR_T>& model)
      : nq_(model.getGenCoordinatesDim()), inputDim_(model.getInputDim()), model_(model.clone()) {}
  ~WbIdPinocchioMappingTpl() override = default;
  WbIdPinocchioMappingTpl* clone() const override { return new WbIdPinocchioMappingTpl(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override {
    return model_ ? model_->getGeneralizedCoordinates(state) : vector_t(state.head(nq_));
  }

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& /*input*/) const override {
    return state.segment(nq_, nq_);
  }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    if (model_ && model_->usesTangentSpace()) {
      matrix_t positionMapping = matrix_t::Zero(nq_, model_->getStateDim());
      positionMapping.leftCols(nq_).setIdentity();
      MATRIX3_T<SCALAR_T> rightJacobian;
      pinocchio::Jexp3(VECTOR3_T<SCALAR_T>(state.template segment<3>(3)), rightJacobian);
      positionMapping.template block<3, 3>(3, 3) = rightJacobian;
      Eigen::Quaternion<SCALAR_T> anchor(state.template tail<4>());
      const SCALAR_T norm = anchor.norm();
      anchor.coeffs() /= norm;
      Eigen::Matrix<SCALAR_T, 3, 4> quaternionToLocal;
      quaternionToLocal.template leftCols<3>() = anchor.w() * MATRIX3_T<SCALAR_T>::Identity() - pinocchio::skew(anchor.vec());
      quaternionToLocal.template rightCols<1>() = -anchor.vec();
      positionMapping.template block<3, 4>(3, model_->getAnchorStartIndex()) =
          (SCALAR_T(2) / norm) * pinocchio::exp3(VECTOR3_T<SCALAR_T>(-state.template segment<3>(3))) * quaternionToLocal;
      matrix_t derivative = Jq * positionMapping;
      derivative.middleCols(nq_, nq_) += Jv;
      return {derivative, matrix_t::Zero(Jq.rows(), inputDim_)};
    }
    matrix_t dfdx(Jq.rows(), 2 * nq_);
    dfdx << Jq, Jv;
    return {dfdx, matrix_t::Zero(Jq.rows(), inputDim_)};
  }

 private:
  WbIdPinocchioMappingTpl(const WbIdPinocchioMappingTpl& rhs)
      : nq_(rhs.nq_), inputDim_(rhs.inputDim_), model_(rhs.model_ ? rhs.model_->clone() : nullptr) {}

  size_t nq_;
  size_t inputDim_;
  std::unique_ptr<WbIdMpcRobotModel<SCALAR_T>> model_;
};

using WbIdPinocchioMapping = WbIdPinocchioMappingTpl<scalar_t>;
using WbIdPinocchioMappingCppAd = WbIdPinocchioMappingTpl<ad_scalar_t>;

}  // namespace ocs2::humanoid
