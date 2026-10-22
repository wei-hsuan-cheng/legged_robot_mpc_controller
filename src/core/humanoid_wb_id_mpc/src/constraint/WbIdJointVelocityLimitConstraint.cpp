#include "humanoid_wb_id_mpc/constraint/WbIdJointVelocityLimitConstraint.h"

#include <stdexcept>

namespace ocs2::humanoid {

WbIdJointVelocityLimitConstraint::WbIdJointVelocityLimitConstraint(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel,
                                                                   vector_t velocityLimits)
    : StateConstraint(ConstraintOrder::Linear),
      velocityLimits_(std::move(velocityLimits)),
      velocityStart_(mpcRobotModel.getJointVelocitiesStartindex()) {
  if (static_cast<size_t>(velocityLimits_.size()) != mpcRobotModel.getJointDim()) {
    throw std::invalid_argument("[WbIdJointVelocityLimitConstraint] expected " + std::to_string(mpcRobotModel.getJointDim()) +
                                " velocity limits, got " + std::to_string(velocityLimits_.size()));
  }
  if ((velocityLimits_.array() <= 0.0).any()) {
    throw std::invalid_argument("[WbIdJointVelocityLimitConstraint] velocity limits must be positive");
  }
}

vector_t WbIdJointVelocityLimitConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  const auto nj = velocityLimits_.size();
  const auto qd = state.segment(static_cast<Eigen::Index>(velocityStart_), nj);
  vector_t h(2 * nj);
  h.head(nj) = velocityLimits_ - qd;
  h.tail(nj) = velocityLimits_ + qd;
  return h;
}

VectorFunctionLinearApproximation WbIdJointVelocityLimitConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                           const PreComputation& preComp) const {
  const auto nj = velocityLimits_.size();
  VectorFunctionLinearApproximation approximation = VectorFunctionLinearApproximation::Zero(2 * nj, state.size(), 0);
  approximation.f = getValue(time, state, preComp);
  approximation.dfdx.block(0, static_cast<Eigen::Index>(velocityStart_), nj, nj) = -matrix_t::Identity(nj, nj);
  approximation.dfdx.block(nj, static_cast<Eigen::Index>(velocityStart_), nj, nj) = matrix_t::Identity(nj, nj);
  return approximation;
}

}  // namespace ocs2::humanoid
