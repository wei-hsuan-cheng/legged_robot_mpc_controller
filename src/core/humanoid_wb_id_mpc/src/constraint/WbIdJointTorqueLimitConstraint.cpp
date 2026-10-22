#include "humanoid_wb_id_mpc/constraint/WbIdJointTorqueLimitConstraint.h"

#include <stdexcept>

namespace ocs2::humanoid {

WbIdJointTorqueLimitConstraint::WbIdJointTorqueLimitConstraint(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel,
                                                               vector_t torqueLimits,
                                                               std::shared_ptr<const WbIdTorqueWindow> torqueWindow)
    : StateInputConstraint(ConstraintOrder::Linear),
      torqueLimits_(std::move(torqueLimits)),
      torqueStart_(mpcRobotModel.getJointTorqueStartIndex()),
      torqueWindow_(std::move(torqueWindow)) {
  if (static_cast<size_t>(torqueLimits_.size()) != mpcRobotModel.getJointDim()) {
    throw std::invalid_argument("[WbIdJointTorqueLimitConstraint] expected " + std::to_string(mpcRobotModel.getJointDim()) +
                                " torque limits, got " + std::to_string(torqueLimits_.size()));
  }
  if ((torqueLimits_.array() <= 0.0).any()) {
    throw std::invalid_argument("[WbIdJointTorqueLimitConstraint] torque limits must be positive");
  }
}

vector_t WbIdJointTorqueLimitConstraint::getValue(scalar_t /*time*/, const vector_t& /*state*/, const vector_t& input,
                                                  const PreComputation& /*preComp*/) const {
  const auto nj = torqueLimits_.size();
  const auto tau = input.segment(static_cast<Eigen::Index>(torqueStart_), nj);
  vector_t h(2 * nj);
  h.head(nj) = torqueLimits_ - tau;
  h.tail(nj) = torqueLimits_ + tau;
  return h;
}

VectorFunctionLinearApproximation WbIdJointTorqueLimitConstraint::getLinearApproximation(scalar_t time,
                                                                                         const vector_t& state,
                                                                                         const vector_t& input,
                                                                                         const PreComputation& preComp) const {
  const auto nj = torqueLimits_.size();
  VectorFunctionLinearApproximation approximation =
      VectorFunctionLinearApproximation::Zero(2 * nj, state.size(), input.size());
  approximation.f = getValue(time, state, input, preComp);
  approximation.dfdu.block(0, static_cast<Eigen::Index>(torqueStart_), nj, nj) = -matrix_t::Identity(nj, nj);
  approximation.dfdu.block(nj, static_cast<Eigen::Index>(torqueStart_), nj, nj) = matrix_t::Identity(nj, nj);
  return approximation;
}

}  // namespace ocs2::humanoid
