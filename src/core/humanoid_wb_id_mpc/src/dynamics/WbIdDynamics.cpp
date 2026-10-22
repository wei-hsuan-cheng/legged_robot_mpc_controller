#include "humanoid_wb_id_mpc/dynamics/WbIdDynamics.h"

namespace ocs2::humanoid {

WbIdDynamics::WbIdDynamics(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel)
    : SystemDynamicsBase(),
      nv_(mpcRobotModel.getGenCoordinatesDim()),
      accelerationStart_(mpcRobotModel.getGeneralizedAccelerationStartIndex()) {
  const auto stateDim = static_cast<Eigen::Index>(mpcRobotModel.getStateDim());
  const auto inputDim = static_cast<Eigen::Index>(mpcRobotModel.getInputDim());
  const auto nv = static_cast<Eigen::Index>(nv_);
  dfdx_ = matrix_t::Zero(stateDim, stateDim);
  dfdx_.topRightCorner(nv, nv).setIdentity();
  dfdu_ = matrix_t::Zero(stateDim, inputDim);
  dfdu_.block(nv, static_cast<Eigen::Index>(accelerationStart_), nv, nv).setIdentity();
}

vector_t WbIdDynamics::computeFlowMap(scalar_t /*time*/, const vector_t& state, const vector_t& input, const PreComputation& /*preComp*/) {
  const auto nv = static_cast<Eigen::Index>(nv_);
  vector_t stateDerivative(state.size());
  stateDerivative.head(nv) = state.tail(nv);
  stateDerivative.tail(nv) = input.segment(static_cast<Eigen::Index>(accelerationStart_), nv);
  return stateDerivative;
}

VectorFunctionLinearApproximation WbIdDynamics::linearApproximation(scalar_t time,
                                                                    const vector_t& state,
                                                                    const vector_t& input,
                                                                    const PreComputation& preComp) {
  VectorFunctionLinearApproximation approximation;
  approximation.f = computeFlowMap(time, state, input, preComp);
  approximation.dfdx = dfdx_;
  approximation.dfdu = dfdu_;
  return approximation;
}

}  // namespace ocs2::humanoid
