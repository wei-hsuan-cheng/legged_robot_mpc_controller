#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Whole-body inverse-dynamics MPC flow map: x = [q, v], u = [W, a, tau], xdot = [v; a].
 *
 * The rigid-body dynamics are NOT in here. They are the RNEA equality constraint
 * (WbIdRneaConstraintCppAd), which is what makes this an inverse-dynamics MPC: the
 * integrator is a plain double integrator, exactly linear, so its Jacobians are
 * constant and need no code generation.
 */
class WbIdDynamics final : public SystemDynamicsBase {
 public:
  explicit WbIdDynamics(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel);
  ~WbIdDynamics() override = default;
  WbIdDynamics* clone() const override { return new WbIdDynamics(*this); }

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
  VectorFunctionLinearApproximation linearApproximation(scalar_t time,
                                                        const vector_t& state,
                                                        const vector_t& input,
                                                        const PreComputation& preComp) override;

 private:
  WbIdDynamics(const WbIdDynamics& rhs) = default;

  size_t nv_;
  size_t accelerationStart_;
  matrix_t dfdx_;
  matrix_t dfdu_;
};

}  // namespace ocs2::humanoid
