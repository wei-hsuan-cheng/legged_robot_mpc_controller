#pragma once

#include <pinocchio/codegen/cppadcg.hpp>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

class WbIdBaseMotionResidual final : public StateInputCostGaussNewtonAd {
 public:
  WbIdBaseMotionResidual(const matrix_t& gains, const SwitchedModelReferenceManager& reference,
                        const WbIdMpcRobotModel<scalar_t>& model, const WbIdMpcRobotModel<ad_scalar_t>& adModel);
  WbIdBaseMotionResidual* clone() const override { return new WbIdBaseMotionResidual(*this); }
  vector_t getParameters(scalar_t, const TargetTrajectories&, const PreComputation&) const override;

 private:
  ad_vector_t costVectorFunction(ad_scalar_t, const ad_vector_t&, const ad_vector_t&, const ad_vector_t&) const override;
  const SwitchedModelReferenceManager* reference_;
  WbIdMpcRobotModel<scalar_t> model_;
  WbIdMpcRobotModel<ad_scalar_t> adModel_;
  vector_t sqrtWeights_;
};

/** State-only facade keeps identical running and terminal Gauss-Newton costs. */
class WbIdBaseMotionCost final : public StateCost {
 public:
  WbIdBaseMotionCost(const matrix_t& gains, const SwitchedModelReferenceManager& reference,
                    const WbIdMpcRobotModel<scalar_t>& model, const WbIdMpcRobotModel<ad_scalar_t>& adModel)
      : residual_(gains, reference, model, adModel), zeroInput_(vector_t::Zero(model.getInputDim())) {}
  WbIdBaseMotionCost* clone() const override { return new WbIdBaseMotionCost(*this); }
  scalar_t getValue(scalar_t t, const vector_t& x, const TargetTrajectories& targets, const PreComputation& pre) const override {
    return residual_.getValue(t, x, zeroInput_, targets, pre);
  }
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t t, const vector_t& x, const TargetTrajectories& targets, const PreComputation& pre) const override {
    const auto full = residual_.getQuadraticApproximation(t, x, zeroInput_, targets, pre);
    auto result = ScalarFunctionQuadraticApproximation::Zero(x.size());
    result.f = full.f;
    result.dfdx = full.dfdx;
    result.dfdxx = full.dfdxx;
    return result;
  }
 private:
  WbIdBaseMotionResidual residual_;
  vector_t zeroInput_;
};

}  // namespace ocs2::humanoid
