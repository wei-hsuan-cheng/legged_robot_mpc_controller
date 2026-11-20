#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Joint velocity box |qd_j| <= qd_max as the inequality h = [qd_max - qd; qd_max + qd] >= 0
 * (to be wrapped in a soft constraint). In the whole-body state the joint velocities are
 * states and can be bounded directly, as in Molnar et al.
 */
class WbIdJointVelocityLimitConstraint final : public StateConstraint {
 public:
  WbIdJointVelocityLimitConstraint(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel, vector_t velocityLimits);
  ~WbIdJointVelocityLimitConstraint() override = default;
  WbIdJointVelocityLimitConstraint* clone() const override { return new WbIdJointVelocityLimitConstraint(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return 2 * static_cast<size_t>(velocityLimits_.size()); }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  WbIdJointVelocityLimitConstraint(const WbIdJointVelocityLimitConstraint& rhs) = default;

  vector_t velocityLimits_;
  size_t velocityStart_;
};

}  // namespace ocs2::humanoid
