#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"
#include "humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h"

namespace ocs2::humanoid {

/**
 * Joint torque box |tau_j| <= tau_max, as the inequality h = [tau_max - tau; tau_max + tau] >= 0
 * (to be wrapped in a soft constraint). Active inside the torque window only, where the
 * joint rows of the inverse dynamics make tau a physical torque (Molnar et al. bound the
 * torques on exactly those nodes).
 */
class WbIdJointTorqueLimitConstraint final : public StateInputConstraint {
 public:
  WbIdJointTorqueLimitConstraint(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel,
                                 vector_t torqueLimits,
                                 std::shared_ptr<const WbIdTorqueWindow> torqueWindow);
  ~WbIdJointTorqueLimitConstraint() override = default;
  WbIdJointTorqueLimitConstraint* clone() const override { return new WbIdJointTorqueLimitConstraint(*this); }

  bool isActive(scalar_t time) const override { return torqueWindow_->contains(time); }
  size_t getNumConstraints(scalar_t /*time*/) const override { return 2 * static_cast<size_t>(torqueLimits_.size()); }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  WbIdJointTorqueLimitConstraint(const WbIdJointTorqueLimitConstraint& rhs) = default;

  vector_t torqueLimits_;
  size_t torqueStart_;
  std::shared_ptr<const WbIdTorqueWindow> torqueWindow_;
};

}  // namespace ocs2::humanoid
