#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Initial guess for new horizon nodes: the weight split over the stance feet, zero
 * acceleration, and the joint torques that make that guess satisfy the RNEA constraint,
 * tau = [RNEA(q, v, 0) - sum_c J_c^T W_c]_joints plus configured joint friction.
 * Wheel moments are seeded at the tire contact. This satisfies the joint torque
 * rows, not necessarily the floating-base or rolling acceleration equalities.
 */
class WbIdInitializer final : public Initializer {
 public:
  WbIdInitializer(PinocchioInterface pinocchioInterface,
                  const SwitchedModelReferenceManager& referenceManager,
                  const WbIdMpcRobotModel<scalar_t>& mpcRobotModel,
                  scalar_t wheelRadius = 0.0,
                  vector3_t wheelAxleAxis = vector3_t::UnitZ());
  ~WbIdInitializer() override = default;
  WbIdInitializer* clone() const override { return new WbIdInitializer(*this); }

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

  /// The inverse-dynamics consistent input for `state`, `contactFlags`. Also used by the controller.
  vector_t weightCompensatingInput(const vector_t& state, const contact_flag_t& contactFlags, scalar_t time = 0.0);

 private:
  WbIdInitializer(const WbIdInitializer& rhs) = default;

  PinocchioInterface pinocchioInterface_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const WbIdMpcRobotModel<scalar_t>* mpcRobotModelPtr_;
  scalar_t wheelRadius_;
  vector3_t wheelAxleAxis_;
};

}  // namespace ocs2::humanoid
