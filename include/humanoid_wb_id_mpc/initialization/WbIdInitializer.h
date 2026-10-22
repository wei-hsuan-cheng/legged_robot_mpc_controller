#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Initial guess for new horizon nodes: the weight split over the stance feet, zero
 * acceleration, and the joint torques that make that guess satisfy the RNEA constraint,
 * tau = [RNEA(q, v, 0) - sum_c J_c^T W_c]_joints. Starting the solver on the constraint
 * manifold rather than at tau = 0 saves it from spending its first iteration on a
 * torque defect of the robot's full weight.
 */
class WbIdInitializer final : public Initializer {
 public:
  WbIdInitializer(PinocchioInterface pinocchioInterface,
                  const SwitchedModelReferenceManager& referenceManager,
                  const WbIdMpcRobotModel<scalar_t>& mpcRobotModel);
  ~WbIdInitializer() override = default;
  WbIdInitializer* clone() const override { return new WbIdInitializer(*this); }

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

  /// The inverse-dynamics consistent input for `state`, `contactFlags`. Also used by the controller.
  vector_t weightCompensatingInput(const vector_t& state, const contact_flag_t& contactFlags);

 private:
  WbIdInitializer(const WbIdInitializer& rhs) = default;

  PinocchioInterface pinocchioInterface_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const WbIdMpcRobotModel<scalar_t>* mpcRobotModelPtr_;
};

}  // namespace ocs2::humanoid
