#pragma once

#include <memory>
#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "legged_robot_mpc_controller/mpc/common/model/ModelSettings.h"
#include "legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Foot contact and swing constraints at the acceleration level.
 *
 * Velocity is part of the state, so a velocity-only equality has dg/du = 0 and
 * cannot be eliminated by state-input equality projection. Differentiating once
 * introduces the generalized acceleration input through J a + Jdot v; velocity
 * and position feedback provide damping and stiffness for the second-order
 * error dynamics.
 *
 *   Stance (6 rows, active while the foot is in contact):
 *     Ka * [a_lin; a_ang] + Kd * [v_lin; omega] + Kp * [p_z - z_ref; tilt] = 0
 *   SwingVertical (1 row, active while the foot is in the air):
 *     ka (a_z - az_ref) + kd (v_z - vz_ref) + kp (p_z - z_ref) = 0
 *
 * a_lin is the CLASSICAL acceleration of the contact frame origin, so it is the time
 * derivative of the LOCAL_WORLD_ALIGNED velocity the damping term acts on. "tilt" is the
 * sole's rotation away from the ground plane (rotationMatrixDistanceToPlane), so yaw is
 * left free.
 *
 * The gains are parameters of the generated library rather than constants baked into
 * it, so changing them in the yaml takes effect without regenerating the model.
 */
class WbIdFootAccelerationConstraintCppAd final : public StateInputConstraintCppAd {
 public:
  enum class Type { Stance, SwingVertical };

  struct Gains {
    scalar_t positionZ{0.0};       // Kp on the foot height
    scalar_t orientation{0.0};     // Kp on the sole tilt
    scalar_t linearVelocityXY{0.0};
    scalar_t linearVelocityZ{0.0};
    scalar_t angularVelocity{0.0};
    scalar_t linearAccelerationXY{1.0};
    scalar_t linearAccelerationZ{1.0};
    scalar_t angularAcceleration{1.0};
  };

  WbIdFootAccelerationConstraintCppAd(Type type,
                                      const SwitchedModelReferenceManager& referenceManager,
                                      const PinocchioInterface& pinocchioInterface,
                                      const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                      size_t contactIndex,
                                      Gains gains,
                                      const ModelSettings& modelSettings);
  ~WbIdFootAccelerationConstraintCppAd() override = default;
  WbIdFootAccelerationConstraintCppAd* clone() const override { return new WbIdFootAccelerationConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override { return type_ == Type::Stance ? 6 : 1; }
  vector_t getParameters(scalar_t time, const PreComputation& preComputation) const override;

  static constexpr size_t kNumParameters = 14;

 private:
  WbIdFootAccelerationConstraintCppAd(const WbIdFootAccelerationConstraintCppAd& rhs);

  ad_vector_t constraintFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;

  Type type_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<WbIdMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;
  size_t contactIndex_;
  size_t frameId_;
  Gains gains_;
};

}  // namespace ocs2::humanoid
