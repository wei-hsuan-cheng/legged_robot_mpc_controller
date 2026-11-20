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
 * Rolling-wheel contact for the WB-ID MPC: the counterpart of WbIdFootAccelerationConstraintCppAd
 * for a wheel instead of a planted foot.
 *
 * The contact frame sits at the tire centre on the wheel link, so it spins with the wheel but its
 * position does not; the contact wrench [f; m] is applied there in world-aligned axes, exactly as
 * for a foot. The tread point touching the ground is p_c = p_h + rho (WheelContactKinematics.h);
 * its material velocity v_c and acceleration a_c = d/dt v_c are what rolling constrains.
 *
 *   Rolling (5 rows, while the wheel is in contact):
 *     ka_t (lon.a_c + lon_dot.v_c) + kd_t lon.v_c             = 0   no longitudinal slip
 *     ka_n n.a_c   + kd_n n.v_c   + kp_n (n.p_c - z_ref)       = 0   no gap
 *     m - rho x f                                              = 0   the force acts at the tire
 *   Lateral (1 row, while any wheel is in contact):
 *     sum_i w_i (ka_t (lat.a_ci + lat_dot.v_ci) + kd_t lat.v_ci) = 0   no mean lateral slip
 *   SwingVertical (1 row, while the wheel is in the air):
 *     ka (a_z - az_ref) + kd (v_z - vz_ref) + kp (z_h - r - z_ref) = 0   tire bottom follows the swing height
 *
 * Lateral is shared because, without hip roll, both wheels' lateral rows are the same function of
 * the base motion in symmetric double support: two such equalities make the projected equality
 * system rank deficient. A common direction from the first wheel avoids cancellation when
 * left/right axle signs are mirrored. This row constrains mean slip, not each wheel separately.
 * w_i = flag_i / sum(flags) is a parameter, so one generated function
 * serves double and single support. Spin, steering by differential wheel speed and the contact
 * point's travel along the tire stay free; moments about the contact point are zero.
 *
 * As in the foot constraint, the acceleration terms give the rows their input dependence and the
 * velocity/position terms are damping and stiffness on the slip/gap error dynamics. The gains,
 * radius and axle are parameters of the generated library, not constants baked into it.
 */
class WbIdWheelContactConstraintCppAd final : public StateInputConstraintCppAd {
 public:
  enum class Type { Rolling, Lateral, SwingVertical };

  struct Settings {
    scalar_t radius{0.055};
    vector3_t axleAxis{0.0, 0.0, 1.0};  // in the contact frame
    scalar_t normalPositionGain{50.0};
    scalar_t normalVelocityGain{50.0};
    scalar_t tangentialVelocityGain{50.0};
    scalar_t normalAccelerationGain{1.0};
    scalar_t tangentialAccelerationGain{1.0};
  };

  /** Rolling and SwingVertical act on one contact; Lateral on all of them (contactIndex ignored). */
  WbIdWheelContactConstraintCppAd(Type type,
                                  const SwitchedModelReferenceManager& referenceManager,
                                  const PinocchioInterface& pinocchioInterface,
                                  const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                  size_t contactIndex,
                                  Settings settings,
                                  const ModelSettings& modelSettings);
  ~WbIdWheelContactConstraintCppAd() override = default;
  WbIdWheelContactConstraintCppAd* clone() const override { return new WbIdWheelContactConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override;
  vector_t getParameters(scalar_t time, const PreComputation& preComputation) const override;

 private:
  WbIdWheelContactConstraintCppAd(const WbIdWheelContactConstraintCppAd& rhs);

  ad_vector_t constraintFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;

  Type type_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<WbIdMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;
  size_t contactIndex_;
  std::vector<size_t> frameIds_;  // all contact frames (Lateral) or the one (Rolling, SwingVertical)
  Settings settings_;
};

}  // namespace ocs2::humanoid
