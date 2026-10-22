#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"
#include "humanoid_wb_id_mpc/constraint/WbIdTorqueWindow.h"

namespace ocs2::humanoid {

/**
 * The whole-body inverse dynamics as a path equality constraint (Molnar et al., RA-L 2025;
 * Khazoom et al., RA-L 2024):
 *
 *   r(q, v, a, W, tau) = RNEA(q, v, a) - sum_c J_c(q)^T W_c - [0_6; tau_j]
 *
 * with J_c the 6 x nv LOCAL_WORLD_ALIGNED Jacobian of contact frame c, matching the
 * world-aligned [f; m] wrench convention the rest of the stack uses (computeJointTorques).
 *
 * Split by rows, as in both papers:
 *   Base   r[0:6]  = 0 at every node - the unactuated floating base, i.e. the Newton-Euler
 *                   (centroidal) equations. This is what makes the plan dynamically feasible.
 *   Joint  r[6:]   = 0 inside the torque window only - defines the joint torques where they
 *                   are actually used (the first nodes are what the robot receives), and
 *                   where the torque limits apply. Outside it tau is a free, regularised
 *                   input that nothing else depends on.
 */
class WbIdRneaConstraintCppAd final : public StateInputConstraintCppAd {
 public:
  enum class Rows { Base, Joint };

  WbIdRneaConstraintCppAd(Rows rows,
                          const PinocchioInterface& pinocchioInterface,
                          const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                          const ModelSettings& modelSettings,
                          std::shared_ptr<const WbIdTorqueWindow> torqueWindow = nullptr);
  ~WbIdRneaConstraintCppAd() override = default;
  WbIdRneaConstraintCppAd* clone() const override { return new WbIdRneaConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override { return numConstraints_; }

 private:
  WbIdRneaConstraintCppAd(const WbIdRneaConstraintCppAd& rhs);

  ad_vector_t constraintFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;

  Rows rows_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<WbIdMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;
  std::vector<size_t> contactFrameIds_;
  size_t numConstraints_;
  std::shared_ptr<const WbIdTorqueWindow> torqueWindow_;
};

}  // namespace ocs2::humanoid
