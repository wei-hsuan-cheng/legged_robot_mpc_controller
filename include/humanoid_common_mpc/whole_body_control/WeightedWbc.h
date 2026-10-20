/******************************************************************************
Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/contact/ContactPolygon.h"
#include "humanoid_common_mpc/whole_body_control/HpipmDenseQp.h"
#include "humanoid_common_mpc/whole_body_control/Task.h"

namespace ocs2::humanoid {

/** Tunable part of the whole-body controller; filled from the controller's ROS parameters. */
struct WbcSettings {
  // --- hard constraints -----------------------------------------------------
  scalar_t frictionCoefficient{0.7};
  scalar_t torsionalFrictionCoefficient{0.1};  //!< |m_z| <= this * f_z
  scalar_t minContactForceZ{10.0};             //!< keeps a stance foot loaded, so the cone stays well posed
  scalar_t copMarginScale{0.9};                //!< shrinks the foot rectangle used for the CoP bound
  vector_t torqueLimits{};                     //!< per joint, symmetric; empty means unbounded

  // --- task gains -----------------------------------------------------------
  scalar_t baseKp{100.0}, baseKd{20.0};    //!< base pose / twist -> base acceleration
  scalar_t jointKp{100.0}, jointKd{20.0};  //!< joint position / velocity -> joint acceleration
  scalar_t swingKp{350.0}, swingKd{37.0};  //!< swing foot pose / twist -> foot acceleration

  // --- task weights (relative; the QP minimises the weighted square sum) -----
  scalar_t weightBaseAccel{200.0};
  scalar_t weightJointAccel{1.0};
  scalar_t weightSwing{100.0};
  scalar_t weightContactWrench{1.0};
  scalar_t weightTorqueReg{1e-3};  //!< regularisation, also makes the Hessian positive definite
  scalar_t weightAccelReg{1e-4};
};

/**
 * Weighted whole-body controller for a biped with two flat (6-DoF) contacts.
 *
 * Replaces the closed-form inverse dynamics in the inner loop. Instead of
 * evaluating RNEA at the MPC's optimised wrenches, it solves one QP per control
 * tick over
 *
 *     x = [ qddot (nv) | w_0 w_1 (6 each) | tau (nj) ]
 *
 * subject to the floating-base equation of motion, the stance contact
 * constraints, friction / CoP / torsional limits and the torque limits, while
 * tracking the MPC solution in a least-squares sense. The advantage over the
 * closed-form version is that the redundancy is resolved against the MEASURED
 * state with the physical limits enforced, rather than applying the planned
 * wrench open loop and hoping it is feasible.
 *
 * Differences from the quadruped reference (qiayuanl/legged_control), which
 * drove the design:
 *   - Contacts are 6-DoF wrenches on flat feet, not 3-DoF point forces, so the
 *     friction cone gains CoP and torsional-friction rows and the contact
 *     Jacobians are the full 6 rows.
 *   - Two contacts instead of four.
 *   - Base and joint references are tracked with an acceleration-level PD on the
 *     MPC policy rather than through a centroidal-momentum-rate task that needs
 *     the desired input to be differentiated numerically.
 *   - HPIPM solves the QP, because qpOASES is not part of this workspace.
 */
class WeightedWbc {
 public:
  struct Solution {
    vector_t jointTorques;                      //!< tau, size getJointDim()
    vector_t jointAccelerations;                //!< qddot of the actuated joints
    feet_array_t<vector6_t> contactWrenches{};  //!< as resolved by the QP, LOCAL_WORLD_ALIGNED
    bool success{false};                        //!< false means the QP did not converge
    int iterations{0};
  };

  WeightedWbc(PinocchioInterface pinocchioInterface,
              const MpcRobotModelBase<scalar_t>& mpcRobotModel,
              const ModelSettings& modelSettings,
              WbcSettings settings);

  ~WeightedWbc();

  WeightedWbc(const WeightedWbc&) = delete;
  WeightedWbc& operator=(const WeightedWbc&) = delete;

  /**
   * One control tick.
   *
   * @param stateDesired  MPC policy state at this time
   * @param inputDesired  MPC policy input at this time (carries the contact wrenches)
   * @param qMeasured     measured generalized coordinates, pinocchio ordering
   * @param vMeasured     measured generalized velocities, pinocchio ordering
   * @param mode          planned contact mode, as returned by the MRT policy
   */
  Solution update(const vector_t& stateDesired,
                  const vector_t& inputDesired,
                  const vector_t& qMeasured,
                  const vector_t& vMeasured,
                  size_t mode);

  const WbcSettings& settings() const { return settings_; }

 private:
  void updateMeasured(const vector_t& qMeasured, const vector_t& vMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  // Hard constraints.
  Task formulateFloatingBaseEomTask() const;
  Task formulateNoContactMotionTask() const;
  Task formulateZeroSwingWrenchTask() const;
  Task formulateContactWrenchConeTask() const;

  // Weighted objectives.
  Task formulateBaseAccelTask() const;
  Task formulateJointAccelTask() const;
  Task formulateSwingFootTask() const;
  Task formulateContactWrenchTask() const;
  Task formulateRegularisationTask() const;

  size_t numDecisionVars() const { return numDecisionVars_; }
  size_t wrenchStartIndex(size_t contact) const { return nv_ + 6 * contact; }
  size_t torqueStartIndex() const { return nv_ + 6 * N_CONTACTS; }

  PinocchioInterface pinocchioMeasured_;
  PinocchioInterface pinocchioDesired_;
  std::unique_ptr<MpcRobotModelBase<scalar_t>> mpcRobotModel_;
  // ModelSettings is not copyable and is only needed while constructing: the
  // contact frame ids and foot rectangles below are resolved from it once.
  WbcSettings settings_;

  size_t nv_{0};  //!< generalized velocity dimension (6 + joints)
  size_t nj_{0};  //!< actuated joints
  size_t numDecisionVars_{0};

  std::array<size_t, N_CONTACTS> contactFrameIds_{};
  //! Foot rectangle per contact, resolved once: the CoP bound needs it every tick.
  std::vector<PolygonBounds> footBounds_;
  feet_array_t<bool> contactFlags_{};
  size_t numStanceContacts_{0};

  // Measured quantities, refreshed every tick.
  vector_t qMeasured_, vMeasured_;
  matrix_t j_;   //!< stacked 6-row contact Jacobians, (6 * N_CONTACTS) x nv
  matrix_t dj_;  //!< their time derivative

  // Desired quantities from the policy.
  vector_t qDesired_, vDesired_;
  feet_array_t<vector3_t> desiredFootPosition_{};
  feet_array_t<matrix3_t> desiredFootOrientation_{};
  feet_array_t<vector6_t> desiredFootVelocity_{};
  feet_array_t<vector6_t> desiredContactWrench_{};

  std::unique_ptr<HpipmDenseQp> qp_;
};

}  // namespace ocs2::humanoid
