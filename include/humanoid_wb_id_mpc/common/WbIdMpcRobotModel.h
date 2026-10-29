#pragma once

// Register Pinocchio's AD/CG specializations before instantiating manifold functions.
#include <pinocchio/codegen/cppadcg.hpp>

#include <cassert>
#include <cstddef>
#include <Eigen/Geometry>
#include <pinocchio/spatial/explog.hpp>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/* State vector of the whole-body inverse-dynamics MPC, nv = 6 + nj.

  Euler mode: x = [q, v], q = [p_world, eulerZYX, q_j],
  v = [v_world, eulerZYX rates, qd_j]; Translation + SphericalZYX, qdot = v.

  Tangent mode: x = [p_world, phi, q_j, v_world, omega_bodyXYZ, qd_j, anchorXYZW].
  R = R_anchor Exp(phi); Translation + Spherical uses quaternion configuration storage.
  The measured anchor has zero flow, and phidot = Jr(phi)^-1 omega_bodyXYZ.
*/
/******************************************************************************************************/
/* Input vector, u = [W_l (6), W_r (6), a (6 + nj), tau_j (nj)]

    W_c    contact wrench [f; m] at contact c, world-aligned axes
    a      generalized acceleration (base and joints), a = dv/dt
    tau_j  joint torques

  (q, v, a, W, tau) are tied together by the inverse dynamics equality constraint
  RNEA(q, v, a) - sum_c J_c^T W_c - [0_6; tau_j] = 0, so the input is redundant by
  design: the solver eliminates it through the state-input constraint projection.
*/
/******************************************************************************************************/

template <typename SCALAR_T>
class WbIdMpcRobotModel : public MpcRobotModelBase<SCALAR_T> {
 public:
  explicit WbIdMpcRobotModel(const ModelSettings& modelSettings, bool tangentSpace = false)
      : MpcRobotModelBase<SCALAR_T>(modelSettings,
                                    2 * (6 + modelSettings.mpc_joint_dim) + (tangentSpace ? 4 : 0),
                                    6 * N_CONTACTS + (6 + modelSettings.mpc_joint_dim) + modelSettings.mpc_joint_dim),
        tangentSpace_(tangentSpace) {}
  ~WbIdMpcRobotModel() override = default;
  WbIdMpcRobotModel* clone() const override { return new WbIdMpcRobotModel(*this); }

  bool usesTangentSpace() const { return tangentSpace_; }
  size_t getAnchorStartIndex() const { return 2 * this->gen_coordinates_dim; }

  Eigen::Quaternion<SCALAR_T> getBaseQuaternion(const VECTOR_T<SCALAR_T>& state) const {
    if (!tangentSpace_) {
      return matrixToQuaternion(getRotationMatrixFromZyxEulerAngles(
          VECTOR3_T<SCALAR_T>(state.template segment<3>(3))));
    }
    Eigen::Quaternion<SCALAR_T> anchor(state.template tail<4>());
    using std::sqrt;
    // Eigen::normalize() branches on the scalar value, which is illegal while
    // recording a CG variable. Measurements are unit length; keep AD algebraic.
    anchor.coeffs() /= sqrt(anchor.squaredNorm() + SCALAR_T(1e-32));
    return anchor * pinocchio::quaternion::exp3(VECTOR3_T<SCALAR_T>(state.template segment<3>(3)));
  }

  //! Only the three rotation increments are free. The anchor is fixed by the
  //! initial condition and zero dynamics; no unit-quaternion constraint is solved.
  void rebase(VECTOR_T<SCALAR_T>& state, const Eigen::Quaternion<SCALAR_T>& anchor) const {
    if (!tangentSpace_) return;
    const auto orientation = getBaseQuaternion(state);
    state.template segment<3>(3) = pinocchio::quaternion::log3(Eigen::Quaternion<SCALAR_T>(anchor.conjugate() * orientation));
    state.template tail<4>() = anchor.coeffs();
  }

  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  size_t getBaseStartindex() const override { return 0; }
  size_t getBaseComVelocityStartindex() const override { return this->gen_coordinates_dim; }
  size_t getJointStartindex() const override { return 6; }
  //! Into the STATE: the joint velocities are part of the state here, not the input.
  size_t getJointVelocitiesStartindex() const override { return this->gen_coordinates_dim + 6; }
  size_t getGeneralizedAccelerationStartIndex() const { return 6 * N_CONTACTS; }
  size_t getJointAccelerationStartIndex() const { return getGeneralizedAccelerationStartIndex() + 6; }
  size_t getJointTorqueStartIndex() const { return getGeneralizedAccelerationStartIndex() + this->gen_coordinates_dim; }

  size_t getContactWrenchStartIndices(size_t contactIndex) const override { return 6 * contactIndex; }
  size_t getContactForceStartIndices(size_t contactIndex) const override { return 6 * contactIndex; }
  size_t getContactMomentStartIndices(size_t contactIndex) const override { return 6 * contactIndex + 3; }

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    if (tangentSpace_) {
      VECTOR_T<SCALAR_T> q(this->gen_coordinates_dim + 1);
      q.template head<3>() = state.template head<3>();
      q.template segment<4>(3) = getBaseQuaternion(state).coeffs();
      q.tail(this->getJointDim()) = getJointAngles(state);
      return q;
    }
    return state.head(this->gen_coordinates_dim);
  }

  VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    VECTOR6_T<SCALAR_T> pose;
    pose << getBasePosition(state), getBaseOrientationEulerZYX(state);
    return pose;
  }

  VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template head<3>();
  }

  VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    if (!tangentSpace_) return state.template segment<3>(3);
    const auto rotation = getBaseQuaternion(state).toRotationMatrix();
    using std::atan2;
    using std::sqrt;
    VECTOR3_T<SCALAR_T> euler;
    euler << atan2(rotation(1, 0), rotation(0, 0)),
        atan2(-rotation(2, 0), sqrt(rotation(2, 1) * rotation(2, 1) + rotation(2, 2) * rotation(2, 2))),
        atan2(rotation(2, 1), rotation(2, 2));
    return euler;
  }

  //! The pelvis linear velocity in world coordinates. The interface calls it the
  //! "CoM" velocity because the centroidal model carries the CoM velocity there; the
  //! reference manager uses it for the capture-point foothold, where the pelvis
  //! velocity is the same signal the centroidal MPC's momentum stands in for.
  VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template segment<3>(this->gen_coordinates_dim);
  }

  //! Logical reference-manager order: world linear, then ZYX Euler rates in Euler
  //! mode, or world angular velocity reversed to ZYX in tangent mode.
  VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    VECTOR6_T<SCALAR_T> velocity = state.template segment<6>(this->gen_coordinates_dim);
    if (tangentSpace_) {
      const VECTOR3_T<SCALAR_T> omegaWorld = getBaseQuaternion(state) * VECTOR3_T<SCALAR_T>(velocity.template tail<3>());
      velocity.template tail<3>() = omegaWorld.reverse();
    }
    return velocity;
  }

  VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(getJointStartindex(), this->modelSettings.mpc_joint_dim);
  }

  VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& /*input*/) const override {
    assert(state.size() == this->state_dim);
    return state.segment(getJointVelocitiesStartindex(), this->modelSettings.mpc_joint_dim);
  }

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& /*input*/) override {
    assert(state.size() == this->state_dim);
    return state.segment(this->gen_coordinates_dim, this->gen_coordinates_dim);
  }

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    return state.segment(this->gen_coordinates_dim, this->gen_coordinates_dim);
  }

  VECTOR_T<SCALAR_T> getGeneralizedAccelerations(const VECTOR_T<SCALAR_T>& input) const {
    assert(input.size() == this->input_dim);
    return input.segment(getGeneralizedAccelerationStartIndex(), this->gen_coordinates_dim);
  }

  VECTOR_T<SCALAR_T> getJointTorques(const VECTOR_T<SCALAR_T>& input) const {
    assert(input.size() == this->input_dim);
    return input.tail(this->modelSettings.mpc_joint_dim);
  }

  void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCoordinates) const override {
    assert(state.size() == this->state_dim);
    if (tangentSpace_) {
      assert(generalizedCoordinates.size() == this->gen_coordinates_dim + 1);
      state.template head<3>() = generalizedCoordinates.template head<3>();
      state.template segment<3>(3).setZero();
      state.segment(6, this->getJointDim()) = generalizedCoordinates.tail(this->getJointDim());
      state.template tail<4>() = generalizedCoordinates.template segment<4>(3);
      return;
    }
    assert(generalizedCoordinates.size() == this->gen_coordinates_dim);
    state.head(this->gen_coordinates_dim) = generalizedCoordinates;
  }

  void setGeneralizedVelocities(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedVelocities) const {
    assert(state.size() == this->state_dim);
    assert(generalizedVelocities.size() == this->gen_coordinates_dim);
    state.segment(this->gen_coordinates_dim, this->gen_coordinates_dim) = generalizedVelocities;
  }

  void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const override {
    assert(state.size() == this->state_dim);
    setBasePosition(state, VECTOR3_T<SCALAR_T>(basePose.template head<3>()));
    setBaseOrientationEulerZYX(state, VECTOR3_T<SCALAR_T>(basePose.template tail<3>()));
  }

  void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const override {
    assert(state.size() == this->state_dim);
    state.template head<3>() = position;
  }

  void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const override {
    assert(state.size() == this->state_dim);
    if (tangentSpace_) {
      state.template segment<3>(3).setZero();
      state.template tail<4>() = matrixToQuaternion(getRotationMatrixFromZyxEulerAngles(eulerAnglesZYX)).coeffs();
    } else {
      state.template segment<3>(3) = eulerAnglesZYX;
    }
  }

  void setBaseVelocity(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& baseVelocity) const {
    assert(state.size() == this->state_dim);
    state.template segment<6>(this->gen_coordinates_dim) = baseVelocity;
    if (tangentSpace_) {
      state.template segment<3>(this->gen_coordinates_dim + 3) =
          getBaseQuaternion(state).conjugate() * VECTOR3_T<SCALAR_T>(baseVelocity.template tail<3>().reverse());
    }
  }

  void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const override {
    assert(state.size() == this->state_dim);
    state.segment(getJointStartindex(), this->modelSettings.mpc_joint_dim) = jointAngles;
  }

  void setJointVelocities(VECTOR_T<SCALAR_T>& state, VECTOR_T<SCALAR_T>& /*input*/, const VECTOR_T<SCALAR_T>& jointVelocities) const override {
    assert(state.size() == this->state_dim);
    state.segment(getJointVelocitiesStartindex(), this->modelSettings.mpc_joint_dim) = jointVelocities;
  }

  void setGeneralizedAccelerations(VECTOR_T<SCALAR_T>& input, const VECTOR_T<SCALAR_T>& accelerations) const {
    assert(input.size() == this->input_dim);
    input.segment(getGeneralizedAccelerationStartIndex(), this->gen_coordinates_dim) = accelerations;
  }

  void setJointTorques(VECTOR_T<SCALAR_T>& input, const VECTOR_T<SCALAR_T>& torques) const {
    assert(input.size() == this->input_dim);
    input.tail(this->modelSettings.mpc_joint_dim) = torques;
  }

  void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const override {
    assert(state.size() == this->state_dim);
    state[2] += heightChange;
  }

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.template segment<6>(getContactWrenchStartIndices(contactIndex));
  }

  VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.template segment<3>(getContactForceStartIndices(contactIndex));
  }

  VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.template segment<3>(getContactMomentStartIndices(contactIndex));
  }

  void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.template segment<6>(getContactWrenchStartIndices(contactIndex)) = wrench;
  }

  void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.template segment<3>(getContactForceStartIndices(contactIndex)) = force;
  }

  void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.template segment<3>(getContactMomentStartIndices(contactIndex)) = moment;
  }

 public:
  WbIdMpcRobotModel(const WbIdMpcRobotModel& rhs) : MpcRobotModelBase<SCALAR_T>(rhs), tangentSpace_(rhs.tangentSpace_) {}
 private:
  bool tangentSpace_;
};

}  // namespace ocs2::humanoid
