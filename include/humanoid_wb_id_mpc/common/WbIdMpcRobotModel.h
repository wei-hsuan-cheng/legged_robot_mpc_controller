#pragma once

#include <cassert>
#include <cstddef>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/* State vector of the whole-body inverse-dynamics MPC, x = [q, v] with nq = nv = 6 + nj

    q = [p_base (3, world), eulerZYX (3), q_j (nj)]
    v = [v_base (3, world), eulerZYX rates (3), qd_j (nj)]

  The floating base is pinocchio's Translation + SphericalZYX composite joint, so v is
  exactly dq/dt and the kinematics integrate as qdot = v with no quaternion map.
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
  explicit WbIdMpcRobotModel(const ModelSettings& modelSettings)
      : MpcRobotModelBase<SCALAR_T>(modelSettings,
                                    2 * (6 + modelSettings.mpc_joint_dim),
                                    6 * N_CONTACTS + (6 + modelSettings.mpc_joint_dim) + modelSettings.mpc_joint_dim) {}
  ~WbIdMpcRobotModel() override = default;
  WbIdMpcRobotModel* clone() const override { return new WbIdMpcRobotModel(*this); }

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
    return state.head(this->gen_coordinates_dim);
  }

  VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template head<6>();
  }

  VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template head<3>();
  }

  VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template segment<3>(3);
  }

  //! The pelvis linear velocity in world coordinates. The interface calls it the
  //! "CoM" velocity because the centroidal model carries the CoM velocity there; the
  //! reference manager uses it for the capture-point foothold, where the pelvis
  //! velocity is the same signal the centroidal MPC's momentum stands in for.
  VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template segment<3>(this->gen_coordinates_dim);
  }

  //! [v_base world (3), eulerZYX rates (3)] - Euler RATES, not the angular velocity.
  VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.template segment<6>(this->gen_coordinates_dim);
  }

  VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(getJointStartindex(), this->modelSettings.mpc_joint_dim);
  }

  VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& /*input*/) const override {
    assert(state.size() == this->state_dim);
    return state.tail(this->modelSettings.mpc_joint_dim);
  }

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& /*input*/) override {
    assert(state.size() == this->state_dim);
    return state.tail(this->gen_coordinates_dim);
  }

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    return state.tail(this->gen_coordinates_dim);
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
    assert(generalizedCoordinates.size() == this->gen_coordinates_dim);
    state.head(this->gen_coordinates_dim) = generalizedCoordinates;
  }

  void setGeneralizedVelocities(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedVelocities) const {
    assert(state.size() == this->state_dim);
    assert(generalizedVelocities.size() == this->gen_coordinates_dim);
    state.tail(this->gen_coordinates_dim) = generalizedVelocities;
  }

  void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const override {
    assert(state.size() == this->state_dim);
    state.template head<6>() = basePose;
  }

  void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const override {
    assert(state.size() == this->state_dim);
    state.template head<3>() = position;
  }

  void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const override {
    assert(state.size() == this->state_dim);
    state.template segment<3>(3) = eulerAnglesZYX;
  }

  void setBaseVelocity(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& baseVelocity) const {
    assert(state.size() == this->state_dim);
    state.template segment<6>(this->gen_coordinates_dim) = baseVelocity;
  }

  void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const override {
    assert(state.size() == this->state_dim);
    state.segment(getJointStartindex(), this->modelSettings.mpc_joint_dim) = jointAngles;
  }

  void setJointVelocities(VECTOR_T<SCALAR_T>& state, VECTOR_T<SCALAR_T>& /*input*/, const VECTOR_T<SCALAR_T>& jointVelocities) const override {
    assert(state.size() == this->state_dim);
    state.tail(this->modelSettings.mpc_joint_dim) = jointVelocities;
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

 private:
  WbIdMpcRobotModel(const WbIdMpcRobotModel& rhs) : MpcRobotModelBase<SCALAR_T>(rhs) {}
};

}  // namespace ocs2::humanoid
