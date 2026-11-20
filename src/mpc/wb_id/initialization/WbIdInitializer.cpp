#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_robot_mpc_controller/mpc/wb_id/initialization/WbIdInitializer.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <cmath>

#include "legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WheelContactKinematics.h"

namespace ocs2::humanoid {

WbIdInitializer::WbIdInitializer(PinocchioInterface pinocchioInterface,
                                 const SwitchedModelReferenceManager& referenceManager,
                                 const WbIdMpcRobotModel<scalar_t>& mpcRobotModel,
                                 scalar_t wheelRadius,
                                 vector3_t wheelAxleAxis)
    : pinocchioInterface_(std::move(pinocchioInterface)), referenceManagerPtr_(&referenceManager), mpcRobotModelPtr_(&mpcRobotModel),
      wheelRadius_(wheelRadius), wheelAxleAxis_(wheelAxleAxis.normalized()) {}

vector_t WbIdInitializer::weightCompensatingInput(const vector_t& state, const contact_flag_t& contactFlags, scalar_t time) {
  vector_t input = ocs2::humanoid::weightCompensatingInput(pinocchioInterface_, contactFlags, *mpcRobotModelPtr_);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(state);
  const vector_t v = mpcRobotModelPtr_->getGeneralizedVelocities(state);
  vector_t tau = pinocchio::rnea(model, data, q, v, vector_t::Zero(model.nv));

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  matrix_t jacobian(6, model.nv);
  for (size_t c = 0; c < N_CONTACTS; ++c) {
    if (!contactFlags[c]) {
      continue;
    }
    jacobian.setZero();
    pinocchio::getFrameJacobian(model, data, getContactFrameIndex(pinocchioInterface_, *mpcRobotModelPtr_, c),
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    if (wheelRadius_ > 0.0) {
      const auto frame = getContactFrameIndex(pinocchioInterface_, *mpcRobotModelPtr_, c);
      const vector3_t axle = data.oMf[frame].rotation() * wheelAxleAxis_;
      const vector3_t normal = referenceManagerPtr_->getContactNormal(c, time);
      const vector3_t rho = wheel::contactOffset<scalar_t>(axle, normal, wheelRadius_);
      mpcRobotModelPtr_->setContactMoment(input, rho.cross(mpcRobotModelPtr_->getContactForce(input, c)), c);
    }
    tau.noalias() -= jacobian.transpose() * mpcRobotModelPtr_->getContactWrench(input, c);
  }
  const auto& settings = mpcRobotModelPtr_->modelSettings;
  const auto nj = static_cast<Eigen::Index>(settings.jointFriction.size());
  for (Eigen::Index i = 0; i < nj; ++i) {
    tau(model.nv - nj + i) += settings.jointFriction[static_cast<size_t>(i)] *
                             std::tanh(v(model.nv - nj + i) / settings.jointFrictionVelocity);
  }
  mpcRobotModelPtr_->setJointTorques(input, tau.tail(static_cast<Eigen::Index>(mpcRobotModelPtr_->getJointDim())));
  return input;
}

void WbIdInitializer::compute(scalar_t time, const vector_t& state, scalar_t /*nextTime*/, vector_t& input, vector_t& nextState) {
  input = weightCompensatingInput(state, referenceManagerPtr_->getContactFlags(time), time);
  nextState = state;
}

}  // namespace ocs2::humanoid
