#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_id_mpc/initialization/WbIdInitializer.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

WbIdInitializer::WbIdInitializer(PinocchioInterface pinocchioInterface,
                                 const SwitchedModelReferenceManager& referenceManager,
                                 const WbIdMpcRobotModel<scalar_t>& mpcRobotModel)
    : pinocchioInterface_(std::move(pinocchioInterface)), referenceManagerPtr_(&referenceManager), mpcRobotModelPtr_(&mpcRobotModel) {}

vector_t WbIdInitializer::weightCompensatingInput(const vector_t& state, const contact_flag_t& contactFlags) {
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
    tau.noalias() -= jacobian.transpose() * mpcRobotModelPtr_->getContactWrench(input, c);
  }
  mpcRobotModelPtr_->setJointTorques(input, tau.tail(static_cast<Eigen::Index>(mpcRobotModelPtr_->getJointDim())));
  return input;
}

void WbIdInitializer::compute(scalar_t time, const vector_t& state, scalar_t /*nextTime*/, vector_t& input, vector_t& nextState) {
  input = weightCompensatingInput(state, referenceManagerPtr_->getContactFlags(time));
  nextState = state;
}

}  // namespace ocs2::humanoid
