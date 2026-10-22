// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_INEKF_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_INEKF_HPP_

#include <memory>

#include "legged_robot_mpc_controller/humanoid_state_estimation/humanoid_state_estimator.hpp"

namespace legged_robot_mpc_controller
{

/**
 * Contact-aided right-invariant EKF floating-base estimator (Hartley et al.
 * 2018). An IMU + contact process model on SE_{N+2}(3), corrected by
 * forward-kinematics position measurements; its error dynamics are log-linear,
 * so convergence does not depend on the operating point - the reason to prefer it
 * over the linearised filter in HumanoidLinearKf.
 *
 * Declares only this filter's parameters, under humanoid_inekf.* on the
 * controller's node, from its humanoid_inekf block in config/g1/ros2_controllers.yaml.
 */
class HumanoidInEkf : public HumanoidStateEstimator
{
public:
  HumanoidInEkf();
  ~HumanoidInEkf() override;

protected:
  state_estimation::EstimatorType estimatorType() const override
  {
    return state_estimation::EstimatorType::InEKF;
  }
  bool declareParameters(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node) override;
  bool readParameters(
    EstimatorCommonSettings & common, state_estimation::EstimatorSettings & filter) override;

private:
  // The generated parameter types are only visible in the .cpp.
  struct Params;
  std::unique_ptr<Params> params_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__HUMANOID_INEKF_HPP_
