// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__GAIT_ADAPTATION_POLICY_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__GAIT_ADAPTATION_POLICY_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

namespace legged_robot_mpc_controller
{

/**
 * A trained gait-adaptation policy, evaluated in the controller (no Python at deployment):
 * an MLP from the controller's policy observation to tanh-squashed actions in [-1, 1] - the
 * deterministic action of the SAC actor written by rl/export_policy.py. Text file:
 *
 *   gait_adaptation_policy 1
 *   obs_dim <n> act_dim <m> obs_clip <c>
 *   obs_mean <n values>  obs_std <n values>
 *   layers <k>
 *   layer <in> <out> <elu|linear>  <out*in weights, row-major> <out biases>   (k times)
 *
 * The last layer is the action mean, before the tanh.
 */
class GaitAdaptationPolicy
{
public:
  //! Throws std::runtime_error on a malformed file or a dimension mismatch.
  GaitAdaptationPolicy(const std::string & file, std::size_t obs_dim, std::size_t act_dim);

  //! observation: the raw controller observation (normalized here). No allocation.
  void evaluate(const std::vector<double> & observation, std::vector<double> & action);

private:
  struct Layer
  {
    Eigen::MatrixXd weight;
    Eigen::VectorXd bias;
    bool elu{false};
  };
  Eigen::VectorXd obs_mean_;
  Eigen::VectorXd obs_std_;
  double obs_clip_{5.0};
  std::vector<Layer> layers_;
  std::vector<Eigen::VectorXd> activations_;  //!< preallocated, one per layer output
  Eigen::VectorXd input_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__GAIT_ADAPTATION_POLICY_HPP_
