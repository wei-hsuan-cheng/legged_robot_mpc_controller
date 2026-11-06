// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#include "legged_robot_mpc_controller/humanoid_whole_body_inverse_dynamics_mpc/gait_adaptation_policy.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace legged_robot_mpc_controller
{

namespace
{

void expect(std::istream & in, const std::string & token)
{
  std::string word;
  if (!(in >> word) || word != token) {
    throw std::runtime_error("gait adaptation policy file: expected '" + token + "', got '" + word + "'");
  }
}

Eigen::VectorXd read_vector(std::istream & in, Eigen::Index n)
{
  Eigen::VectorXd v(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    if (!(in >> v[i])) {
      throw std::runtime_error("gait adaptation policy file: truncated");
    }
  }
  return v;
}

}  // namespace

GaitAdaptationPolicy::GaitAdaptationPolicy(const std::string & file, std::size_t obs_dim, std::size_t act_dim)
{
  std::ifstream in(file);
  if (!in) {
    throw std::runtime_error("cannot open gait adaptation policy " + file);
  }
  int version = 0;
  std::size_t n = 0, m = 0, k = 0;
  expect(in, "gait_adaptation_policy");
  in >> version;
  expect(in, "obs_dim");
  in >> n;
  expect(in, "act_dim");
  in >> m;
  expect(in, "obs_clip");
  in >> obs_clip_;
  if (version != 1 || n != obs_dim || m != act_dim) {
    throw std::runtime_error("gait adaptation policy " + file + ": version " + std::to_string(version) + ", obs " +
                             std::to_string(n) + ", act " + std::to_string(m) + "; the controller needs version 1, obs " +
                             std::to_string(obs_dim) + ", act " + std::to_string(act_dim));
  }
  expect(in, "obs_mean");
  obs_mean_ = read_vector(in, static_cast<Eigen::Index>(n));
  expect(in, "obs_std");
  obs_std_ = read_vector(in, static_cast<Eigen::Index>(n));
  expect(in, "layers");
  in >> k;
  std::size_t width = n;
  for (std::size_t l = 0; l < k; ++l) {
    std::size_t rows_in = 0, rows_out = 0;
    std::string activation;
    expect(in, "layer");
    in >> rows_in >> rows_out >> activation;
    if (rows_in != width || (activation != "elu" && activation != "linear")) {
      throw std::runtime_error("gait adaptation policy " + file + ": bad layer " + std::to_string(l));
    }
    Layer layer;
    layer.elu = activation == "elu";
    layer.weight.resize(static_cast<Eigen::Index>(rows_out), static_cast<Eigen::Index>(rows_in));
    for (Eigen::Index r = 0; r < layer.weight.rows(); ++r) {
      layer.weight.row(r) = read_vector(in, layer.weight.cols()).transpose();
    }
    layer.bias = read_vector(in, static_cast<Eigen::Index>(rows_out));
    layers_.push_back(std::move(layer));
    activations_.emplace_back(static_cast<Eigen::Index>(rows_out));
    width = rows_out;
  }
  if (layers_.empty() || width != act_dim) {
    throw std::runtime_error("gait adaptation policy " + file + ": the last layer must have act_dim outputs");
  }
  input_.resize(static_cast<Eigen::Index>(n));
}

void GaitAdaptationPolicy::evaluate(const std::vector<double> & observation, std::vector<double> & action)
{
  for (Eigen::Index i = 0; i < input_.size(); ++i) {
    const double x = std::isfinite(observation[static_cast<std::size_t>(i)]) ? observation[static_cast<std::size_t>(i)] : 0.0;
    input_[i] = std::clamp((x - obs_mean_[i]) / obs_std_[i], -obs_clip_, obs_clip_);
  }
  const Eigen::VectorXd * x = &input_;
  for (std::size_t l = 0; l < layers_.size(); ++l) {
    auto & y = activations_[l];
    y.noalias() = layers_[l].weight * *x;
    y += layers_[l].bias;
    if (layers_[l].elu) {
      y = y.unaryExpr([](double v) { return v > 0.0 ? v : std::expm1(v); });
    }
    x = &y;
  }
  action.resize(static_cast<std::size_t>(x->size()));
  for (Eigen::Index i = 0; i < x->size(); ++i) {
    action[static_cast<std::size_t>(i)] = std::tanh((*x)[i]);
  }
}

}  // namespace legged_robot_mpc_controller
