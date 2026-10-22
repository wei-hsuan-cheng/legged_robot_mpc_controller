// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__ESTIMATOR_STATE_INTERFACES_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__ESTIMATOR_STATE_INTERFACES_HPP_

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimate.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimator_interface.hpp"

namespace legged_robot_mpc_controller::estimator_interfaces
{

struct EstimatorStatus
{
  bool ready{false};          //!< the estimate may drive control
  std::uint32_t epoch{0};     //!< initialization epoch the filter last acted on
  double running_time{0.0};   //!< seconds since that initialization
};

/**
 * The state interfaces an estimator controller exports, as <controller>/<name>.
 * This visitor is the only definition of the layout: names(), pack() and unpack()
 * are all generated from it, so the exporter and the consumer cannot disagree on
 * the order. Works on const and non-const arguments alike.
 *
 * Per-contact diagnostics are not part of it: their names depend on the
 * estimator's contact frames, which a consumer cannot know when it declares the
 * interfaces it claims.
 */
template <class Estimate, class Diagnostics, class Status, class F>
void visit(Estimate & e, Diagnostics & d, Status & s, F && f)
{
  f("position.x", e.position.x());
  f("position.y", e.position.y());
  f("position.z", e.position.z());
  f("orientation.w", e.orientation.w());
  f("orientation.x", e.orientation.x());
  f("orientation.y", e.orientation.y());
  f("orientation.z", e.orientation.z());
  f("linear_velocity_world.x", e.linear_velocity_world.x());
  f("linear_velocity_world.y", e.linear_velocity_world.y());
  f("linear_velocity_world.z", e.linear_velocity_world.z());
  f("angular_velocity.x", e.angular_velocity_local.x());
  f("angular_velocity.y", e.angular_velocity_local.y());
  f("angular_velocity.z", e.angular_velocity_local.z());
  f("valid", e.valid);

  f("status.ready", s.ready);
  f("status.epoch", s.epoch);
  f("status.running_time", s.running_time);

  f("diagnostics.valid", d.valid);
  f("diagnostics.correction_applied", d.correction_applied);
  f("diagnostics.correction_dim", d.correction_dim);
  f("diagnostics.nis", d.nis);
  f("diagnostics.nis_per_dof", d.nis_per_dof);
  f("diagnostics.num_augmented_contacts", d.num_augmented_contacts);
  f("diagnostics.attitude_std.x", d.attitude_std.x());
  f("diagnostics.attitude_std.y", d.attitude_std.y());
  f("diagnostics.attitude_std.z", d.attitude_std.z());
  f("diagnostics.velocity_std.x", d.velocity_std.x());
  f("diagnostics.velocity_std.y", d.velocity_std.y());
  f("diagnostics.velocity_std.z", d.velocity_std.z());
  f("diagnostics.position_std.x", d.position_std.x());
  f("diagnostics.position_std.y", d.position_std.y());
  f("diagnostics.position_std.z", d.position_std.z());
  f("diagnostics.gyroscope_bias_std.x", d.gyroscope_bias_std.x());
  f("diagnostics.gyroscope_bias_std.y", d.gyroscope_bias_std.y());
  f("diagnostics.gyroscope_bias_std.z", d.gyroscope_bias_std.z());
  f("diagnostics.accelerometer_bias_std.x", d.accelerometer_bias_std.x());
  f("diagnostics.accelerometer_bias_std.y", d.accelerometer_bias_std.y());
  f("diagnostics.accelerometer_bias_std.z", d.accelerometer_bias_std.z());
  f("diagnostics.gyroscope_bias.x", d.gyroscope_bias.x());
  f("diagnostics.gyroscope_bias.y", d.gyroscope_bias.y());
  f("diagnostics.gyroscope_bias.z", d.gyroscope_bias.z());
  f("diagnostics.accelerometer_bias.x", d.accelerometer_bias.x());
  f("diagnostics.accelerometer_bias.y", d.accelerometer_bias.y());
  f("diagnostics.accelerometer_bias.z", d.accelerometer_bias.z());
  f("diagnostics.estimating_bias", d.estimating_bias);
  f("diagnostics.inekf_height", d.inekf_height);
  f("diagnostics.kinematic_height", d.kinematic_height);
  f("diagnostics.kinematic_height_valid", d.kinematic_height_valid);
  f("diagnostics.reported_height", d.reported_height);
}

/**
 * The reference interfaces an estimator controller exports, as <controller>/<name>: the
 * consumer's inputs to the filter, written by it as command interfaces. Only the
 * consumer claims anything, so the data flows both ways without a cycle in the
 * controller manager's chaining.
 *  - contact.left / contact.right: 1 = in stance, 0 = in swing.
 *  - initialize_epoch: the filter initializes from stance whenever it changes, and
 *    exports the epoch it acted on as status.epoch. 0 keeps it off.
 */
enum ReferenceIndex : std::size_t { kContactLeft = 0, kContactRight = 1, kInitializeEpoch = 2 };

inline const std::vector<std::string> & reference_names()
{
  static const std::vector<std::string> list = {"contact.left", "contact.right", "initialize_epoch"};
  return list;
}

inline const std::vector<std::string> & names()
{
  static const std::vector<std::string> list = [] {
    std::vector<std::string> n;
    state_estimation::FloatingBaseEstimate e;
    state_estimation::EstimatorDiagnostics d;
    EstimatorStatus s;
    visit(e, d, s, [&n](const char * name, auto &) { n.emplace_back(name); });
    return n;
  }();
  return list;
}

/// values must already have names().size() entries; nothing is allocated.
inline void pack(
  const state_estimation::FloatingBaseEstimate & e, const state_estimation::EstimatorDiagnostics & d,
  const EstimatorStatus & s, std::vector<double> & values)
{
  std::size_t i = 0;
  visit(e, d, s, [&](const char *, const auto & field) { values[i++] = static_cast<double>(field); });
}

inline void unpack(
  const std::vector<double> & values, state_estimation::FloatingBaseEstimate & e,
  state_estimation::EstimatorDiagnostics & d, EstimatorStatus & s)
{
  std::size_t i = 0;
  visit(e, d, s, [&](const char *, auto & field) {
    field = static_cast<std::remove_reference_t<decltype(field)>>(values[i++]);
  });
}

}  // namespace legged_robot_mpc_controller::estimator_interfaces

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_STATE_ESTIMATION__ESTIMATOR_STATE_INTERFACES_HPP_
