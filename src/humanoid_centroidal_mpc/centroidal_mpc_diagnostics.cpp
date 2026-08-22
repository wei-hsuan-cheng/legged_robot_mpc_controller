#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/centroidal_mpc_diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/ComputationRequest.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

namespace legged_robot_mpc_controller
{

namespace
{

/// Replace a "%t" placeholder in the path prefix with the local start time, so a
/// rerun does not silently overwrite the log it is meant to be compared against.
std::string expandPathPrefix(const std::string & prefix)
{
  const auto marker = prefix.find("%t");
  if (marker == std::string::npos) {
    return prefix;
  }
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local{};
  localtime_r(&now, &local);
  std::ostringstream stamp;
  stamp << std::put_time(&local, "%Y%m%d_%H%M%S");
  return prefix.substr(0, marker) + stamp.str() + prefix.substr(marker + 2);
}

void addVector3Columns(
  std::vector<std::string> & columns, const std::string & name)
{
  columns.emplace_back(name + "_x");
  columns.emplace_back(name + "_y");
  columns.emplace_back(name + "_z");
}

void addMomentumColumns(std::vector<std::string> & columns, const std::string & name)
{
  for (const char * axis : {"lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"}) {
    columns.emplace_back(name + "_" + axis);
  }
}

/// Write a 3-vector into consecutive row slots, advancing the cursor.
void put(double * row, std::size_t & cursor, const Eigen::Vector3d & v)
{
  row[cursor++] = v.x();
  row[cursor++] = v.y();
  row[cursor++] = v.z();
}

void put(double * row, std::size_t & cursor, const ocs2::vector_t & v)
{
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    row[cursor++] = v[i];
  }
}

void put(double * row, std::size_t & cursor, double value)
{
  row[cursor++] = value;
}

void put(double * row, std::size_t & cursor, bool value)
{
  row[cursor++] = value ? 1.0 : 0.0;
}

}  // namespace

CentroidalMpcDiagnostics::CentroidalMpcDiagnostics(
  Settings settings,
  const ocs2::humanoid::CentroidalMpcInterface & mpc_interface,
  const ocs2::PinocchioInterface & pinocchio_interface,
  std::vector<std::string> contact_frames)
: settings_(std::move(settings)),
  mpc_interface_(&mpc_interface),
  pinocchio_(pinocchio_interface),
  info_(mpc_interface.getCentroidalModelInfo()),
  contact_frames_(std::move(contact_frames))
{
  if (!settings_.enabled) {
    return;
  }

  // Deep copy of the problem: the cost terms carry their own PreComputation and
  // their own AD models, so evaluating them here cannot disturb the solver.
  problem_ = std::make_unique<ocs2::OptimalControlProblem>(mpc_interface.getOptimalControlProblem());

  running_cost_names_ = mpc_interface.getCostNames();
  terminal_cost_names_ = mpc_interface.getTerminalCostNames();
  // getCostNames() enumerates an unordered_map, so the order is unspecified.
  // Sort it, otherwise the CSV column order changes between runs and two logs
  // cannot be diffed column-wise.
  std::sort(running_cost_names_.begin(), running_cost_names_.end());
  std::sort(terminal_cost_names_.begin(), terminal_cost_names_.end());

  const auto joint_dim = static_cast<Eigen::Index>(info_.actuatedDofNum);
  const auto gen_dim = static_cast<Eigen::Index>(info_.generalizedCoordinatesNum);
  q_scratch_ = ocs2::vector_t::Zero(gen_dim);
  v_scratch_ = ocs2::vector_t::Zero(gen_dim);
  momentum_estimate_ = ocs2::vector_t::Zero(6);
  momentum_ground_truth_ = ocs2::vector_t::Zero(6);
  static_cast<void>(joint_dim);

  buildStateSchema();
  buildCostSchema();
}

CentroidalMpcDiagnostics::~CentroidalMpcDiagnostics()
{
  stop();
}

void CentroidalMpcDiagnostics::buildStateSchema()
{
  auto & c = state_columns_;
  c.clear();
  c.emplace_back("t");
  c.emplace_back("phase");
  c.emplace_back("mode");
  c.emplace_back("contact_left");
  c.emplace_back("contact_right");

  addVector3Columns(c, "gt_p");
  addVector3Columns(c, "gt_rpy");
  addVector3Columns(c, "gt_v_world");
  addVector3Columns(c, "gt_w_local");

  addVector3Columns(c, "est_p");
  addVector3Columns(c, "est_rpy");
  addVector3Columns(c, "est_v_world");
  addVector3Columns(c, "est_w_local");

  addVector3Columns(c, "err_p");
  addVector3Columns(c, "err_rpy");
  addVector3Columns(c, "err_v_world");
  addVector3Columns(c, "err_w_local");

  // Momentum through the SAME A_G(q) from both feedback sources: the difference
  // is exactly the error the estimator injects into the dominant MPC state.
  addMomentumColumns(c, "hbar_est");
  addMomentumColumns(c, "hbar_gt");
  addMomentumColumns(c, "hbar_err");

  addVector3Columns(c, "P_att_std");
  addVector3Columns(c, "P_vel_std");
  addVector3Columns(c, "P_pos_std");
  addVector3Columns(c, "P_bg_std");
  addVector3Columns(c, "P_ba_std");
  addVector3Columns(c, "bias_gyro");
  addVector3Columns(c, "bias_accel");
  c.emplace_back("estimating_bias");

  c.emplace_back("corr_applied");
  c.emplace_back("corr_dim");
  c.emplace_back("nis");
  c.emplace_back("nis_per_dof");
  c.emplace_back("num_augmented_contacts");

  for (const auto & frame : contact_frames_) {
    addVector3Columns(c, frame + "_innov");
    addVector3Columns(c, frame + "_innov_std");
    addVector3Columns(c, frame + "_landmark");
    c.emplace_back(frame + "_corrected");
    c.emplace_back(frame + "_added");
    c.emplace_back(frame + "_removed");
    c.emplace_back(frame + "_in_stance");
    c.emplace_back(frame + "_landmark_valid");
    c.emplace_back(frame + "_ground_anchor");
  }

  c.emplace_back("height_inekf");
  c.emplace_back("height_kinematic");
  c.emplace_back("height_kinematic_valid");
  c.emplace_back("height_reported");
}

void CentroidalMpcDiagnostics::buildCostSchema()
{
  auto & c = cost_columns_;
  c.clear();
  c.emplace_back("t");
  c.emplace_back("mode");
  c.emplace_back("advance_ms");
  c.emplace_back("running_cost_total");
  c.emplace_back("terminal_cost_total");

  // The state the terms are evaluated at and the reference they compare against,
  // so a term's value can be read together with the error that produced it.
  addMomentumColumns(c, "x_hbar");
  c.emplace_back("x_base_x");
  c.emplace_back("x_base_y");
  c.emplace_back("x_base_z");
  c.emplace_back("x_base_yaw");
  c.emplace_back("x_base_pitch");
  c.emplace_back("x_base_roll");
  addMomentumColumns(c, "ref_hbar");
  c.emplace_back("ref_base_x");
  c.emplace_back("ref_base_y");
  c.emplace_back("ref_base_z");
  c.emplace_back("ref_base_yaw");
  c.emplace_back("ref_base_pitch");
  c.emplace_back("ref_base_roll");

  for (const auto & name : running_cost_names_) {
    c.emplace_back("run_" + name);
  }
  for (const auto & name : terminal_cost_names_) {
    c.emplace_back("term_" + name);
  }
}

void CentroidalMpcDiagnostics::start()
{
  if (!settings_.enabled) {
    return;
  }
  const std::string prefix = expandPathPrefix(settings_.pathPrefix);
  state_path_ = prefix + "_state.csv";
  state_log_.open(state_path_, state_columns_, settings_.bufferRows);
  if (settings_.logCost) {
    cost_path_ = prefix + "_cost.csv";
    cost_log_.open(cost_path_, cost_columns_, settings_.bufferRows);
  }
  next_state_time_ = -1.0;
  next_cost_time_ = -1.0;
}

void CentroidalMpcDiagnostics::stop()
{
  state_log_.close();
  cost_log_.close();
}

ocs2::vector_t CentroidalMpcDiagnostics::normalizedMomentum(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & linear_velocity_world,
  const Eigen::Vector3d & angular_velocity_local,
  const ocs2::vector_t & joint_positions,
  const ocs2::vector_t & joint_velocities)
{
  const auto joint_dim = joint_positions.size();
  Eigen::Quaterniond quaternion = orientation;
  if (quaternion.norm() < 1e-12) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }
  // Raw ZYX here, not the controller's unwrapped yaw: momentum depends on the
  // rotation, not on which branch of the yaw the controller is tracking.
  const ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(quaternion);

  q_scratch_.head<3>() = position;
  q_scratch_.segment<3>(3) = euler_zyx;
  q_scratch_.tail(joint_dim) = joint_positions;

  v_scratch_.head<3>() = linear_velocity_world;
  v_scratch_.segment<3>(3) =
    ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(
    euler_zyx, angular_velocity_local);
  v_scratch_.tail(joint_dim) = joint_velocities;

  ocs2::updateCentroidalDynamics(pinocchio_, info_, q_scratch_);
  const auto & momentum_matrix = ocs2::getCentroidalMomentumMatrix(pinocchio_);
  return momentum_matrix * v_scratch_ / info_.robotMass;
}

void CentroidalMpcDiagnostics::recordState(
  const StateSample & sample,
  const state_estimation::FloatingBaseEstimate & estimate,
  const state_estimation::InekfFloatingBaseEstimator::Diagnostics & filter)
{
  if (!settings_.enabled || !state_log_.isOpen()) {
    return;
  }
  const double period = 1.0 / std::max(1.0, settings_.stateRate);
  if (next_state_time_ >= 0.0 && sample.time < next_state_time_) {
    return;
  }
  // Advance from the sample time rather than accumulating the period, so a
  // hiccup in the control loop does not permanently shift the sampling grid.
  next_state_time_ = sample.time + period;

  double * row = state_log_.beginRow();
  if (row == nullptr) {
    return;
  }
  std::size_t k = 0;

  put(row, k, sample.time);
  put(row, k, static_cast<double>(sample.phase));
  put(row, k, static_cast<double>(sample.mode));
  put(row, k, sample.contact_left);
  put(row, k, sample.contact_right);

  Eigen::Quaterniond gt_quaternion = sample.gt_orientation;
  if (gt_quaternion.norm() < 1e-12) {
    gt_quaternion = Eigen::Quaterniond::Identity();
  } else {
    gt_quaternion.normalize();
  }
  const ocs2::vector3_t gt_rpy = ocs2::humanoid::quaternionToEulerZYX(gt_quaternion);
  put(row, k, sample.gt_position);
  put(row, k, gt_rpy);
  put(row, k, sample.gt_linear_velocity_world);
  put(row, k, sample.gt_angular_velocity_local);

  Eigen::Quaterniond est_quaternion = estimate.orientation;
  if (est_quaternion.norm() < 1e-12) {
    est_quaternion = Eigen::Quaterniond::Identity();
  } else {
    est_quaternion.normalize();
  }
  const ocs2::vector3_t est_rpy = ocs2::humanoid::quaternionToEulerZYX(est_quaternion);
  put(row, k, estimate.position);
  put(row, k, est_rpy);
  put(row, k, estimate.linear_velocity_world);
  put(row, k, estimate.angular_velocity_local);

  put(row, k, Eigen::Vector3d(estimate.position - sample.gt_position));
  // Orientation error as the ZYX angles of the residual rotation, so it stays
  // meaningful across the +-pi wrap of either input.
  const Eigen::Quaterniond orientation_error = est_quaternion * gt_quaternion.conjugate();
  put(row, k, ocs2::vector3_t(ocs2::humanoid::quaternionToEulerZYX(orientation_error)));
  put(row, k, Eigen::Vector3d(estimate.linear_velocity_world - sample.gt_linear_velocity_world));
  put(row, k, Eigen::Vector3d(estimate.angular_velocity_local - sample.gt_angular_velocity_local));

  const bool momentum_available =
    sample.joint_valid && sample.gt_valid && estimate.valid &&
    sample.joint_positions.size() == q_scratch_.size() - 6 &&
    sample.joint_velocities.size() == v_scratch_.size() - 6;
  if (momentum_available) {
    momentum_estimate_ = normalizedMomentum(
      estimate.position, estimate.orientation, estimate.linear_velocity_world,
      estimate.angular_velocity_local, sample.joint_positions, sample.joint_velocities);
    momentum_ground_truth_ = normalizedMomentum(
      sample.gt_position, sample.gt_orientation, sample.gt_linear_velocity_world,
      sample.gt_angular_velocity_local, sample.joint_positions, sample.joint_velocities);
    put(row, k, momentum_estimate_);
    put(row, k, momentum_ground_truth_);
    put(row, k, ocs2::vector_t(momentum_estimate_ - momentum_ground_truth_));
  } else {
    k += 18;  // leave the three momentum blocks as NaN
  }

  put(row, k, filter.attitude_std);
  put(row, k, filter.velocity_std);
  put(row, k, filter.position_std);
  put(row, k, filter.gyroscope_bias_std);
  put(row, k, filter.accelerometer_bias_std);
  put(row, k, filter.gyroscope_bias);
  put(row, k, filter.accelerometer_bias);
  put(row, k, filter.estimating_bias);

  put(row, k, filter.correction_applied);
  put(row, k, static_cast<double>(filter.correction_dim));
  put(row, k, filter.nis);
  put(row, k, filter.nis_per_dof);
  put(row, k, static_cast<double>(filter.num_augmented_contacts));

  for (std::size_t i = 0; i < contact_frames_.size(); ++i) {
    const bool in_range = i < filter.contact_innovation.size();
    put(row, k, in_range ? filter.contact_innovation[i] : Eigen::Vector3d::Zero());
    put(row, k, in_range ? filter.contact_innovation_std[i] : Eigen::Vector3d::Zero());
    put(row, k, in_range ? filter.contact_landmark_position[i] : Eigen::Vector3d::Zero());
    put(row, k, in_range && filter.contact_corrected[i]);
    put(row, k, in_range && filter.contact_added[i]);
    put(row, k, in_range && filter.contact_removed[i]);
    put(row, k, in_range && filter.contact_in_stance[i]);
    put(row, k, in_range && filter.contact_landmark_valid[i]);
    put(row, k, in_range && i < filter.contact_ground_anchor.size() ?
      filter.contact_ground_anchor[i] : 0.0);
  }

  put(row, k, filter.inekf_height);
  put(row, k, filter.kinematic_height);
  put(row, k, filter.kinematic_height_valid);
  put(row, k, filter.reported_height);

  if (k != state_log_.width()) {
    schema_mismatch_ = true;
  }
  state_log_.commitRow();
}

void CentroidalMpcDiagnostics::recordCost(
  const ocs2::SystemObservation & observation, double advance_ms)
{
  if (!settings_.enabled || !settings_.logCost || !cost_log_.isOpen() || problem_ == nullptr) {
    return;
  }
  const double period = 1.0 / std::max(1.0, settings_.costRate);
  if (next_cost_time_ >= 0.0 && observation.time < next_cost_time_) {
    return;
  }
  next_cost_time_ = observation.time + period;

  const auto reference_manager = mpc_interface_->getReferenceManagerPtr();
  if (!reference_manager) {
    return;
  }
  const auto & targets = reference_manager->getTargetTrajectories();
  if (targets.empty()) {
    return;
  }

  const auto & t = observation.time;
  const auto & x = observation.state;
  const auto & u = observation.input;

  double running_total = 0.0;
  double terminal_total = 0.0;
  std::vector<double> running_values(running_cost_names_.size(), 0.0);
  std::vector<double> terminal_values(terminal_cost_names_.size(), 0.0);

  // Cost terms may throw (AD library reload, out-of-range reference lookup). A
  // diagnostic must never take the solver thread down with it.
  try {
    problem_->preComputationPtr->request(ocs2::Request::Cost, t, x, u);
    for (std::size_t i = 0; i < running_cost_names_.size(); ++i) {
      running_values[i] = problem_->costPtr->get<ocs2::StateInputCost>(running_cost_names_[i])
        .getValue(t, x, u, targets, *problem_->preComputationPtr);
      running_total += running_values[i];
    }
    problem_->preComputationPtr->requestFinal(ocs2::Request::Cost, t, x);
    for (std::size_t i = 0; i < terminal_cost_names_.size(); ++i) {
      terminal_values[i] = problem_->finalCostPtr->get<ocs2::StateCost>(terminal_cost_names_[i])
        .getValue(t, x, targets, *problem_->preComputationPtr);
      terminal_total += terminal_values[i];
    }
  } catch (const std::exception &) {
    return;
  }

  const ocs2::vector_t reference_state = targets.getDesiredState(t);

  double * row = cost_log_.beginRow();
  if (row == nullptr) {
    return;
  }
  std::size_t k = 0;
  put(row, k, t);
  put(row, k, static_cast<double>(observation.mode));
  put(row, k, advance_ms);
  put(row, k, running_total);
  put(row, k, terminal_total);

  // Centroidal state layout: [normalized momentum (6), base pose (6), joints].
  for (Eigen::Index i = 0; i < 12 && i < x.size(); ++i) {
    put(row, k, x[i]);
  }
  for (Eigen::Index i = x.size(); i < 12; ++i) {
    ++k;
  }
  for (Eigen::Index i = 0; i < 12 && i < reference_state.size(); ++i) {
    put(row, k, reference_state[i]);
  }
  for (Eigen::Index i = reference_state.size(); i < 12; ++i) {
    ++k;
  }

  for (const double value : running_values) {
    put(row, k, value);
  }
  for (const double value : terminal_values) {
    put(row, k, value);
  }

  if (k != cost_log_.width()) {
    schema_mismatch_ = true;
  }
  cost_log_.commitRow();
}

}  // namespace legged_robot_mpc_controller
