#include "legged_robot_mpc_controller/humanoid_state_estimation/linear_kf_floating_base_estimator.hpp"

#include <algorithm>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace legged_robot_mpc_controller::state_estimation
{

int EstimatorSettings::numFeet() const
{
  if (num_feet > 0) {
    return num_feet;
  }
  // Derive from the contact-point -> foot map. On the G1 this is [0,0,1,1],
  // i.e. 4 contact points belonging to 2 feet.
  int64_t highest = -1;
  for (const int64_t index : contact_foot_indices) {
    highest = std::max(highest, index);
  }
  return highest >= 0 ? static_cast<int>(highest + 1) : 0;
}

LinearKfFloatingBaseEstimator::LinearKfFloatingBaseEstimator(const EstimatorSettings & settings)
: settings_(settings)
{
  pinocchio::urdf::buildModel(settings_.urdf_path, pinocchio::JointModelFreeFlyer(), model_);
  data_ = std::make_unique<pinocchio::Data>(model_);
  num_estimator_joints_ = static_cast<int>(model_.nq) - 7;
  if (num_estimator_joints_ < 0) {
    throw std::runtime_error("[LinearKfFloatingBaseEstimator] URDF has no actuated joints");
  }

  controller_to_estimator_.assign(settings_.controller_joint_names.size(), -1);
  for (size_t i = 0; i < settings_.controller_joint_names.size(); ++i) {
    const std::string & name = settings_.controller_joint_names[i];
    if (!model_.existJointName(name)) {
      throw std::invalid_argument(
              "[LinearKfFloatingBaseEstimator] controller joint '" + name +
              "' not found in estimator URDF");
    }
    controller_to_estimator_[i] = static_cast<int>(model_.idx_qs[model_.getJointId(name)]) - 7;
  }

  contact_frame_ids_.reserve(settings_.contact_frames.size());
  for (const std::string & frame : settings_.contact_frames) {
    if (!model_.existFrame(frame)) {
      throw std::invalid_argument(
              "[LinearKfFloatingBaseEstimator] contact frame '" + frame +
              "' not found in estimator URDF");
    }
    contact_frame_ids_.push_back(model_.getFrameId(frame));
  }

  // N is the number of CONTACT POINTS, not feet: the G1 has 2 feet but 4 points.
  num_contacts_ = static_cast<Eigen::Index>(contact_frame_ids_.size());
  if (num_contacts_ == 0) {
    throw std::invalid_argument("[LinearKfFloatingBaseEstimator] no contact frames configured");
  }
  dim_contacts_ = 3 * num_contacts_;
  num_state_ = 6 + dim_contacts_;
  num_observe_ = 2 * dim_contacts_ + num_contacts_;

  x_hat_.setZero(num_state_);
  ps_.setZero(dim_contacts_);
  vs_.setZero(dim_contacts_);
  feet_heights_.setZero(num_contacts_);

  a_.setIdentity(num_state_, num_state_);
  b_.setZero(num_state_, 3);

  // Measurement matrix. Per contact point i:
  //   rows [3i, 3i+3)                       -> base position, minus that point's position
  //   rows [3(N+i), 3(N+i)+3)               -> base velocity
  //   row  2*dimContacts + i                -> that point's world z
  Eigen::MatrixXd c1(3, 6);
  Eigen::MatrixXd c2(3, 6);
  c1 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
  c2 << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();
  c_.setZero(num_observe_, num_state_);
  for (Eigen::Index i = 0; i < num_contacts_; ++i) {
    c_.block(3 * i, 0, 3, 6) = c1;
    c_.block(3 * (num_contacts_ + i), 0, 3, 6) = c2;
    c_(2 * dim_contacts_ + i, 6 + 3 * i + 2) = 1.0;
  }
  c_.block(0, 6, dim_contacts_, dim_contacts_) =
    -Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);

  q_.setIdentity(num_state_, num_state_);
  // Large initial covariance: the seed pose is a guess and the filter should not
  // defend it against the first kinematic correction.
  p_ = 100.0 * q_;
  r_.setIdentity(num_observe_, num_observe_);
}

std::vector<bool> LinearKfFloatingBaseEstimator::expandContactFlags(
  const std::vector<bool> & foot_flags) const
{
  // The gait schedule produces one flag per FOOT; the filter needs one per
  // CONTACT POINT. contact_foot_indices ([0,0,1,1] on the G1) is that mapping.
  std::vector<bool> point_flags(static_cast<size_t>(num_contacts_), true);
  for (size_t i = 0; i < point_flags.size(); ++i) {
    int64_t foot = 0;
    if (i < settings_.contact_foot_indices.size()) {
      foot = settings_.contact_foot_indices[i];
    }
    point_flags[i] = (foot >= 0 && foot < static_cast<int64_t>(foot_flags.size()))
      ? foot_flags[static_cast<size_t>(foot)]
      : true;
  }
  return point_flags;
}

void LinearKfFloatingBaseEstimator::remapToEstimatorOrder(
  const std::vector<double> & controller_values, Eigen::VectorXd & estimator_vector) const
{
  estimator_vector.setZero();
  for (size_t i = 0; i < controller_to_estimator_.size() && i < controller_values.size(); ++i) {
    const int idx = controller_to_estimator_[i];
    if (idx >= 0 && idx < estimator_vector.size()) {
      estimator_vector(idx) = controller_values[i];
    }
  }
}

void LinearKfFloatingBaseEstimator::initialize(
  const Eigen::Vector3d & base_position,
  const Eigen::Quaterniond & base_orientation,
  const std::vector<double> & joint_positions)
{
  base_orientation_ = base_orientation.normalized();

  x_hat_.setZero(num_state_);
  x_hat_.segment<3>(0) = base_position;

  // Seed each contact point from forward kinematics at the seeded base pose, so
  // the first correction sees a consistent state rather than a 1 m residual.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
  q.segment<3>(0) = base_position;
  q(3) = base_orientation_.x();
  q(4) = base_orientation_.y();
  q(5) = base_orientation_.z();
  q(6) = base_orientation_.w();
  Eigen::VectorXd qj = Eigen::VectorXd::Zero(num_estimator_joints_);
  remapToEstimatorOrder(joint_positions, qj);
  q.tail(num_estimator_joints_) = qj;

  pinocchio::forwardKinematics(model_, *data_, q);
  pinocchio::updateFramePlacements(model_, *data_);
  for (Eigen::Index i = 0; i < num_contacts_; ++i) {
    x_hat_.segment<3>(6 + 3 * i) =
      data_->oMf[contact_frame_ids_[static_cast<size_t>(i)]].translation();
  }

  p_ = 100.0 * Eigen::MatrixXd::Identity(num_state_, num_state_);
  feet_heights_.setZero(num_contacts_);
  initialized_ = true;
}

FloatingBaseEstimate LinearKfFloatingBaseEstimator::update(
  const Eigen::Vector3d & imu_angular_velocity_body,
  const Eigen::Vector3d & imu_linear_acceleration_body,
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_velocities,
  const std::vector<double> & /*joint_torques*/)
{
  // No contact information supplied: treat every point as planted, which is the
  // stance assumption. The scheduled-contact overload below is the normal path.
  const std::vector<bool> all_in_contact(static_cast<size_t>(num_contacts_), true);
  return updateImpl(
    imu_angular_velocity_body, imu_linear_acceleration_body, joint_positions, joint_velocities,
    all_in_contact);
}

FloatingBaseEstimate LinearKfFloatingBaseEstimator::update(
  const Eigen::Vector3d & imu_angular_velocity_body,
  const Eigen::Vector3d & imu_linear_acceleration_body,
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_velocities,
  const std::vector<double> & /*joint_torques*/,
  const std::vector<bool> & foot_contacts)
{
  return updateImpl(
    imu_angular_velocity_body, imu_linear_acceleration_body, joint_positions, joint_velocities,
    expandContactFlags(foot_contacts));
}

FloatingBaseEstimate LinearKfFloatingBaseEstimator::updateImpl(
  const Eigen::Vector3d & imu_angular_velocity_body,
  const Eigen::Vector3d & imu_linear_acceleration_body,
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_velocities,
  const std::vector<bool> & point_contact_flags)
{
  const double dt = settings_.sampling_time;
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d R = base_orientation_.toRotationMatrix();

  // ---- process model: constant acceleration on the base, static contacts ----
  a_.block(0, 3, 3, 3) = dt * I3;
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * I3;
  b_.block(3, 0, 3, 3) = dt * I3;

  // Nominal process noise shape, scaled per block below. The 1/20 factors and
  // the 9.81 on the velocity block are Cheetah 3's, kept so the tuning numbers
  // from that work and from legged_control transfer directly.
  q_.setIdentity(num_state_, num_state_);
  q_.block(0, 0, 3, 3) = (dt / 20.0) * I3;
  q_.block(3, 3, 3, 3) = (dt * 9.81 / 20.0) * I3;
  q_.block(6, 6, dim_contacts_, dim_contacts_) =
    dt * Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);

  Eigen::MatrixXd q = Eigen::MatrixXd::Identity(num_state_, num_state_);
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * settings_.lkf_imu_process_noise_position;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * settings_.lkf_imu_process_noise_velocity;
  q.block(6, 6, dim_contacts_, dim_contacts_) =
    q_.block(6, 6, dim_contacts_, dim_contacts_) * settings_.lkf_foot_process_noise_position;

  Eigen::MatrixXd r = Eigen::MatrixXd::Identity(num_observe_, num_observe_);
  r.block(0, 0, dim_contacts_, dim_contacts_) *= settings_.lkf_foot_sensor_noise_position;
  r.block(dim_contacts_, dim_contacts_, dim_contacts_, dim_contacts_) *=
    settings_.lkf_foot_sensor_noise_velocity;
  r.block(2 * dim_contacts_, 2 * dim_contacts_, num_contacts_, num_contacts_) *=
    settings_.lkf_foot_height_sensor_noise;

  // ---- forward kinematics of the contact points, in the BASE frame ----
  //
  // Position is left at the origin and linear velocity at zero on purpose: the
  // measurement is the foot RELATIVE to the base, so only orientation, joint
  // positions and joint velocities may enter. Feeding the estimated base pose in
  // here would make the measurement depend on the state it is meant to correct.
  Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model_.nq);
  q_pin(3) = base_orientation_.x();
  q_pin(4) = base_orientation_.y();
  q_pin(5) = base_orientation_.z();
  q_pin(6) = base_orientation_.w();
  Eigen::VectorXd qj = Eigen::VectorXd::Zero(num_estimator_joints_);
  remapToEstimatorOrder(joint_positions, qj);
  q_pin.tail(num_estimator_joints_) = qj;

  Eigen::VectorXd v_pin = Eigen::VectorXd::Zero(model_.nv);
  v_pin.segment<3>(3) = imu_angular_velocity_body;
  Eigen::VectorXd vj = Eigen::VectorXd::Zero(num_estimator_joints_);
  remapToEstimatorOrder(joint_velocities, vj);
  v_pin.tail(num_estimator_joints_) = vj;

  pinocchio::forwardKinematics(model_, *data_, q_pin, v_pin);
  pinocchio::updateFramePlacements(model_, *data_);

  const double swing = settings_.lkf_swing_noise_multiplier;
  for (Eigen::Index i = 0; i < num_contacts_; ++i) {
    const auto frame_id = contact_frame_ids_[static_cast<size_t>(i)];
    const Eigen::Vector3d foot_position = data_->oMf[frame_id].translation();
    const Eigen::Vector3d foot_velocity =
      pinocchio::getFrameVelocity(model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();

    // Base position measured as (foot world position) - (foot relative position).
    ps_.segment<3>(3 * i) = -foot_position;
    ps_(3 * i + 2) += settings_.lkf_foot_radius;
    vs_.segment<3>(3 * i) = -foot_velocity;

    // A swing foot is neither static nor a valid measurement, so inflate both its
    // process and measurement noise instead of removing it from the problem -
    // that keeps the matrix dimensions fixed across contact transitions.
    const bool in_contact = point_contact_flags[static_cast<size_t>(i)];
    const double scale = in_contact ? 1.0 : swing;
    const Eigen::Index qi = 6 + 3 * i;
    q.block(qi, qi, 3, 3) *= scale;
    r.block(3 * i, 3 * i, 3, 3) *= scale;
    r.block(dim_contacts_ + 3 * i, dim_contacts_ + 3 * i, 3, 3) *= scale;
    r(2 * dim_contacts_ + i, 2 * dim_contacts_ + i) *= scale;
  }

  // ---- predict ----
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d accel_world = R * imu_linear_acceleration_body + gravity;

  x_hat_ = a_ * x_hat_ + b_ * accel_world;
  const Eigen::MatrixXd p_minus = a_ * p_ * a_.transpose() + q;

  // ---- correct ----
  Eigen::VectorXd y(num_observe_);
  y << ps_, vs_, feet_heights_;
  const Eigen::VectorXd innovation = y - c_ * x_hat_;
  const Eigen::MatrixXd s = c_ * p_minus * c_.transpose() + r;

  const Eigen::MatrixXd ct = c_.transpose();
  x_hat_ += p_minus * ct * s.lu().solve(innovation);
  p_ = (Eigen::MatrixXd::Identity(num_state_, num_state_) - p_minus * ct * s.lu().solve(c_)) *
    p_minus;
  p_ = 0.5 * (p_ + p_.transpose().eval());   // keep it symmetric against round-off

  // ---- diagnostics, in the same shape the InEKF reports ----
  diagnostics_.correction_applied = true;
  diagnostics_.nis = innovation.dot(s.lu().solve(innovation));
  diagnostics_.correction_dim = static_cast<int>(num_observe_);
  diagnostics_.nis_per_dof =
    num_observe_ > 0 ? diagnostics_.nis / static_cast<double>(num_observe_) : 0.0;

  FloatingBaseEstimate estimate;
  estimate.position = x_hat_.segment<3>(0);
  estimate.orientation = base_orientation_;
  estimate.linear_velocity_world = x_hat_.segment<3>(3);
  estimate.angular_velocity_local = imu_angular_velocity_body;
  estimate.valid = true;
  return estimate;
}

}  // namespace legged_robot_mpc_controller::state_estimation
