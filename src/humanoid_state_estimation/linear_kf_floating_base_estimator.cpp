#include "legged_robot_mpc_controller/humanoid_state_estimation/linear_kf_floating_base_estimator.hpp"

#include <algorithm>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
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

  if (settings_.lkf_imu_extrinsics) {
    if (!model_.existFrame(settings_.imu_frame)) {
      throw std::invalid_argument(
              "[LinearKfFloatingBaseEstimator] IMU frame '" + settings_.imu_frame +
              "' not found in estimator URDF");
    }
    // With the free flyer at the identity, a frame fixed to the base sits at its mounting.
    pinocchio::Data data(model_);
    pinocchio::framesForwardKinematics(model_, data, pinocchio::neutral(model_));
    const pinocchio::SE3 & mounting = data.oMf[model_.getFrameId(settings_.imu_frame)];
    base_to_imu_ = Eigen::Quaterniond(mounting.rotation()).normalized();
    imu_position_base_ = mounting.translation();
    imu_extrinsics_ = true;
  }

  if (settings_.lkf_contact_model == "rolling_wheel") {
    if (!(settings_.lkf_wheel_radius > 0.0) || settings_.lkf_wheel_axle_axis.norm() < 1e-9) {
      throw std::invalid_argument(
              "[LinearKfFloatingBaseEstimator] rolling_wheel needs a positive wheel radius and a nonzero axle axis");
    }
    rolling_wheels_ = true;
  } else if (settings_.lkf_contact_model != "point") {
    throw std::invalid_argument(
            "[LinearKfFloatingBaseEstimator] contact model must be 'point' or 'rolling_wheel', got '" +
            settings_.lkf_contact_model + "'");
  }

  dim_contacts_ = 3 * num_contacts_;
  if (rolling_wheels_) {
    // [base position, base velocity]; per wheel 3 velocity rows, then one height row per wheel.
    num_state_ = 6;
    num_observe_ = 4 * num_contacts_;
    x_hat_.setZero(num_state_);
    vs_.setZero(dim_contacts_);
    a_.setIdentity(num_state_, num_state_);
    b_.setZero(num_state_, 3);
    c_.setZero(num_observe_, num_state_);
    for (Eigen::Index i = 0; i < num_contacts_; ++i) {
      c_.block(3 * i, 3, 3, 3) = Eigen::Matrix3d::Identity();
      c_(dim_contacts_ + i, 2) = 1.0;
    }
    q_.setIdentity(num_state_, num_state_);
    p_ = 100.0 * q_;
    r_.setIdentity(num_observe_, num_observe_);
    return;
  }
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
  if (rolling_wheels_) {
    p_ = 100.0 * Eigen::MatrixXd::Identity(num_state_, num_state_);
    initialized_ = true;
    return;
  }

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

void LinearKfFloatingBaseEstimator::initializeFromStance(
  const Eigen::Quaterniond & imu_orientation_world,
  const std::vector<double> & joint_positions)
{
  // Stage 1 takes attitude straight from the IMU, so seed it the same way.
  const Eigen::Quaterniond base_orientation = imuToBaseOrientation(imu_orientation_world);
  if (rolling_wheels_) {
    // Standing on both tires: the height that puts their contact points on the ground plane.
    base_orientation_ = base_orientation;
    const std::vector<double> zero_velocities(joint_positions.size(), 0.0);
    baseRelativeKinematics(Eigen::Vector3d::Zero(), joint_positions, zero_velocities);
    double sum = 0.0;
    for (Eigen::Index i = 0; i < num_contacts_; ++i) {
      Eigen::Vector3d position, velocity;
      wheelContact(i, position, velocity);
      sum += settings_.height_ground_z - position.z();
    }
    initialize(
      Eigen::Vector3d(0.0, 0.0, sum / static_cast<double>(num_contacts_)), base_orientation, joint_positions);
    return;
  }
  // Seed once at height 0: the contacts' z in the state are then their heights
  // relative to the base, and the height that puts them on the ground plane is
  // ground - mean(relative). Re-seed at that height.
  initialize(Eigen::Vector3d::Zero(), base_orientation, joint_positions);
  double sum = 0.0;
  for (Eigen::Index i = 0; i < num_contacts_; ++i) {
    sum += settings_.height_ground_z - x_hat_(6 + 3 * i + 2);
  }
  const double height = num_contacts_ > 0 ? sum / static_cast<double>(num_contacts_) : 0.0;
  initialize(Eigen::Vector3d(0.0, 0.0, height), base_orientation, joint_positions);
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
  // The base's own rate, and the specific force at the base origin (the IMU's lever arm
  // costs w x (w x r); the angular-acceleration term is left out, the velocity rows absorb it).
  Eigen::Vector3d gyro = imu_angular_velocity_body;
  Eigen::Vector3d accel = imu_linear_acceleration_body;
  if (imu_extrinsics_) {
    gyro = base_to_imu_ * imu_angular_velocity_body;
    accel = base_to_imu_ * imu_linear_acceleration_body - gyro.cross(gyro.cross(imu_position_base_));
  }
  if (rolling_wheels_) {
    baseRelativeKinematics(gyro, joint_positions, joint_velocities);
    return updateRollingWheels(gyro, accel, point_contact_flags);
  }

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
  v_pin.segment<3>(3) = gyro;
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
  const Eigen::Vector3d accel_world = R * accel + gravity;

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
  estimate.angular_velocity_local = gyro;
  estimate.valid = true;
  return estimate;
}

Eigen::Quaterniond LinearKfFloatingBaseEstimator::imuToBaseOrientation(
  const Eigen::Quaterniond & imu_orientation) const
{
  // R_world_base = R_world_imu * R_base_imu^T
  return imu_extrinsics_ ? (imu_orientation.normalized() * base_to_imu_.conjugate()).normalized()
                         : imu_orientation.normalized();
}

void LinearKfFloatingBaseEstimator::baseRelativeKinematics(
  const Eigen::Vector3d & base_angular_velocity,
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_velocities)
{
  // As in the point model: the measurement is relative to the base, so the estimated base
  // position and velocity must not enter it.
  Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model_.nq);
  q_pin(3) = base_orientation_.x();
  q_pin(4) = base_orientation_.y();
  q_pin(5) = base_orientation_.z();
  q_pin(6) = base_orientation_.w();
  Eigen::VectorXd qj = Eigen::VectorXd::Zero(num_estimator_joints_);
  remapToEstimatorOrder(joint_positions, qj);
  q_pin.tail(num_estimator_joints_) = qj;

  Eigen::VectorXd v_pin = Eigen::VectorXd::Zero(model_.nv);
  v_pin.segment<3>(3) = base_angular_velocity;
  Eigen::VectorXd vj = Eigen::VectorXd::Zero(num_estimator_joints_);
  remapToEstimatorOrder(joint_velocities, vj);
  v_pin.tail(num_estimator_joints_) = vj;

  pinocchio::forwardKinematics(model_, *data_, q_pin, v_pin);
  pinocchio::updateFramePlacements(model_, *data_);
}

void LinearKfFloatingBaseEstimator::wheelContact(
  Eigen::Index i, Eigen::Vector3d & position, Eigen::Vector3d & material_velocity) const
{
  const auto frame_id = contact_frame_ids_[static_cast<size_t>(i)];
  const pinocchio::SE3 & placement = data_->oMf[frame_id];
  const pinocchio::Motion velocity =
    pinocchio::getFrameVelocity(model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
  const Eigen::Vector3d axle = (placement.rotation() * settings_.lkf_wheel_axle_axis).normalized();
  // Lowest point of the tire circle: straight down without its component along the axle.
  Eigen::Vector3d down = -Eigen::Vector3d::UnitZ() + axle.z() * axle;
  const double norm = down.norm();
  down = norm > 1e-6 ? Eigen::Vector3d(down / norm) : Eigen::Vector3d(-Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d to_contact =
    placement.rotation() * settings_.lkf_wheel_center_offset + settings_.lkf_wheel_radius * down;
  position = placement.translation() + to_contact;
  material_velocity = velocity.linear() + velocity.angular().cross(to_contact);
}

FloatingBaseEstimate LinearKfFloatingBaseEstimator::updateRollingWheels(
  const Eigen::Vector3d & base_angular_velocity,
  const Eigen::Vector3d & base_linear_acceleration,
  const std::vector<bool> & wheel_contact_flags)
{
  const double dt = settings_.sampling_time;
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  a_.setIdentity(num_state_, num_state_);
  a_.block(0, 3, 3, 3) = dt * I3;
  b_.setZero(num_state_, 3);
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * I3;
  b_.block(3, 0, 3, 3) = dt * I3;

  // Same noise scaling as the point model, so the shared parameters mean the same thing.
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(num_state_, num_state_);
  q.block(0, 0, 3, 3) = (dt / 20.0) * settings_.lkf_imu_process_noise_position * I3;
  q.block(3, 3, 3, 3) = (dt * 9.81 / 20.0) * settings_.lkf_imu_process_noise_velocity * I3;

  Eigen::MatrixXd r = Eigen::MatrixXd::Identity(num_observe_, num_observe_);
  Eigen::VectorXd y(num_observe_);
  int stance = 0;
  double height_sum = 0.0;
  for (Eigen::Index i = 0; i < num_contacts_; ++i) {
    Eigen::Vector3d position, material_velocity;
    wheelContact(i, position, material_velocity);
    y.segment<3>(3 * i) = -material_velocity;
    y(dim_contacts_ + i) = settings_.height_ground_z - position.z();
    // An airborne wheel spins freely and its tread is not at rest: inflate rather than drop its
    // rows, which keeps the dimensions fixed across contact changes.
    const bool in_contact = wheel_contact_flags[static_cast<size_t>(i)];
    const double scale = in_contact ? 1.0 : settings_.lkf_swing_noise_multiplier;
    r.block(3 * i, 3 * i, 3, 3) *= settings_.lkf_foot_sensor_noise_velocity * scale;
    r(dim_contacts_ + i, dim_contacts_ + i) *= settings_.lkf_foot_height_sensor_noise * scale;
    if (in_contact) {
      ++stance;
      height_sum += y(dim_contacts_ + i);
    }
  }

  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d accel_world = base_orientation_ * base_linear_acceleration + gravity;
  x_hat_ = a_ * x_hat_ + b_ * accel_world;
  const Eigen::MatrixXd p_minus = a_ * p_ * a_.transpose() + q;

  const Eigen::VectorXd innovation = y - c_ * x_hat_;
  const Eigen::MatrixXd s = c_ * p_minus * c_.transpose() + r;
  const auto s_lu = s.lu();
  const Eigen::MatrixXd gain = p_minus * c_.transpose() * s_lu.inverse();
  x_hat_ += gain * innovation;
  p_ = (Eigen::MatrixXd::Identity(num_state_, num_state_) - gain * c_) * p_minus;
  p_ = 0.5 * (p_ + p_.transpose().eval());

  diagnostics_.valid = true;
  diagnostics_.correction_applied = stance > 0;
  diagnostics_.correction_dim = static_cast<int>(num_observe_);
  diagnostics_.nis = innovation.dot(s_lu.solve(innovation));
  diagnostics_.nis_per_dof = diagnostics_.nis / static_cast<double>(num_observe_);
  diagnostics_.position_std = p_.diagonal().segment<3>(0).cwiseMax(0.0).cwiseSqrt();
  diagnostics_.velocity_std = p_.diagonal().segment<3>(3).cwiseMax(0.0).cwiseSqrt();
  diagnostics_.kinematic_height_valid = stance > 0;
  diagnostics_.kinematic_height = stance > 0 ? height_sum / stance : 0.0;
  diagnostics_.reported_height = x_hat_(2);
  diagnostics_.foot_stance_left = num_contacts_ > 0 && wheel_contact_flags[0] ? 1.0 : 0.0;
  diagnostics_.foot_stance_right = num_contacts_ > 1 && wheel_contact_flags[1] ? 1.0 : 0.0;

  FloatingBaseEstimate estimate;
  estimate.position = x_hat_.segment<3>(0);
  estimate.orientation = base_orientation_;
  estimate.linear_velocity_world = x_hat_.segment<3>(3);
  estimate.angular_velocity_local = base_angular_velocity;
  estimate.valid = true;
  return estimate;
}

}  // namespace legged_robot_mpc_controller::state_estimation
