#include "legged_robot_mpc_controller/humanoid_wb_mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <Eigen/Geometry>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pluginlib/class_list_macros.hpp>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>

#include "legged_robot_mpc_controller/config/wb_mpc_config_builder.hpp"

namespace legged_robot_mpc_controller
{

namespace
{

ocs2::vector_t make_vector(
  const std::vector<double>& values,
  const Eigen::Index expected_size,
  const double default_value,
  const std::string& name)
{
  if (values.empty()) {
    return ocs2::vector_t::Constant(expected_size, default_value);
  }
  if (static_cast<Eigen::Index>(values.size()) != expected_size) {
    throw std::invalid_argument(
      name + " size (" + std::to_string(values.size()) + ") must be empty or equal to " +
      std::to_string(expected_size));
  }
  return Eigen::Map<const ocs2::vector_t>(values.data(), expected_size);
}

std::optional<size_t> interface_offset(
  const std::vector<std::string>& interface_names,
  const std::string& interface_name)
{
  const auto it = std::find(interface_names.begin(), interface_names.end(), interface_name);
  if (it == interface_names.end()) {
    return std::nullopt;
  }
  return static_cast<size_t>(std::distance(interface_names.begin(), it));
}

}  // namespace

HumanoidWbMpcController::~HumanoidWbMpcController()
{
  stop_solver_thread();
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    parameters_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] init failed: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "[HumanoidWbMpcController] init success");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_configure(
  const rclcpp_lifecycle::State&)
{
  if (param_listener_->is_old(parameters_)) {
    parameters_ = param_listener_->get_params();
  }

  if (parameters_.paths.urdfFile.empty() || parameters_.paths.libFolder.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] paths.urdfFile or paths.libFolder is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController] CppAD library folder: %s | recompileLibraries=%s",
    parameters_.paths.libFolder.c_str(),
    parameters_.ocs2.model.recompileLibrariesCppAd ? "true" : "false");

  // Build the whole-body MPC problem from ROS 2 parameters. The first run generates and
  // compiles the CppAD model libraries into paths.libFolder, which can take several minutes;
  // subsequent runs load the cached libraries.
  try {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] Constructing WBMpcInterface (CppAD codegen folder: %s, "
      "recompile: %s). First-time codegen can take several minutes...",
      parameters_.paths.libFolder.c_str(),
      parameters_.ocs2.model.recompileLibrariesCppAd ? "true" : "false");
    mpc_interface_ = std::make_unique<ocs2::humanoid::WBMpcInterface>(buildWbMpcConfig(parameters_));
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] WBMpcInterface ready (state dim %zu, input dim %zu)",
      static_cast<size_t>(mpc_interface_->getMpcRobotModel().getStateDim()),
      static_cast<size_t>(mpc_interface_->getMpcRobotModel().getInputDim()));
    control_model_ =
      std::make_unique<ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>>(
      mpc_interface_->modelSettings());
    const auto joint_dim = static_cast<Eigen::Index>(control_model_->getJointDim());
    mpc_joint_kp_ = make_vector(parameters_.control.mpcJointKp, joint_dim, 1200.0, "control.mpcJointKp");
    mpc_joint_kd_ = make_vector(parameters_.control.mpcJointKd, joint_dim, 10.0, "control.mpcJointKd");
    torque_limit_ = make_vector(parameters_.control.torqueLimit, joint_dim, 0.0, "control.torqueLimit");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "[HumanoidWbMpcController] Failed to build WBMpcInterface: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (parameters_.floatingBase.source == "mujoco_ground_truth_odom") {
    odometry_subscription_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      parameters_.floatingBase.odometryTopic,
      rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odometry_callback(msg); });
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController] configured whole-body MPC controller | joints=%zu solver=%s "
    "floatingBase.source=%s stateInterfaceName=%s odom=%s baseFrame=%s",
    parameters_.robot.jointNames.size(),
    parameters_.ocs2.mpc.solverType.c_str(),
    parameters_.floatingBase.source.c_str(),
    parameters_.floatingBase.stateInterfaceName.c_str(),
    parameters_.floatingBase.odometryTopic.c_str(),
    parameters_.floatingBase.baseFrame.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_activate(
  const rclcpp_lifecycle::State&)
{
  if (!mpc_interface_ || !control_model_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] activation requested before MPC interface was configured.");
    return controller_interface::CallbackReturn::ERROR;
  }

  initial_observation_state_ = mpc_interface_->getInitialState();
  const auto initial_observation = build_observation(get_node()->now());
  write_torque_command(compute_weight_compensating_torque(initial_observation));
  try {
    start_solver_thread(initial_observation);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] failed to start MPC solver: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !mrt_interface_->initialPolicyReceived()) {
    if ((std::chrono::steady_clock::now() - wait_start) > std::chrono::seconds(20)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[HumanoidWbMpcController] timed out waiting for the initial MPC policy.");
      stop_solver_thread();
      return controller_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController] activated with initial MPC policy | solver=%s, joints=%zu",
    parameters_.ocs2.mpc.solverType.c_str(),
    parameters_.robot.jointNames.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HumanoidWbMpcController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  stop_solver_thread();
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HumanoidWbMpcController::command_interface_configuration()
  const
{
  return make_joint_interface_configuration({parameters_.robot.commandInterface});
}

controller_interface::InterfaceConfiguration HumanoidWbMpcController::state_interface_configuration()
  const
{
  auto config = make_joint_interface_configuration(parameters_.robot.stateInterfaces);

  if (parameters_.floatingBase.source == "state_interfaces") {
    if (config.type == controller_interface::interface_configuration_type::NONE) {
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    }
    const auto floating_base_interfaces = floating_base_state_interface_names();
    config.names.insert(
      config.names.end(),
      floating_base_interfaces.begin(),
      floating_base_interfaces.end());
  }

  return config;
}

controller_interface::return_type HumanoidWbMpcController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type HumanoidWbMpcController::update_and_write_commands(
  const rclcpp::Time& time,
  const rclcpp::Duration&)
{
  if (!mpc_interface_ || !control_model_) {
    return controller_interface::return_type::ERROR;
  }

  const auto observation = build_observation(time);
  vector_t torque;
  if (mrt_interface_ && mrt_interface_->initialPolicyReceived()) {
    torque = compute_mpc_torque_command(observation);
  } else {
    torque = compute_weight_compensating_torque(observation);
  }
  write_torque_command(torque);
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
HumanoidWbMpcController::on_export_reference_interfaces()
{
  // controller_manager rejects chainable controllers with zero reference interfaces,
  // so export a single dummy one (same pattern as dynamics_mpc_controller).
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.emplace_back(
    std::string(get_node()->get_name()),
    std::string("dummy_humanoid_wb_mpc/") + hardware_interface::HW_IF_EFFORT,
    reference_interfaces_.data());
  return reference_interfaces;
}

controller_interface::InterfaceConfiguration
HumanoidWbMpcController::make_joint_interface_configuration(
  const std::vector<std::string>& interface_names) const
{
  controller_interface::InterfaceConfiguration config;
  if (parameters_.robot.jointNames.empty() || interface_names.empty()) {
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : parameters_.robot.jointNames) {
    for (const auto& interface_name : interface_names) {
      if (!interface_name.empty()) {
        config.names.emplace_back(joint_name + "/" + interface_name);
      }
    }
  }
  return config;
}

std::vector<std::string> HumanoidWbMpcController::floating_base_state_interface_names() const
{
  const std::string& name = parameters_.floatingBase.stateInterfaceName;
  return {
    name + "/position.x",
    name + "/position.y",
    name + "/position.z",
    name + "/orientation.w",
    name + "/orientation.x",
    name + "/orientation.y",
    name + "/orientation.z",
    name + "/linear_velocity.x",
    name + "/linear_velocity.y",
    name + "/linear_velocity.z",
    name + "/angular_velocity.x",
    name + "/angular_velocity.y",
    name + "/angular_velocity.z",
  };
}

std::optional<double> HumanoidWbMpcController::get_state_interface_value(
  const std::string& prefix_name,
  const std::string& interface_name) const
{
  const auto state_handle = std::find_if(
    state_interfaces_.begin(),
    state_interfaces_.end(),
    [&prefix_name, &interface_name](const auto& interface) {
      return interface.get_prefix_name() == prefix_name &&
             interface.get_interface_name() == interface_name;
    });
  if (state_handle == state_interfaces_.end()) {
    return std::nullopt;
  }
  return state_handle->get_value();
}

bool HumanoidWbMpcController::read_joint_state(vector_t& q, vector_t& v)
{
  const size_t n = parameters_.robot.jointNames.size();
  const auto position_offset = interface_offset(
    parameters_.robot.stateInterfaces,
    hardware_interface::HW_IF_POSITION);
  const auto velocity_offset = interface_offset(
    parameters_.robot.stateInterfaces,
    hardware_interface::HW_IF_VELOCITY);
  if (!position_offset || !velocity_offset) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidWbMpcController] stateInterfaces must include position and velocity.");
    return false;
  }

  const size_t stride = parameters_.robot.stateInterfaces.size();
  if (state_interfaces_.size() < n * stride) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidWbMpcController] expected at least %zu state interfaces, got %zu.",
      n * stride,
      state_interfaces_.size());
    return false;
  }

  q.resize(static_cast<Eigen::Index>(n));
  v.resize(static_cast<Eigen::Index>(n));
  for (size_t i = 0; i < n; ++i) {
    q[static_cast<Eigen::Index>(i)] =
      state_interfaces_[i * stride + *position_offset].get_value();
    v[static_cast<Eigen::Index>(i)] =
      state_interfaces_[i * stride + *velocity_offset].get_value();
  }
  return true;
}

bool HumanoidWbMpcController::read_floating_base_state(ocs2::SystemObservation& observation)
{
  const std::string& name = parameters_.floatingBase.stateInterfaceName;
  const auto px = get_state_interface_value(name, "position.x");
  const auto py = get_state_interface_value(name, "position.y");
  const auto pz = get_state_interface_value(name, "position.z");
  const auto qw = get_state_interface_value(name, "orientation.w");
  const auto qx = get_state_interface_value(name, "orientation.x");
  const auto qy = get_state_interface_value(name, "orientation.y");
  const auto qz = get_state_interface_value(name, "orientation.z");
  const auto lvx = get_state_interface_value(name, "linear_velocity.x");
  const auto lvy = get_state_interface_value(name, "linear_velocity.y");
  const auto lvz = get_state_interface_value(name, "linear_velocity.z");
  const auto avx = get_state_interface_value(name, "angular_velocity.x");
  const auto avy = get_state_interface_value(name, "angular_velocity.y");
  const auto avz = get_state_interface_value(name, "angular_velocity.z");

  if (!px || !py || !pz || !qw || !qx || !qy || !qz || !lvx || !lvy || !lvz || !avx || !avy || !avz) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "[HumanoidWbMpcController] missing floating-base state interfaces for '%s'.",
      name.c_str());
    return false;
  }

  auto& model = *control_model_;
  model.setBasePosition(
    observation.state,
    (ocs2::vector3_t() << *px, *py, *pz).finished());

  Eigen::Quaterniond q(*qw, *qx, *qy, *qz);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  const ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(q);
  model.setBaseOrientationEulerZYX(observation.state, euler_zyx);

  const ocs2::vector3_t linear_velocity_local(*lvx, *lvy, *lvz);
  const ocs2::vector3_t angular_velocity_local(*avx, *avy, *avz);
  model.setBaseLinearVelocity(observation.state, q * linear_velocity_local);
  model.setBaseOrientationEulerZYXDerivatives(
    observation.state,
    ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(
      euler_zyx,
      angular_velocity_local));
  return true;
}

ocs2::SystemObservation HumanoidWbMpcController::build_observation(const rclcpp::Time& time)
{
  const auto& model = *control_model_;
  ocs2::SystemObservation observation;
  observation.time = time.seconds();
  observation.state = initial_observation_state_.size() == static_cast<Eigen::Index>(model.getStateDim()) ?
    initial_observation_state_ :
    mpc_interface_->getInitialState();
  observation.input = vector_t::Zero(static_cast<Eigen::Index>(model.getInputDim()));
  observation.mode = ocs2::humanoid::STANCE;

  nav_msgs::msg::Odometry odometry;
  bool has_odometry = false;
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    if (latest_odometry_) {
      odometry = *latest_odometry_;
      has_odometry = true;
    }
  }

  if (parameters_.floatingBase.source == "state_interfaces") {
    read_floating_base_state(observation);
  } else if (has_odometry) {
    model.setBasePosition(
      observation.state,
      (ocs2::vector3_t() <<
        odometry.pose.pose.position.x,
        odometry.pose.pose.position.y,
        odometry.pose.pose.position.z).finished());

    Eigen::Quaterniond q(
      odometry.pose.pose.orientation.w,
      odometry.pose.pose.orientation.x,
      odometry.pose.pose.orientation.y,
      odometry.pose.pose.orientation.z);
    q.normalize();

    const ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(q);
    model.setBaseOrientationEulerZYX(observation.state, euler_zyx);

    const ocs2::vector3_t linear_velocity_local(
      odometry.twist.twist.linear.x,
      odometry.twist.twist.linear.y,
      odometry.twist.twist.linear.z);
    const ocs2::vector3_t angular_velocity_local(
      odometry.twist.twist.angular.x,
      odometry.twist.twist.angular.y,
      odometry.twist.twist.angular.z);

    model.setBaseLinearVelocity(observation.state, q * linear_velocity_local);
    model.setBaseOrientationEulerZYXDerivatives(
      observation.state,
      ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(
        euler_zyx,
        angular_velocity_local));
  }

  vector_t q_joint;
  vector_t v_joint;
  if (read_joint_state(q_joint, v_joint)) {
    model.setJointAngles(observation.state, q_joint);
    model.setJointVelocities(observation.state, observation.input, v_joint);
  }

  return observation;
}

ocs2::TargetTrajectories HumanoidWbMpcController::current_observation_to_reset_trajectory(
  const ocs2::SystemObservation& observation)
{
  const auto& model = *control_model_;
  vector_t target_state = observation.state;
  target_state.tail(static_cast<Eigen::Index>(model.getGenCoordinatesDim())).setZero();
  if (target_state.size() >= 6) {
    target_state.segment<2>(4).setZero();
  }
  return ocs2::TargetTrajectories(
    {observation.time},
    {target_state},
    {vector_t::Zero(observation.input.size())});
}

void HumanoidWbMpcController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  latest_odometry_ = *msg;
}

void HumanoidWbMpcController::start_solver_thread(
  const ocs2::SystemObservation& initial_observation)
{
  stop_solver_thread();

  const std::string& solver_type = parameters_.ocs2.mpc.solverType;
  if (solver_type == "sqp") {
    mpc_solver_ = std::make_unique<ocs2::SqpMpc>(
      mpc_interface_->mpcSettings(),
      mpc_interface_->sqpSettings(),
      mpc_interface_->getOptimalControlProblem(),
      mpc_interface_->getInitializer());
  } else if (solver_type == "ddp") {
    mpc_solver_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      mpc_interface_->mpcSettings(),
      mpc_interface_->ddpSettings(),
      mpc_interface_->getRollout(),
      mpc_interface_->getOptimalControlProblem(),
      mpc_interface_->getInitializer());
  } else {
    throw std::runtime_error("[HumanoidWbMpcController] unsupported solver type: " + solver_type);
  }

  mpc_solver_->getSolverPtr()->setReferenceManager(mpc_interface_->getReferenceManagerPtr());
  mrt_interface_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_solver_);
  mrt_interface_->initRollout(&mpc_interface_->getRollout());
  mrt_interface_->setCurrentObservation(initial_observation);
  mrt_interface_->resetMpcNode(current_observation_to_reset_trajectory(initial_observation));

  terminate_solver_thread_.store(false);
  solver_thread_ = std::jthread([this] { solver_worker(); });
}

void HumanoidWbMpcController::stop_solver_thread()
{
  terminate_solver_thread_.store(true);
  if (solver_thread_.joinable()) {
    solver_thread_.join();
  }
  mrt_interface_.reset();
  mpc_solver_.reset();
}

void HumanoidWbMpcController::solver_worker()
{
  const double mpc_frequency = parameters_.ocs2.mpc.mpcDesiredFrequency;
  const bool sleep_between_solves = mpc_frequency > 0.0;
  const auto period = sleep_between_solves ?
    std::chrono::duration<double>(1.0 / mpc_frequency) :
    std::chrono::duration<double>(0.0);

  while (!terminate_solver_thread_.load() && rclcpp::ok()) {
    const auto wakeup_time = std::chrono::steady_clock::now() + period;
    try {
      mrt_interface_->advanceMpc();
      if (!mrt_interface_->updatePolicy()) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(),
          2000,
          "[HumanoidWbMpcController] MPC solver failed to update policy.");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "[HumanoidWbMpcController] MPC solver exception: %s",
        e.what());
    }

    if (sleep_between_solves) {
      std::this_thread::sleep_until(wakeup_time);
    }
  }
}

HumanoidWbMpcController::vector_t HumanoidWbMpcController::compute_weight_compensating_torque(
  const ocs2::SystemObservation& observation)
{
  const ocs2::contact_flag_t contact_flags{true, true};
  vector_t input = ocs2::humanoid::weightCompensatingInput(
    mpc_interface_->getPinocchioInterface(),
    contact_flags,
    *control_model_);
  return ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
    observation.state,
    input,
    mpc_interface_->getPinocchioInterface(),
    *control_model_);
}

HumanoidWbMpcController::vector_t HumanoidWbMpcController::compute_mpc_torque_command(
  const ocs2::SystemObservation& observation)
{
  mrt_interface_->setCurrentObservation(observation);

  vector_t policy_state;
  vector_t policy_input;
  size_t policy_mode = ocs2::humanoid::STANCE;
  mrt_interface_->evaluatePolicy(
    observation.time + parameters_.control.policyTimeOffset,
    observation.state,
    policy_state,
    policy_input,
    policy_mode);

  vector_t tau = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
    policy_state,
    policy_input,
    mpc_interface_->getPinocchioInterface(),
    *control_model_);

  const vector_t q = control_model_->getJointAngles(observation.state);
  const vector_t v = control_model_->getJointVelocities(observation.state, observation.input);
  const vector_t q_policy = control_model_->getJointAngles(policy_state);
  const vector_t v_policy = control_model_->getJointVelocities(policy_state, policy_input);

  tau += mpc_joint_kp_.cwiseProduct(q_policy - q) + mpc_joint_kd_.cwiseProduct(v_policy - v);
  return tau;
}

void HumanoidWbMpcController::write_torque_command(const vector_t& torque)
{
  const size_t n = std::min(command_interfaces_.size(), static_cast<size_t>(torque.size()));
  for (size_t i = 0; i < n; ++i) {
    double value = torque[static_cast<Eigen::Index>(i)];
    if (i < static_cast<size_t>(torque_limit_.size()) && torque_limit_[static_cast<Eigen::Index>(i)] > 0.0) {
      const double limit = torque_limit_[static_cast<Eigen::Index>(i)];
      value = std::clamp(value, -limit, limit);
    }
    if (!std::isfinite(value)) {
      value = 0.0;
    }
    command_interfaces_[i].set_value(value);
  }
}

}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidWbMpcController,
  controller_interface::ChainableControllerInterface)
