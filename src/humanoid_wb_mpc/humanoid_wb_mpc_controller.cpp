#include "legged_robot_mpc_controller/humanoid_wb_mpc/humanoid_wb_mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
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
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_robot_mpc_controller/common/config/config_builder_utils.hpp"
#include "legged_robot_mpc_controller/humanoid_wb_mpc/wb_mpc_config_builder.hpp"

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

template<typename Derived>
std::string format_vector(const Eigen::MatrixBase<Derived>& value)
{
  std::ostringstream stream;
  stream << "[" << value.transpose() << "]";
  return stream.str();
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

  diagnostics_active_ = parameters_.diagnostics.activate;
  diagnostics_period_ms_ = static_cast<std::uint64_t>(std::max(
    1.0, std::round(parameters_.diagnostics.period * 1000.0)));

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
    const auto fixed_joint_dim = static_cast<Eigen::Index>(parameters_.ocs2.model.fixedJointNames.size());
    fixed_joint_kp_ = make_vector(parameters_.control.fixedJointKp, fixed_joint_dim, 100.0, "control.fixedJointKp");
    fixed_joint_kd_ = make_vector(parameters_.control.fixedJointKd, fixed_joint_dim, 1.0, "control.fixedJointKd");
    torque_limit_ = make_vector(parameters_.control.torqueLimit, joint_dim, 0.0, "control.torqueLimit");

    const auto reference_config = common::buildReferenceConfig(parameters_);
    if (parameters_.ocs2.gait.gaitFile.empty()) {
      throw std::invalid_argument("[HumanoidWbMpcController] ocs2.gait.gaitFile is empty.");
    }
    target_trajectories_calculator_ =
      std::make_unique<ocs2::humanoid::WBMpcTargetTrajectoriesCalculator>(
      reference_config,
      mpc_interface_->getMpcRobotModel(),
      mpc_interface_->mpcSettings().timeHorizon_);
    auto velocity_target_to_target_trajectories =
      [this](
      const ocs2::vector4_t& velocity_target,
      ocs2::scalar_t init_time,
      ocs2::scalar_t /* final_time */,
      const ocs2::vector_t& init_state) {
        auto target_trajectories = target_trajectories_calculator_->commandedVelocityToTargetTrajectories(
          velocity_target,
          init_time,
          init_state);
        heading_reference_.apply(
          velocity_target(3),
          init_time,
          control_model_->getBasePose(init_state)[3],
          target_trajectories);
        return target_trajectories;
      };
    auto base_pose_target_to_target_trajectories =
      [this](
      const ocs2::vector6_t& base_pose_target,
      ocs2::scalar_t init_time,
      ocs2::scalar_t /* final_time */,
      const ocs2::vector_t& init_state) {
        return target_trajectories_calculator_->commandedBasePoseToTargetTrajectories(
          base_pose_target, init_time, init_state);
      };
    auto motion_manager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      common::loadGaitMap(parameters_.ocs2.gait.gaitFile),
      reference_config,
      mpc_interface_->getSwitchedModelReferenceManagerPtr(),
      mpc_interface_->getMpcRobotModel(),
      std::move(velocity_target_to_target_trajectories),
      std::move(base_pose_target_to_target_trajectories),
      parameters_.target.defaultMode);
    motion_manager_ = std::move(motion_manager);
    rclcpp::QoS command_qos(1);
    command_qos.best_effort();
    motion_manager_->subscribe(
      get_node(), command_qos, parameters_.target.walkingVelocityTopic,
      parameters_.target.basePoseTopic, parameters_.target.modeTopic,
      parameters_.target.globalFrame);
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

  diag_pinocchio_ = std::make_unique<ocs2::PinocchioInterface>(mpc_interface_->getPinocchioInterface());

  try {
    performance_visualization_ = std::make_unique<visualization::PerformanceVisualization>(
      get_node(),
      mpc_interface_->getPinocchioInterface(),
      mpc_interface_->getMpcRobotModel(),
      visualization::makePerformanceVisualizationSettings(parameters_));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[HumanoidWbMpcController] Failed to configure performance visualization: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
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

  heading_reference_.reset();
  yaw_unwrapper_.reset();
  initial_observation_state_ = mpc_interface_->getInitialState();
  const auto initial_observation = build_observation(get_node()->now());
  if (parameters_.robot.commandInterface == "effort_pd") {
    const vector_t q_hold = control_model_->getJointAngles(initial_observation.state);
    write_joint_action_command(
      q_hold, vector_t::Zero(q_hold.size()), compute_weight_compensating_torque(initial_observation));
  } else {
    write_torque_command(compute_weight_compensating_torque(initial_observation));
  }
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
  log_interface_order();
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
  const bool effort_pd = parameters_.robot.commandInterface == "effort_pd";
  const std::vector<std::string> command_ifaces = effort_pd ?
    std::vector<std::string>{"position", "velocity", "effort"} :
    std::vector<std::string>{parameters_.robot.commandInterface};
  auto config = make_joint_interface_configuration(command_ifaces);
  for (const auto& joint_name : parameters_.ocs2.model.fixedJointNames) {
    for (const auto& iface : command_ifaces) {
      config.names.emplace_back(joint_name + "/" + iface);
    }
  }
  if (!config.names.empty()) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  }
  return config;
}

controller_interface::InterfaceConfiguration HumanoidWbMpcController::state_interface_configuration()
  const
{
  auto config = make_joint_interface_configuration(parameters_.robot.stateInterfaces);

  for (const auto& joint_name : parameters_.ocs2.model.fixedJointNames) {
    for (const auto& interface_name : parameters_.robot.stateInterfaces) {
      if (!interface_name.empty()) {
        config.names.emplace_back(joint_name + "/" + interface_name);
      }
    }
  }

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
  const bool effort_pd = parameters_.robot.commandInterface == "effort_pd";
  TorqueCommand command;
  if (mrt_interface_ && mrt_interface_->initialPolicyReceived()) {
    command = compute_mpc_torque_command(observation);
  } else {
    command.feedforward = compute_weight_compensating_torque(observation);
    command.requested = command.feedforward;
    // Hold the measured posture while waiting for the first policy.
    command.policy_position = control_model_->getJointAngles(observation.state);
    command.policy_velocity = vector_t::Zero(command.policy_position.size());
  }

  vector_t applied_torque;
  if (effort_pd) {
    // Joint-action mode: hardware runs the PD at the physics rate (legacy architecture);
    // we only forward the desired state and the MPC feedforward torque.
    write_joint_action_command(command.policy_position, command.policy_velocity, command.feedforward);
    applied_torque = command.feedforward;
  } else {
    applied_torque = write_torque_command(command.requested);
  }
  log_runtime_diagnostics(observation, command, applied_torque);
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

  ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(q);
  euler_zyx(0) = yaw_unwrapper_.unwrap(euler_zyx(0));
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
  // Observed contact mode: track the planned gait phase (legacy fed measured contact
  // flags; the planned mode is the flat-ground equivalent and keeps the MRT policy
  // evaluation synchronized with single/double-support phases while walking).
  observation.mode = ocs2::humanoid::STANCE;
  if (mpc_interface_) {
    const auto reference_manager = mpc_interface_->getSwitchedModelReferenceManagerPtr();
    if (reference_manager) {
      observation.mode = reference_manager->getModeSchedule().modeAtTime(observation.time);
    }
  }

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

    ocs2::vector3_t euler_zyx = ocs2::humanoid::quaternionToEulerZYX(q);
    euler_zyx(0) = yaw_unwrapper_.unwrap(euler_zyx(0));
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

  // Low-pass the generalized-velocity half of the state. Raw simulator velocity
  // dither at node 0 keeps the SQP baseline dynamics defect above g_max, trapping
  // the linesearch in constraint-repair mode where the tracking cost is ignored.
  const double cutoff_hz = parameters_.control.observationVelocityFilterCutoffHz;
  if (cutoff_hz > 0.0) {
    const auto vel_dim = static_cast<Eigen::Index>(model.getGenCoordinatesDim());
    const double dt = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt);
    auto velocity = observation.state.tail(vel_dim);
    if (filtered_generalized_velocity_.size() != vel_dim) {
      filtered_generalized_velocity_ = velocity;
    } else {
      filtered_generalized_velocity_ =
        alpha * velocity + (1.0 - alpha) * filtered_generalized_velocity_;
    }
    velocity = filtered_generalized_velocity_;
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
  mpc_solver_->getSolverPtr()->addSynchronizedModule(motion_manager_);

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

HumanoidWbMpcController::TorqueCommand HumanoidWbMpcController::compute_mpc_torque_command(
  const ocs2::SystemObservation& observation)
{
  mrt_interface_->setCurrentObservation(observation);

  const double diagnostics_period_s = static_cast<double>(diagnostics_period_ms_) / 1000.0;
  diagnostics_due_ = diagnostics_active_ &&
    (last_diagnostics_time_ < 0.0 ||
     observation.time - last_diagnostics_time_ >= diagnostics_period_s);
  if (diagnostics_due_) {
    last_diagnostics_time_ = observation.time;
  }

  TorqueCommand command;
  vector_t policy_state;
  vector_t policy_input;
  command.policy_mode = ocs2::humanoid::STANCE;
  mrt_interface_->evaluatePolicy(
    observation.time + parameters_.control.policyTimeOffset,
    observation.state,
    policy_state,
    policy_input,
    command.policy_mode);

  command.feedforward = ocs2::humanoid::computeJointTorques<ocs2::scalar_t>(
    policy_state,
    policy_input,
    mpc_interface_->getPinocchioInterface(),
    *control_model_);

  const vector_t q = control_model_->getJointAngles(observation.state);
  const vector_t v = control_model_->getJointVelocities(observation.state, observation.input);
  command.policy_position = control_model_->getJointAngles(policy_state);
  command.policy_velocity = control_model_->getJointVelocities(policy_state, policy_input);

  command.feedback =
    mpc_joint_kp_.cwiseProduct(command.policy_position - q) +
    mpc_joint_kd_.cwiseProduct(command.policy_velocity - v);
  command.requested = command.feedforward + command.feedback;

  // 10 Hz hand-off: copying the policy trajectory every RT update starves the solver.
  if (performance_visualization_ &&
      (last_visualization_time_ < 0.0 || observation.time - last_visualization_time_ >= 0.1)) {
    last_visualization_time_ = observation.time;
    performance_visualization_->update_visualization(mrt_interface_->getPolicy().stateTrajectory_);
  }

  if (diagnostics_due_) {
    const auto& target = mrt_interface_->getCommand().mpcTargetTrajectories_;
    if (!target.stateTrajectory.empty()) {
      command.target_final_base_pose = control_model_->getBasePose(target.stateTrajectory.back());
      command.target_final_time = target.timeTrajectory.back();
    }
    const auto& policy = mrt_interface_->getPolicy();
    if (!policy.stateTrajectory_.empty()) {
      command.plan_final_base_pose = control_model_->getBasePose(policy.stateTrajectory_.back());
      command.plan_final_time = policy.timeTrajectory_.back();
      command.policy_age = observation.time - policy.timeTrajectory_.front();
    }
    command.left_contact_wrench = control_model_->getContactWrench(policy_input, 0);
    command.right_contact_wrench = control_model_->getContactWrench(policy_input, 1);
    command.policy_base_pose = control_model_->getBasePose(policy_state);
    command.policy_base_velocity = control_model_->getBaseComVelocity(policy_state);
  }
  return command;
}

void HumanoidWbMpcController::log_interface_order() const
{
  if (!diagnostics_active_) {
    return;
  }

  std::ostringstream command_names;
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (i > 0) {
      command_names << ", ";
    }
    command_names << command_interfaces_[i].get_prefix_name() << "/" <<
      command_interfaces_[i].get_interface_name();
  }

  std::ostringstream state_names;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (i > 0) {
      state_names << ", ";
    }
    state_names << state_interfaces_[i].get_prefix_name() << "/" <<
      state_interfaces_[i].get_interface_name();
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController][INTERFACES] command_order=[%s] state_order=[%s]",
    command_names.str().c_str(), state_names.str().c_str());
}

void HumanoidWbMpcController::log_runtime_diagnostics(
  const ocs2::SystemObservation& observation,
  const TorqueCommand& command,
  const vector_t& applied_torque) const
{
  // All formatting/FK below only runs when the diagnostics gate fired this update.
  if (!diagnostics_active_ || !diagnostics_due_) {
    return;
  }

  const vector_t q = control_model_->getJointAngles(observation.state);
  const vector_t v = control_model_->getJointVelocities(observation.state, observation.input);
  const auto base_pose = control_model_->getBasePose(observation.state);
  const auto base_velocity = control_model_->getBaseComVelocity(observation.state);

  const size_t total_joint_count =
    parameters_.robot.jointNames.size() + parameters_.ocs2.model.fixedJointNames.size();
  vector_t effort_state = vector_t::Constant(
    static_cast<Eigen::Index>(total_joint_count),
    std::numeric_limits<double>::quiet_NaN());
  for (size_t i = 0; i < total_joint_count; ++i) {
    const std::string& joint_name = i < parameters_.robot.jointNames.size() ?
      parameters_.robot.jointNames[i] :
      parameters_.ocs2.model.fixedJointNames[i - parameters_.robot.jointNames.size()];
    const auto effort = get_state_interface_value(joint_name, hardware_interface::HW_IF_EFFORT);
    if (effort) {
      effort_state[static_cast<Eigen::Index>(i)] = *effort;
    }
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController][DIAGNOSTICS] t=%.3f basePose=%s baseVelocity=%s "
    "q=%s qd=%s policyMode=%zu policyQ=%s policyQd=%s "
    "tauFeedforward=%s tauFeedback=%s tauRequested=%s tauCommandClamped=%s "
    "effortStatePreviousStep=%s",
    observation.time,
    format_vector(base_pose).c_str(),
    format_vector(base_velocity).c_str(),
    format_vector(q).c_str(),
    format_vector(v).c_str(),
    command.policy_mode,
    format_vector(command.policy_position).c_str(),
    format_vector(command.policy_velocity).c_str(),
    format_vector(command.feedforward).c_str(),
    format_vector(command.feedback).c_str(),
    format_vector(command.requested).c_str(),
    format_vector(applied_torque).c_str(),
    format_vector(effort_state).c_str());

  // Measured foot contact-frame poses from FK: discriminates a stance foot pivoting
  // on the ground from the pelvis yawing through the hip joints.
  std::string foot_info;
  if (diag_pinocchio_) {
    const auto& pin_model = diag_pinocchio_->getModel();
    auto& pin_data = diag_pinocchio_->getData();
    const auto log_feet = [&](const vector_t& state, const char* tag, std::ostringstream& os) {
        pinocchio::forwardKinematics(
          pin_model, pin_data, control_model_->getGeneralizedCoordinates(state));
        pinocchio::updateFramePlacements(pin_model, pin_data);
        for (const auto& contact_name : control_model_->modelSettings.contactNames) {
          if (!pin_model.existFrame(contact_name)) {
            continue;
          }
          const auto& placement = pin_data.oMf[pin_model.getFrameId(contact_name)];
          os << " " << tag << (contact_name.find("_l_") != std::string::npos ? "L" : "R")
             << "=[" << placement.translation().x() << " " << placement.translation().y() << " "
             << placement.translation().z() << " yaw "
             << std::atan2(placement.rotation()(1, 0), placement.rotation()(0, 0)) << "]";
        }
      };
    std::ostringstream os;
    log_feet(observation.state, "meas", os);
    // Planned feet at the end of the active policy: shows where the solver wants to step.
    if (mrt_interface_ && mrt_interface_->initialPolicyReceived()) {
      const auto& policy = mrt_interface_->getPolicy();
      if (!policy.stateTrajectory_.empty()) {
        log_feet(policy.stateTrajectory_.back(), "plan", os);
      }
    }
    foot_info = os.str();
  }

  // Reference vs plan vs actual base motion: distinguishes a non-advancing reference,
  // a non-advancing optimized plan, and a tracking failure.
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[HumanoidWbMpcController][WALK] t=%.3f obsBase=%s obsBaseVel=%s policyBaseNow=%s "
    "policyBaseVelNow=%s targetFinal=%s @%.3f planFinal=%s @%.3f "
    "wrenchL=%s wrenchR=%s policyAge=%.3f feet:%s",
    observation.time,
    format_vector(base_pose).c_str(),
    format_vector(base_velocity).c_str(),
    format_vector(command.policy_base_pose).c_str(),
    format_vector(command.policy_base_velocity).c_str(),
    format_vector(command.target_final_base_pose).c_str(),
    command.target_final_time,
    format_vector(command.plan_final_base_pose).c_str(),
    command.plan_final_time,
    format_vector(command.left_contact_wrench).c_str(),
    format_vector(command.right_contact_wrench).c_str(),
    command.policy_age,
    foot_info.c_str());
}

void HumanoidWbMpcController::write_joint_action_command(
  const vector_t& q_des, const vector_t& qd_des, const vector_t& tau_ff)
{
  // effort_pd mode: hardware evaluates kp*(q_des-q)+kd*(qd_des-qd)+tau_ff every physics
  // step (legacy WBMpcMrtJointController actuator servo). 3 command interfaces per joint.
  const size_t mpc_joint_count = parameters_.robot.jointNames.size();
  const size_t fixed_joint_count = parameters_.ocs2.model.fixedJointNames.size();
  if (command_interfaces_.size() < (mpc_joint_count + fixed_joint_count) * 3) {
    return;
  }
  auto safe = [](double v) { return std::isfinite(v) ? v : 0.0; };
  for (size_t i = 0; i < mpc_joint_count; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    command_interfaces_[3 * i + 0].set_value(safe(i < static_cast<size_t>(q_des.size()) ? q_des[idx] : 0.0));
    command_interfaces_[3 * i + 1].set_value(safe(i < static_cast<size_t>(qd_des.size()) ? qd_des[idx] : 0.0));
    command_interfaces_[3 * i + 2].set_value(safe(i < static_cast<size_t>(tau_ff.size()) ? tau_ff[idx] : 0.0));
  }
  // Joints excluded from the MPC model (wrists): hold zero, soft hardware gains.
  for (size_t i = 0; i < fixed_joint_count; ++i) {
    const size_t base = 3 * (mpc_joint_count + i);
    command_interfaces_[base + 0].set_value(0.0);
    command_interfaces_[base + 1].set_value(0.0);
    command_interfaces_[base + 2].set_value(0.0);
  }
}

HumanoidWbMpcController::vector_t HumanoidWbMpcController::write_torque_command(
  const vector_t& torque)
{
  const size_t mpc_joint_count = parameters_.robot.jointNames.size();
  const size_t n = std::min(mpc_joint_count, static_cast<size_t>(torque.size()));
  const size_t fixed_joint_count = parameters_.ocs2.model.fixedJointNames.size();
  vector_t applied_torque = vector_t::Zero(
    static_cast<Eigen::Index>(mpc_joint_count + fixed_joint_count));
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
    applied_torque[static_cast<Eigen::Index>(i)] = value;
  }

  // The legacy controller kept joints excluded from the MPC model at zero with a
  // small joint-space impedance action. Keep the same behavior for the six wrist joints.
  const auto position_offset = interface_offset(
    parameters_.robot.stateInterfaces,
    hardware_interface::HW_IF_POSITION);
  const auto velocity_offset = interface_offset(
    parameters_.robot.stateInterfaces,
    hardware_interface::HW_IF_VELOCITY);
  const size_t state_stride = parameters_.robot.stateInterfaces.size();
  if (!position_offset || !velocity_offset ||
    state_interfaces_.size() < (mpc_joint_count + fixed_joint_count) * state_stride ||
    command_interfaces_.size() < mpc_joint_count + fixed_joint_count)
  {
    return applied_torque;
  }

  for (size_t i = 0; i < fixed_joint_count; ++i) {
    const size_t state_index = (mpc_joint_count + i) * state_stride;
    const double q = state_interfaces_[state_index + *position_offset].get_value();
    const double v = state_interfaces_[state_index + *velocity_offset].get_value();
    const double fixed_torque =
      fixed_joint_kp_[static_cast<Eigen::Index>(i)] * (0.0 - q) +
      fixed_joint_kd_[static_cast<Eigen::Index>(i)] * (0.0 - v);
    const double applied_fixed_torque = std::isfinite(fixed_torque) ? fixed_torque : 0.0;
    command_interfaces_[mpc_joint_count + i].set_value(applied_fixed_torque);
    applied_torque[static_cast<Eigen::Index>(mpc_joint_count + i)] = applied_fixed_torque;
  }
  return applied_torque;
}

}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::HumanoidWbMpcController,
  controller_interface::ChainableControllerInterface)
