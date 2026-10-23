#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_wb_id_mpc/WbIdMpcInterface.h>
#include <humanoid_wb_id_mpc/command/WbIdMpcTargetTrajectoriesCalculator.h>
#include <humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h>

#include "legged_robot_mpc_controller/common/ros2_procedural_mpc_motion_manager.hpp"
#include "legged_robot_mpc_controller/common/yaw_unwrapper.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/estimator_state_interfaces.hpp"
#include "legged_robot_mpc_controller/humanoid_whole_body_inverse_dynamics_mpc_controller_parameters.hpp"
#include "legged_robot_mpc_controller/visualization/performance_visualization.hpp"

namespace legged_robot_mpc_controller
{

/**
 * Whole-body inverse-dynamics MPC as a ros2_control controller.
 *
 * Uses its own generated parameter schema, with shared estimator/command interfaces.
 * The model is ocs2::humanoid::WbIdMpcInterface. The MPC's
 * input already contains the joint torques, so there is no separate inverse dynamics or
 * whole-body QP - the policy's tau is the feedforward, with the joint servo's PD around
 * the policy's (q, qd) (Molnar et al., RA-L 2025).
 */
class HumanoidWholeBodyInverseDynamicsMpcController : public controller_interface::ChainableControllerInterface
{
public:
  using Params = humanoid_whole_body_inverse_dynamics_mpc_controller::Params;
  using ParamListener = humanoid_whole_body_inverse_dynamics_mpc_controller::ParamListener;

  ~HumanoidWholeBodyInverseDynamicsMpcController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

private:
  using vector_t = ocs2::vector_t;

  struct JointActionCommand
  {
    vector_t policy_position;
    vector_t policy_velocity;
    vector_t feedforward;
  };

  controller_interface::InterfaceConfiguration make_joint_interface_configuration(
    const std::vector<std::string>& interface_names) const;
  std::vector<std::string> floating_base_state_interface_names() const;
  bool read_joint_state(vector_t& q, vector_t& v);
  std::optional<double> get_state_interface_value(
    const std::string& prefix_name,
    const std::string& interface_name) const;
  ocs2::SystemObservation build_observation(const rclcpp::Time& time);
  ocs2::SystemObservation hold_last_hardware_observation(
    const ocs2::SystemObservation& fallback) const;
  void read_state_estimator();
  void request_estimator_initialization();
  void write_estimator_references(double time);
  bool estimator_initialized_for_request() const;
  ocs2::TargetTrajectories current_observation_to_reset_trajectory(
    const ocs2::SystemObservation& observation);
  //! Start the worker that builds the solver from start_observation_ once the
  //! control loop sets locomotion_requested_, then runs it.
  void start_deferred_solver_thread();
  void construct_solver(const ocs2::SystemObservation& initial_observation);
  void stop_solver_thread();
  void solver_worker();
  vector_t compute_weight_compensating_torque(const ocs2::SystemObservation& observation);
  JointActionCommand compute_mpc_joint_action(const ocs2::SystemObservation& observation);
  void clamp_torque(vector_t& tau) const;

  // Bring-up: identical to HumanoidCentroidalMpcController (see there), so the two
  // controllers start a robot the same way: Passive -> StandUp -> Settle -> Converge
  // -> StartingMpc -> Blend -> Locomotion, Fault on excessive tilt.
  enum class BringUpPhase { Passive, StandUp, Settle, Converge, StartingMpc, Blend, Locomotion, Fault };
  JointActionCommand run_bring_up(const rclcpp::Time& time, const ocs2::SystemObservation& observation);
  JointActionCommand hold_command(const vector_t& q_target, const vector_t& q_measured,
                                  const std::optional<Eigen::Quaterniond>& imu_orientation);
  JointActionCommand damping_command(const vector_t& q_measured) const;
  std::optional<Eigen::Quaterniond> read_imu_orientation();
  bool read_imu_rates(Eigen::Vector3d& gyro, Eigen::Vector3d& accel);
  void enter_bring_up_phase(BringUpPhase phase, double time);
  static const char* to_string(BringUpPhase phase);

  //! Where the joint torque feedforward comes from (control.feedforward).
  enum class Feedforward { PolicyTorque, InverseDynamics };
  Feedforward feedforward_{Feedforward::PolicyTorque};

  void write_joint_action_command(
    const vector_t& q_des, const vector_t& qd_des, const vector_t& tau_ff);

  std::shared_ptr<ParamListener> param_listener_;
  Params parameters_;

  std::unique_ptr<ocs2::humanoid::WbIdMpcInterface> mpc_interface_;
  std::unique_ptr<ocs2::humanoid::WbIdMpcRobotModel<ocs2::scalar_t>> control_model_;
  //! Controller-owned pinocchio copy for the stand's gravity compensation and the
  //! optional measured-state inverse dynamics (the solver owns its own instances).
  std::unique_ptr<ocs2::PinocchioInterface> control_pinocchio_;
  std::unique_ptr<ocs2::humanoid::WbIdMpcTargetTrajectoriesCalculator> target_trajectories_calculator_;
  std::shared_ptr<Ros2ProceduralMpcMotionManager> motion_manager_;
  std::unique_ptr<ocs2::MPC_BASE> mpc_solver_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_interface_;
  std::unique_ptr<visualization::PerformanceVisualization> performance_visualization_;
  vector_t torque_limit_;  //!< empty = no software clamp
  //! effort_pd servo gains per MPC joint (control.jointEffortPd); empty = keep the hardware's.
  vector_t joint_kp_;
  vector_t joint_kd_;
  //! Command-interface indices of <joint>/kp, <joint>/kd, interleaved per MPC joint.
  std::vector<std::size_t> gain_command_index_;

  //! Tracking debug (diagnostics.trackingRate): one row per published tick of the MPC
  //! policy at the current time, the command written and the measured / estimated state.
  void configure_tracking();
  void publish_tracking(const ocs2::SystemObservation& observation, const JointActionCommand& command);
  std::vector<std::string> tracking_names_;
  std_msgs::msg::Float64MultiArray tracking_row_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_publisher_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> tracking_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_names_publisher_;
  std::size_t tracking_decimation_{0};
  std::size_t tracking_tick_{0};
  std::vector<std::size_t> tracking_foot_frames_;
  //! Solver metrics (same switch): one row per solve on ~/solver, from the solver thread.
  void publish_solver_metrics(double advance_ms);
  std::unique_ptr<ocs2::OptimalControlProblem> solver_diag_problem_;  //!< private copy: never races the solver
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solver_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr solver_names_publisher_;
  std::vector<std::pair<std::string, std::size_t>> solver_eq_terms_;
  std::vector<std::string> solver_cost_terms_;
  std::vector<std::string> solver_state_cost_terms_;
  std::vector<std::string> solver_soft_terms_;
  std::vector<std::string> solver_state_soft_terms_;
  ocs2::PrimalSolution solver_primal_;
  std_msgs::msg::Float64MultiArray solver_row_;
  //! The policy compute_mpc_joint_action evaluated this tick, if any.
  bool tracking_policy_valid_{false};
  vector_t tracking_policy_state_;
  vector_t tracking_policy_input_;
  std::size_t tracking_policy_mode_{0};
  //! Ticks whose policy time fell past the torque window (see compute_mpc_joint_action).
  //! Atomic: counted on the control thread, reported by the solver thread.
  std::atomic<uint64_t> torque_window_overruns_{0};

  common::YawUnwrapper yaw_unwrapper_;

  //! Control-thread snapshot of the solver's mode schedule; see the centroidal
  //! controller for why the reference manager must not be read from this thread.
  realtime_tools::RealtimeBuffer<ocs2::ModeSchedule> mode_schedule_buffer_;
  ocs2::contact_flag_t contact_flags_at(double time);
  size_t mode_at(double time);
  void publish_mode_schedule();

  vector_t filtered_generalized_velocity_;
  double last_visualization_time_{-1.0};

  bool has_estimator() const { return parameters_.floatingBase.estimator != "none"; }
  std::vector<std::size_t> estimator_interface_index_;
  std::vector<double> estimator_values_;
  state_estimation::FloatingBaseEstimate last_estimate_;
  state_estimation::EstimatorDiagnostics estimator_diagnostics_;
  estimator_interfaces::EstimatorStatus estimator_status_;
  bool estimator_driving_control_{false};
  std::vector<std::size_t> estimator_command_index_;
  std::uint32_t estimator_epoch_{0};

  std::jthread solver_thread_;
  std::atomic_bool terminate_solver_thread_{false};

  std::atomic<BringUpPhase> bring_up_phase_{BringUpPhase::Passive};
  std::atomic_bool at_rest_{false};
  vector_t passive_hold_joints_;
  double phase_start_time_{-1.0};
  double settled_since_{-1.0};
  Eigen::Vector3d rest_gyro_lpf_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rest_accel_lpf_{Eigen::Vector3d::Zero()};
  bool rest_filter_initialized_{false};
  vector_t stand_up_start_joints_;
  vector_t stand_nominal_joints_;
  ocs2::SystemObservation start_observation_;
  std::atomic_bool locomotion_requested_{false};
  std::atomic_bool mpc_ready_{false};
  bool active_policy_ready_{false};  // Owned by the control thread, like the active MRT policy.
  std::atomic_bool stand_up_requested_{false};
  std::atomic_bool start_requested_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_locomotion_service_;

  ocs2::SystemObservation last_hardware_observation_;
  bool has_hardware_observation_{false};
  bool observation_from_hardware_{false};
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC__HUMANOID_WHOLE_BODY_INVERSE_DYNAMICS_MPC_CONTROLLER_HPP_
