#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h>

#include "legged_robot_mpc_controller/common/yaw_unwrapper.hpp"
#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/centroidal_mpc_diagnostics.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/inekf_floating_base_estimator.hpp"
#include "legged_robot_mpc_controller/common/ros2_procedural_mpc_motion_manager.hpp"
#include "legged_robot_mpc_controller/humanoid_centroidal_mpc_controller_parameters.hpp"
#include "legged_robot_mpc_controller/visualization/performance_visualization.hpp"

namespace legged_robot_mpc_controller
{

class HumanoidCentroidalMpcController : public controller_interface::ChainableControllerInterface
{
public:
  using Params = humanoid_centroidal_mpc_controller::Params;
  using ParamListener = humanoid_centroidal_mpc_controller::ParamListener;

  ~HumanoidCentroidalMpcController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_reference_from_subscribers() override;
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
  /// Last observation fully read from hardware, with the time/mode refreshed, or
  /// the supplied fallback if none has been read yet.
  ocs2::SystemObservation hold_last_hardware_observation(
    const ocs2::SystemObservation& fallback) const;
  // Runs the InEKF state estimator each control tick (bootstraps from GT, publishes its odom).
  void update_state_estimator(const rclcpp::Time& time);
  void log_state_estimator_validation(const rclcpp::Time& time);
  // Samples one row of the Phase-0 state diagnostics log (no-op when disabled).
  void record_diagnostics_state(
    const rclcpp::Time& time,
    const ocs2::SystemObservation& observation,
    const JointActionCommand& command);
  ocs2::TargetTrajectories current_observation_to_reset_trajectory(
    const ocs2::SystemObservation& observation);
  void start_solver_thread(const ocs2::SystemObservation& initial_observation);
  void stop_solver_thread();
  void solver_worker();
  vector_t compute_weight_compensating_torque(const ocs2::SystemObservation& observation);
  JointActionCommand compute_mpc_joint_action(const ocs2::SystemObservation& observation);

  //! Which generalized velocity feeds the RNEA feedforward. See
  //! compute_mpc_joint_action() for why this is a choice and not a constant.
  enum class InverseDynamicsVelocity { Zero, Measured, Policy };
  InverseDynamicsVelocity inverse_dynamics_velocity_{InverseDynamicsVelocity::Measured};
  void write_joint_action_command(
    const vector_t& q_des, const vector_t& qd_des, const vector_t& tau_ff);

  std::shared_ptr<ParamListener> param_listener_;
  Params parameters_;

  std::unique_ptr<ocs2::humanoid::CentroidalMpcInterface> mpc_interface_;
  std::unique_ptr<ocs2::humanoid::CentroidalMpcRobotModel<ocs2::scalar_t>> control_model_;
  // Controller-owned pinocchio copy for the observation momentum mapping and the
  // feedforward inverse dynamics (the solver owns its own instances).
  std::unique_ptr<ocs2::PinocchioInterface> control_pinocchio_;
  std::unique_ptr<ocs2::humanoid::CentroidalMpcTargetTrajectoriesCalculator> target_trajectories_calculator_;
  std::shared_ptr<Ros2ProceduralMpcMotionManager> motion_manager_;
  std::unique_ptr<ocs2::MPC_BASE> mpc_solver_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_interface_;
  std::unique_ptr<visualization::PerformanceVisualization> performance_visualization_;

  // Keeps the observed yaw continuous across the +-pi wrap (update thread only).
  common::YawUnwrapper yaw_unwrapper_;

  /**
   * Control-thread snapshot of the solver's mode schedule.
   *
   * The reference manager stores its ModeSchedule in an ocs2::BufferedValue,
   * whose contract is explicit that get() is NOT thread-safe with respect to
   * updateFromBuffer() - and updateFromBuffer() runs on the solver thread inside
   * preSolverRun(), i.e. during advanceMpc(). Calling getContactFlags() or
   * getModeSchedule() straight from the 1 kHz control loop therefore reads two
   * std::vectors while they may be being move-assigned. The visible consequence
   * is a wrong contact flag handed to the InEKF, which adds or drops a landmark
   * at the wrong instant - exactly the profile of a rare unreproducible fall.
   *
   * The solver thread publishes the schedule after each iteration completes,
   * when reading it from that thread is safe, and the control loop reads this
   * snapshot instead. RealtimeBuffer's reader never blocks: if the writer holds
   * the lock it returns the previous snapshot, which is the right trade here
   * since the schedule changes far more slowly than the control period.
   */
  realtime_tools::RealtimeBuffer<ocs2::ModeSchedule> mode_schedule_buffer_;

  /// Contact flags at `time` from the snapshot above. Control-thread safe.
  ocs2::contact_flag_t contact_flags_at(double time);
  /// Mode at `time` from the snapshot above. Control-thread safe.
  size_t mode_at(double time);
  /// Publish the solver's current mode schedule. Solver thread only.
  void publish_mode_schedule();

  // Observation velocity low-pass state (see build_observation).
  vector_t filtered_generalized_velocity_;
  double last_visualization_time_{-1.0};

  // Optional InEKF floating-base estimator. When enabled it runs in parallel and
  // publishes its odometry; when floatingBase.source == "state_estimator" its
  // output drives the observation instead of the MuJoCo ground-truth body frame.
  std::unique_ptr<state_estimation::FloatingBaseEstimatorInterface> state_estimator_;
  state_estimation::FloatingBaseEstimate last_estimate_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_estimate_odom_publisher_;
  double last_estimate_publish_time_{-1.0};
  // Filter convergence window: the estimate only drives control after this time.
  double estimator_warmup_end_time_{-1.0};
  bool estimator_driving_control_{false};
  double last_estimator_validation_time_{-1.0};
  Eigen::Vector3d previous_gyroscope_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_accelerometer_bias_{Eigen::Vector3d::Zero()};
  bool estimator_bias_validation_initialized_{false};

  // Phase-0 instrumentation. Null unless diagnostics.log.enabled is set; the
  // control thread feeds its state log and the solver thread feeds its cost log.
  std::unique_ptr<CentroidalMpcDiagnostics> diagnostics_;
  // Scratch for the diagnostics sample, kept alive so the control thread does
  // not allocate a joint vector per tick.
  CentroidalMpcDiagnostics::StateSample diagnostics_sample_;
  // Latest observation handed to the solver thread for the cost breakdown. The
  // control thread only ever try_locks this: a diagnostic is never worth
  // blocking the control loop for, and a one-tick-stale observation changes
  // nothing about which cost term dominates.
  std::mutex diagnostics_observation_mutex_;
  ocs2::SystemObservation diagnostics_observation_;
  bool diagnostics_observation_valid_{false};

  std::jthread solver_thread_;
  std::atomic_bool terminate_solver_thread_{false};

  /**
   * The MPC observation is built entirely from the hardware interfaces - all 35
   * states are overwritten from the measured joint and floating-base feedback.
   * The configured ocs2.initialState is only a construction-time placeholder for
   * the interface; the robot's actual starting pose comes from wherever the
   * hardware says it is, which in simulation is initial_pose.yaml via the
   * ros2_control xacro.
   *
   * These hold the last observation that was fully read from hardware, so a
   * transient interface dropout holds the last KNOWN state instead of silently
   * asserting a configured pose the robot is not in.
   */
  ocs2::SystemObservation last_hardware_observation_;
  bool has_hardware_observation_{false};
  //! Set by build_observation(): true when every interface it needs was readable.
  bool observation_from_hardware_{false};
  vector_t mpc_joint_kp_;
  vector_t mpc_joint_kd_;
  vector_t fixed_joint_kp_;
  vector_t fixed_joint_kd_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_
