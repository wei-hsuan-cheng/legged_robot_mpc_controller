#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__HUMANOID_CENTROIDAL_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <std_srvs/srv/trigger.hpp>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/u_int32.hpp>
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
#include <humanoid_common_mpc/whole_body_control/WeightedWbc.h>
#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h>

#include "legged_robot_mpc_controller/common/yaw_unwrapper.hpp"
#include "legged_robot_mpc_controller/humanoid_centroidal_mpc/centroidal_mpc_diagnostics.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/estimator_state_interfaces.hpp"
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
  /// Last observation fully read from hardware, with the time/mode refreshed, or
  /// the supplied fallback if none has been read yet.
  ocs2::SystemObservation hold_last_hardware_observation(
    const ocs2::SystemObservation& fallback) const;
  // Reads the estimator controller's exported state interfaces. The controller
  // manager runs that controller before this one, so this is this tick's estimate.
  void read_state_estimator();
  // Asks the estimator to initialize from stance, with an epoch one past the one
  // it last acted on, so a ready flag from before the request cannot pass for one
  // after it. Published from the control thread through a realtime publisher,
  // retried every tick until it goes out.
  void request_estimator_initialization();
  bool estimator_initialized_for_request() const;
  // Samples one row of the Phase-0 state diagnostics log (no-op when disabled).
  void record_diagnostics_state(
    const rclcpp::Time& time,
    const ocs2::SystemObservation& observation,
    const JointActionCommand& command);
  ocs2::TargetTrajectories current_observation_to_reset_trajectory(
    const ocs2::SystemObservation& observation);
  void start_solver_thread(const ocs2::SystemObservation& initial_observation);
  /// Build the SQP solver and the MRT, seeded from `initial_observation`. Heavy
  /// (allocations, problem setup), so it never runs on the control thread.
  void construct_solver(const ocs2::SystemObservation& initial_observation);
  void stop_solver_thread();
  void solver_worker();
  vector_t compute_weight_compensating_torque(const ocs2::SystemObservation& observation);
  JointActionCommand compute_mpc_joint_action(const ocs2::SystemObservation& observation);

  // ---- bring-up (floatingBase.source == state_estimator) -----------------------
  //
  // A real robot has no ground truth to start from, so the MPC must not run until
  // the estimator has something trustworthy to give it - and the estimator can
  // only be initialised once the robot is standing still on both feet. The
  // bring-up therefore follows MIT Cheetah's Passive -> StandUp -> Locomotion,
  // adapted to a humanoid that starts crouched with its feet flat (the humanoid
  // equivalent of Cheetah lying on its folded legs):
  //
  //   Passive      hold the pose measured at activation on the joint servo, no
  //                feed-forward - valid hanging on a hoist or crouched on the floor.
  //                Pure damping is NOT safe here: a humanoid on its feet with only
  //                damping folds up. Waits for ~/stand_up (or bringUp.autoStart).
  //   StandUp      joint PD from the measured pose to the nominal one, with gravity
  //                compensation from the IMU attitude. Uses no floating-base state.
  //   Settle       hold the nominal pose until the joints are still.
  //   Converge     waits for ~/start_locomotion (or autoStart) - so on hardware the
  //                operator can lower the robot onto its feet first - then
  //                initialises the estimator from stance and lets it converge.
  //   StartingMpc  hold while a worker thread builds the solver, seeded from the
  //                ESTIMATED observation, and computes the first policy.
  //   Blend        stand command blended into the MPC command.
  //   Locomotion   MPC only.
  //   Fault        joint damping, latched, if the trunk tilts past bringUp.tiltLimit.
  //
  // Ground truth plays no part in any of it.
  enum class BringUpPhase { Passive, StandUp, Settle, Converge, StartingMpc, Blend, Locomotion, Fault };
  JointActionCommand run_bring_up(const rclcpp::Time& time, const ocs2::SystemObservation& observation);
  /// Hold `q_target` with gravity compensated from the IMU attitude and the
  /// measured joints - the stand phases' command, independent of the estimator.
  JointActionCommand hold_command(const vector_t& q_target, const vector_t& q_measured,
                                  const std::optional<Eigen::Quaterniond>& imu_orientation);
  /// q_des = measured, so the joint servo's stiffness term is zero and only its
  /// damping acts.
  JointActionCommand damping_command(const vector_t& q_measured) const;
  std::optional<Eigen::Quaterniond> read_imu_orientation();
  /// IMU gyro and accelerometer, body frame; false if not readable.
  bool read_imu_rates(Eigen::Vector3d& gyro, Eigen::Vector3d& accel);
  void enter_bring_up_phase(BringUpPhase phase, double time);
  static const char* to_string(BringUpPhase phase);

  //! Which generalized velocity feeds the RNEA feedforward. See
  //! compute_mpc_joint_action() for why this is a choice and not a constant.
  //! Which inner loop turns the MPC policy into joint torques.
  enum class JointCommandMode { InverseDynamics, Wbc };
  JointCommandMode joint_command_mode_{JointCommandMode::InverseDynamics};

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
  //! Whole-body controller; built only when jointCommandMode is 'wbc'.
  std::unique_ptr<ocs2::humanoid::WeightedWbc> wbc_;
  //! Counts QP failures, so a persistent problem is visible instead of silent.
  uint64_t wbc_failure_count_{0};
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

  // The estimator controller named by floatingBase.estimator, through its
  // exported state interfaces (estimator_state_interfaces.hpp); not claimed for none.
  bool has_estimator() const { return parameters_.floatingBase.estimator != "none"; }
  std::vector<std::size_t> estimator_interface_index_;  //!< into state_interfaces_, set on activate
  std::vector<double> estimator_values_;
  state_estimation::FloatingBaseEstimate last_estimate_;
  state_estimation::EstimatorDiagnostics estimator_diagnostics_;
  estimator_interfaces::EstimatorStatus estimator_status_;
  bool estimator_driving_control_{false};

  // This controller -> the estimator: the gait, for its contact flags, and the
  // initialization epoch. Topics, not chaining: ros2_control cannot chain both ways.
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr mode_schedule_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr estimator_initialize_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::UInt32>>
    rt_estimator_initialize_publisher_;
  std::uint32_t estimator_epoch_{0};
  bool estimator_epoch_pending_{false};

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

  // Bring-up state. The phase and the times are control-thread only; the atomics
  // and start_observation_ are the hand-off to the worker that builds the solver.
  //! Atomic because the service callbacks read it, on the executor's thread.
  std::atomic<BringUpPhase> bring_up_phase_{BringUpPhase::Passive};
  //! Settle's rest test, published for ~/start_locomotion to check.
  std::atomic_bool at_rest_{false};
  vector_t passive_hold_joints_;  //!< measured at activation
  double phase_start_time_{-1.0};
  double settled_since_{-1.0};    //!< first tick the trunk was at rest, or -1
  // Rest detection from the IMU (zero-velocity detection): low-passed gyro speed
  // and accelerometer magnitude. Joint velocities are the wrong test - the ankles
  // chatter at tens of Hz under the stiff joint servo while the trunk is still.
  // Filtered as VECTORS, then normed: the norm of g plus a zero-mean vibration is
  // biased upward (~sqrt(g^2 + sigma^2)), and the ankle chatter alone shakes the
  // pelvis accelerometer by several m/s^2.
  Eigen::Vector3d rest_gyro_lpf_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rest_accel_lpf_{Eigen::Vector3d::Zero()};
  bool rest_filter_initialized_{false};
  vector_t stand_up_start_joints_;  //!< measured on entering StandUp
  vector_t stand_nominal_joints_;   //!< reference.defaultJointState
  ocs2::SystemObservation start_observation_;     //!< written before locomotion_requested_ is set
  std::atomic_bool locomotion_requested_{false};  //!< control thread -> worker: build the solver
  std::atomic_bool mpc_ready_{false};             //!< worker -> control thread: mrt_interface_ is usable
  std::atomic_bool stand_up_requested_{false};    //!< ~/stand_up was called
  std::atomic_bool start_requested_{false};       //!< ~/start_locomotion was called
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_locomotion_service_;

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
