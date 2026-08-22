#ifndef LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__CENTROIDAL_MPC_DIAGNOSTICS_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__CENTROIDAL_MPC_DIAGNOSTICS_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>

#include "legged_robot_mpc_controller/common/diagnostics_csv_logger.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/floating_base_estimate.hpp"
#include "legged_robot_mpc_controller/humanoid_state_estimation/inekf_floating_base_estimator.hpp"

namespace legged_robot_mpc_controller
{

/**
 * Phase-0 instrumentation for the centroidal MPC + InEKF stack.
 *
 * Two questions need per-tick evidence and neither is answerable from the
 * existing rate-limited RCLCPP_INFO validation line:
 *
 *  1. Is the filter *consistent*, not merely close to ground truth? That needs
 *     the covariance it reports for itself and the innovation it saw relative to
 *     the covariance it predicted (NIS), sampled densely enough to see a single
 *     touchdown.
 *  2. Which cost term actually decides the posture the solver picks? Summed cost
 *     tells you nothing about the balance between, say, leg torque and base-z
 *     tracking; only the per-term values do.
 *
 * These come from two different threads at two different rates, so they go to
 * two files rather than being forced into one schema:
 *
 *  - `<prefix>_state.csv`, produced by the control thread, decimated from the
 *    controller rate. Estimator internals, estimate-vs-ground-truth error, and
 *    the centroidal momentum recomputed from BOTH feedback sources through the
 *    same A_G(q) - which is what shows how much momentum error the estimator
 *    injects into the dominant state.
 *  - `<prefix>_cost.csv`, produced by the solver thread once per MPC iteration.
 *    Per-cost-term values at the current observation, plus the reference the
 *    terms are tracking.
 *
 * Splitting by thread is not just tidiness. The cost terms read the reference
 * manager (swing trajectories, mode schedule), which the solver thread writes in
 * preSolverRun(); evaluating them from the control thread would race with that.
 * Evaluating them on the solver thread, between iterations, does not.
 */
class CentroidalMpcDiagnostics
{
public:
  struct Settings
  {
    bool enabled{false};
    //! Output path prefix; "_state.csv" / "_cost.csv" are appended. A "%t" in the
    //! prefix is replaced by the local start time, so consecutive runs do not
    //! overwrite each other.
    std::string pathPrefix;
    double stateRate{200.0};    //!< sampling rate [Hz] of the control-thread log
    bool logCost{true};         //!< also produce the solver-thread cost log
    double costRate{50.0};      //!< max sampling rate [Hz] of the cost log
    std::size_t bufferRows{4096};
  };

  CentroidalMpcDiagnostics(
    Settings settings,
    const ocs2::humanoid::CentroidalMpcInterface & mpc_interface,
    const ocs2::PinocchioInterface & pinocchio_interface,
    std::vector<std::string> contact_frames);

  ~CentroidalMpcDiagnostics();

  /// Open both logs and start their writer threads. Call from on_activate().
  void start();
  /// Flush and close both logs. Call from on_deactivate(). Safe to call twice.
  void stop();

  bool enabled() const { return settings_.enabled; }

  /// Everything the control thread samples for one row of `<prefix>_state.csv`.
  struct StateSample
  {
    double time{0.0};
    //! 0 = estimator not yet seeded, 1 = warm-up (ground truth still drives
    //! control), 2 = the estimate drives control. Lets a run be segmented
    //! without guessing where the hand-off happened.
    int phase{0};
    int mode{0};
    bool contact_left{false};
    bool contact_right{false};

    // Ground truth from the simulator body, in the representation build_observation uses.
    Eigen::Vector3d gt_position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond gt_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d gt_linear_velocity_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gt_angular_velocity_local{Eigen::Vector3d::Zero()};
    bool gt_valid{false};

    //! Joint positions/velocities in the MPC (pinocchio) ordering, needed to
    //! recompute the centroidal momentum from each feedback source.
    ocs2::vector_t joint_positions;
    ocs2::vector_t joint_velocities;
    bool joint_valid{false};
  };

  /**
   * Sample one control tick. Cheap and rate-limited: returns immediately unless
   * this tick falls on the configured sampling period. Called from the control
   * thread.
   */
  void recordState(
    const StateSample & sample,
    const state_estimation::FloatingBaseEstimate & estimate,
    const state_estimation::InekfFloatingBaseEstimator::Diagnostics & filter);

  /**
   * Evaluate and log the per-cost-term breakdown at the given observation.
   * Called from the solver thread, between MPC iterations.
   */
  void recordCost(const ocs2::SystemObservation & observation, double advance_ms);

  /**
   * True if a row was ever filled with a number of values other than the number
   * of declared columns. That would silently shift every column after the
   * mismatch, so it is checked rather than assumed: the schema and the fill code
   * are two separate lists that have to be kept in step by hand.
   */
  bool schemaMismatch() const { return schema_mismatch_; }

  std::size_t droppedStateRows() const { return state_log_.droppedRows(); }
  std::size_t droppedCostRows() const { return cost_log_.droppedRows(); }
  const std::string & statePath() const { return state_path_; }
  const std::string & costPath() const { return cost_path_; }

private:
  void buildStateSchema();
  void buildCostSchema();
  /// Centroidal normalized momentum A_G(q) v / m for the given base feedback.
  ocs2::vector_t normalizedMomentum(
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & linear_velocity_world,
    const Eigen::Vector3d & angular_velocity_local,
    const ocs2::vector_t & joint_positions,
    const ocs2::vector_t & joint_velocities);

  Settings settings_;
  const ocs2::humanoid::CentroidalMpcInterface * mpc_interface_;

  //! Private pinocchio copy: recomputing momentum must not touch the instances
  //! the control loop or the solver are using.
  ocs2::PinocchioInterface pinocchio_;
  ocs2::CentroidalModelInfo info_;

  //! Private deep copy of the problem, so evaluating cost terms uses its own
  //! PreComputation rather than one the solver is mutating.
  std::unique_ptr<ocs2::OptimalControlProblem> problem_;
  std::vector<std::string> running_cost_names_;
  std::vector<std::string> terminal_cost_names_;

  std::vector<std::string> contact_frames_;

  DiagnosticsCsvLogger state_log_;
  DiagnosticsCsvLogger cost_log_;
  std::vector<std::string> state_columns_;
  std::vector<std::string> cost_columns_;
  std::string state_path_;
  std::string cost_path_;

  double next_state_time_{-1.0};
  double next_cost_time_{-1.0};
  bool schema_mismatch_{false};

  // Scratch, preallocated so recordState() does not allocate.
  ocs2::vector_t q_scratch_;
  ocs2::vector_t v_scratch_;
  ocs2::vector_t momentum_estimate_;
  ocs2::vector_t momentum_ground_truth_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__HUMANOID_CENTROIDAL_MPC__CENTROIDAL_MPC_DIAGNOSTICS_HPP_
