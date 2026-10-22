#pragma once

#include <atomic>
#include <limits>
#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2::humanoid {

/**
 * The part of the horizon in which the joint torques are decision variables.
 *
 * Both reference formulations drop the actuated rows of the inverse dynamics, and with
 * them the torque limits, after the first few nodes: Khazoom et al. keep them for the
 * first 2 time steps, Molnar et al. (wb-mpc-locoman, tau_nodes = 3) for the first 3.
 * Only the torques at the start of the horizon are ever sent to the robot, while the
 * unactuated base rows - the Newton-Euler equations - are what shapes the motion over
 * the rest of it, so this trades no relevant accuracy for a much smaller problem.
 *
 * The window is a time span from the start of the current horizon, which the solver
 * reports through preSolverRun(); duration < 0 means the whole horizon.
 */
class WbIdTorqueWindow {
 public:
  explicit WbIdTorqueWindow(scalar_t duration) : duration_(duration) {}

  void setHorizonStart(scalar_t initTime) { horizonStart_.store(initTime, std::memory_order_release); }

  bool contains(scalar_t time) const {
    if (duration_ < 0.0) {
      return true;
    }
    return time < horizonStart_.load(std::memory_order_acquire) + duration_;
  }

  scalar_t duration() const { return duration_; }

  // A policy may be older than the solver's current horizon. Check its own
  // timestamps, and every knot contributing to the interpolated input.
  bool containsInterpolatedInput(scalar_t time, const scalar_array_t& policyTimes) const {
    if (policyTimes.empty() || time > policyTimes.back()) {
      return false;
    }
    if (duration_ < 0.0) {
      return true;
    }
    const auto [index, alpha] = LinearInterpolation::timeSegment(time, policyTimes);
    const scalar_t end = policyTimes.front() + duration_;
    if (alpha > 0.0 && policyTimes[index] >= end) {
      return false;
    }
    return alpha >= 1.0 || policyTimes[index + 1] < end;
  }

 private:
  std::atomic<scalar_t> horizonStart_{-std::numeric_limits<scalar_t>::infinity()};
  const scalar_t duration_;
};

/** Hands the start of every horizon to the torque window before the solver runs. */
class WbIdTorqueWindowSynchronizedModule final : public SolverSynchronizedModule {
 public:
  explicit WbIdTorqueWindowSynchronizedModule(std::shared_ptr<WbIdTorqueWindow> window) : window_(std::move(window)) {}

  void preSolverRun(scalar_t initTime, scalar_t /*finalTime*/, const vector_t& /*initState*/,
                    const ReferenceManagerInterface& /*referenceManager*/) override {
    window_->setHorizonStart(initTime);
  }
  void postSolverRun(const PrimalSolution& /*primalSolution*/) override {}

 private:
  std::shared_ptr<WbIdTorqueWindow> window_;
};

}  // namespace ocs2::humanoid
