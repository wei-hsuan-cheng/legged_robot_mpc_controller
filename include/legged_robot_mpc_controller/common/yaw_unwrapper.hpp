#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__YAW_UNWRAPPER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__YAW_UNWRAPPER_HPP_

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged_robot_mpc_controller::common
{

/// Keeps the observed base yaw continuous across the +-pi atan2 wrap.
///
/// The MPC state stores yaw as a plain number: if the observation jumps from +pi to
/// -pi while turning, the warm-started solution and the pinned initial state disagree
/// by 2*pi and the solver output degrades for several solves (the "backward-facing
/// blow-up"). Unwrapping against the previous observation keeps the state continuous;
/// every downstream consumer (rotation matrices, target transforms) is 2*pi-periodic,
/// so an unbounded yaw is valid. Call from the update thread only.
class YawUnwrapper
{
public:
  void reset() { initialized_ = false; }

  double unwrap(double yaw)
  {
    if (!initialized_) {
      unwrapped_yaw_ = yaw;
      initialized_ = true;
    } else {
      unwrapped_yaw_ = ocs2::moduloAngleWithReference(yaw, unwrapped_yaw_);
    }
    return unwrapped_yaw_;
  }

private:
  bool initialized_{false};
  double unwrapped_yaw_{0.0};
};

}  // namespace legged_robot_mpc_controller::common

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__YAW_UNWRAPPER_HPP_
