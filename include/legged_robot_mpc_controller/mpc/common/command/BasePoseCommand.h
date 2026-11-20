#pragma once

#include "legged_robot_mpc_controller/mpc/common/model/Types.h"

namespace ocs2::humanoid {

/** Absolute floating-base pose command expressed in the configured global frame. */
struct BasePoseCommand {
  vector3_t position{vector3_t::Zero()};
  quaternion_t orientation{quaternion_t::Identity()};
};

}  // namespace ocs2::humanoid
