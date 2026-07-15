#pragma once

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

/** Absolute floating-base pose command expressed in the configured global frame. */
struct BasePoseCommand {
  vector3_t position{vector3_t::Zero()};
  quaternion_t orientation{quaternion_t::Identity()};
};

}  // namespace ocs2::humanoid
