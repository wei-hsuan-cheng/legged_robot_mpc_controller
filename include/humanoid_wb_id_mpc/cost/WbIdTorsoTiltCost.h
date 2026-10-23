#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_wb_id_mpc/common/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/** Soft gravity-relative tilt corridor and transverse angular-velocity regularization.
 * The tracked frame's local +Z is its upright axis. There is no pose trajectory or yaw target.
 */
class WbIdTorsoTiltCost final : public StateInputCostGaussNewtonAd {
 public:
  struct Config {
    bool activate = false;
    std::string linkName = "torso_link";
    scalar_t freeTiltAngle = 0.08726646259971647;  // 5 degrees, in radians.
    scalar_t tiltWeight = 10.0;
    scalar_t angularVelocityWeight = 5.0;
  };

  WbIdTorsoTiltCost(const Config& config,
                   const PinocchioInterface& pinocchioInterface,
                   const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                   const ModelSettings& modelSettings);
  ~WbIdTorsoTiltCost() override = default;
  WbIdTorsoTiltCost* clone() const override { return new WbIdTorsoTiltCost(*this); }

  vector_t getParameters(scalar_t time,
                         const TargetTrajectories& targetTrajectories,
                         const PreComputation& preComputation) const override;

 private:
  WbIdTorsoTiltCost(const WbIdTorsoTiltCost& other);

  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;

  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<WbIdMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;
  std::size_t frameId_;
  vector_t parameters_;
};

}  // namespace ocs2::humanoid
