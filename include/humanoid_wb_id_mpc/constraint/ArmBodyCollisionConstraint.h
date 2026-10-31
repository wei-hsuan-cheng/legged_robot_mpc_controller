#pragma once

#include <pinocchio/fwd.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

struct ArmBodyCollisionSphere {
  std::string name;
  std::string linkName;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  scalar_t radius = 0.0;
};

struct ArmBodyCollisionConfig {
  bool activate = false;
  scalar_t minimumDistance = 0.015;
  std::string implementation = "polynomial";
  scalar_t mu = 3000.0;
  scalar_t delta = 0.02;
  std::vector<ArmBodyCollisionSphere> arms;
  std::vector<ArmBodyCollisionSphere> bodies;
};

// Arm x body pairs only. This does not replace the existing leg constraint.
class ArmBodyCollisionConstraint final : public StateConstraint {
 public:
  ArmBodyCollisionConstraint(const PinocchioInterface& pinocchio,
                             const PinocchioStateInputMapping<scalar_t>& mapping,
                             size_t stateDim, ArmBodyCollisionConfig config);
  ArmBodyCollisionConstraint(const ArmBodyCollisionConstraint& other);
  ArmBodyCollisionConstraint* clone() const override;

  bool isActive(scalar_t /*time*/) const override { return config_.activate; }
  size_t getNumConstraints(scalar_t /*time*/) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximationForRows(
      const vector_t& state, const std::vector<Eigen::Index>& rows) const;

  // For an independent visualization-thread instance, not the solver's instance.
  std::vector<Eigen::Vector3d> getWorldCenters(const vector_t& state) const;
  const ArmBodyCollisionConfig& getConfig() const { return config_; }

 private:
  void updateKinematics(const vector_t& state, bool needJacobians) const;
  Eigen::Vector3d centerInWorld(size_t sphereIndex) const;
  matrix_t centerJacobianInState(size_t sphereIndex, const vector_t& state) const;
  const ArmBodyCollisionSphere& sphere(size_t sphereIndex) const;

  // Each OCS2 worker clone owns its Data and mapping; no cross-worker mutable cache.
  mutable PinocchioInterface pinocchio_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mapping_;
  size_t stateDim_;
  ArmBodyCollisionConfig config_;
  std::vector<size_t> frameIds_;
};

// Same penalty objective as StateSoftConstraint, without assembling zero polynomial rows.
class ArmBodyCollisionSoftConstraint final : public StateCost {
 public:
  ArmBodyCollisionSoftConstraint(const PinocchioInterface& pinocchio,
                                 const PinocchioStateInputMapping<scalar_t>& mapping,
                                 size_t stateDim, const ArmBodyCollisionConfig& config);
  ArmBodyCollisionSoftConstraint* clone() const override { return new ArmBodyCollisionSoftConstraint(*this); }
  bool isActive(scalar_t time) const override { return constraint_.isActive(time); }
  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targets,
                    const PreComputation& preComp) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state, const TargetTrajectories& targets,
      const PreComputation& preComp) const override;

 private:
  ArmBodyCollisionConstraint constraint_;
  MultidimensionalPenalty penalty_;
  bool polynomial_;
  scalar_t delta_;
};

}  // namespace ocs2::humanoid
