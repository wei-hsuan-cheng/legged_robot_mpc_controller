#include <pinocchio/fwd.hpp>

#include "humanoid_wb_id_mpc/constraint/ArmBodyCollisionConstraint.h"

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>

namespace ocs2::humanoid {
namespace {
constexpr scalar_t kDistanceEpsilon = 1e-9;

void requirePositive(scalar_t value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("[ArmBodyCollision] invalid positive parameter: ") + name);
  }
}

std::unique_ptr<PenaltyBase> makePenalty(const ArmBodyCollisionConfig& config) {
  if (config.implementation == "rbf") {
    return std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{config.mu, config.delta});
  }
  if (config.implementation == "polynomial") {
    return std::make_unique<PieceWisePolynomialBarrierPenalty>(PieceWisePolynomialBarrierPenalty::Config{config.mu, config.delta});
  }
  throw std::invalid_argument("[ArmBodyCollision] implementation must be rbf or polynomial");
}
}  // namespace

ArmBodyCollisionConstraint::ArmBodyCollisionConstraint(
    const PinocchioInterface& pinocchio, const PinocchioStateInputMapping<scalar_t>& mapping,
    size_t stateDim, ArmBodyCollisionConfig config)
    : StateConstraint(ConstraintOrder::Linear),
      pinocchio_(pinocchio), mapping_(mapping.clone()), stateDim_(stateDim), config_(std::move(config)) {
  if (!std::isfinite(config_.minimumDistance) || config_.minimumDistance < 0.0) {
    throw std::invalid_argument("[ArmBodyCollision] minimumDistance must be finite and nonnegative");
  }
  requirePositive(config_.mu, "mu");
  requirePositive(config_.delta, "delta");
  if (stateDim_ == 0 || config_.arms.empty() || config_.bodies.empty()) {
    throw std::invalid_argument("[ArmBodyCollision] nonempty state, arms, and bodies are required");
  }
  // The current WB-ID mapping uses Translation + SphericalZYX and nq == nv.
  // A future manifold model must supply its own coordinate-to-tangent Jacobian mapping.
  if (pinocchio_.getModel().nq != pinocchio_.getModel().nv) {
    throw std::invalid_argument("[ArmBodyCollision] current draft requires nq == nv");
  }
  std::unordered_set<std::string> names;
  for (size_t i = 0; i < config_.arms.size() + config_.bodies.size(); ++i) {
    const auto& entry = sphere(i);
    requirePositive(entry.radius, "sphere radius");
    if (entry.name.empty() || !names.insert(entry.name).second || !entry.center.allFinite()) {
      throw std::invalid_argument("[ArmBodyCollision] sphere names must be unique and centers finite");
    }
    const auto frameId = pinocchio_.getModel().getFrameId(entry.linkName);
    if (frameId >= static_cast<pinocchio::FrameIndex>(pinocchio_.getModel().nframes)) {
      throw std::invalid_argument("[ArmBodyCollision] missing frame: " + entry.linkName);
    }
    frameIds_.push_back(frameId);
  }
  mapping_->setPinocchioInterface(pinocchio_);
}

ArmBodyCollisionConstraint::ArmBodyCollisionConstraint(const ArmBodyCollisionConstraint& other)
    : StateConstraint(other), pinocchio_(other.pinocchio_), mapping_(other.mapping_->clone()),
      stateDim_(other.stateDim_), config_(other.config_), frameIds_(other.frameIds_) {
  mapping_->setPinocchioInterface(pinocchio_);
}

ArmBodyCollisionConstraint* ArmBodyCollisionConstraint::clone() const {
  return new ArmBodyCollisionConstraint(*this);
}

size_t ArmBodyCollisionConstraint::getNumConstraints(scalar_t /*time*/) const {
  return config_.arms.size() * config_.bodies.size();
}

const ArmBodyCollisionSphere& ArmBodyCollisionConstraint::sphere(size_t index) const {
  return index < config_.arms.size() ? config_.arms.at(index) : config_.bodies.at(index - config_.arms.size());
}

void ArmBodyCollisionConstraint::updateKinematics(const vector_t& state, bool needJacobians) const {
  if (state.size() != static_cast<Eigen::Index>(stateDim_) || !state.allFinite()) {
    throw std::invalid_argument("[ArmBodyCollision] invalid state dimensions or nonfinite state");
  }
  const auto& model = pinocchio_.getModel();
  auto& data = pinocchio_.getData();
  const vector_t q = mapping_->getPinocchioJointPosition(state);
  if (q.size() != model.nq) {
    throw std::invalid_argument("[ArmBodyCollision] mapping returned the wrong configuration dimension");
  }
  if (needJacobians) {
    pinocchio::computeJointJacobians(model, data, q);
  } else {
    pinocchio::forwardKinematics(model, data, q);
  }
  pinocchio::updateFramePlacements(model, data);
}

Eigen::Vector3d ArmBodyCollisionConstraint::centerInWorld(size_t index) const {
  return pinocchio_.getData().oMf[frameIds_.at(index)].act(sphere(index).center);
}

matrix_t ArmBodyCollisionConstraint::centerJacobianInState(size_t index, const vector_t& state) const {
  const auto& model = pinocchio_.getModel();
  auto& data = pinocchio_.getData();
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> frameJacobian(6, model.nv);
  frameJacobian.setZero();
  pinocchio::getFrameJacobian(model, data, frameIds_.at(index), pinocchio::LOCAL_WORLD_ALIGNED, frameJacobian);
  const Eigen::Vector3d offset = data.oMf[frameIds_.at(index)].rotation() * sphere(index).center;
  matrix_t Jq = frameJacobian.topRows(3);
  for (Eigen::Index col = 0; col < model.nv; ++col) {
    Jq.col(col) += frameJacobian.bottomRows<3>().col(col).cross(offset);
  }
  return mapping_->getOcs2Jacobian(state, Jq, matrix_t::Zero(3, model.nv)).first;
}

std::vector<Eigen::Vector3d> ArmBodyCollisionConstraint::getWorldCenters(const vector_t& state) const {
  updateKinematics(state, false);
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(frameIds_.size());
  for (size_t i = 0; i < frameIds_.size(); ++i) {
    centers.push_back(centerInWorld(i));
  }
  return centers;
}

vector_t ArmBodyCollisionConstraint::getValue(
    scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  const auto centers = getWorldCenters(state);
  vector_t result(getNumConstraints(0.0));
  size_t row = 0;
  for (size_t arm = 0; arm < config_.arms.size(); ++arm) {
    for (size_t body = 0; body < config_.bodies.size(); ++body) {
      const auto delta = (centers[arm] - centers[config_.arms.size() + body]).eval();
      // Conservative regularization: never overestimates actual center distance.
      const scalar_t distance = std::sqrt(delta.squaredNorm() + kDistanceEpsilon * kDistanceEpsilon) - kDistanceEpsilon;
      result(row++) = distance - config_.arms[arm].radius - config_.bodies[body].radius - config_.minimumDistance;
    }
  }
  return result;
}

VectorFunctionLinearApproximation ArmBodyCollisionConstraint::getLinearApproximation(
    scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  std::vector<Eigen::Index> rows(getNumConstraints(0.0));
  std::iota(rows.begin(), rows.end(), 0);
  return getLinearApproximationForRows(state, rows);
}

VectorFunctionLinearApproximation ArmBodyCollisionConstraint::getLinearApproximationForRows(
    const vector_t& state, const std::vector<Eigen::Index>& rows) const {
  updateKinematics(state, true);
  std::vector<Eigen::Vector3d> centers(frameIds_.size());
  std::vector<matrix_t> jacobians(frameIds_.size());
  std::vector<bool> ready(frameIds_.size(), false);
  const auto prepare = [&](size_t i) {
    if (!ready[i]) {
      centers[i] = centerInWorld(i);
      jacobians[i] = centerJacobianInState(i, state);
      ready[i] = true;
    }
  };
  auto result = VectorFunctionLinearApproximation::Zero(rows.size(), stateDim_, 0);
  for (size_t row = 0; row < rows.size(); ++row) {
    if (rows[row] < 0 || rows[row] >= static_cast<Eigen::Index>(getNumConstraints(0.0))) {
      throw std::invalid_argument("[ArmBodyCollision] requested linearization row is out of range");
    }
    const size_t arm = static_cast<size_t>(rows[row]) / config_.bodies.size();
    const size_t body = static_cast<size_t>(rows[row]) % config_.bodies.size();
    const size_t j = config_.arms.size() + body;
    prepare(arm);
    prepare(j);
    const Eigen::Vector3d delta = centers[arm] - centers[j];
    const scalar_t smoothNorm = std::sqrt(delta.squaredNorm() + kDistanceEpsilon * kDistanceEpsilon);
    result.f(row) = smoothNorm - kDistanceEpsilon - config_.arms[arm].radius -
                    config_.bodies[body].radius - config_.minimumDistance;
    result.dfdx.row(row) = (delta.transpose() / smoothNorm) * (jacobians[arm] - jacobians[j]);
  }
  return result;
}

ArmBodyCollisionSoftConstraint::ArmBodyCollisionSoftConstraint(
    const PinocchioInterface& pinocchio, const PinocchioStateInputMapping<scalar_t>& mapping,
    size_t stateDim, const ArmBodyCollisionConfig& config)
    : constraint_(pinocchio, mapping, stateDim, config), penalty_(makePenalty(config)),
      polynomial_(config.implementation == "polynomial"), delta_(config.delta) {}

scalar_t ArmBodyCollisionSoftConstraint::getValue(
    scalar_t time, const vector_t& state, const TargetTrajectories& /*targets*/, const PreComputation& preComp) const {
  return penalty_.getValue(time, constraint_.getValue(time, state, preComp));
}

ScalarFunctionQuadraticApproximation ArmBodyCollisionSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& /*targets*/, const PreComputation& preComp) const {
  if (!polynomial_) {
    return penalty_.getQuadraticApproximation(time, constraint_.getLinearApproximation(time, state, preComp));
  }
  const vector_t values = constraint_.getValue(time, state, preComp);
  std::vector<Eigen::Index> active;
  active.reserve(values.size());
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (values(i) < delta_) active.push_back(i);
  }
  // Outside the polynomial buffer, value, gradient and curvature are exactly zero.
  if (active.empty()) return ScalarFunctionQuadraticApproximation::Zero(state.size(), 0);
  const auto selected = constraint_.getLinearApproximationForRows(state, active);
  return penalty_.getQuadraticApproximation(time, selected);
}

}  // namespace ocs2::humanoid
