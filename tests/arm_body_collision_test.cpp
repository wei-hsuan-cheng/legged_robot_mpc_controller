#include <pinocchio/fwd.hpp>

#include <cmath>
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-translation.hpp>
#include <pinocchio/multibody/joint/joint-spherical-ZYX.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>

#include "humanoid_wb_id_mpc/common/WbIdPinocchioMapping.h"
#include "humanoid_wb_id_mpc/constraint/ArmBodyCollisionConstraint.h"

using namespace ocs2;
using namespace ocs2::humanoid;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

template <typename Function>
void requireInvalid(Function function) {
  try {
    function();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error("invalid configuration was accepted");
}

int main(int argc, char** argv) {
  try {
    pinocchio::Model model;
    const auto translation = model.addJoint(0, pinocchio::JointModelTranslation(), pinocchio::SE3::Identity(), "base_xyz");
    const auto attitude = model.addJoint(translation, pinocchio::JointModelSphericalZYX(), pinocchio::SE3::Identity(), "base_zyx");
    model.addFrame(pinocchio::Frame("torso", attitude, 0, pinocchio::SE3::Identity(), pinocchio::BODY));
    const auto arm = model.addJoint(attitude, pinocchio::JointModelRY(),
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.02, 0.3, 0.1)), "arm");
    model.addFrame(pinocchio::Frame("hand", arm, 0,
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.03, 0.02, -0.2)), pinocchio::BODY));
    const PinocchioInterface interface(model);
    const WbIdPinocchioMapping mapping(model.nq, 1);
    ArmBodyCollisionConfig config;
    config.activate = true;
    config.arms = {{"hand_sphere", "hand", Eigen::Vector3d(0.02, 0.01, -0.12), 0.05}};
    config.bodies = {{"chest", "torso", Eigen::Vector3d(0.0, 0.0, -0.1), 0.10},
                     {"pelvis", "torso", Eigen::Vector3d(0.01, 0.30, -0.3), 0.09}};
    ArmBodyCollisionConstraint constraint(interface, mapping, 2 * model.nq, config);
    require(constraint.getNumConstraints(0.0) == 2, "wrong arm x body pair count");
    require(constraint.isActive(0.0) && constraint.isActive(99.0), "must remain active in all gait phases");
    PreComputation preComp;
    TargetTrajectories targets(0);
    ArmBodyCollisionSoftConstraint compact(interface, mapping, 2 * model.nq, config);
    StateSoftConstraint reference(std::unique_ptr<StateConstraint>(constraint.clone()),
        std::make_unique<PieceWisePolynomialBarrierPenalty>(PieceWisePolynomialBarrierPenalty::Config{config.mu, config.delta}));
    auto rbfConfig = config;
    rbfConfig.implementation = "rbf";
    rbfConfig.mu = 0.01;
    ArmBodyCollisionSoftConstraint compactRbf(interface, mapping, 2 * model.nq, rbfConfig);
    StateSoftConstraint referenceRbf(std::unique_ptr<StateConstraint>(constraint.clone()),
        std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{rbfConfig.mu, rbfConfig.delta}));
    size_t activePolynomialSamples = 0;
    size_t inactivePolynomialSamples = 0;
    std::mt19937 generator(1029);
    std::uniform_real_distribution<double> distribution(-0.8, 0.8);
    scalar_t worstError = 0.0;
    for (size_t sample = 0; sample < 100; ++sample) {
      vector_t state(2 * model.nq);
      for (auto& value : state) value = distribution(generator);
      const auto approximation = constraint.getLinearApproximation(0.0, state, preComp);
      const auto reversed = constraint.getLinearApproximationForRows(state, {1, 0});
      require(reversed.f(0) == approximation.f(1) && reversed.f(1) == approximation.f(0), "selected row ordering changed values");
      require(reversed.dfdx.row(0).isApprox(approximation.dfdx.row(1), 1e-12), "selected row Jacobian is wrong");
      ((approximation.f.array() < config.delta).any() ? activePolynomialSamples : inactivePolynomialSamples)++;
      for (const auto& costs : {std::make_pair(static_cast<const StateCost*>(&compact), static_cast<const StateCost*>(&reference)),
                                std::make_pair(static_cast<const StateCost*>(&compactRbf), static_cast<const StateCost*>(&referenceRbf))}) {
        const auto optimized = costs.first->getQuadraticApproximation(0.0, state, targets, preComp);
        const auto original = costs.second->getQuadraticApproximation(0.0, state, targets, preComp);
        require(std::abs(optimized.f - original.f) < 1e-10 * (1 + std::abs(original.f)), "cost value changed by row filtering");
        require(optimized.dfdx.isApprox(original.dfdx, 1e-10), "cost gradient changed by row filtering");
        require(optimized.dfdxx.isApprox(original.dfdxx, 1e-10), "Gauss-Newton Hessian changed by row filtering");
        require(std::abs(costs.first->getValue(0.0, state, targets, preComp) - optimized.f) < 1e-10 * (1 + std::abs(optimized.f)),
                "soft cost value and approximation disagree");
      }
      require(approximation.f.isApprox(constraint.getValue(0.0, state, preComp), 1e-12), "value/linearization disagree");
      require(approximation.dfdx.rightCols(model.nq).norm() == 0.0, "collision depends on velocity");
      require(approximation.dfdx.leftCols(6).norm() < 1e-11, "self-clearance must be invariant to global base pose");
      matrix_t finiteDifference(2, state.size());
      for (Eigen::Index col = 0; col < state.size(); ++col) {
        auto plus = state.eval();
        auto minus = state.eval();
        plus(col) += 1e-6;
        minus(col) -= 1e-6;
        finiteDifference.col(col) = (constraint.getValue(0.0, plus, preComp) - constraint.getValue(0.0, minus, preComp)) / 2e-6;
      }
      worstError = std::max(worstError, (finiteDifference - approximation.dfdx).cwiseAbs().maxCoeff());
      require(worstError < 1e-7, "analytic sphere-offset Jacobian failed finite-difference verification");
      auto clone = std::unique_ptr<ArmBodyCollisionConstraint>(constraint.clone());
      constraint.getValue(0.0, vector_t::Zero(state.size()), preComp);
      require(clone->getValue(0.0, state, preComp).isApprox(approximation.f, 1e-12), "worker clone shares mutable kinematic data");
    }
    require(activePolynomialSamples > 0 && inactivePolynomialSamples > 0, "row-filtering test missed active or inactive cases");
    auto coincident = config;
    coincident.arms.front().linkName = "torso";
    coincident.arms.front().center = coincident.bodies.front().center;
    ArmBodyCollisionConstraint overlapping(interface, mapping, 2 * model.nq, coincident);
    const auto atOverlap = overlapping.getLinearApproximation(0.0, vector_t::Zero(2 * model.nq), preComp);
    require(atOverlap.f.allFinite() && atOverlap.dfdx.allFinite() && atOverlap.f(0) < 0, "coincident centers produced nonfinite values");
    auto disabled = config;
    disabled.activate = false;
    require(!ArmBodyCollisionConstraint(interface, mapping, 2 * model.nq, disabled).isActive(0.0), "activation switch ignored");
    requireInvalid([&] { auto bad = config; bad.arms[0].radius = -1; ArmBodyCollisionConstraint c(interface, mapping, 2 * model.nq, bad); });
    requireInvalid([&] { auto bad = config; bad.arms[0].linkName = "missing"; ArmBodyCollisionConstraint c(interface, mapping, 2 * model.nq, bad); });
    requireInvalid([&] { auto bad = config; bad.arms[0].name = "chest"; ArmBodyCollisionConstraint c(interface, mapping, 2 * model.nq, bad); });
    requireInvalid([&] { auto bad = config; bad.bodies.clear(); ArmBodyCollisionConstraint c(interface, mapping, 2 * model.nq, bad); });
    requireInvalid([&] { constraint.getValue(0.0, vector_t::Zero(1), preComp); });
    requireInvalid([&] { constraint.getLinearApproximationForRows(vector_t::Zero(2 * model.nq), {-1}); });
    RelaxedBarrierPenalty rbf({0.01, 0.02});
    PieceWisePolynomialBarrierPenalty polynomial({1000.0, 0.02});
    require(std::abs(rbf.getDerivative(0.0, 0.0) + 1.0) < 1e-12, "RBF boundary slope mismatch");
    require(std::abs(rbf.getSecondDerivative(0.0, 0.0) - 25.0) < 1e-12, "RBF boundary curvature mismatch");
    require(polynomial.getValue(0.0, 0.03) == 0.0, "polynomial should be inactive beyond its buffer");
    require(rbf.getDerivative(0.0, 0.10) < 0.0, "RBF must retain its feasible-region gradient");
    std::cout << "PASS: 100 randomized analytic-Jacobian checks, base-pose invariance, clones, overlap, activation, validation, penalties.\n"
              << "Polynomial/RBF cost, gradient and Hessian match StateSoftConstraint; active/inactive cases: "
              << activePolynomialSamples << '/' << inactivePolynomialSamples << '\n'
              << "Worst Jacobian absolute error: " << worstError << '\n';
    if (argc > 1 && std::string(argv[1]) == "--benchmark") {
      // H2-sized state and pair counts; synthetic kinematics, not an end-to-end MPC benchmark.
      auto benchModel = model;
      for (size_t i = 0; i < 24; ++i) {
        benchModel.addJoint(attitude, pinocchio::JointModelRY(), pinocchio::SE3::Identity(), "aux_" + std::to_string(i));
      }
      const PinocchioInterface benchInterface(benchModel);
      const WbIdPinocchioMapping benchMapping(benchModel.nq, 1);
      const vector_t state = vector_t::Zero(2 * benchModel.nq);
      scalar_t sink = 0.0;
      for (const bool near : {false, true}) {
        auto benchConfig = config;
        benchConfig.arms.clear();
        benchConfig.bodies.clear();
        for (size_t i = 0; i < 16; ++i) {
          benchConfig.arms.push_back({"arm_" + std::to_string(i), "hand", Eigen::Vector3d(0.02 + .003 * i, .01, -.12), .05});
        }
        for (size_t i = 0; i < 8; ++i) {
          benchConfig.bodies.push_back({"body_" + std::to_string(i), "torso", Eigen::Vector3d(0, 0, -.05 * i), .10});
        }
        if (near) benchConfig.bodies.front().center = Eigen::Vector3d(.08, .30, -.22);
        ArmBodyCollisionSoftConstraint selected(benchInterface, benchMapping, state.size(), benchConfig);
        StateSoftConstraint generic(
            std::make_unique<ArmBodyCollisionConstraint>(benchInterface, benchMapping, state.size(), benchConfig),
            std::make_unique<PieceWisePolynomialBarrierPenalty>(PieceWisePolynomialBarrierPenalty::Config{benchConfig.mu, benchConfig.delta}));
        const auto expected = generic.getQuadraticApproximation(0.0, state, targets, preComp);
        const auto actual = selected.getQuadraticApproximation(0.0, state, targets, preComp);
        require(actual.dfdxx.isApprox(expected.dfdxx, 1e-10), "representative-size benchmark Hessian differs");
        const auto measure = [&](const StateCost& cost, size_t count) {
          const auto begin = std::chrono::steady_clock::now();
          for (size_t i = 0; i < count; ++i) {
            const auto result = cost.getQuadraticApproximation(0.0, state, targets, preComp);
            sink += result.f + result.dfdx.squaredNorm();
          }
          return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count();
        };
        measure(generic, 50);
        measure(selected, 50);
        double genericUs = 0.0, selectedUs = 0.0;
        for (size_t block = 0; block < 6; ++block) {
          if (block % 2 == 0) {
            genericUs += measure(generic, 200);
            selectedUs += measure(selected, 200);
          } else {
            selectedUs += measure(selected, 200);
            genericUs += measure(generic, 200);
          }
        }
        std::cout << "NODE_BENCH state_dim=" << state.size() << " pairs=128 near=" << near
                  << " generic_us=" << genericUs / 1200 << " selected_us=" << selectedUs / 1200 << '\n';
      }
      require(std::isfinite(sink), "benchmark produced nonfinite cost");
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
