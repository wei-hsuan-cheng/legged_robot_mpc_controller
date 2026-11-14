#include <pinocchio/fwd.hpp>

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"
#include "humanoid_wb_id_mpc/common/WheelContactKinematics.h"
#include "humanoid_wb_id_mpc/constraint/WbIdRneaConstraintCppAd.h"
#include "humanoid_wb_id_mpc/constraint/WbIdWheelContactConstraintCppAd.h"
#include "humanoid_wb_id_mpc/initialization/WbIdInitializer.h"

using namespace ocs2;
using namespace ocs2::humanoid;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    require(argc == 3, "usage: wheel_model_consistency_test UPKIE_URDF CODEGEN_ROOT");
    ModelSettings::Params p;
    p.robotName = "upkie";
    p.modelFolderCppAd = argv[2];
    p.recompileLibrariesCppAd = false;
    p.verboseCppAd = false;
    p.contactNames6DoF = {"left_tire_center", "right_tire_center"};
    p.contactParentJointNames = {"left_wheel", "right_wheel"};
    p.contactFrameTranslation = vector3_t(0, 0, -.0312);
    p.contactRectangleXMin = p.contactRectangleYMin = -.01;
    p.contactRectangleXMax = p.contactRectangleYMax = .01;
    p.jointArmature.assign(6, .005);
    p.jointFriction.assign(6, .1);
    ModelSettings settings(p, argv[1], "wb_id_mpc_");
    auto interface = createCustomPinocchioInterface(argv[1], settings, false, 0, false);
    WbIdMpcRobotModel<double> robot(settings);
    WbIdMpcRobotModel<ad_scalar_t> robotAD(settings);
    const ModeSchedule schedule({-1., 2.}, {STANCE, STANCE, STANCE});
    auto gait = std::make_shared<GaitSchedule>(schedule, ModeSequenceTemplate({0., .5}, {STANCE}), 0.);
    auto swing = std::make_shared<SwingTrajectoryPlanner>(SwingTrajectoryPlanner::Config{}, 2);
    SwitchedModelReferenceManager reference(gait, swing, interface, robot);
    vector_t x = vector_t::Zero(24), u = vector_t::Zero(30);
    x(2) = .3;
    x.segment<6>(6) << -.6021396734839873, -.9655142016056263, 0, .6021396734839873, .9655142016056263, 0;
    reference.setTargetTrajectories(TargetTrajectories({0., 1.}, {x, x}, {u, u}));
    reference.setModeSchedule(schedule);
    reference.preSolverRun(0., 1., x);
    auto window = std::make_shared<WbIdTorqueWindow>(-1.);
    WbIdRneaConstraintCppAd base(WbIdRneaConstraintCppAd::Rows::Base, interface, robotAD, settings);
    WbIdRneaConstraintCppAd joint(WbIdRneaConstraintCppAd::Rows::Joint, interface, robotAD, settings, window);
    using Contact = WbIdWheelContactConstraintCppAd;
    Contact::Settings gains;
    Contact left(Contact::Type::Rolling, reference, interface, robotAD, 0, gains, settings);
    Contact right(Contact::Type::Rolling, reference, interface, robotAD, 1, gains, settings);
    Contact lateral(Contact::Type::Lateral, reference, interface, robotAD, 0, gains, settings);
    WbIdInitializer initializer(interface, reference, robot, gains.radius, gains.axleAxis);
    const auto& model = interface.getModel();
    const auto frame = [&](size_t i) { return model.getFrameId(settings.contactNames.at(i)); };
    PreComputation pre;
    std::mt19937 rng(731);
    std::uniform_real_distribution<double> random(-.4, .4);
    const vector_t nominal = x;
    double worstValue = 0, worstJacobian = 0, minimumSingular = 1e100;
    struct Wheel { vector3_t point, rho, slip, longitudinal, lateral; };
    const auto wheelAt = [&](const vector_t& q, const vector_t& v, size_t i) {
      auto data = interface.getData();
      pinocchio::forwardKinematics(model, data, q, v);
      const auto& placement = pinocchio::updateFramePlacement(model, data, frame(i));
      const auto velocity = pinocchio::getFrameVelocity(model, data, frame(i), pinocchio::LOCAL_WORLD_ALIGNED);
      const vector3_t axle = placement.rotation() * vector3_t::UnitZ();
      Wheel k;
      k.rho = wheel::contactOffset<double>(axle, vector3_t::UnitZ(), .055);
      k.point = placement.translation() + k.rho;
      k.slip = velocity.linear() + velocity.angular().cross(k.rho);
      k.lateral = wheel::lateralDirection<double>(axle, vector3_t::UnitZ());
      k.longitudinal = vector3_t::UnitZ().cross(k.lateral);
      return k;
    };
    for (int sample = 0; sample < 12; ++sample) {
      x = nominal;
      if (sample > 0) {
        for (auto& value : x) value += random(rng);
        x(2) = .3 + .1 * random(rng);
        x(4) *= .5;
        x(5) *= .5;
      }
      for (auto& value : u) value = random(rng);
      u(2) = u(8) = 26.8;
      const vector_t q = x.head(12), v = x.tail(12), a = u.segment(12, 12);
      auto data = interface.getData();
      vector_t expected = pinocchio::rnea(model, data, q, v, a);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      for (size_t i = 0; i < 2; ++i) {
        matrix_t jacobian = matrix_t::Zero(6, 12);
        pinocchio::getFrameJacobian(model, data, frame(i), pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
        expected -= jacobian.transpose() * robot.getContactWrench(u, i);
        const auto k = wheelAt(q, v, i);
        const vector3_t force(.3, -.2, 26.8);
        const vector_t direct = jacobian.topRows(3).transpose() * force +
                                jacobian.bottomRows(3).transpose() * k.rho.cross(force);
        const vector_t pointForce = (jacobian.topRows(3) -
            pinocchio::skew(k.rho) * jacobian.bottomRows(3)).transpose() * force;
        require((direct - pointForce).norm() < 1e-12, "tire-center wrench violates virtual-work identity");
      }
      expected.tail(6) -= u.tail(6);
      for (int j = 0; j < 6; ++j) expected(6 + j) += .1 * std::tanh(v(6 + j) / .05);
      worstValue = std::max(worstValue, (base.getValue(.3, x, u, pre) - expected.head(6)).norm());
      worstValue = std::max(worstValue, (joint.getValue(.3, x, u, pre) - expected.tail(6)).norm());
      const double dt = 1e-6;
      const vector_t qPlus = q + dt*v + .5*dt*dt*a, qMinus = q - dt*v + .5*dt*dt*a;
      const vector_t vPlus = v + dt*a, vMinus = v - dt*a;
      double meanLateral = 0;
      const auto refPlus = wheelAt(qPlus, vPlus, 0), refMinus = wheelAt(qMinus, vMinus, 0);
      const auto refNow = wheelAt(q, v, 0);
      for (size_t i = 0; i < 2; ++i) {
        const auto now = wheelAt(q, v, i), plus = wheelAt(qPlus, vPlus, i), minus = wheelAt(qMinus, vMinus, i);
        vector_t rows(5);
        rows(0) = (plus.longitudinal.dot(plus.slip) - minus.longitudinal.dot(minus.slip)) / (2*dt)
                  + 50*now.longitudinal.dot(now.slip);
        rows(1) = (plus.slip.z() - minus.slip.z()) / (2*dt) + 50*now.slip.z() + 50*now.point.z();
        rows.tail(3) = robot.getContactMoment(u, i) - now.rho.cross(robot.getContactForce(u, i));
        const auto& constraint = i == 0 ? left : right;
        worstValue = std::max(worstValue, (constraint.getValue(.3, x, u, pre) - rows).norm());
        meanLateral += .5*((refPlus.lateral.dot(plus.slip) - refMinus.lateral.dot(minus.slip))/(2*dt)
                           + 50*refNow.lateral.dot(now.slip));
      }
      worstValue = std::max(worstValue, std::abs(lateral.getValue(.3, x, u, pre)(0) - meanLateral));
      matrix_t inputRows(23, 30);
      Eigen::Index offset = 0;
      const std::array<StateInputConstraintCppAd*, 5> constraints{&base, &joint, &left, &right, &lateral};
      for (auto* constraint : constraints) {
        const auto linear = constraint->getLinearApproximation(.3, x, u, pre);
        inputRows.middleRows(offset, linear.f.size()) = linear.dfdu;
        offset += linear.f.size();
        for (Eigen::Index column = 0; column < x.size() + u.size(); ++column) {
          auto xp = x.eval(), xm = x.eval(), up = u.eval(), um = u.eval();
          if (column < x.size()) { xp(column) += dt; xm(column) -= dt; }
          else { up(column-x.size()) += dt; um(column-x.size()) -= dt; }
          const vector_t difference = (constraint->getValue(.3, xp, up, pre) - constraint->getValue(.3, xm, um, pre))/(2*dt);
          const vector_t derivative = column < x.size() ? vector_t(linear.dfdx.col(column)) : vector_t(linear.dfdu.col(column-x.size()));
          worstJacobian = std::max(worstJacobian, (difference-derivative).norm()/(1+derivative.norm()));
        }
      }
      Eigen::JacobiSVD<matrix_t> svd(inputRows);
      minimumSingular = std::min(minimumSingular, svd.singularValues().tail(1)(0));
      const vector_t seed = initializer.weightCompensatingInput(x, {true, true}, .3);
      require(joint.getValue(.3, x, seed, pre).norm() < 1e-9, "initializer violates joint RNEA/friction rows");
      require(left.getValue(.3, x, seed, pre).tail(3).norm() < 1e-12, "initializer violates left tire moment rows");
      require(right.getValue(.3, x, seed, pre).tail(3).norm() < 1e-12, "initializer violates right tire moment rows");
    }
    std::cout << "max equation error=" << worstValue << ", max normalized AD Jacobian error=" << worstJacobian
              << ", minimum input-Jacobian singular value=" << minimumSingular << '\n';
    require(worstValue < 2e-7, "generated model disagrees with direct equations");
    require(worstJacobian < 2e-7, "generated Jacobian disagrees with finite differences");
    require(minimumSingular > 1e-6, "double-support input equality Jacobian is rank deficient");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
