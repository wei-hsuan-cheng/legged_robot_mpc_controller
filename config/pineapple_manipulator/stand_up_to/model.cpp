// Pineapple stand-up model: Pinocchio RNEA, frame placements and CoM as CasADi functions, for plan.py.
// Scalar conversion follows Pinocchio's official example:
// https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/casadi/casadi-rnea.cpp
// Accepts either Pineapple variant. If present, the arm is locked at zero as in the MPC.
// Usage: pineapple_manipulator_stand_up_model ROBOT.urdf PREFIX (writes CasADi RNEA and frame functions).
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <casadi/casadi.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  using SX = casadi::SX;
  pinocchio::Model full;
  pinocchio::urdf::buildModel(argv[1], pinocchio::JointModelFreeFlyer(), full);
  std::vector<pinocchio::JointIndex> locked;
  for (const char *name : {"arm_joint", "arm_base_joint", "upper_arm_joint", "fore_arm_joint", "5dof_joint",
                           "gripper_case_joint"}) {
    if (full.existJointName(name)) locked.push_back(full.getJointId(name));
  }
  const pinocchio::Model model = pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));
  if (model.nv != 14 || model.nq != 15) throw std::runtime_error("expected the eight-joint Pineapple leg model");
  pinocchio::Model armed = model;
  // MJCF joint armature (pinappleV3 default) on the leg and wheel joints; regenerate after changing it.
  armed.armature.tail(8).setConstant(.004);
  for (int i = 0; i < armed.njoints; ++i) std::cout << i << " " << armed.names[i] << "\n";
  auto am = armed.cast<SX>();
  pinocchio::DataTpl<SX> data(am);
  SX q = SX::sym("q", armed.nq), v = SX::sym("v", armed.nv), a = SX::sym("a", armed.nv);
  Eigen::Matrix<SX, Eigen::Dynamic, 1> eq(armed.nq), ev(armed.nv), ea(armed.nv);
  for (int i = 0; i < armed.nq; ++i) eq[i] = q(i);
  for (int i = 0; i < armed.nv; ++i) { ev[i] = v(i); ea[i] = a(i); }
  pinocchio::rnea(am, data, eq, ev, ea);
  SX tau = SX::zeros(armed.nv);
  for (int i = 0; i < armed.nv; ++i) tau(i) = data.tau[i];
  casadi::Function("rnea", {q, v, a}, {tau}).save(std::string(argv[2]) + "_rnea.casadi");
  pinocchio::forwardKinematics(am, data, eq);
  pinocchio::updateFramePlacements(am, data);
  pinocchio::centerOfMass(am, data, eq);
  // Frame origins and rotations only: plan.py reads tire/seat offsets from geometry.yaml
  // and ground-clearance box corners from the selected MJCF.
  std::vector<std::string> names = {"wheel_l", "wheel_r", "base_link", "thigh_l", "thigh_r", "calf_l", "calf_r"};
  if (!locked.empty()) {
    names.push_back("upper_arm");
    names.push_back("fore_arm");
  }
  const int count = names.size();
  SX fk = SX::zeros(count * 12 + 3);
  for (int j = 0; j < count; ++j) {
    const auto &f = data.oMf[armed.getFrameId(names[j])];
    for (int i = 0; i < 3; ++i) fk(12 * j + i) = f.translation()[i];
    for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r) fk(12 * j + 3 + 3 * c + r) = f.rotation()(r, c);
  }
  for (int i = 0; i < 3; ++i) fk(12 * count + i) = data.com[0][i];
  casadi::Function("frames", {q}, {fk}).save(std::string(argv[2]) + "_frames.casadi");
}
