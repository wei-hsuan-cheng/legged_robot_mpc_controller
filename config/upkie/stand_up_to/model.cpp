// Scalar conversion follows Pinocchio's official example:
// https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/casadi/casadi-rnea.cpp
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <casadi/casadi.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if(argc!=3) return 2;
  using SX=casadi::SX;
  pinocchio::Model model;
  pinocchio::urdf::buildModel(argv[1],pinocchio::JointModelFreeFlyer(),model);
  if(model.nv!=12 || model.nq!=13) throw std::runtime_error("expected the six-joint Upkie model");
  // Match Upkie's MJCF and WB-ID model armature; regenerate after changing it.
  model.armature.tail(6).setConstant(.005);
  for(int i=0;i<model.njoints;++i) std::cout<<i<<" "<<model.names[i]<<"\n";
  auto am=model.cast<SX>();
  pinocchio::DataTpl<SX> data(am);
  SX q=SX::sym("q",model.nq),v=SX::sym("v",model.nv),a=SX::sym("a",model.nv);
  Eigen::Matrix<SX,Eigen::Dynamic,1> eq(model.nq),ev(model.nv),ea(model.nv);
  for(int i=0;i<model.nq;++i) eq[i]=q(i);
  for(int i=0;i<model.nv;++i) {ev[i]=v(i);ea[i]=a(i);}
  pinocchio::rnea(am,data,eq,ev,ea);
  SX tau=SX::zeros(model.nv);
  for(int i=0;i<model.nv;++i)tau(i)=data.tau[i];
  casadi::Function("rnea",{q,v,a},{tau}).save(std::string(argv[2])+"_rnea.casadi");
  pinocchio::forwardKinematics(am,data,eq);
  pinocchio::updateFramePlacements(am,data);
  pinocchio::centerOfMass(am,data,eq);
  // Frame origins and rotations only: plan.py places the tire and knee-housing centres in these frames from
  // config/upkie/geometry.yaml, so the model carries no contact geometry.
  SX fk=SX::zeros(4*12+3);
  const char* names[]={"left_wheel_link","right_wheel_link","left_femur","right_femur"};
  for(int j=0;j<4;++j){
    const auto &f=data.oMf[model.getFrameId(names[j])];
    for(int i=0;i<3;++i) fk(12*j+i)=f.translation()[i];
    for(int c=0;c<3;++c) for(int r=0;r<3;++r)fk(12*j+3+3*c+r)=f.rotation()(r,c);
  }
  for(int i=0;i<3;++i) fk(48+i)=data.com[0][i];
  casadi::Function("frames",{q},{fk}).save(std::string(argv[2])+"_frames.casadi");
}
