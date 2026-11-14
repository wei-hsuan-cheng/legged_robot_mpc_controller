#pragma once

#include <Eigen/Core>

namespace ocs2::humanoid::wheel {

/*
  Rolling contact of a rigid thin disk on a plane, shared by the MPC constraint (CppAD scalar)
  and the controller's measured slip/gap logging (double).

  A tire of radius r has its centre at p_h, a unit axle e (world) and the plane a unit normal n.
  The lowest point of the disk is p_c = p_h + rho with

    rho = -r (n - (e.n) e) / |n - (e.n) e|.

  rho depends on the configuration through e only; with the wheel's full angular velocity omega
  (spin included) e moves as e_dot = omega x e, and the spin component drops out.

  Velocities at the contact:
    v_c = v_h + omega x rho        material velocity of the tread point now at p_c (slip)
    a_c = a_h + alpha x rho + omega x rho_dot   its time derivative, d/dt v_c(q, v)

  v_c is zero when the tire rolls without slipping, while d/dt p_c = v_h + rho_dot is the
  geometric contact point moving along the ground. Because p_c minimises the height over the
  disk, the gap rate is n . v_c (envelope theorem), so n . a_c is the gap acceleration.

  With a wheel-centre wrench [f; m] in the same world-aligned basis as the centre Jacobian,
  m = rho x f makes J_h^T [f; m] = J_c^T f with J_c = J_v - [rho]x J_omega: the force acting at
  the tire contact.
*/

template <typename SCALAR>
using Vector3 = Eigen::Matrix<SCALAR, 3, 1>;

/** w = n - (e.n) e, the in-plane direction of the disk towards the plane (unnormalized). */
template <typename SCALAR>
Vector3<SCALAR> downInDisk(const Vector3<SCALAR>& axle, const Vector3<SCALAR>& normal) {
  return normal - axle.dot(normal) * axle;
}

/** rho, from the tire centre to the lowest point of the disk. Needs the axle not parallel to n. */
template <typename SCALAR>
Vector3<SCALAR> contactOffset(const Vector3<SCALAR>& axle, const Vector3<SCALAR>& normal, SCALAR radius) {
  const Vector3<SCALAR> w = downInDisk(axle, normal);
  return -radius * w / w.norm();
}

/** d rho / dt for axleRate = omega x axle. */
template <typename SCALAR>
Vector3<SCALAR> contactOffsetRate(const Vector3<SCALAR>& axle,
                                  const Vector3<SCALAR>& axleRate,
                                  const Vector3<SCALAR>& normal,
                                  SCALAR radius) {
  const Vector3<SCALAR> w = downInDisk(axle, normal);
  const Vector3<SCALAR> wRate = -axleRate.dot(normal) * axle - axle.dot(normal) * axleRate;
  const SCALAR norm = w.norm();
  return -radius * (wRate / norm - w * (w.dot(wRate) / (norm * norm * norm)));
}

/** Unit lateral direction of the wheel in the plane: the axle projected onto the plane. */
template <typename SCALAR>
Vector3<SCALAR> lateralDirection(const Vector3<SCALAR>& axle, const Vector3<SCALAR>& normal) {
  const Vector3<SCALAR> lateral = axle - axle.dot(normal) * normal;
  return lateral / lateral.norm();
}

/** Unit rolling direction in the plane, n x lateral. */
template <typename SCALAR>
Vector3<SCALAR> longitudinalDirection(const Vector3<SCALAR>& axle, const Vector3<SCALAR>& normal) {
  return normal.cross(lateralDirection(axle, normal));
}

}  // namespace ocs2::humanoid::wheel
