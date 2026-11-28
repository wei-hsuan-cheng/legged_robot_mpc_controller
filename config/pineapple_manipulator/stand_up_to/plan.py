#!/usr/bin/env python3
"""Offline Pineapple stand-up: full-body inverse dynamics with prescribed seat/tire contacts (Upkie's method).

    plan.py --models PREFIX [--phase-seconds MIN MAX] [--output config/pineapple_manipulator/stand_up_trajectory.yaml]

PREFIX_rnea.casadi and PREFIX_frames.casadi come from model.cpp (pineapple_manipulator_stand_up_model pineapple_manipulator.urdf PREFIX).
Two contact phases from the seated idle (rollout.IDLE), each lasting at most MAX seconds:

  1  both tires and the base box's back bottom edge: roll the CoM over the tires while the seat unloads
  2  both tires only: rise off the seat to 0.30 m in the posture table's balanced posture

The decision variables per node are the Euler state, its rates and accelerations, the eight joint torques and three
world point forces (two tires, the seat); the inverse dynamics hold at every node. The tracking gains and the export
are in feedback.py; simulate.py replays the export in MuJoCo.
"""

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

import casadi as ca
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rollout as r  # noqa: E402

NX = 14                      # [p (3), yaw, pitch, roll, 8 joints]
NJ = 8
COUNTS = [24, 36]            # intervals per phase
GOAL_HEIGHT = 0.30           # [m] base height at the end of the plan
TORQUE = np.array([19.0, 19.0, 28.0, 7.7] * 2)  # 70 % of the actuator ranges: feedback reserve
FRICTION = 0.7
# model.cpp and rollout.FRAMES share the same order for the selected robot's clearance links.
CLEARANCE = 0.005            # [m] ground clearance of the leg and arm collision boxes


def seat_points(geometry):
    """Back and front bottom edges of the seat box, in the base frame (the knee_housings disk's ends along x)."""
    seat = geometry["knee_housings"][0]
    assert seat["frame"] == "base_link"
    c, radius, half = np.array(seat["center"], float), seat["radius"], seat["half_width"]
    return c + [-radius, 0.0, -half], c + [radius, 0.0, -half]


def box_corners():
    """(frame index, corner in the frame) of every collision box of the clearance links, from the MJCF."""
    m = r.Rollout().m
    corners = []
    for index, body in enumerate(r.FRAMES[3:], start=3):
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != m.body(body).id or not m.geom_contype[g] or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, m.geom_quat[g])
            for sign in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
                corners.append((index, m.geom_pos[g] + rotation.reshape(3, 3) @ (sign * m.geom_size[g])))
    return corners


def functions(model_prefix, geometry):
    rnea = ca.Function.load(str(model_prefix) + "_rnea.casadi")
    fk = ca.Function.load(str(model_prefix) + "_frames.casadi")
    if fk.size1_out(0) != len(r.FRAMES) * 12 + 3:
        raise ValueError("CasADi frame model does not match --robot; regenerate its model first")
    x, v, a = ca.SX.sym("x", NX), ca.SX.sym("v", NX), ca.SX.sym("a", NX)
    F, u = ca.SX.sym("F", 9), ca.SX.sym("u", NJ)
    yaw, th, roll = x[3], x[4], x[5]
    cy, sy = ca.cos(yaw / 2), ca.sin(yaw / 2)
    cp, sp = ca.cos(th / 2), ca.sin(th / 2)
    cr, sr = ca.cos(roll / 2), ca.sin(roll / 2)
    quat = ca.vertcat(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    q = ca.vertcat(x[:3], quat, x[6:])
    g = fk(q)
    Rz = ca.vertcat(ca.horzcat(ca.cos(yaw), -ca.sin(yaw), 0), ca.horzcat(ca.sin(yaw), ca.cos(yaw), 0),
                    ca.horzcat(0, 0, 1))
    Ry = ca.vertcat(ca.horzcat(ca.cos(th), 0, ca.sin(th)), ca.horzcat(0, 1, 0),
                    ca.horzcat(-ca.sin(th), 0, ca.cos(th)))
    Rx = ca.vertcat(ca.horzcat(1, 0, 0), ca.horzcat(0, ca.cos(roll), -ca.sin(roll)),
                    ca.horzcat(0, ca.sin(roll), ca.cos(roll)))
    R = Rz @ Ry @ Rx
    omega = ca.vertcat(v[5] - ca.sin(th) * v[3],
                       ca.cos(roll) * v[4] + ca.sin(roll) * ca.cos(th) * v[3],
                       -ca.sin(roll) * v[4] + ca.cos(roll) * ca.cos(th) * v[3])
    pv = ca.vertcat(R.T @ v[:3], omega, v[6:])
    T = ca.jacobian(pv, v)
    pa = T @ a + ca.jacobian(pv, x) @ v
    tau = rnea(q, pv, pa)
    # MJCF joint friction loss (0.1 N m, smoothed as in the MPC) and damping (0.02 N m s).
    tau[6:] += 0.1 * ca.tanh(v[6:] / 0.05) + 0.02 * v[6:]
    res = T.T @ tau - ca.vertcat(ca.SX.zeros(6), u)
    wheel = geometry["wheel"]
    n = ca.DM([0, 0, 1])
    points, gaps, vels = [], [], []
    back, front = seat_points(geometry)
    # Contacts: left tire, right tire, seat back edge (with forces), seat front edge (gap only).
    for frame, kind in ((0, "tire"), (1, "tire"), (2, back), (2, front)):
        rot = ca.reshape(g[12 * frame + 3:12 * frame + 12], 3, 3)
        origin = g[12 * frame:12 * frame + 3]
        if isinstance(kind, str):
            center = origin + rot @ ca.DM(wheel["center_offset"])
            axle = rot @ ca.DM(np.array(wheel["axle_axis"]) / np.linalg.norm(wheel["axle_axis"]))
            w = n - ca.dot(axle, n) * axle
            # Smooth finite-width support point (the lower tire edge under roll), as Upkie's plan.
            rho = (-wheel["radius"] * w / ca.norm_2(w)
                   - wheel["half_width"] * axle[2] / ca.sqrt(axle[2] ** 2 + 0.02 ** 2) * axle)
        else:
            center, rho = origin, rot @ ca.DM(kind)
        Rdot = ca.reshape(ca.jacobian(ca.reshape(rot, 9, 1), x) @ v, 3, 3)
        cross = Rdot @ rot.T
        om = ca.vertcat(cross[2, 1], cross[0, 2], cross[1, 0])
        material = ca.jacobian(center, x) @ v + ca.cross(om, rho)
        if len(points) < 3:
            res -= ca.jacobian(material, v).T @ F[3 * len(points):3 * len(points) + 3]
        points.append(center + rho)
        gaps.append((center + rho)[2])
        vels.append(material)
    heights = [(g[12 * frame:12 * frame + 3] + ca.reshape(g[12 * frame + 3:12 * frame + 12], 3, 3) @ ca.DM(corner))[2]
               for frame, corner in box_corners()]
    com = g[12 * len(r.FRAMES):12 * len(r.FRAMES) + 3]
    return ca.Function("full_model", [x, v, a, F, u],
                       [res, ca.vertcat(*gaps), ca.vertcat(*vels), com, ca.vertcat(*points), q, pv, pa,
                        ca.vertcat(*heights)])


def initial_state(rollout):
    rollout.spawn_idle()
    yaw, pitch, roll = rollout.euler()
    q, _ = rollout.joints()
    return np.r_[rollout.d.qpos[:3], yaw, pitch, roll, q]


def balanced_joint_state(height):
    spec = importlib.util.spec_from_file_location("posture", r.ROOT / "config" / r.ROBOT / "compute_posture_table.py")
    posture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(posture)
    thigh, calf = posture.Model().solve(height, 1.1, -1.95)
    return np.array([0.0, thigh, calf, 0.0, 0.0, thigh, calf, 0.0])


def solve(model_prefix, seed_path, solution_path, phase_seconds, geometry):
    fn = functions(model_prefix, geometry)
    rollout = r.Rollout()
    x0 = initial_state(rollout)
    goal_joints = balanced_joint_state(GOAL_HEIGHT)
    goal = np.r_[x0[:2], GOAL_HEIGHT, x0[3], 0, 0, goal_joints]
    opt = ca.Opti()
    N = sum(COUNTS)
    knots = np.r_[0, np.cumsum(COUNTS)]
    symmetric = r.ROBOT == "pineapple"
    # A symmetric arm-free rise needs only forward translation, height, pitch and the
    # common thigh/calf/wheel angles. Keep all 14 RNEA rows and independent contact forces.
    rows = [0, 2, 4, 7, 8, 9] if symmetric else list(range(NX))
    Z, W, B = [opt.variable(len(rows), N + 1) for _ in range(3)]
    lift, offset = np.eye(NX), np.zeros(NX)
    nominal = x0.copy()
    if symmetric:
        heading = x0[3]
        lift = np.zeros((NX, len(rows)))
        lift[0, 0], lift[1, 0] = np.cos(heading), np.sin(heading)
        lift[2, 1], lift[4, 2] = 1, 1
        for index in (7, 11):
            lift[index:index + 3, 3:6] = np.eye(3)
        offset[[0, 1, 3]] = x0[[0, 1, 3]]
        nominal[0] = 0
        nominal[7:10] = (x0[7:10] + x0[11:14]) / 2
    X = ca.repmat(ca.DM(offset), 1, N + 1) + ca.DM(lift) @ Z
    V, A = ca.DM(lift) @ W, ca.DM(lift) @ B
    F, U, D = opt.variable(9, N + 1), opt.variable(NJ, N + 1), opt.variable(2)
    # Optimize both support-phase durations within the requested maneuver-time bounds.
    opt.subject_to(opt.bounded(phase_seconds[0], D, phase_seconds[1]))
    opt.set_initial(D, np.full(2, phase_seconds[1]))
    tolerance = np.r_[0, 0, 0.003, 0, 0.015, 0.01, np.full(NJ, 0.015)]
    if r.ROBOT == "pineapple":
        # Without the arm, idle servo deflection shifts the measured fold. Permit a nearby
        # contact-consistent nominal start; validate tracking from the actual settled pose.
        tolerance = np.r_[0, 0, 0.01, 0, 0.07, 0.01, 0.06, 0.15, 0.15, 0.15, 0.06, 0.15, 0.15, 0.15]
    opt.subject_to(opt.bounded(nominal[rows] - tolerance[rows], Z[:, 0], nominal[rows] + tolerance[rows]))
    opt.subject_to(W[:, 0] == 0)
    opt.subject_to(W[:, -1] == 0)
    opt.subject_to(B[:, -1] == 0)
    opt.subject_to(X[2, -1] == GOAL_HEIGHT)
    opt.subject_to(X[4, -1] == 0)
    if not symmetric:
        opt.subject_to(X[5, -1] == 0)
    legs = [1, 2] if symmetric else [0, 1, 2, 4, 5, 6]
    opt.subject_to(X[[6 + i for i in legs], -1] == goal[[6 + i for i in legs]])
    mg = rollout.m.body_subtreemass[1] * 9.81
    max_displacement = 0.4 if r.ROBOT == "pineapple" else 0.25
    max_yaw_change = 0.0 if symmetric else 0.3
    low = np.array([-0.61, 0.0, -3.26, -1e3, -1.05, 0.0, -3.26, -1e3])
    high = np.array([1.05, 1.745, 0.0, 1e3, 0.61, 1.745, 0.0, 1e3])
    if symmetric:
        # Unlimited wheel joints have a [0, 0] range in MuJoCo; it is not a fixed-angle bound.
        low, high = np.array([rollout.m.joint(name).range if rollout.m.joint(name).limited[0]
                              else [-1e3, 1e3] for name in r.JOINTS]).T
    vmax = np.r_[0.5, 0.3, 0.5, 0.5, 1.0, 0.5, np.array([1.5, 3, 3, 8] * 2)]
    amax = np.r_[3, 3, 4, 3, 6, 3, np.array([8, 10, 10, 20] * 2)]
    if symmetric:
        # Offline maneuver bounds, not MPC tuning or actuator limits. The short seat
        # release needs more acceleration than the heavier manipulator's slow rocking.
        vmax[rows] = [1, 1, 3, 5.5, 6, 15]
        amax[rows] = [10, 12, 30, 50, 50, 100]
    cost = 0
    warm_multipliers = False
    for j in range(N + 1):
        phase = min(np.searchsorted(knots, j, side="right") - 1, 1)
        dt = D[phase] / COUNTS[phase]
        res, gaps, vels, com, points, _, _, _, heights = fn(X[:, j], V[:, j], A[:, j], F[:, j], U[:, j])
        # At a fixed, statically balanced terminal pose, pitch balance is implied by force
        # balance. Keeping that dependent equality makes the terminal KKT block singular.
        # Check the complete terminal residual again before accepting any export.
        if j == N:
            opt.subject_to(res[[0, 1, 2, 3, 5] + list(range(6, NX))] / mg == 0)
        else:
            opt.subject_to(res / mg == 0)
        # Thighs, calves and arm off the ground (the seated spawn may rest a thigh on it: start at -2 mm).
        opt.subject_to(heights >= min(CLEARANCE, -0.002 + 0.001 * j))
        forward = ca.DM(lift[:3, 0])
        for i in range(1 if symmetric else 2):  # symmetric right contact is identical; verify it before export
            if j < N:
                opt.subject_to(gaps[i] == 0)
            if 0 < j < N:
                opt.subject_to(ca.dot(forward, vels[:3]) == 0 if symmetric else vels[3 * i:3 * i + 2] == 0)
        seat = phase == 0
        if seat:
            if j < N:
                opt.subject_to(gaps[2] == 0)
            if 0 < j:
                opt.subject_to(ca.dot(forward, vels[6:9]) == 0 if symmetric else vels[6:8] == 0)
            # Unload the seat before the release, not in one force jump.
            force_scale = mg if symmetric else 1.0
            opt.subject_to(F[8, j] / force_scale <= mg / force_scale * min(1.0, (knots[1] - j) / COUNTS[0] * 3))
            opt.subject_to(gaps[3] >= 0)
        else:
            opt.subject_to(F[6:9, j] == 0)
            since = j - knots[1]
            clearance = 0.03 * min(since / 8, 1)
            opt.subject_to(gaps[2] >= clearance)
            opt.subject_to(gaps[3] >= clearance)
        for i in range(3 if seat else 2):
            f = F[3 * i:3 * i + 3, j] / (mg if symmetric else 1.0)
            opt.subject_to(f[2] >= 0)
            opt.subject_to(opt.bounded(-FRICTION * f[2], f[:2], FRICTION * f[2]))
        opt.subject_to(opt.bounded(0.0, X[2, j], 0.40))
        opt.subject_to(opt.bounded(-0.6, X[4, j], 0.6))
        if not symmetric:
            opt.subject_to(opt.bounded(-0.2, X[5, j], 0.2))
            opt.subject_to(opt.bounded(x0[3] - max_yaw_change, X[3, j], x0[3] + max_yaw_change))
        opt.subject_to(opt.bounded(-max_displacement, Z[0, j] if symmetric else X[:2, j] - x0[:2], max_displacement))
        joint_rows = [i for i in rows if i >= 6]
        opt.subject_to(opt.bounded(low[np.array(joint_rows) - 6], X[joint_rows, j], high[np.array(joint_rows) - 6]))
        opt.subject_to(opt.bounded(-vmax[rows], W[:, j], vmax[rows]))
        opt.subject_to(opt.bounded(-amax[rows], B[:, j], amax[rows]))
        opt.subject_to(opt.bounded(-1, U[:, j] / ca.DM(TORQUE), 1) if symmetric
                       else opt.bounded(-TORQUE, U[:, j], TORQUE))
        # Hips stay near zero (wheels upright): the seated tires sit inward of the standing track.
        cost += dt * (5 * ca.sumsqr(V[3:6, j]) + ca.sumsqr(V[6:, j]) + 0.03 * ca.sumsqr(A[:, j])
                      + 0.1 * ca.sumsqr(U[:, j] / ca.DM(TORQUE)) + 0.1 * ca.sumsqr(F[:, j] / mg)
                      + ca.sumsqr(X[4, j]) + 10 * ca.sumsqr(X[5, j]) + 2 * ca.sumsqr(X[:2, j] - x0[:2])
                      + 2 * ca.sumsqr(X[[6, 10], j])
                      # Left/right symmetry: hips mirrored (same axis, opposite abduction), thighs and calves equal.
                      + 5 * ca.sumsqr(X[6, j] + X[10, j]) + 5 * ca.sumsqr(X[7:9, j] - X[11:13, j]))
        if j < N:
            opt.subject_to(Z[:, j + 1] == Z[:, j] + dt / 2 * (W[:, j] + W[:, j + 1]))
            opt.subject_to(W[:, j + 1] == W[:, j] + dt / 2 * (B[:, j] + B[:, j + 1]))
            cost += dt * (0.002 * ca.sumsqr((U[:, j + 1] - U[:, j]) / dt)
                          + 0.2 * ca.sumsqr((F[:, j + 1] - F[:, j]) / (mg * dt)))
        s = j / N
        target = goal[rows].copy()
        if symmetric:
            target[0] = 0
        opt.set_initial(Z[:, j], (1 - s) * nominal[rows] + s * target)
        forces = np.zeros((3, 3))
        forces[:3 if seat else 2, 2] = mg / (3 if seat else 2)
        opt.set_initial(F[:, j], forces.reshape(-1))
    if seed_path is not None and Path(seed_path).exists():
        old = json.loads(Path(seed_path).read_text())
        durations = np.clip(np.array(old["duration"]), *phase_seconds)
        opt.set_initial(D, durations)
        warm_x = np.array(old["x"])[rows].copy()
        if symmetric:
            warm_x[0] = (np.cos(heading) * (np.array(old["x"])[0] - x0[0])
                         + np.sin(heading) * (np.array(old["x"])[1] - x0[1]))
        opt.set_initial(Z, warm_x)
        for key, value in dict(force=F, torque=U).items():
            opt.set_initial(value, np.array(old[key]))
        # Preserve feasible phase timing instead of stretching the first phase to the upper
        # bound and introducing a derivative jump at the contact switch in every warm start.
        old_seconds = np.repeat(np.array(old["duration"]) / COUNTS, COUNTS)
        ratio = np.r_[old_seconds, old_seconds[-1]] / np.repeat(
            durations / COUNTS, np.r_[COUNTS[0], COUNTS[1] + 1])
        for key, value, power in (("v", W, 1), ("a", B, 2)):
            warm = np.array(old[key])[rows].copy()
            if symmetric:
                warm[0] = np.cos(heading) * np.array(old[key])[0] + np.sin(heading) * np.array(old[key])[1]
            opt.set_initial(value, warm * ratio ** power)
        multipliers = np.array(old.get("constraint_multipliers", []))
        same_bounds = (old.get("max_displacement", 0.25) == max_displacement
                       and old.get("max_yaw_change", 0.3) == max_yaw_change)
        if same_bounds and multipliers.size == opt.g.numel() and np.isfinite(multipliers).all():
            opt.set_initial(opt.lam_g, multipliers)
            warm_multipliers = True
    opt.minimize(cost + 0.05 * ca.sum1(D))
    options = dict(max_iter=1000, tol=1e-6, print_level=5, mu_strategy="adaptive", print_frequency_iter=25)
    if warm_multipliers:
        options.update(warm_start_init_point="yes", warm_start_bound_push=1e-6,
                       warm_start_slack_bound_push=1e-6, warm_start_mult_bound_push=1e-6)
    opt.solver("ipopt", {"expand": True}, options)
    violation = ca.mmax(ca.fmax(opt.lbg - opt.g, opt.g - opt.ubg))
    evaluate = ca.Function("evaluate", [opt.x], [X, V, A, F, U, D, cost, opt.f, violation]).expand()
    best = dict(objective=np.inf, iteration=None)

    def keep_best(iteration):  # IPOPT may end oscillating: keep the cheapest feasible iterate
        z = opt.debug.value(opt.x)
        objective, error = (float(v) for v in evaluate(z)[-2:])
        if error <= 1e-6 and objective < best["objective"]:
            if best["iteration"] is None:
                print("first feasible iterate", iteration, "constraint violation", error, flush=True)
            best.update(objective=objective, iteration=iteration, z=z)

    opt.callback(keep_best)
    success = False
    try:
        sol = opt.solve()
        success = True
    except RuntimeError:
        sol = opt.debug
    z = sol.value(opt.x)
    retained = None
    if float(evaluate(z)[-1]) > 1e-5 and best["iteration"] is not None:
        z, retained = best["z"], best["iteration"]
        print("final iterate infeasible; retaining iteration", retained, flush=True)
    *trajectory, total, _, error = evaluate(z)
    opt.callback()  # Break the callback/Opti reference cycle when solving multiple offline problems.
    out = {k: np.array(y).squeeze().tolist() for k, y in zip(("x", "v", "a", "force", "torque", "duration"), trajectory)}
    out.update(success=success, initial=x0.tolist(), goal=goal.tolist(), cost=float(total),
               return_status=opt.stats()["return_status"], retained_iteration=retained,
               max_constraint_violation=float(error), robot=r.ROBOT,
               model_mass_kg=float(mg / 9.81), counts=COUNTS, initial_pose_tolerance=tolerance.tolist(),
               max_displacement=max_displacement, max_yaw_change=max_yaw_change, symmetric=symmetric,
               velocity_limits=vmax[rows].tolist(), acceleration_limits=amax[rows].tolist())
    contact_values = [fn(*z) for z in zip(*(np.array(out[key]).T for key in ("x", "v", "a", "force", "torque")))]
    gaps = np.array([np.array(z[1]).reshape(-1) for z in contact_values])
    velocities = np.array([np.array(z[2]).reshape(-1) for z in contact_values])
    # Includes every tire and both XY directions omitted as analytic duplicates in the
    # symmetric NLP. Vertical material velocity is not the rolling no-slip equality.
    out["contact_error"] = float(max(np.abs(gaps[:, :2]).max(), np.abs(gaps[:COUNTS[0], 2]).max(),
                                     np.abs(velocities[:, [0, 1, 3, 4]]).max(),
                                     np.abs(velocities[:COUNTS[0], 6:8]).max()))
    out["initial_reference_error"] = float(max(0, (np.abs(np.array(out["x"])[:, 0] - x0) - tolerance).max()))
    terminal_residual = fn(np.array(out["x"])[:, -1], np.array(out["v"])[:, -1],
                           np.array(out["a"])[:, -1], np.array(out["force"])[:, -1],
                           np.array(out["torque"])[:, -1])[0]
    out["terminal_dynamics_error"] = float(ca.norm_inf(terminal_residual) / mg)
    if success and retained is None:
        out["constraint_multipliers"] = np.array(sol.value(opt.lam_g)).reshape(-1).tolist()
    Path(solution_path).parent.mkdir(parents=True, exist_ok=True)
    Path(solution_path).write_text(json.dumps(out))
    print("NLP status:", out["return_status"], "constraint violation:", out["max_constraint_violation"],
          "durations:", out["duration"], flush=True)
    if not np.isfinite(out["max_constraint_violation"]) or out["max_constraint_violation"] > 1e-5:
        values, lower, upper = [np.array(y).reshape(-1) for y in ca.Function(
            "constraint_check", [opt.x], [opt.g, opt.lbg, opt.ubg])(z)]
        errors = np.maximum(lower - values, values - upper)
        for index in np.argsort(errors)[-5:][::-1]:
            print("constraint error", errors[index], opt.debug.g_describe(int(index)), flush=True)
        raise RuntimeError("stand-up optimization is infeasible; refusing trajectory export")
    if not np.isfinite(out["terminal_dynamics_error"]) or out["terminal_dynamics_error"] > 1e-5:
        raise RuntimeError("complete terminal inverse dynamics failed; refusing trajectory export")
    if not np.isfinite(out["contact_error"]) or out["contact_error"] > 1e-5:
        raise RuntimeError("complete rolling/seat contact check failed; refusing trajectory export")
    if not np.isfinite(out["initial_reference_error"]) or out["initial_reference_error"] > 1e-5:
        raise RuntimeError("initial reference is outside the settled-pose envelope; refusing trajectory export")
    return out


def check_model(model_prefix, geometry):
    """Inverse dynamics and CoM against MuJoCo at perturbed 3D states (arm at zero, at rest)."""
    from scipy.spatial.transform import Rotation

    model = functions(model_prefix, geometry)
    rnea = ca.Function.load(str(model_prefix) + "_rnea.casadi")
    frames = ca.Function.load(str(model_prefix) + "_frames.casadi")
    rollout = r.Rollout()
    m, d = rollout.m, rollout.d
    rng = np.random.default_rng(9)
    legs = rollout.va[:8]
    for _ in range(5):
        x = np.r_[0.0, 0.0, 0.3, 0.1, 0.2, 0.03, 0.1, 1.1, -1.9, 0.5, -0.1, 1.0, -1.8, -0.4] + rng.normal(0, 0.02, NX)
        v, a = rng.normal(0, 0.3, NX), rng.normal(0, 0.5, NX)
        _, _, _, com, _, q, pv, pa, _ = model(x, v, a, np.zeros(9), np.zeros(NJ))
        q, pv, pa = [np.array(z).reshape(-1) for z in (q, pv, pa)]
        rotation = Rotation.from_quat(q[3:7]).as_matrix()
        mujoco.mj_resetData(m, d)
        d.qpos[:3], d.qpos[3:7] = q[:3], np.r_[q[6], q[3:6]]
        d.qpos[rollout.qa[:8]] = q[7:]
        d.qvel[:3], d.qvel[3:6] = rotation @ pv[:3], pv[3:6]
        d.qvel[legs] = pv[6:]
        mujoco.mj_forward(m, d)
        placements = np.array(frames(q)).reshape(-1)
        for index, name in enumerate(r.FRAMES):
            body = m.body(name).id
            np.testing.assert_allclose(placements[12 * index:12 * index + 3], d.xpos[body], atol=1e-7)
            np.testing.assert_allclose(placements[12 * index + 3:12 * index + 12].reshape(3, 3, order="F"),
                                       d.xmat[body].reshape(3, 3), atol=1e-7)
        mass = np.zeros((m.nv, m.nv))
        try:
            mujoco.mj_fullM(m, d, mass)  # MuJoCo >= 3.4
        except TypeError:
            mujoco.mj_fullM(m, mass, d.qM)
        acceleration = np.zeros(m.nv)
        acceleration[:3], acceleration[3:6] = rotation @ (pa[:3] + np.cross(pv[3:6], pv[:3])), pa[3:6]
        acceleration[legs] = pa[6:]
        rows = np.r_[0:6, legs]
        expected = (mass @ acceleration + d.qfrc_bias)[rows]
        predicted = np.array(rnea(q, pv, pa)).reshape(-1)
        predicted[:3] = rotation @ predicted[:3]
        np.testing.assert_allclose(predicted, expected, atol=1e-5, rtol=1e-6)
        np.testing.assert_allclose(np.array(com).reshape(-1), d.subtree_com[1], atol=1e-7)
    print("Pinocchio/CasADi and MuJoCo dynamics/CoM/frame parity passed (5 states).", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", choices=("pineapple", "pineapple_manipulator"), default="pineapple_manipulator")
    parser.add_argument("--models", type=Path)
    parser.add_argument("--seed", type=Path)
    parser.add_argument("--solution", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--phase-seconds", type=float, nargs=2, metavar=("MIN", "MAX"))
    parser.add_argument("--geometry", type=Path)
    parser.add_argument("--check-model-only", action="store_true")
    parser.add_argument("--export-only", action="store_true", help="skip the solve: export --solution's plan")
    args = parser.parse_args()
    r.configure(args.robot)
    args.phase_seconds = args.phase_seconds or ((0.05, 2.5) if args.robot == "pineapple" else (0.5, 1.5))
    work = Path(tempfile.gettempdir()) / (args.robot + "_stand_up")
    args.models = args.models or work / args.robot
    args.solution = args.solution or work / "solution.json"
    own_seed = r.ROOT / "config" / args.robot / "stand_up_seed.json"
    args.seed = args.seed or (own_seed if own_seed.is_file() else HERE / "seed.json")
    args.output = args.output or r.ROOT / "config" / args.robot / "stand_up_trajectory.yaml"
    if not args.check_model_only and args.output.is_symlink():
        raise ValueError("output is a shared trajectory symlink; choose a separate candidate path before installation")
    geometry = r.load_geometry(args.geometry)
    check_model(args.models, geometry)
    if args.check_model_only:
        return
    if not args.export_only:
        solve(args.models, args.seed, args.solution, args.phase_seconds, geometry)
    from feedback import Tracking

    tracking = Tracking(args.models, args.solution, geometry)
    tracking.sit_down = Tracking(args.models, args.solution, geometry, reverse=True)
    tracking.export(args.output)


if __name__ == "__main__":
    main()
