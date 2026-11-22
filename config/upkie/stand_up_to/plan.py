"""Offline Upkie stand-up: full-body inverse dynamics and prescribed knee/wheel contacts."""

import argparse
import importlib.util
import json
from pathlib import Path
import tempfile

import casadi as ca
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def load_helper(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


p = load_helper("upkie_rollout", HERE.parent / "plan_stand_up.py")
posture = load_helper("upkie_posture", HERE.parent / "compute_posture_table.py")


def functions(model_prefix):
    rnea = ca.Function.load(str(model_prefix) + "_rnea.casadi")
    fk = ca.Function.load(str(model_prefix) + "_fk.casadi")
    x = ca.SX.sym("x", 12)
    v = ca.SX.sym("v", 12)
    a = ca.SX.sym("a", 12)
    F = ca.SX.sym("F", 12)
    u = ca.SX.sym("u", 6)
    yaw, th, roll = x[3], x[4], x[5]
    cy, sy = ca.cos(yaw / 2), ca.sin(yaw / 2)
    cp, sp = ca.cos(th / 2), ca.sin(th / 2)
    cr, sr = ca.cos(roll / 2), ca.sin(roll / 2)
    quat = ca.vertcat(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    q = ca.vertcat(x[:3], quat, x[6:])
    g = fk(q)
    Rz = ca.vertcat(
        ca.horzcat(ca.cos(yaw), -ca.sin(yaw), 0),
        ca.horzcat(ca.sin(yaw), ca.cos(yaw), 0),
        ca.horzcat(0, 0, 1),
    )
    Ry = ca.vertcat(
        ca.horzcat(ca.cos(th), 0, ca.sin(th)),
        ca.horzcat(0, 1, 0),
        ca.horzcat(-ca.sin(th), 0, ca.cos(th)),
    )
    Rx = ca.vertcat(
        ca.horzcat(1, 0, 0),
        ca.horzcat(0, ca.cos(roll), -ca.sin(roll)),
        ca.horzcat(0, ca.sin(roll), ca.cos(roll)),
    )
    R = Rz @ Ry @ Rx
    omega = ca.vertcat(
        v[5] - ca.sin(th) * v[3],
        ca.cos(roll) * v[4] + ca.sin(roll) * ca.cos(th) * v[3],
        -ca.sin(roll) * v[4] + ca.cos(roll) * ca.cos(th) * v[3],
    )
    pv = ca.vertcat(R.T @ v[:3], omega, v[6:])
    T = ca.jacobian(pv, v)
    pa = T @ a + ca.jacobian(pv, x) @ v
    tau = rnea(q, pv, pa)
    tau[6:] += 0.1 * ca.tanh(v[6:] / 0.05)
    res = T.T @ tau - ca.vertcat(ca.SX.zeros(6), u)
    gaps = []
    vels = []
    points = []
    n = ca.DM([0, 0, 1])
    for i in range(4):
        center = g[12 * i : 12 * i + 3]
        rot = ca.reshape(g[12 * i + 3 : 12 * i + 12], 3, 3)
        axle = rot[:, 2]
        w = n - ca.dot(axle, n) * axle
        halfwidth = 0.022 if i < 2 else 0.02685
        # Smooth finite-width support point; unlike a thin disk, this accounts
        # for the lower tire/housing edge under body roll.
        rho = (
            -(0.08 if i < 2 else 0.05) * w / ca.norm_2(w)
            - halfwidth * axle[2] / ca.sqrt(axle[2] ** 2 + 0.02**2) * axle
        )
        Rdot = ca.reshape(ca.jacobian(ca.reshape(rot, 9, 1), x) @ v, 3, 3)
        cross = Rdot @ rot.T
        om = ca.vertcat(cross[2, 1], cross[0, 2], cross[1, 0])
        material = ca.jacobian(center, x) @ v + ca.cross(om, rho)
        J = ca.jacobian(material, v)
        res -= J.T @ F[3 * i : 3 * i + 3]
        points.append(center + rho)
        gaps.append((center + rho)[2])
        vels.append(material)
    return ca.Function(
        "full_model",
        [x, v, a, F, u],
        [
            res,
            ca.vertcat(*gaps),
            ca.vertcat(*vels),
            g[48:51],
            ca.vertcat(*points),
            q,
            pv,
            pa,
        ],
    )


def solve(model_prefix, seed_path, solution_path):
    fn = functions(model_prefix)
    r = p.Rollout()
    r.spawn_idle()
    q, _ = r.joints()
    x0 = np.r_[r.d.qpos[:3], np.pi, r.pitch(), 0, q]
    h, k = posture.Model().solve(0.18, -1.5, -2.1)
    goal = np.r_[x0[:2], 0.18, np.pi, 0, 0, p.mirror(h, k)]
    opt = ca.Opti()
    counts = [16, 16, 28]
    N = sum(counts)
    knots = np.r_[0, np.cumsum(counts)]
    X = opt.variable(12, N + 1)
    V = opt.variable(12, N + 1)
    A = opt.variable(12, N + 1)
    F = opt.variable(12, N + 1)
    U = opt.variable(6, N + 1)
    D = opt.variable(3)
    opt.subject_to(opt.bounded(1.0, D, 5.0))
    opt.set_initial(D, [2.0, 2.0, 2.0])
    tolerance = np.r_[0, 0, 0.003, 0, 0.015, 0.01, np.full(6, 0.015)]
    opt.subject_to(opt.bounded(x0 - tolerance, X[:, 0], x0 + tolerance))
    opt.subject_to(V[:, 0] == 0)
    opt.subject_to(V[:, -1] == 0)
    opt.subject_to(A[:, -1] == 0)
    opt.subject_to(X[2, -1] == 0.18)
    opt.subject_to(X[4:6, -1] == 0)
    opt.subject_to(X[[6, 7, 9, 10], -1] == goal[[6, 7, 9, 10]])
    mg = r.m.body_subtreemass[1] * 9.81
    cost = 0
    for j in range(N + 1):
        phase = min(np.searchsorted(knots, j, side="right") - 1, 2)
        dt = D[phase] / counts[phase]
        res, gaps, vels, com, points, _, _, _ = fn(
            X[:, j], V[:, j], A[:, j], F[:, j], U[:, j]
        )
        opt.subject_to(res / mg == 0)
        active = [True, True, phase == 0, phase < 2]
        for i in range(4):
            if active[i]:
                if j < N:
                    opt.subject_to(gaps[i] == 0)
                if 0 < j < N:
                    opt.subject_to(vels[3 * i : 3 * i + 2] == 0)
                # Unload each knee before the prescribed release, not in one abrupt force jump.
                if i >= 2:
                    release = knots[1 if i == 2 else 2]
                    opt.subject_to(
                        F[3 * i + 2, j]
                        <= mg * min(1.0, (release - j) / counts[phase] * 3)
                    )
            else:
                opt.subject_to(F[3 * i : 3 * i + 3, j] == 0)
                since = j - knots[1 if i == 2 else 2]
                opt.subject_to(gaps[i] >= 0.015 * min(since / 12, 1))
            if active[i]:
                f = F[3 * i : 3 * i + 3, j]
                opt.subject_to(f[2] >= 0)
                opt.subject_to(opt.bounded(-0.7 * f[2], f[:2], 0.7 * f[2]))
        opt.subject_to(opt.bounded(0.08, X[2, j], 0.3))
        opt.subject_to(opt.bounded(-0.85, X[4, j], 0.85))
        opt.subject_to(opt.bounded(-0.22, X[5, j], 0.22))
        opt.subject_to(opt.bounded(np.pi - 0.3, X[3, j], np.pi + 0.3))
        opt.subject_to(opt.bounded(-0.20, X[:2, j] - x0[:2], 0.20))
        low = np.array([-2.7, -2.38, -100, -0.8, 0.8, -100])
        high = np.array([0.8, -0.8, 100, 2.7, 2.38, 100])
        opt.subject_to(opt.bounded(low, X[6:, j], high))
        vmax = np.r_[0.5, 0.3, 0.3, 0.5, 0.8, 0.5, 2, 2, 6, 2, 2, 6]
        amax = np.r_[3, 3, 3, 3, 3, 3, 6, 6, 15, 6, 6, 15]
        opt.subject_to(opt.bounded(-vmax, V[:, j], vmax))
        opt.subject_to(opt.bounded(-amax, A[:, j], amax))
        limits = np.array([12, 12, 1.2, 12, 12, 1.2])
        opt.subject_to(opt.bounded(-limits, U[:, j], limits))
        cost += dt * (
            5 * ca.sumsqr(V[3:6, j])
            + ca.sumsqr(V[6:, j])
            + 0.03 * ca.sumsqr(A[:, j])
            + 0.1 * ca.sumsqr(U[:, j] / ca.DM(limits))
            + 0.1 * ca.sumsqr(F[:, j] / mg)
            + ca.sumsqr(X[4:6, j])
            + 2 * ca.sumsqr(X[:2, j] - x0[:2])
        )
        if j < N:
            opt.subject_to(X[:, j + 1] == X[:, j] + dt / 2 * (V[:, j] + V[:, j + 1]))
            opt.subject_to(V[:, j + 1] == V[:, j] + dt / 2 * (A[:, j] + A[:, j + 1]))
            cost += dt * (
                0.002 * ca.sumsqr((U[:, j + 1] - U[:, j]) / dt)
                + 0.2 * ca.sumsqr((F[:, j + 1] - F[:, j]) / (mg * dt))
            )
        s = j / N
        opt.set_initial(X[:, j], (1 - s) * x0 + s * goal)
        forces = np.zeros((4, 3))
        forces[active, 2] = mg / sum(active)
        opt.set_initial(F[:, j], forces.reshape(-1))
    previous = Path(seed_path)

    if previous.exists():
        old = json.loads(previous.read_text())
        for key, value in dict(x=X, v=V, a=A, force=F, torque=U, duration=D).items():
            opt.set_initial(value, np.array(old[key]))
    opt.minimize(cost + 0.05 * ca.sum1(D))
    opt.solver(
        "ipopt",
        {"expand": True},
        dict(
            max_iter=1000,
            tol=1e-6,
            print_level=5,
            mu_strategy="adaptive",
            print_frequency_iter=25,
        ),
    )
    success = False
    try:
        sol = opt.solve()
        success = True
    except RuntimeError:
        sol = opt.debug
        opt.debug.show_infeasibilities(1e-4)
    out = {
        k: np.array(sol.value(y)).tolist()
        for k, y in dict(x=X, v=V, a=A, force=F, torque=U, duration=D).items()
    }
    out.update(
        success=success,
        initial=x0.tolist(),
        goal=goal.tolist(),
        cost=float(sol.value(cost)),
        stats=opt.stats(),
    )
    values = np.array(sol.value(opt.g)).reshape(-1)
    lower = np.array(sol.value(opt.lbg)).reshape(-1)
    upper = np.array(sol.value(opt.ubg)).reshape(-1)
    out["max_constraint_violation"] = float(
        np.max(np.maximum(lower - values, values - upper))
    )
    solution_path.parent.mkdir(parents=True, exist_ok=True)
    solution_path.write_text(json.dumps(out))
    print(
        "NLP status:",
        out["stats"]["return_status"],
        "constraint violation:",
        out["max_constraint_violation"],
        flush=True,
    )
    if (
        not np.isfinite(out["max_constraint_violation"])
        or out["max_constraint_violation"] > 1e-5
    ):
        raise RuntimeError(
            "stand-up optimization is infeasible; refusing trajectory export"
        )
    for key in ("x", "v", "a", "force", "torque", "duration"):
        if not np.all(np.isfinite(out[key])):
            raise RuntimeError(f"nonfinite {key}; refusing trajectory export")
    return out


def check_model(model_prefix):
    from scipy.spatial.transform import Rotation
    from xml.etree import ElementTree

    xml = ElementTree.parse(ROOT / "description/mujoco/robots/upkie/upkie.xml")
    tire = xml.find(".//mesh[@name='tire_collision']")
    vertices = np.fromstring(tire.attrib["vertex"], sep=" ").reshape(-1, 3)
    if not np.isclose(
        np.max(np.linalg.norm(vertices[:, :2], axis=1)), 0.08, atol=1e-6
    ) or not np.isclose(np.ptp(vertices[:, 2]), 0.044, atol=1e-6):
        raise ValueError(
            "this stand-up model expects the 80 mm-radius, 44 mm-wide collision tire"
        )

    model = functions(model_prefix)
    rnea = ca.Function.load(str(model_prefix) + "_rnea.casadi")
    rollout = p.Rollout()
    rng = np.random.default_rng(9)
    for _ in range(5):
        x = np.r_[0.0, 0.0, 0.2, np.pi, 0.2, 0.03, p.mirror(-1.2, -2.1)]
        x += rng.normal(0, 0.02, 12)
        v = rng.normal(0, 0.3, 12)
        a = rng.normal(0, 0.5, 12)
        _, _, _, com, _, q, pv, pa = model(x, v, a, np.zeros(12), np.zeros(6))
        q, pv, pa = [np.array(z).reshape(-1) for z in (q, pv, pa)]
        rotation = Rotation.from_quat(q[3:7]).as_matrix()
        rollout.d.qpos[:] = np.r_[q[:3], q[6], q[3:6], q[7:]]
        rollout.d.qvel[:] = np.r_[rotation @ pv[:3], pv[3:]]
        acceleration = np.r_[rotation @ (pa[:3] + np.cross(pv[3:6], pv[:3])), pa[3:]]
        mujoco.mj_forward(rollout.m, rollout.d)
        mass = np.zeros((12, 12))
        mujoco.mj_fullM(rollout.m, rollout.d, mass)
        expected = mass @ acceleration + rollout.d.qfrc_bias
        predicted = np.array(rnea(q, pv, pa)).reshape(-1)
        predicted[:3] = rotation @ predicted[:3]
        np.testing.assert_allclose(predicted, expected, atol=1e-5, rtol=1e-6)
        np.testing.assert_allclose(
            np.array(com).reshape(-1), rollout.d.subtree_com[1], atol=1e-7
        )
    print(
        "Pinocchio/CasADi and MuJoCo dynamics/CoM parity passed (5 states).", flush=True
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--models",
        type=Path,
        default=Path(tempfile.gettempdir()) / "upkie_stand_up" / "upkie",
    )
    parser.add_argument("--seed", type=Path, default=HERE / "seed.json")
    parser.add_argument(
        "--solution",
        type=Path,
        default=Path(tempfile.gettempdir()) / "upkie_stand_up" / "solution.json",
    )
    parser.add_argument(
        "--output", type=Path, default=HERE.parent / "stand_up_trajectory.yaml"
    )
    parser.add_argument("--check-model-only", action="store_true")
    args = parser.parse_args()
    check_model(args.models)
    if args.check_model_only:
        return
    solve(args.models, args.seed, args.solution)
    from feedback import Tracking

    tracker = Tracking(args.models, args.solution)
    tracker.export(args.output)


if __name__ == "__main__":
    main()
