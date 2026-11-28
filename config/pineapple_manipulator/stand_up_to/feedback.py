"""Finite-horizon LQR tracking gains for the Pineapple stand-up plan and its export (Upkie's feedback.py)."""

import json
import hashlib
from pathlib import Path

import casadi as ca
import numpy as np
from scipy.interpolate import CubicHermiteSpline, interp1d
from scipy.linalg import expm

import plan as to
import rollout as r

NS = 2 * to.NX  # [x; x_dot], the controller's state layout


def linearizations(model):
    """Contact-constrained forward dynamics, linearized, per phase: tires + seat, then tires only."""
    x, v, a = ca.SX.sym("x", to.NX), ca.SX.sym("v", to.NX), ca.SX.sym("a", to.NX)
    f, u = ca.SX.sym("f", 9), ca.SX.sym("u", to.NJ)
    res, gap, vel, *_ = model(x, v, a, f, u)
    M = ca.jacobian(res, a)
    J = -ca.jacobian(res, f).T
    bias = ca.substitute(res, ca.vertcat(a, f), ca.SX.zeros(to.NX + 9))
    cbias = ca.jacobian(vel[:9], x) @ v + 10 * vel[:9]
    for i in range(3):
        cbias[3 * i + 2] += 50 * gap[i]
    terms = ca.Function("terms", [x, v, u], [M, J, bias, cbias])
    result = []
    for indices in (list(range(9)), list(range(6))):
        state, torque = ca.MX.sym("state", NS), ca.MX.sym("torque", to.NJ)
        mat, jac, b, c = terms(state[:to.NX], state[to.NX:], torque)
        jac, c = jac[indices, :], c[indices]
        nc = len(indices)
        # The arm-free folded posture needs a less rigid linearization to avoid large
        # terminal sit-down gains. This regularizes the LQR model, not the TO contacts.
        regularization = 1e-3 if r.ROBOT == "pineapple" else 1e-5
        K = ca.blockcat([[mat, -jac.T], [jac, ca.MX.eye(nc) * regularization]])
        sol = ca.solve(K, ca.vertcat(-b, -c))
        ode = ca.vertcat(state[to.NX:], sol[:to.NX])
        result.append(ca.Function("linearize" + str(nc), [state, torque],
                                  [ca.jacobian(ode, state), ca.jacobian(ode, torque)]))
    return result


def joint_friction(v):
    """plan.functions' joint friction torque: smoothed friction loss and damping, odd in v."""
    return 0.1 * np.tanh(v / 0.05) + 0.02 * v


class Tracking:
    """Gains along the stand-up, or with reverse=True along the sit-down: the stand-up played backwards.

    Rigid-body dynamics and rolling contacts are time-reversible: q(T - t) with the velocity negated and the same
    accelerations and contact forces needs the same torques, except for the joint friction, which is odd in the
    velocity, so the sit-down torque is U - 2 friction(v). It ends seated at rest, the seat taking its load gradually.
    """

    def __init__(self, model_prefix, solution_path, geometry, reverse=False):
        self.geometry = geometry
        self.reverse = reverse
        self.plan = json.loads(Path(solution_path).read_text())
        if self.plan.get("robot", r.ROBOT) != r.ROBOT:
            raise ValueError("solution robot does not match the selected model")
        for key in ("max_constraint_violation", "terminal_dynamics_error", "contact_error", "initial_reference_error"):
            error = self.plan.get(key, float("inf"))
            if not np.isfinite(error) or error > 1e-5:
                raise ValueError(f"refusing export without a passing {key} check")
        mass = self.plan.get("model_mass_kg", float("nan"))
        if not np.isfinite(mass) or not np.isclose(mass, r.Rollout().m.body_subtreemass[1], atol=1e-8, rtol=1e-8):
            raise ValueError("solution mass does not match the physical robot; refusing intermediate-model export")
        samples = sum(self.plan.get("counts", to.COUNTS)) + 1
        for key, rows in (("x", to.NX), ("v", to.NX), ("torque", to.NJ)):
            values = np.array(self.plan[key])
            if values.shape != (rows, samples) or not np.isfinite(values).all():
                raise ValueError(f"invalid {key} trajectory")
        duration = np.array(self.plan["duration"])
        if duration.shape != (2,) or not np.isfinite(duration).all() or np.any(duration <= 0):
            raise ValueError("invalid contact-phase durations")
        self.X, self.V = np.array(self.plan["x"]), np.array(self.plan["v"])
        self.U = np.array(self.plan["torque"])
        D = self.plan["duration"]
        counts = self.plan.get("counts", to.COUNTS)
        self.switch = np.cumsum(D)
        self.times = np.r_[0, np.cumsum(np.repeat(np.array(D) / counts, counts))]
        if reverse:
            self.U = self.U - 2 * joint_friction(self.V[6:])
            self.X, self.V, self.U = self.X[:, ::-1], -self.V[:, ::-1], self.U[:, ::-1]
            self.times = self.times[-1] - self.times[::-1]
        self.spline = CubicHermiteSpline(self.times, self.X.T, self.V.T)
        self.torque = interp1d(self.times, self.U, axis=1)
        self.fns = linearizations(to.functions(model_prefix, geometry))
        self.end = self.times[-1]
        self.grid = np.r_[np.arange(0, self.end, 0.01), self.end]
        joint_q, joint_v = np.array([8, 8, 8, 0.1] * 2), np.array([1, 1, 1, 0.05] * 2)
        self.Q = np.diag(np.r_[0.2, 0.2, 15, 2, 150, 150, joint_q, 1, 1, 1, 2, 15, 15, joint_v])
        self.R = np.diag(np.array([0.1, 0.1, 0.1, 2] * 2))
        self.PD = np.zeros((to.NJ, NS))
        self.PD[:, 6:to.NX] = np.diag(r.KP)
        self.PD[:, to.NX + 6:] = np.diag(r.KD)
        self.gains = np.zeros((len(self.grid), to.NJ, NS))
        A, B = self.matrices(self.end, 0.01)
        # Contact invariants and zero-speed lateral translation are neutral uncontrollable modes: a finite
        # terminal hold instead of the DARE.
        P = self.Q * 2
        for _ in range(500):
            K = np.linalg.solve(self.R * 0.01 + B.T @ P @ B, B.T @ P @ A)
            P = self.Q * 0.01 + A.T @ P @ (A - B @ K)
        for i in range(len(self.grid) - 1, -1, -1):
            dt = self.grid[i + 1] - self.grid[i] if i < len(self.grid) - 1 else 0.01
            A, B = self.matrices(self.grid[i], dt)
            K = np.linalg.solve(self.R * dt + B.T @ P @ B, B.T @ P @ A)
            self.gains[i] = K
            P = self.Q * dt + A.T @ P @ (A - B @ K)
        print("gains ready, max |K|", np.max(abs(self.gains)), flush=True)

    def matrices(self, t, dt):
        seated = t >= self.end - self.switch[0] if self.reverse else t < self.switch[0]
        phase = 0 if seated else 1
        A, B = [np.array(z) for z in self.fns[phase](np.r_[self.spline(t), self.spline(t, 1)], self.torque(t))]
        A -= B @ self.PD
        M = np.zeros((NS + to.NJ, NS + to.NJ))
        M[:NS, :NS], M[:NS, NS:] = A, B
        E = expm(M * dt)
        return E[:NS, :NS], E[:NS, NS:]

    def export_sit_down(self):
        """The reversed plan, whole: from rest at the plan's end height to seated at rest."""
        times = np.unique(np.r_[np.arange(0, self.end, 0.02), self.end])
        states = np.array([np.r_[self.spline(t), self.spline(t, 1)] for t in times])
        gains = np.array([self.gains[min(int(round(t / 0.01)), len(self.grid) - 1)].reshape(-1) for t in times])
        return dict(
            joint_names=r.JOINTS,
            # The controller loads this map like the stand-up: it plays it to the end (handover_time), no MPC.
            handover_time=float(self.end),
            mpc_start_time=0.0,
            seat_contact_time=float(self.plan["duration"][-1]),  # the seat takes load (informational)
            kp=r.KP.tolist(),
            kd=r.KD.tolist(),
            time=times.tolist(),
            position=states[:, 6:to.NX].tolist(),
            velocity=states[:, to.NX + 6:].tolist(),
            effort=np.array([self.torque(t) for t in times]).tolist(),
            state=states.tolist(),
            feedback=gains.tolist(),
            wheel_radius=float(self.geometry["wheel"]["radius"]),
        )

    sit_down = None

    def export(self, path):
        import yaml

        last_phase = self.plan["duration"][-1]
        seat_release = self.end - last_phase
        handover = self.end - 0.4 * last_phase
        times = np.unique(np.r_[np.arange(0, handover, 0.02), self.switch[:-1], handover])
        states = np.array([np.r_[self.spline(t), self.spline(t, 1)] for t in times])
        gains = np.array([self.gains[min(int(t / 0.01), len(self.grid) - 1)].reshape(-1) for t in times])
        data = dict(
            joint_names=r.JOINTS,
            handover_time=float(handover),
            mpc_start_time=float(seat_release + 0.25 * last_phase),
            seat_release_time=float(seat_release),  # end of the seated phases (informational)
            kp=r.KP.tolist(),
            kd=r.KD.tolist(),
            time=times.tolist(),
            position=states[:, 6:to.NX].tolist(),
            velocity=states[:, to.NX + 6:].tolist(),
            effort=np.array([self.torque(t) for t in times]).tolist(),
            state=states.tolist(),
            feedback=gains.tolist(),
            # The controller's support guard: the seat (knee_contacts) at least this far above the tire plane.
            minimum_knee_clearance=0.015,
            clearance_hold_time=0.05,
            wheel_radius=float(self.geometry["wheel"]["radius"]),
            wheel_half_width=float(self.geometry["wheel"]["half_width"]),
            knee_contacts=[dict(frame=k["frame"], center=[float(c) for c in k["center"]], radius=float(k["radius"]),
                                half_width=float(k["half_width"])) for k in self.geometry["knee_housings"]],
        )

        class Dumper(yaml.SafeDumper):
            pass

        Dumper.add_representer(list, lambda dumper, values: dumper.represent_sequence(
            "tag:yaml.org,2002:seq", values, flow_style=all(isinstance(v, (int, float)) for v in values)))
        header = ("# Offline full-body ID stand-up: seat and tire contacts, then tire-only support.\n"
                  f"# Generated by config/pineapple_manipulator/stand_up_to/plan.py --robot {r.ROBOT}; validate after regenerating.\n")
        urdf = r.ROOT / "description/urdf/robots" / r.ROBOT / (r.ROBOT + ".urdf")
        maps = dict(generation=dict(
            robot=r.ROBOT,
            model_mass_kg=self.plan["model_mass_kg"],
            urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
            max_constraint_violation=self.plan["max_constraint_violation"],
            terminal_dynamics_error=self.plan["terminal_dynamics_error"],
            contact_error=self.plan["contact_error"],
            initial_reference_error=self.plan["initial_reference_error"],
            initial_pose_tolerance=self.plan["initial_pose_tolerance"],
            phase_durations=self.plan["duration"],
            max_displacement=self.plan["max_displacement"],
            max_yaw_change=self.plan["max_yaw_change"],
            symmetric=self.plan["symmetric"],
            velocity_limits=self.plan["velocity_limits"],
            acceleration_limits=self.plan["acceleration_limits"],
            contact_regularization=1e-3 if r.ROBOT == "pineapple" else 1e-5,
        ), stand_up=data)
        if self.sit_down is not None:
            maps["sit_down"] = self.sit_down.export_sit_down()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(header + yaml.dump(maps, Dumper=Dumper, sort_keys=False, width=100000))
        print("exported", path, len(times), "samples, handover", round(handover, 3), "s", flush=True)
