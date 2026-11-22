import json
from pathlib import Path
import casadi as ca
import numpy as np
from scipy.interpolate import CubicHermiteSpline, interp1d
from scipy.linalg import expm
import plan as to

p = to.p


def linearizations(model):
    x = ca.SX.sym("x", 12)
    v = ca.SX.sym("v", 12)
    a = ca.SX.sym("a", 12)
    f = ca.SX.sym("f", 12)
    u = ca.SX.sym("u", 6)
    res, gap, vel, *_ = model(x, v, a, f, u)
    M = ca.jacobian(res, a)
    J = -ca.jacobian(res, f).T
    bias = ca.substitute(res, ca.vertcat(a, f), ca.SX.zeros(24))
    cbias = ca.jacobian(vel, x) @ v + 10 * vel
    for i in range(4):
        cbias[3 * i + 2] += 50 * gap[i]
    terms = ca.Function("terms", [x, v, u], [M, J, bias, cbias])
    result = []
    for indices in (
        list(range(12)),
        list(range(6)) + list(range(9, 12)),
        list(range(6)),
    ):
        state = ca.MX.sym("state", 24)
        torque = ca.MX.sym("torque", 6)
        mat, jac, b, c = terms(state[:12], state[12:], torque)
        jac = jac[indices, :]
        c = c[indices]
        nc = len(indices)
        K = ca.blockcat([[mat, -jac.T], [jac, ca.MX.eye(nc) * 1e-5]])
        sol = ca.solve(K, ca.vertcat(-b, -c))
        ode = ca.vertcat(state[12:], sol[:12])
        result.append(
            ca.Function(
                "linearize" + str(nc),
                [state, torque],
                [ca.jacobian(ode, state), ca.jacobian(ode, torque)],
            )
        )
    return result


class Tracking:
    def __init__(self, model_prefix, solution_path):
        self.plan = json.loads(Path(solution_path).read_text())
        self.X = np.array(self.plan["x"])
        self.V = np.array(self.plan["v"])
        self.U = np.array(self.plan["torque"])
        D = self.plan["duration"]
        self.switch = np.cumsum(D)
        self.times = np.r_[
            np.linspace(0, D[0], 17),
            D[0] + np.linspace(0, D[1], 17)[1:],
            D[0] + D[1] + np.linspace(0, D[2], 29)[1:],
        ]
        self.spline = CubicHermiteSpline(self.times, self.X.T, self.V.T)
        self.torque = interp1d(self.times, self.U, axis=1)
        self.model = to.functions(model_prefix)
        self.fns = linearizations(self.model)
        self.end = self.times[-1]
        self.grid = np.arange(0, self.end, 0.01)
        self.grid = np.r_[self.grid, self.end]
        self.Q = np.diag(
            np.r_[
                0.2,
                0.2,
                15,
                2,
                150,
                150,
                np.array([8, 8, 0.1, 8, 8, 0.1]),
                1,
                1,
                1,
                2,
                15,
                15,
                np.array([1, 1, 0.05, 1, 1, 0.05]),
            ]
        )
        self.R = np.diag([0.2, 0.2, 2, 0.2, 0.2, 2])
        self.PD = np.zeros((6, 24))
        self.PD[:, 6:12] = np.diag(p.KP)
        self.PD[:, 18:] = np.diag(p.KD)
        self.gains = np.zeros((len(self.grid), 6, 24))
        A, B = self.matrices(self.end, 0.01)
        # Contact invariants and zero-speed lateral translation include neutral
        # uncontrollable modes. A finite terminal hold avoids an invalid DARE.
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
        print("gains ready", np.max(abs(self.gains)), flush=True)

    def matrices(self, t, dt):
        phase = min(np.searchsorted(self.switch, t, side="right"), 2)
        A, B = [
            np.array(z)
            for z in self.fns[phase](
                np.r_[self.spline(t), self.spline(t, 1)], self.torque(t)
            )
        ]
        A -= B @ self.PD
        M = np.zeros((30, 30))
        M[:24, :24] = A
        M[:24, 24:] = B
        E = expm(M * dt)
        return E[:24, :24], E[:24, 24:]

    def export(self, path):
        import yaml

        Path(path).parent.mkdir(parents=True, exist_ok=True)

        last_phase = self.plan["duration"][2]
        handover = self.end - 0.4 * last_phase
        times = np.unique(
            np.r_[np.arange(0, handover, 0.02), self.switch[:2], handover]
        )
        states = np.array([np.r_[self.spline(t), self.spline(t, 1)] for t in times])
        gains = np.array(
            [
                self.gains[min(int(t / 0.01), len(self.grid) - 1)].reshape(-1)
                for t in times
            ]
        )
        data = dict(
            joint_names=p.JOINTS,
            handover_time=float(handover),
            mpc_start_time=float(self.switch[1] + 0.2 * last_phase),
            kp=p.KP.tolist(),
            kd=p.KD.tolist(),
            time=times.tolist(),
            position=states[:, 6:12].tolist(),
            velocity=states[:, 18:24].tolist(),
            effort=np.array([self.torque(t) for t in times]).tolist(),
            state=states.tolist(),
            feedback=gains.tolist(),
            minimum_knee_clearance=0.015,
            clearance_hold_time=0.05,
            wheel_half_width=0.022,
            knee_contacts=[
                dict(
                    frame=name,
                    center=[0.0, -0.17015, 0.0700987],
                    radius=0.05,
                    half_width=0.02685,
                )
                for name in ("left_femur", "right_femur")
            ],
        )

        class Dumper(yaml.SafeDumper):
            pass

        def represent_list(dumper, values):
            return dumper.represent_sequence(
                "tag:yaml.org,2002:seq",
                values,
                flow_style=all(isinstance(v, (int, float)) for v in values),
            )

        Dumper.add_representer(list, represent_list)
        header = (
            "# Offline full-body ID trajectory: knee/wheel contacts, then wheel-only support.\n"
            "# Generated by config/upkie/stand_up_to/plan.py; validate in MuJoCo after regenerating.\n"
        )
        Path(path).write_text(
            header
            + yaml.dump(
                dict(stand_up=data), Dumper=Dumper, sort_keys=False, width=100000
            )
        )
        print("exported", path, len(times), "samples", flush=True)
