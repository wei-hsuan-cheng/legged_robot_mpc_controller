#!/usr/bin/env python3
"""Plan the Upkie stand-up from the kneeling idle pose; writes stand_up_trajectory.yaml.

    pip install mujoco numpy          # offline tool; not needed at run time
    python3 plan_stand_up.py                      # re-export the stored parameters
    python3 plan_stand_up.py --search 10          # cross-entropy search first, then export

Why a planned push-off. Kneeling (both knee-motor housings and both wheels on the floor, the pose
of initial_pose.yaml) the whole-body CoM is 5-16 cm ahead of the wheel contacts for every
hip/knee configuration that keeps the knees down, with the knees carrying 29-93% of the weight
(checked over the reachable joint ranges). The CoM can therefore not be moved over the wheels
quasi-statically, and the wheel torque alone (2 x 1.7 N m) cannot lift the knees: the wheels
just drive the robot forward on its knees. As in Ascento's "apply torque" phase and Upkie's own
getting-up motion (open loop until just before the body leaves the floor, then the MPC), the
stand-up is a short dynamic push:

  1. fold: hip/knee to (hip1, knee1) over T_fold, wheels free - the knees fully folded bring the
     CoM closest to the wheels (about 5 cm);
  2. push: hip/knee to (hip2, knee2) over T_push with a forward wheel torque - the hips extend,
     pressing the knee housings into the floor, and the body pivots back about the wheel axle.

The parameters minimise the pendulum's divergent component at knee lift-off,
xi = (x_com - x_wheel) + (v_com - v_wheel) / omega, omega = sqrt(g / z_com): with xi ~ 0 the
body coasts towards the CoM-over-wheel posture by itself and the MPC, switched on at lift-off
(handover_time), only has to correct the rest. Every sample is a MuJoCo rollout of the project
MJCF with the playback's own servo (PD gains kp/kd plus feed-forward), torques clipped at the
motor limits, so the trajectory is dynamically feasible in that model by construction.

The exported samples are the rollout's measured joint positions/velocities and the total torque
it applied. Played back with the same gains from the same idle state, the servo's PD terms are
zero and the applied torque is the planned one; a deviating replay gets PD correction around
the planned motion.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
SCENE = HERE.parents[1] / "description" / "mujoco" / "robots" / "upkie" / "scene.xml"
OUT = HERE / "stand_up_trajectory.yaml"

JOINTS = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
LIMITS = np.array([16.0, 16.0, 1.7, 16.0, 16.0, 1.7])
KP = np.array([20.0, 20.0, 0.0, 20.0, 20.0, 0.0])
KD = np.array([0.5, 0.5, 0.05, 0.5, 0.5, 0.05])
IDLE = (-1.3, -2.4)  # kneeling idle pose (left hip, left knee); right = -left
G = 9.81
SAMPLE_DT = 0.005

# [hip1, knee1, T_fold, hip2, knee2, T_push, wheel torque]
PARAMETERS = [-0.5042924447052395, -2.6867604846496587, 0.5333189637279592,
              -0.20721165891497184, -2.195829021770047, 0.3765912289040746, 1.361214009259519]
LOWER = np.array([-1.6, -2.7, 0.4, -1.6, -2.7, 0.08, -1.7])
UPPER = np.array([0.2, -1.2, 1.5, 0.6, -0.6, 0.5, 1.7])


def mirror(hip, knee, wheel=0.0):
    return np.array([hip, knee, wheel, -hip, -knee, -wheel])


class Rollout:
    def __init__(self, scene=SCENE):
        self.m = mujoco.MjModel.from_xml_path(str(scene))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.act = [m.actuator(n).id for n in JOINTS]
        self.qa = [m.joint(n).qposadr[0] for n in JOINTS]
        self.va = [m.joint(n).dofadr[0] for n in JOINTS]
        self.floor = m.geom("floor").id
        self.knees = [m.geom(n).id for n in ("left_femur_collision1", "left_femur_collision4",
                                             "right_femur_collision1", "right_femur_collision4")]
        self.tire = m.geom("left_foot_collision").id

    def joints(self):
        return (np.array([self.d.qpos[a] for a in self.qa]), np.array([self.d.qvel[a] for a in self.va]))

    def apply(self, q_des, v_des, feedforward):
        q, v = self.joints()
        tau = np.clip(KP * (q_des - q) + KD * (v_des - v) + feedforward, -LIMITS, LIMITS)
        self.d.ctrl[self.act] = tau
        mujoco.mj_step(self.m, self.d)
        return tau

    def spawn_idle(self, settle=3.0):
        """Upright spawn with the idle joints, 1 cm above the floor; the robot tips onto its knees."""
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        q_idle = mirror(*IDLE)
        for i, a in enumerate(self.qa):
            d.qpos[a] = q_idle[i]
        mujoco.mj_forward(m, d)
        lowest = min(self._lowest_point(g) for g in range(m.ngeom) if g != self.floor and m.geom_contype[g])
        d.qpos[2] = -lowest + 0.01
        spawn_height = float(d.qpos[2])
        while d.time < settle:
            self.apply(q_idle, np.zeros(6), np.zeros(6))
        return spawn_height

    def _lowest_point(self, geom):
        # conservative: the geom's bounding sphere
        return self.d.geom_xpos[geom][2] - self.m.geom_rbound[geom]

    def knee_force(self):
        total = 0.0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if self.floor in (c.geom1, c.geom2) and (c.geom1 in self.knees or c.geom2 in self.knees):
                f = np.zeros(6)
                mujoco.mj_contactForce(self.m, self.d, i, f)
                total += f[0]
        return total

    def divergent_component(self):
        com = self.d.subtree_com[1]
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_GEOM, self.tire, vel, 0)
        omega = math.sqrt(G / max(com[2], 0.05))
        offset = com[0] - self.d.geom_xpos[self.tire][0]
        relative_velocity = self.d.subtree_linvel[1][0] - vel[3]
        return offset + relative_velocity / omega, offset, relative_velocity

    def pitch(self):
        w, x, y, z = self.d.qpos[3:7]
        return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))

    def stand_up(self, p, record=False, liftoff_hold=0.02, after=0.6):
        """Fold + push from the settled idle pose; returns the lift-off state and the samples."""
        hip1, knee1, t_fold, hip2, knee2, t_push, wheel_torque = p
        self.spawn_idle()
        d = self.d
        t0 = d.time
        q0, _ = self.joints()
        q1, q2 = mirror(hip1, knee1), mirror(hip2, knee2)
        lift, free_time, saturation = None, 0.0, 0.0
        samples, next_sample = [], 0.0
        while d.time - t0 < t_fold + t_push + after:
            t = d.time - t0
            if t < t_fold:
                s = t / t_fold
                q_des = q0 + (s * s * (3 - 2 * s)) * (q1 - q0)
                v_des = (6 * s * (1 - s) / t_fold) * (q1 - q0)
                wheel = 0.0
            else:
                s = min((t - t_fold) / t_push, 1.0)
                q_des = q1 + (s * s * (3 - 2 * s)) * (q2 - q1)
                v_des = (6 * s * (1 - s) / t_push) * (q2 - q1) if s < 1.0 else np.zeros(6)
                wheel = wheel_torque
            feedforward = np.array([0.0, 0.0, wheel, 0.0, 0.0, -wheel])
            q, v = self.joints()
            raw = KP * (q_des - q) + KD * (v_des - v) + feedforward
            saturation = max(saturation, float(np.max(np.abs(raw) / LIMITS)))
            if record and t >= next_sample - 1e-9:
                samples.append((t, q.copy(), v.copy()))
                next_sample += SAMPLE_DT
            tau = self.apply(q_des, v_des, feedforward)
            if record and len(samples) and len(samples[-1]) == 3:
                samples[-1] = samples[-1] + (tau.copy(),)
            if t > t_fold:
                free_time = free_time + self.m.opt.timestep if self.knee_force() < 0.1 else 0.0
                if lift is None and free_time >= liftoff_hold:
                    xi, offset, relative_velocity = self.divergent_component()
                    lift = dict(t=d.time - t0, xi=xi, com_offset=offset, relative_velocity=relative_velocity,
                                pitch=self.pitch(), pitch_rate=float(d.qvel[4]),
                                com_height=float(d.subtree_com[1][2]), base_height=float(d.qpos[2]))
                    if record:
                        break
        return lift, saturation, samples


def cost(rollout, p):
    lift, saturation, _ = rollout.stand_up(p)
    if lift is None:
        return 10.0, None
    return abs(lift["xi"]) + 0.02 * abs(lift["pitch_rate"]) + 2.0 * max(0.0, saturation - 1.0), lift


def search(rollout, iterations, population=32, elites=8, seed=1):
    rng = np.random.default_rng(seed)
    mean, std = np.array(PARAMETERS), (UPPER - LOWER) / 8
    best = (math.inf, mean, None)
    for iteration in range(iterations):
        population_p = np.clip(mean + std * rng.standard_normal((population, len(mean))), LOWER, UPPER)
        scored = sorted(((*cost(rollout, p), p) for p in population_p), key=lambda s: s[0])
        if scored[0][0] < best[0]:
            best = (scored[0][0], scored[0][2], scored[0][1])
        elite = np.array([s[2] for s in scored[:elites]])
        mean, std = elite.mean(axis=0), elite.std(axis=0) + 1e-3
        print(f"iteration {iteration}: best cost {best[0]:.4f}", flush=True)
    return best[1]


def export(rollout, p, path=OUT):
    lift, saturation, samples = rollout.stand_up(p, record=True)
    if lift is None:
        raise RuntimeError("the knees never left the floor: no handover point")
    rows = [s for s in samples if len(s) == 4]
    # Lift-off can fall between the 5 ms samples. Include its post-step state so
    # the controller's handover timestamp lies within the exported trajectory.
    if rows[-1][0] < lift["t"] - 1e-9:
        q, v = rollout.joints()
        rows.append((lift["t"], q.copy(), v.copy(), rollout.d.ctrl[rollout.act].copy()))
    spawn = rollout.spawn_idle(settle=0.0)
    lines = [
        "# Upkie stand-up from the kneeling idle pose, planned by plan_stand_up.py; do not edit by hand.",
        f"# Parameters [hip1, knee1, T_fold, hip2, knee2, T_push, wheel torque]: {json.dumps([round(x, 6) for x in p])}",
        f"# Knee lift-off (handover) at {lift['t']:.3f} s: CoM {lift['com_offset']:+.3f} m from the wheel contact, "
        f"moving {lift['relative_velocity']:+.3f} m/s, xi {lift['xi']:+.3f} m, pitch {lift['pitch']:+.3f} rad, "
        f"pitch rate {lift['pitch_rate']:+.3f} rad/s, CoM height {lift['com_height']:.3f} m.",
        f"# Largest servo command / limit: {saturation:.2f}. Idle spawn height (initial_pose.yaml): {spawn:.4f} m.",
        "stand_up:",
        f"  joint_names: [{', '.join(JOINTS)}]",
        f"  handover_time: {lift['t']:.4f}",
        f"  kp: [{', '.join(f'{x:g}' for x in KP)}]",
        f"  kd: [{', '.join(f'{x:g}' for x in KD)}]",
        f"  time: [{', '.join(f'{t:.4f}' for t, *_ in rows)}]",
        "  position:",
        *[f"    - [{', '.join(f'{x:.6f}' for x in q)}]" for _, q, _, _ in rows],
        "  velocity:",
        *[f"    - [{', '.join(f'{x:.5f}' for x in v)}]" for _, _, v, _ in rows],
        "  effort:",
        *[f"    - [{', '.join(f'{x:.5f}' for x in tau)}]" for *_, tau in rows],
    ]
    Path(path).write_text("\n".join(lines) + "\n")
    print(f"wrote {path}: {len(rows)} samples, handover {lift['t']:.3f} s, xi {lift['xi']:+.4f} m, "
          f"saturation {saturation:.2f}, idle spawn height {spawn:.4f} m")
    return lift


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--search", type=int, default=0, help="Cross-entropy iterations before exporting")
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()
    rollout = Rollout(args.scene)
    p = search(rollout, args.search) if args.search > 0 else np.array(PARAMETERS)
    print("parameters:", json.dumps([float(x) for x in p]))
    export(rollout, p, args.output)


if __name__ == "__main__":
    main()
