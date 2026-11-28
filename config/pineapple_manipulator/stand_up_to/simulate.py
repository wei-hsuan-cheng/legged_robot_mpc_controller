#!/usr/bin/env python3
"""Replay an exported Pineapple stand-up in MuJoCo, as the controller plays it, up to the MPC handover.

    simulate.py [PLAN.yaml]   (default config/pineapple_manipulator/stand_up_trajectory.yaml)

From the seated idle spawn: the servo kp (q* - q) + kd (v* - v) + tau* - K (x - x*) every physics step, the state
from MuJoCo's ground truth (the controller uses its estimate), the arm held at zero. Prints the base height, tilt,
seat clearance and ground contacts at the plan's switch, MPC start and handover, and the peak torque use. An offline
check before the full ROS test; the MPC catch after the handover is not simulated here.
"""

import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rollout as r  # noqa: E402


def seat_clearance(rollout, geometry):
    """The controller's guard: lowest point of the seat disk above the mean tire bottom."""
    d = rollout.d
    seat = geometry["knee_housings"][0]
    rot = d.xmat[rollout.base].reshape(3, 3)
    center = d.xpos[rollout.base] + rot @ np.array(seat["center"])
    az = min(1.0, abs(rot[2, 2]))
    lowest = center[2] - seat["radius"] * np.sqrt(max(0.0, 1 - az * az)) - seat["half_width"] * az
    wheel = geometry["wheel"]
    bottoms = []
    for body in ("wheel_l", "wheel_r"):
        wr = d.xmat[rollout.m.body(body).id].reshape(3, 3)
        axle = wr @ np.array(wheel["axle_axis"])
        bottoms.append(d.xpos[rollout.m.body(body).id][2] - wheel["radius"] * np.sqrt(max(0.0, 1 - axle[2] ** 2))
                       - wheel["half_width"] * abs(axle[2]))
    return lowest - np.mean(bottoms)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("plan", nargs="?", type=Path, default=HERE.parent / "stand_up_trajectory.yaml")
    parser.add_argument("--idle", type=float, default=3.0, help="seated settle before the plan [s]")
    parser.add_argument("--sit-down", action="store_true", help="replay the sit_down map instead, from its first "
                        "state (standing at rest); reports the seat touchdown and the final seated state")
    args = parser.parse_args()
    plan = yaml.safe_load(args.plan.read_text())["sit_down" if args.sit_down else "stand_up"]
    geometry = r.load_geometry()
    times = np.array(plan["time"])
    pos, vel, eff = (np.array(plan[k]) for k in ("position", "velocity", "effort"))
    states, gains = np.array(plan["state"]), np.array(plan["feedback"]).reshape(len(times), 8, -1)
    kp, kd = np.array(plan["kp"]), np.array(plan["kd"])
    rollout = r.Rollout()
    if args.sit_down:
        rollout.place(states[0])
        marks = {"seat_on": plan["seat_contact_time"], "end": plan["handover_time"]}
    else:
        rollout.spawn_idle(args.idle)
        marks = {"seat_off": plan["seat_release_time"], "mpc_start": plan["mpc_start_time"],
                 "handover": plan["handover_time"]}
    start = rollout.d.time
    anchor = rollout.state()
    peak_seat = 0.0
    peak = np.zeros(8)
    max_tilt = 0.0
    reported = set()
    while True:
        t = rollout.d.time - start
        k1 = int(np.clip(np.searchsorted(times, t, side="right"), 1, len(times) - 1))
        k0 = k1 - 1
        a = float(np.clip((t - times[k0]) / (times[k1] - times[k0]), 0, 1))
        x = rollout.state()
        # Anchor XY, heading and the free wheel angles to the plan's start, as the controller does.
        offsets = np.zeros(8)
        offsets[kp == 0] = anchor[6:14][kp == 0] - states[0][6:14][kp == 0]
        x[:2] = states[0][:2] + (x[:2] - anchor[:2])
        x[3] = states[0][3] + (x[3] - anchor[3])
        x[6:14] -= offsets
        ref = (1 - a) * states[k0] + a * states[k1]
        ff = (1 - a) * eff[k0] + a * eff[k1] - ((1 - a) * gains[k0] + a * gains[k1]) @ (x - ref)
        tau = rollout.apply((1 - a) * pos[k0] + a * pos[k1] + offsets, (1 - a) * vel[k0] + a * vel[k1], ff, kp, kd)
        peak = np.maximum(peak, np.abs(tau) / rollout.limits[:8])
        max_tilt = max(max_tilt, rollout.tilt())
        for name, when in marks.items():
            if name not in reported and t >= when:
                reported.add(name)
                print(f"{name:9s} t={t:.2f}s z={rollout.d.qpos[2]:.3f} tilt={rollout.tilt():.3f} "
                      f"pitch={rollout.euler()[1]:+.3f} seat clearance={seat_clearance(rollout, geometry) * 1e3:.1f} mm "
                      f"contacts={ {k: round(v, 1) for k, v in rollout.ground_contacts().items()} }")
        peak_seat = max(peak_seat, rollout.ground_contacts().get("base_link", 0.0))
        if t >= plan["handover_time"] + (1.0 if args.sit_down else 0.0) or rollout.tilt() > 0.8:
            break
    print("max tilt %.3f rad, peak torque use %s, peak seat force %.1f N" % (max_tilt, np.round(peak, 2), peak_seat))
    if args.sit_down:
        print("final (1 s after the plan): z %.3f, pitch %+.3f, speed %.3f, contacts %s" % (
            rollout.d.qpos[2], rollout.euler()[1], np.abs(rollout.d.qvel[:6]).max(),
            {k: round(v, 1) for k, v in rollout.ground_contacts().items()}))
    print("tracking error at handover:", np.round(x - ref, 3)[[2, 4, 5, 7, 8, 11, 12]])
    return 0 if rollout.tilt() < 0.3 else 1


if __name__ == "__main__":
    raise SystemExit(main())
