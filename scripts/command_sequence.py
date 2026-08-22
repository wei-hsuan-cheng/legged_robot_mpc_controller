#!/usr/bin/env python3
"""Drive the humanoid through a scripted command sequence for a diagnosis run.

The base-command GUI is fine for exploring, but a tuning run has to be
repeatable: two logs are only comparable if the robot was asked to do the same
thing at the same times. This publishes the same WalkingVelocityCommand topic the
GUI does, on a fixed schedule, and prints each phase boundary so the timestamps
can be matched against the diagnostics CSV.

Usage:
    python3 command_sequence.py                 # the default diagnosis sequence
    python3 command_sequence.py --sequence squat
    python3 command_sequence.py --list

The default sequence is built around the questions Phase 0 exists to answer:

  settle   stance at the nominal height, so the estimator warm-up and hand-off
           are visible in isolation before anything moves.
  squat    a step change to a lower pelvis height. This is the height-command
           test: if the pelvis does not follow, the per-cost-term log says
           whether the leg-torque term is outbidding base-z tracking.
  rise     back to nominal, to see whether the height error is symmetric (a cost
           imbalance is; a reference bug usually is not).
  walk     forward walking, which is the only phase that produces contact
           transitions - and therefore the only phase in which the filter
           consistency (NIS) numbers mean anything at all. A standing robot
           never breaks contact, so its NIS is trivially near zero.
  stop     back to stance, to check the estimator settles rather than ringing.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ocs2_msgs.msg import WalkingVelocityCommand
except ImportError as exc:  # pragma: no cover
    sys.exit(f"needs a sourced ROS 2 environment with ocs2_msgs: {exc}")


NOMINAL_HEIGHT = 0.7925
PUBLISH_HZ = 50.0


class Phase:
    """One commanded segment.

    `ramp` linearly interpolates from the previous phase's command to this one
    over the phase duration, instead of stepping. This matters: a step from
    standing to a walking velocity asks the MPC to acquire the whole momentum in
    one horizon, and a robot that falls on the step may be perfectly stable on a
    ramp. Distinguishing "cannot walk at this speed" from "cannot accept this
    acceleration" needs both to be runnable.
    """

    def __init__(self, name: str, duration: float, vx: float = 0.0, vy: float = 0.0,
                 yaw_rate: float = 0.0, height: float = NOMINAL_HEIGHT,
                 ramp: bool = False) -> None:
        self.name = name
        self.duration = duration
        self.vx = vx
        self.vy = vy
        self.yaw_rate = yaw_rate
        self.height = height
        self.ramp = ramp

    def command(self, alpha: float, previous: "Phase | None") -> tuple[float, float, float, float]:
        target = (self.vx, self.vy, self.yaw_rate, self.height)
        if not self.ramp or previous is None:
            return target
        start = (previous.vx, previous.vy, previous.yaw_rate, previous.height)
        return tuple(s + (e - s) * alpha for s, e in zip(start, target))  # type: ignore[return-value]


SEQUENCES: dict[str, list[Phase]] = {
    # Full sweep: height first (no gait in the way), then gait.
    "diagnosis": [
        Phase("settle", 15.0),
        Phase("squat", 20.0, height=0.75),
        Phase("rise", 15.0),
        Phase("walk", 40.0, vx=0.3),
        Phase("stop", 20.0),
    ],
    # Isolates the height question with no gait in the way.
    "squat": [
        Phase("settle", 15.0),
        Phase("squat", 25.0, height=0.75),
        Phase("rise", 20.0),
    ],
    # Height sweep, to see whether tracking error grows smoothly with the
    # commanded drop or hits a wall at some depth.
    "height_sweep": [
        Phase("settle", 10.0),
        Phase("z_770", 12.0, height=0.770),
        Phase("z_750", 12.0, height=0.750),
        Phase("z_720", 12.0, height=0.720),
        Phase("z_690", 12.0, height=0.690),
        Phase("rise", 12.0),
    ],
    # Speed ladder with ramps between steps: finds the speed at which balance is
    # lost, rather than only showing that one particular speed fails.
    "speed_ladder": [
        Phase("settle", 12.0),
        Phase("ramp_010", 5.0, vx=0.10, ramp=True),
        Phase("hold_010", 15.0, vx=0.10),
        Phase("ramp_020", 5.0, vx=0.20, ramp=True),
        Phase("hold_020", 15.0, vx=0.20),
        Phase("ramp_030", 5.0, vx=0.30, ramp=True),
        Phase("hold_030", 15.0, vx=0.30),
        Phase("ramp_stop", 5.0, vx=0.0, ramp=True),
        Phase("stop", 10.0),
    ],
    # The same 0.3 m/s that fails as a step, reached as a ramp instead.
    "walk_ramp": [
        Phase("settle", 12.0),
        Phase("ramp", 12.0, vx=0.3, ramp=True),
        Phase("hold", 45.0, vx=0.3),
        Phase("ramp_stop", 6.0, vx=0.0, ramp=True),
        Phase("stop", 12.0),
    ],
    # Slow steady walk, long enough for the height-drift rate to be readable.
    "walk_slow": [
        Phase("settle", 12.0),
        Phase("ramp", 6.0, vx=0.1, ramp=True),
        Phase("hold", 90.0, vx=0.1),
        Phase("ramp_stop", 4.0, vx=0.0, ramp=True),
        Phase("stop", 12.0),
    ],
    # Short bursts: does it survive a few steps and settle, or does each burst
    # leave it worse off than the last?
    "bursts": [
        Phase("settle", 12.0),
        Phase("burst_1", 4.0, vx=0.15),
        Phase("rest_1", 8.0),
        Phase("burst_2", 4.0, vx=0.15),
        Phase("rest_2", 8.0),
        Phase("burst_3", 6.0, vx=0.2),
        Phase("rest_3", 10.0),
    ],
    # Turning and lateral motion, which load the yaw/roll axes the forward walk
    # does not exercise.
    "turn_strafe": [
        Phase("settle", 12.0),
        Phase("ramp_yaw", 5.0, yaw_rate=0.3, ramp=True),
        Phase("yaw", 15.0, yaw_rate=0.3),
        Phase("rest", 8.0),
        Phase("ramp_vy", 5.0, vy=0.15, ramp=True),
        Phase("strafe", 15.0, vy=0.15),
        Phase("stop", 10.0),
    ],
    # Stance only, as a control: any estimator drift seen here is not gait-driven.
    "stand": [Phase("stand", 120.0)],
}


class CommandSequencer(Node):
    def __init__(self) -> None:
        super().__init__("command_sequence")
        # Match the GUI's QoS exactly, or the controller may not receive these.
        qos = QoSProfile(depth=25, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(
            WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos)

    def publish(self, vx: float, vy: float, yaw_rate: float, height: float) -> None:
        message = WalkingVelocityCommand()
        # Same clamps the GUI applies, so a scripted run cannot command something
        # the interactive one could not.
        message.linear_velocity_x = float(max(-1.0, min(1.0, vx)))
        message.linear_velocity_y = float(max(-1.0, min(1.0, vy)))
        message.angular_velocity_z = float(max(-1.0, min(1.0, yaw_rate)))
        message.desired_pelvis_height = float(max(0.2, min(1.0, height)))
        self._publisher.publish(message)


def run(sequence: list[Phase], settle_before: float) -> None:
    rclpy.init()
    node = CommandSequencer()
    period = 1.0 / PUBLISH_HZ

    start = time.time()
    if settle_before > 0.0:
        # Hold the nominal command while the controller activates, so the first
        # scripted phase starts from a known state rather than mid-startup.
        print(f"[{0.0:7.2f}] waiting {settle_before:.1f}s for the controller", flush=True)
        while time.time() - start < settle_before:
            node.publish(0.0, 0.0, 0.0, NOMINAL_HEIGHT)
            time.sleep(period)

    previous: Phase | None = None
    for phase in sequence:
        phase_start = time.time()
        kind = "ramp to" if phase.ramp else "hold   "
        print(f"[{phase_start - start:7.2f}] {phase.name:<10} {kind} "
              f"v=({phase.vx:+.2f}, {phase.vy:+.2f}) yaw={phase.yaw_rate:+.2f} "
              f"z={phase.height:.4f} for {phase.duration:.1f}s", flush=True)
        while True:
            elapsed = time.time() - phase_start
            if elapsed >= phase.duration:
                break
            alpha = min(1.0, elapsed / phase.duration) if phase.duration > 0 else 1.0
            vx, vy, yaw_rate, height = phase.command(alpha, previous)
            node.publish(vx, vy, yaw_rate, height)
            time.sleep(period)
        previous = phase

    print(f"[{time.time() - start:7.2f}] done", flush=True)
    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sequence", default="diagnosis", choices=sorted(SEQUENCES))
    parser.add_argument("--settle-before", type=float, default=8.0,
                        help="seconds to hold the nominal command before the first "
                             "phase, covering controller activation and the "
                             "estimator warm-up window")
    parser.add_argument("--list", action="store_true", help="print the sequences and exit")
    args = parser.parse_args()

    if args.list:
        for name, phases in SEQUENCES.items():
            total = sum(p.duration for p in phases)
            print(f"{name} ({total:.0f}s): " +
                  ", ".join(f"{p.name}[{p.duration:.0f}s]" for p in phases))
        return 0

    run(SEQUENCES[args.sequence], args.settle_before)
    return 0


if __name__ == "__main__":
    sys.exit(main())
