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

# name, duration [s], vx, vy, yaw rate, pelvis height [m]
SEQUENCES: dict[str, list[tuple[str, float, float, float, float, float]]] = {
    "diagnosis": [
        ("settle", 15.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
        ("squat", 20.0, 0.0, 0.0, 0.0, 0.75),
        ("rise", 15.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
        ("walk", 40.0, 0.3, 0.0, 0.0, NOMINAL_HEIGHT),
        ("stop", 20.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
    ],
    # Isolates the height question with no gait in the way.
    "squat": [
        ("settle", 15.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
        ("squat", 25.0, 0.0, 0.0, 0.0, 0.75),
        ("rise", 20.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
    ],
    # Long walk: the phase that generates contact transitions, for NIS statistics
    # and for the height-drift rate (which needs 60 s+ to be readable).
    "walk": [
        ("settle", 15.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
        ("walk", 90.0, 0.3, 0.0, 0.0, NOMINAL_HEIGHT),
        ("stop", 15.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
    ],
    # Stance only, as a control: any estimator drift seen here is not gait-driven.
    "stand": [
        ("stand", 120.0, 0.0, 0.0, 0.0, NOMINAL_HEIGHT),
    ],
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


def run(sequence: list[tuple[str, float, float, float, float, float]],
        settle_before: float) -> None:
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

    for name, duration, vx, vy, yaw_rate, height in sequence:
        phase_start = time.time()
        print(f"[{phase_start - start:7.2f}] {name:<8} "
              f"v=({vx:+.2f}, {vy:+.2f}) yaw={yaw_rate:+.2f} z={height:.4f} "
              f"for {duration:.1f}s", flush=True)
        while time.time() - phase_start < duration:
            node.publish(vx, vy, yaw_rate, height)
            time.sleep(period)

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
            total = sum(p[1] for p in phases)
            print(f"{name} ({total:.0f}s): " +
                  ", ".join(f"{p[0]}[{p[1]:.0f}s]" for p in phases))
        return 0

    run(SEQUENCES[args.sequence], args.settle_before)
    return 0


if __name__ == "__main__":
    sys.exit(main())
