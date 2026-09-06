#!/usr/bin/env python3
"""Drive the humanoid around a figure-eight with a sinusoidal pelvis height.

Why a figure-eight rather than another straight-line ladder: it exercises forward
speed, both turn directions, and the sign reversal of curvature at the crossing,
continuously and without ever leaving a small patch of floor. The velocity ladders
answer "how fast", but they only ask for one thing at a time and they walk the
robot several metres away; this asks for coupled vx and omega that never stop
changing, which is much closer to how the machine actually gets used.

Geometry (Gerono lemniscate), parameterised by a phase s:

    x(s) = (lx/2) * sin(s)
    y(s) = (ly/2) * sin(2s)

spanning lx by ly and crossing itself at the origin twice per lap.

CONSTANT SPEED, not constant phase rate. Advancing s linearly in time makes the
commanded forward speed vary around the lap, and for this curve the variation
cannot be tuned away: the speed ratio is 16(r+1)/(r(8-r)) under the square root
with r = (lx/2)^2/ly^2, which is minimised at r = 2 and still equals exactly 2.0.
A 2:1 spread means asking for a 0.2 m/s minimum forces a 0.4 m/s maximum, above
what the machine currently sustains. Integrating s by arc length instead holds
|v| fixed, so the speed requirement is met everywhere by construction and the
only thing that varies is the yaw rate.

The commanded twist is expressed in the PELVIS frame - the controller integrates
it with integrateBodyTwistTargetBasePose() and rotates it to world by the target
yaw. Because the heading is defined tangential to the path, the lateral component
is identically zero:

    vx = speed (constant),  vy = 0,  omega = speed * curvature(s)

Height is an independent sinusoid between zmin and zmax, deliberately not
commensurate with the lap period unless you make it so.

Usage:
    python3 figure_eight_command.py --dry-run          # print the envelope only
    python3 figure_eight_command.py                    # defaults, 2 laps
    python3 figure_eight_command.py --speed 0.25 --lx 1.4 --ly 0.5
"""

from __future__ import annotations

import argparse
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ocs2_msgs.msg import WalkingVelocityCommand
    _IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover
    rclpy = None
    _IMPORT_ERROR = exc
    # Keeps --dry-run usable off-robot, where the geometry is all you want to
    # check. Publisher is defined but never instantiated on this path.
    Node = object


PUBLISH_HZ = 50.0

# Envelope measured on the flat scene with the tuned foot placement.
SAFE_VX = 0.30
SAFE_YAW_RATE = 0.50


class FigureEight:
    """Gerono lemniscate traversed at constant speed with a tangential heading."""

    def __init__(self, lx: float, ly: float) -> None:
        if lx <= 0.0 or ly <= 0.0:
            raise ValueError("lx and ly must be positive")
        self.lx = lx
        self.ly = ly

    def position(self, s: float) -> tuple[float, float]:
        return (0.5 * self.lx * math.sin(s), 0.5 * self.ly * math.sin(2.0 * s))

    def _derivatives(self, s: float) -> tuple[float, float, float, float]:
        """(dx/ds, dy/ds, d2x/ds2, d2y/ds2)."""
        ax, ay = 0.5 * self.lx, 0.5 * self.ly
        return (ax * math.cos(s),
                2.0 * ay * math.cos(2.0 * s),
                -ax * math.sin(s),
                -4.0 * ay * math.sin(2.0 * s))

    def speed_per_phase(self, s: float) -> float:
        """|dp/ds| - how much arc length one radian of phase buys."""
        dx, dy, _, _ = self._derivatives(s)
        return math.hypot(dx, dy)

    def curvature(self, s: float) -> float:
        dx, dy, ddx, ddy = self._derivatives(s)
        denom = (dx * dx + dy * dy) ** 1.5
        # The curve has no cusp, so denom is bounded away from zero for any
        # positive lx, ly; the guard is for degenerate parameters only.
        return (dx * ddy - dy * ddx) / denom if denom > 1e-12 else 0.0

    def path_length(self, samples: int = 20000) -> float:
        step = 2.0 * math.pi / samples
        return sum(self.speed_per_phase(i * step) for i in range(samples)) * step

    def envelope(self, speed: float, samples: int = 4000) -> dict:
        step = 2.0 * math.pi / samples
        curvatures = [abs(self.curvature(i * step)) for i in range(samples)]
        length = self.path_length()
        return {
            "path_length": length,
            "lap_time": length / speed,
            "yaw_max": speed * max(curvatures),
            "yaw_mean": speed * sum(curvatures) / len(curvatures),
            "radius_min": 1.0 / max(curvatures) if max(curvatures) > 0 else float("inf"),
        }


def height_at(t: float, zmin: float, zmax: float, period: float, phase: float) -> float:
    mid = 0.5 * (zmin + zmax)
    amplitude = 0.5 * (zmax - zmin)
    return mid + amplitude * math.sin(2.0 * math.pi * t / period + phase)


class Publisher(Node):
    def __init__(self) -> None:
        super().__init__("figure_eight_command")
        # Match the GUI's QoS exactly or the controller may not receive these.
        qos = QoSProfile(depth=25, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(
            WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos)

    def publish(self, vx: float, vy: float, yaw_rate: float, height: float) -> None:
        message = WalkingVelocityCommand()
        message.linear_velocity_x = float(max(-1.0, min(1.0, vx)))
        message.linear_velocity_y = float(max(-1.0, min(1.0, vy)))
        message.angular_velocity_z = float(max(-1.0, min(1.0, yaw_rate)))
        message.desired_pelvis_height = float(max(0.2, min(1.0, height)))
        self._publisher.publish(message)


def describe(curve: FigureEight, args) -> dict:
    env = curve.envelope(args.speed)
    print(f"figure-eight  lx={curve.lx:.2f} m  ly={curve.ly:.2f} m  "
          f"speed={args.speed:.3f} m/s  laps={args.laps:.2f}")
    print(f"  path length per lap : {env['path_length']:.2f} m")
    print(f"  lap time            : {env['lap_time']:.1f} s  "
          f"(total {args.laps * env['lap_time']:.1f} s)")
    print(f"  forward speed       : {args.speed:.3f} m/s constant")
    print(f"  yaw rate            : mean {env['yaw_mean']:.3f}  max {env['yaw_max']:.3f} rad/s")
    print(f"  tightest turn radius: {env['radius_min']:.3f} m")
    print(f"  pelvis height       : {args.zmin:.3f} -> {args.zmax:.3f} m, "
          f"period {args.zperiod:.1f} s")
    warn = []
    if args.speed > SAFE_VX:
        warn.append(f"speed {args.speed:.3f} > {SAFE_VX} m/s")
    if env["yaw_max"] > SAFE_YAW_RATE:
        warn.append(f"peak yaw rate {env['yaw_max']:.3f} > {SAFE_YAW_RATE} rad/s")
    if warn:
        print("  NOTE: outside the measured-safe envelope (" + "; ".join(warn) + ").")
        print("        Widen --lx/--ly to reduce curvature, or lower --speed.")
    return env


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # Figure-eight geometry and speed
    parser.add_argument("--lx", type=float, default=4.0, help="path extent along x [m]")
    parser.add_argument("--ly", type=float, default=1.6, help="path extent along y [m]")
    parser.add_argument("--speed", type=float, default=0.22,
                        help="constant forward speed [m/s]; held everywhere on the lap. "
                             "Keep at or above 0.2 - below that is no longer of interest.")
    parser.add_argument("--laps", type=float, default=2.0, help="number of laps to run")
    # Height sinusoid
    parser.add_argument("--zmin", type=float, default=0.77, help="minimum pelvis height [m]")
    parser.add_argument("--zmax", type=float, default=0.80, help="maximum pelvis height [m]")
    parser.add_argument("--zperiod", type=float, default=13.0, help="height period [s]")
    parser.add_argument("--zphase", type=float, default=0.0, help="height phase [rad]")
    # Run shaping
    parser.add_argument("--settle", type=float, default=8.0,
                        help="seconds of stance at the mean height before starting")
    parser.add_argument("--ramp", type=float, default=5.0,
                        help="seconds to ramp the twist in from zero, so the MPC is not "
                             "asked to acquire the whole momentum in one horizon")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the commanded envelope and exit without publishing")
    args = parser.parse_args()

    if args.zmin > args.zmax:
        return "zmin must not exceed zmax"
    if args.speed <= 0.0:
        return "speed must be positive"

    curve = FigureEight(args.lx, args.ly)
    env = describe(curve, args)

    if args.dry_run:
        return 0
    if rclpy is None:
        return f"needs a sourced ROS 2 environment with ocs2_msgs: {_IMPORT_ERROR}"

    rclpy.init()
    node = Publisher()
    dt = 1.0 / PUBLISH_HZ
    mid_height = 0.5 * (args.zmin + args.zmax)

    start = time.time()
    if args.settle > 0.0:
        print(f"[{0.0:7.2f}] settle {args.settle:.1f}s at z={mid_height:.4f}", flush=True)
        while time.time() - start < args.settle:
            node.publish(0.0, 0.0, 0.0, mid_height)
            time.sleep(dt)

    total = args.laps * env["lap_time"]
    print(f"[{args.settle:7.2f}] figure-eight for {total:.1f}s "
          f"({args.laps:.2f} laps)", flush=True)

    traj_start = time.time()
    phase = 0.0
    previous = traj_start
    next_report = 0.0
    while True:
        now = time.time()
        elapsed = now - traj_start
        if elapsed >= total:
            break
        step_dt = now - previous
        previous = now

        # Ramp the twist, not the height: the height command is already smooth and
        # starts at the mid value the settle phase held.
        scale = min(1.0, elapsed / args.ramp) if args.ramp > 0.0 else 1.0
        commanded_speed = scale * args.speed

        # Arc-length advance: ds = v dt / |dp/ds|. This is what keeps |v| constant
        # around the lap instead of the phase rate.
        per_phase = curve.speed_per_phase(phase)
        if per_phase > 1e-9:
            phase += commanded_speed * step_dt / per_phase
        phase %= 2.0 * math.pi

        yaw_rate = commanded_speed * curve.curvature(phase)
        height = height_at(elapsed, args.zmin, args.zmax, args.zperiod, args.zphase)
        node.publish(commanded_speed, 0.0, yaw_rate, height)

        if elapsed >= next_report:
            x, y = curve.position(phase)
            print(f"[{args.settle + elapsed:7.2f}] pos=({x:+.2f},{y:+.2f}) "
                  f"vx={commanded_speed:+.3f} w={yaw_rate:+.3f} z={height:.4f}", flush=True)
            next_report += 5.0
        time.sleep(dt)

    print(f"[{args.settle + total:7.2f}] settling", flush=True)
    settle_end = time.time()
    while time.time() - settle_end < 8.0:
        node.publish(0.0, 0.0, 0.0, mid_height)
        time.sleep(dt)

    print(f"[{time.time() - start:7.2f}] done", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
