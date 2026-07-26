#!/bin/bash
# =============================================================================
# Closed-loop InEKF regression test for the G1 centroidal MPC.
#
# The test keeps the robot in stance through the estimator warm-up, then commands
# a flat-ground motion away from the staircase while the InEKF drives the MPC
# observation. VX, VY, and YAW_RATE are normalized commands.
#
#   VERDICT: SUCCESS       - handoff completed, robot stayed upright and walked
#   VERDICT: NO_HANDOFF    - estimator never became the active feedback source
#   VERDICT: NO_ODOM       - GT or estimator odometry was unavailable
#   VERDICT: EST_DIVERGED  - estimator error exceeded the configured safety bound
#   VERDICT: FALL          - GT pelvis height or attitude crossed the fall threshold
#   VERDICT: NO_WALK       - robot remained upright but made insufficient progress
#
# Usage:
#   ros2 run legged_robot_mpc_controller state_estimator_closed_loop_test.sh [log]
# =============================================================================
set -u

LOG="${1:-/tmp/state_estimator_closed_loop_test.log}"
STARTUP_WAIT="${STARTUP_WAIT:-180}"
STANCE_SECONDS="${STANCE_SECONDS:-8.0}"
WALK_SECONDS="${WALK_SECONDS:-12.0}"
SETTLE_SECONDS="${SETTLE_SECONDS:-3.0}"
VX="${VX:--0.15}"
VY="${VY:-0.0}"
YAW_RATE="${YAW_RATE:-0.0}"
PELVIS_HEIGHT="${PELVIS_HEIGHT:-0.7925}"
MIN_TRANSLATIONAL_PROGRESS="${MIN_TRANSLATIONAL_PROGRESS:-0.12}"
MIN_YAW_CHANGE="${MIN_YAW_CHANGE:-0.12}"
FLOATING_BASE_SOURCE="${FLOATING_BASE_SOURCE:-state_estimator}"

if ! command -v ros2 >/dev/null; then
  echo "ERROR: ros2 not found; source ROS 2 and the workspace first." >&2
  exit 2
fi

WS_ROOT="$(dirname "${COLCON_PREFIX_PATH%%:*}")"
cd "$WS_ROOT" || exit 2

LAUNCH_PID=""
cleanup()
{
  if [ -n "$LAUNCH_PID" ]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
  pkill -9 -f 'g1[.]launch' 2>/dev/null || true
  pkill -9 -f '[m]ujoco_ros2_control' 2>/dev/null || true
  pkill -9 -f '[r]obot_state_publisher' 2>/dev/null || true
  pkill -9 -f '[c]ontroller_sequence' 2>/dev/null || true
}
trap cleanup EXIT

cleanup
sleep 3

ros2 launch legged_robot_mpc_controller g1.launch.py \
  mujoco_headless:=true baseCommandGui:=false rviz:=false \
  mpcControllerName:=humanoid_centroidal_mpc_controller \
  floatingBaseSource:="$FLOATING_BASE_SOURCE" > "$LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 "$STARTUP_WAIT"); do
  grep -q "MuJoCo simulation started" "$LOG" && break
  sleep 1
done
if ! grep -q "MuJoCo simulation started" "$LOG"; then
  echo "VERDICT: LAUNCH_FAILED"
  tail -40 "$LOG"
  exit 1
fi

python3 - "$LOG" "$STANCE_SECONDS" "$WALK_SECONDS" "$SETTLE_SECONDS" \
  "$VX" "$VY" "$YAW_RATE" "$PELVIS_HEIGHT" \
  "$MIN_TRANSLATIONAL_PROGRESS" "$MIN_YAW_CHANGE" "$FLOATING_BASE_SOURCE" <<'PYEOF'
import math
import sys
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.node import Node

(
    log_path,
    stance_seconds,
    walk_seconds,
    settle_seconds,
    vx,
    vy,
    yaw_rate,
    pelvis_height,
    min_translational_progress,
    min_yaw_change,
    floating_base_source,
) = sys.argv[1:12]
stance_seconds, walk_seconds, settle_seconds = map(
    float, (stance_seconds, walk_seconds, settle_seconds)
)
vx, vy, yaw_rate, pelvis_height, min_translational_progress, min_yaw_change = map(
    float,
    (
        vx,
        vy,
        yaw_rate,
        pelvis_height,
        min_translational_progress,
        min_yaw_change,
    ),
)


def rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return roll, pitch, yaw


class Probe(Node):
    def __init__(self):
        super().__init__("state_estimator_closed_loop_probe")
        self.gt = None
        self.est = None
        self.gt_history = deque(maxlen=20)
        self.samples = []
        self.fall = False
        self.est_diverged = False
        self.bad_estimate_count = 0
        self.publisher = self.create_publisher(
            WalkingVelocityCommand, "/humanoid/walking_velocity_command", 10
        )
        self.create_subscription(
            Odometry, "/mujoco/ground_truth/odom", self._gt_callback, 30
        )
        self.create_subscription(
            Odometry, "/humanoid/state_estimate/odom", self._est_callback, 30
        )

    def _gt_callback(self, msg):
        self.gt = msg
        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        self.gt_history.append((stamp, msg))
        roll, pitch, _ = rpy(msg.pose.pose.orientation)
        if msg.pose.pose.position.z < 0.5 or abs(roll) > 0.6 or abs(pitch) > 0.6:
            self.fall = True

    def _est_callback(self, msg):
        self.est = msg
        if not self.gt_history:
            return
        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        gt_stamp, gt = min(self.gt_history, key=lambda item: abs(item[0] - stamp))
        if abs(gt_stamp - stamp) > 0.03:
            return
        gp = gt.pose.pose.position
        ep = msg.pose.pose.position
        gr = rpy(gt.pose.pose.orientation)
        er = rpy(msg.pose.pose.orientation)
        gv = gt.twist.twist.linear
        ev = msg.twist.twist.linear
        velocity_errors = (abs(gv.x - ev.x), abs(gv.y - ev.y), abs(gv.z - ev.z))
        velocity_error = math.sqrt(sum(value * value for value in velocity_errors))
        yaw_error = abs((gr[2] - er[2] + math.pi) % (2.0 * math.pi) - math.pi)
        sample = {
            "roll": abs(gr[0] - er[0]),
            "pitch": abs(gr[1] - er[1]),
            "yaw": yaw_error,
            "xy": math.hypot(gp.x - ep.x, gp.y - ep.y),
            "z": abs(gp.z - ep.z),
            "velocity": velocity_error,
            "vx": velocity_errors[0],
            "vy": velocity_errors[1],
            "vz": velocity_errors[2],
        }
        self.samples.append(sample)
        bad = (
            sample["z"] > 0.25
            or sample["velocity"] > 1.0
            or sample["roll"] > 0.5
            or sample["pitch"] > 0.5
        )
        self.bad_estimate_count = self.bad_estimate_count + 1 if bad else 0
        if self.bad_estimate_count >= 25:
            self.est_diverged = True

    def command(self, command_x, command_y, command_yaw):
        msg = WalkingVelocityCommand()
        msg.linear_velocity_x = command_x
        msg.linear_velocity_y = command_y
        msg.desired_pelvis_height = pelvis_height
        msg.angular_velocity_z = command_yaw
        self.publisher.publish(msg)


def run_phase(node, duration, command_x, command_y=0.0, command_yaw=0.0):
    end = time.monotonic() + duration
    while time.monotonic() < end and not node.fall and not node.est_diverged:
        node.command(command_x, command_y, command_yaw)
        rclpy.spin_once(node, timeout_sec=0.02)


rclpy.init()
node = Probe()

run_phase(node, stance_seconds, 0.0)
if node.gt is None or node.est is None:
    print("VERDICT: NO_ODOM")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

with open(log_path, "r", encoding="utf-8", errors="replace") as stream:
    handed_off = "state estimator warm-up complete" in stream.read()
if floating_base_source == "state_estimator" and not handed_off:
    print("VERDICT: NO_HANDOFF")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

start_x = node.gt.pose.pose.position.x
start_y = node.gt.pose.pose.position.y
start_yaw = rpy(node.gt.pose.pose.orientation)[2]
run_phase(node, walk_seconds, vx, vy, yaw_rate)
end_x = node.gt.pose.pose.position.x if node.gt is not None else start_x
end_y = node.gt.pose.pose.position.y if node.gt is not None else start_y
end_yaw = rpy(node.gt.pose.pose.orientation)[2] if node.gt is not None else start_yaw
run_phase(node, settle_seconds, 0.0)

if node.samples:
    mean = lambda key: sum(s[key] for s in node.samples) / len(node.samples)
    maximum = lambda key: max(s[key] for s in node.samples)
    print(
        "ESTIMATE:"
        f" samples={len(node.samples)}"
        f" mean_rp=({math.degrees(mean('roll')):.3f},"
        f"{math.degrees(mean('pitch')):.3f})deg"
        f" max_rp=({math.degrees(maximum('roll')):.3f},"
        f"{math.degrees(maximum('pitch')):.3f})deg"
        f" mean_v={mean('velocity'):.4f}m/s"
        f" max_v={maximum('velocity'):.4f}m/s"
        f" mean_vxyz=({mean('vx'):.4f},{mean('vy'):.4f},{mean('vz'):.4f})m/s"
        f" max_vxyz=({maximum('vx'):.4f},{maximum('vy'):.4f},{maximum('vz'):.4f})m/s"
        f" mean_z={mean('z'):.4f}m"
        f" mean_xy={mean('xy'):.4f}m"
        f" mean_yaw={math.degrees(mean('yaw')):.3f}deg"
    )

displacement_x = end_x - start_x
displacement_y = end_y - start_y
command_norm = math.hypot(vx, vy)
directional_progress = (
    (displacement_x * vx + displacement_y * vy) / command_norm
    if command_norm > 1.0e-9
    else 0.0
)
yaw_change = (end_yaw - start_yaw + math.pi) % (2.0 * math.pi) - math.pi
directional_yaw_change = (
    yaw_change * math.copysign(1.0, yaw_rate) if abs(yaw_rate) > 1.0e-9 else 0.0
)
final_z = node.gt.pose.pose.position.z if node.gt is not None else float("nan")
print(
    f"MOTION: displacement_xy=({displacement_x:.3f},{displacement_y:.3f})"
    f" directional_progress={directional_progress:.3f}"
    f" yaw_change={yaw_change:.3f}rad final_z={final_z:.3f}"
)

if node.est_diverged:
    verdict = "EST_DIVERGED"
elif node.fall:
    verdict = "FALL"
elif command_norm > 1.0e-9 and directional_progress < min_translational_progress:
    verdict = "NO_WALK"
elif command_norm <= 1.0e-9 and abs(yaw_rate) > 1.0e-9 and directional_yaw_change < min_yaw_change:
    verdict = "NO_WALK"
else:
    verdict = "SUCCESS"
print(f"VERDICT: {verdict}")

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if verdict == "SUCCESS" else 1)
PYEOF
RESULT=$?

echo "=== estimator/controller log summary ==="
grep -E "InEKF|state estimator|MPC_TIMING|Increasing to gait|Decreasing to gait" "$LOG" | tail -20
echo "=== warnings/errors ==="
grep -iE "error|exception|nan|rejected|failed" "$LOG" | \
  grep -viE "Parameter|error_gain|CppAdInterface" | tail -10
exit "$RESULT"
