#!/bin/bash
# =============================================================================
# Autonomous stair climbing test for the G1 centroidal MPC.
#
# Launches the MuJoCo simulation headless, triggers the fixed-sequence stair
# climb ("stair_climb" target mode), monitors the pelvis ground-truth odometry
# and the foot TF frames, and prints a VERDICT line:
#
#   VERDICT: SUCCESS     - pelvis reached the top of the staircase, no fall
#   VERDICT: INCOMPLETE  - no fall, but the top was not reached in time
#   VERDICT: FALL        - pelvis dropped or tipped over
#   VERDICT: NO_ODOM     - no ground-truth odometry received
#
# Exit code: 0 only on SUCCESS (usable in CI-style automation).
#
# Usage (workspace must be sourced; run from anywhere):
#   ros2 run legged_robot_mpc_controller stair_climbing_test.sh [log] [monitor_s] [startup_wait_s]
# or directly:
#   ./tests/stair_climbing_test.sh /tmp/stair_test.log 45 90
#
# Success thresholds match the default staircase in config/g1/stair_climbing.yaml
# (5 x 0.10 m risers, 0.30 m treads, base at x=0.75). Override for a different
# staircase with env vars EXPECT_MIN_X / EXPECT_MIN_Z.
# =============================================================================
set -u

LOG="${1:-/tmp/stair_climbing_test.log}"
MONITOR="${2:-45}"        # seconds to monitor after triggering the climb
STARTUP_WAIT="${3:-90}"   # seconds to wait for the simulation to come up
EXPECT_MIN_X="${EXPECT_MIN_X:-1.85}"
EXPECT_MIN_Z="${EXPECT_MIN_Z:-1.15}"

if ! command -v ros2 >/dev/null; then
  echo "ERROR: ros2 not found - source your ROS 2 + workspace setup first." >&2
  exit 2
fi

# The launch resolves libFolder (CppAD codegen cache) relative to the CWD, so run
# from the workspace root (parent of install/), derived from COLCON_PREFIX_PATH.
WS_ROOT="$(dirname "${COLCON_PREFIX_PATH%%:*}")"
if [ ! -d "$WS_ROOT" ]; then
  echo "ERROR: cannot derive workspace root from COLCON_PREFIX_PATH." >&2
  exit 2
fi
cd "$WS_ROOT" || exit 2

# [.] avoids pkill matching wrapper shells that carry this pattern in argv.
pkill -f 'g1[.]launch' 2>/dev/null
pkill -f mujoco_ros2_control 2>/dev/null
sleep 2

ros2 launch legged_robot_mpc_controller g1.launch.py \
  mujoco_headless:=true velocityCommandGui:=false rviz:=false \
  mpcControllerName:=humanoid_centroidal_mpc_controller > "$LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 "$STARTUP_WAIT"); do
  grep -q "MuJoCo simulation started" "$LOG" && break
  sleep 1
done
if ! grep -q "MuJoCo simulation started" "$LOG"; then
  echo "=== LAUNCH FAILED, last lines:"
  tail -40 "$LOG"
  kill "$LAUNCH_PID" 2>/dev/null
  exit 1
fi
if ! grep -q "stair climbing config loaded" "$LOG"; then
  echo "WARNING: controller did not report a stair climbing config (stairClimbingFile empty?)"
fi
echo "=== launched OK, standing for 5 s"
sleep 5

echo "=== triggering stair climb"
ros2 topic pub --once /humanoid/target_mode std_msgs/msg/String "{data: stair_climb}" > /dev/null 2>&1

echo "=== monitoring pelvis GT odom for $MONITOR s"
python3 - "$MONITOR" "$EXPECT_MIN_X" "$EXPECT_MIN_Z" <<'PYEOF'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

monitor_s = float(sys.argv[1])
expect_min_x = float(sys.argv[2])
expect_min_z = float(sys.argv[3])

rclpy.init()
node = Node("stair_climbing_probe")
state = {"last_print": 0.0, "fall": False, "final": None, "max_x": -1e9, "max_z": -1e9}

def cb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    sinr = 2 * (q.w * q.x + q.y * q.z)
    cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    state["final"] = (p.x, p.y, p.z, roll, pitch, yaw)
    state["max_x"] = max(state["max_x"], p.x)
    state["max_z"] = max(state["max_z"], p.z)
    if p.z < 0.5 or abs(roll) > 0.6 or abs(pitch) > 0.6:
        state["fall"] = True
    now = time.time()
    if now - state["last_print"] > 1.5:
        state["last_print"] = now
        print(f"t={m.header.stamp.sec}.{m.header.stamp.nanosec // 10**8}"
              f" x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
              f" roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}", flush=True)

node.create_subscription(Odometry, "/mujoco/ground_truth/odom", cb, 10)

# Foot world positions via TF (world -> ankle_roll_link), printed with the odom line.
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
last_feet = [0.0]

def print_feet():
    now = time.time()
    if now - last_feet[0] < 1.5:
        return
    last_feet[0] = now
    out = []
    for name in ("left_ankle_roll_link", "right_ankle_roll_link"):
        try:
            t = tf_buffer.lookup_transform("world", name, rclpy.time.Time())
            v = t.transform.translation
            out.append(f"{name[:1].upper()}=({v.x:.3f},{v.y:.3f},{v.z:.3f})")
        except Exception:
            out.append(f"{name[:1].upper()}=n/a")
    print("feet " + " ".join(out), flush=True)

end = time.time() + monitor_s
while time.time() < end and not state["fall"]:
    rclpy.spin_once(node, timeout_sec=0.2)
    print_feet()

if state["final"] is None:
    print("VERDICT: NO_ODOM")
    sys.exit(1)
x, y, z, roll, pitch, yaw = state["final"]
if state["fall"]:
    print(f"VERDICT: FALL at x={x:.3f} z={z:.3f} roll={roll:.3f} pitch={pitch:.3f}")
    sys.exit(1)
climbed = z > expect_min_z and x > expect_min_x
print(f"VERDICT: {'SUCCESS' if climbed else 'INCOMPLETE'}"
      f" final x={x:.3f} y={y:.3f} z={z:.3f} (max x={state['max_x']:.3f} z={state['max_z']:.3f})")
sys.exit(0 if climbed else 1)
PYEOF
RESULT=$?

kill "$LAUNCH_PID" 2>/dev/null
sleep 4
pkill -f mujoco_ros2_control 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null

echo "=== stair plan log lines:"
grep -E "StairClimbingPlan|Stair climbing" "$LOG" | tail -6
echo "=== warnings/errors:"
grep -iE "error|crash|failed|threw|exception" "$LOG" | grep -viE "Parameter|error_gain|CppAdInterface" | tail -8
echo "=== done, full log at $LOG"
exit "$RESULT"
