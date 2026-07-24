#!/bin/bash
# =============================================================================
# Autonomous terrain-aware stair walking test for the G1 centroidal MPC.
#
# Phase-1 perceptive-locomotion test: no pre-scripted sequence. The robot is
# switched to the "terrain_walk" target mode and simply commanded a forward
# velocity; the online TerrainFootholdPlanner selects footholds over the
# ground-truth staircase. The velocity command is zeroed (closed loop) once
# the pelvis passes the top of the stairs, and the robot must stand there.
#
#   VERDICT: SUCCESS     - pelvis reached the top and stands there, no fall
#   VERDICT: INCOMPLETE  - no fall, but the top was not reached in time
#   VERDICT: FALL        - pelvis dropped or tipped over
#   VERDICT: NO_ODOM     - no ground-truth odometry received
#
# Exit code: 0 only on SUCCESS.
#
# Usage (workspace must be sourced; runs from anywhere):
#   ros2 run legged_robot_mpc_controller terrain_walk_test.sh [log] [max_walk_s] [startup_wait_s]
# Env overrides: VX (0.25), PELVIS_HEIGHT (0.72), STOP_X (1.95),
#                EXPECT_MIN_X (1.85), EXPECT_MIN_Z (1.15), MAX_X (2.45),
#                TERRAIN_CONFIG (alternative terrain_walking yaml for terrainWalkingFile)
# =============================================================================
set -u

LOG="${1:-/tmp/terrain_walk_test.log}"
MAX_WALK="${2:-60}"       # seconds of walking before giving up
STARTUP_WAIT="${3:-90}"
VX="${VX:-0.25}"
PELVIS_HEIGHT="${PELVIS_HEIGHT:-0.72}"
STOP_X="${STOP_X:-1.95}"
EXPECT_MIN_X="${EXPECT_MIN_X:-1.85}"
EXPECT_MIN_Z="${EXPECT_MIN_Z:-1.15}"
MAX_X="${MAX_X:-2.45}"    # walking past this means it walked off the top
TERRAIN_CONFIG="${TERRAIN_CONFIG:-}"

if ! command -v ros2 >/dev/null; then
  echo "ERROR: ros2 not found - source your ROS 2 + workspace setup first." >&2
  exit 2
fi
WS_ROOT="$(dirname "${COLCON_PREFIX_PATH%%:*}")"
cd "$WS_ROOT" || exit 2

pkill -f 'g1[.]launch' 2>/dev/null
pkill -f mujoco_ros2_control 2>/dev/null
sleep 2

EXTRA_ARGS=()
if [ -n "$TERRAIN_CONFIG" ]; then
  EXTRA_ARGS+=("terrainWalkingFile:=$TERRAIN_CONFIG")
fi
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mujoco_headless:=true velocityCommandGui:=false rviz:=false \
  mpcControllerName:=humanoid_centroidal_mpc_controller "${EXTRA_ARGS[@]}" > "$LOG" 2>&1 &
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
echo "=== launched OK, standing for 5 s"
sleep 5

echo "=== selecting terrain_walk mode"
ros2 topic pub --once /humanoid/target_mode std_msgs/msg/String "{data: terrain_walk}" > /dev/null 2>&1
sleep 1

echo "=== walking vx=$VX (pelvis height $PELVIS_HEIGHT) until x > $STOP_X (max ${MAX_WALK}s)"
python3 - "$MAX_WALK" "$VX" "$PELVIS_HEIGHT" "$STOP_X" "$EXPECT_MIN_X" "$EXPECT_MIN_Z" "$MAX_X" <<'PYEOF'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import WalkingVelocityCommand
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

max_walk_s, vx, pelvis_height, stop_x, expect_min_x, expect_min_z, max_x = map(float, sys.argv[1:8])

rclpy.init()
node = Node("terrain_walk_probe")
state = {"last_print": 0.0, "fall": False, "final": None, "max_x": -1e9, "max_z": -1e9,
         "stopped_at": None}

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
cmd_pub = node.create_publisher(WalkingVelocityCommand, "/humanoid/walking_velocity_command", 10)

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

def publish_cmd(v):
    msg = WalkingVelocityCommand()
    msg.linear_velocity_x = v
    msg.linear_velocity_y = 0.0
    msg.desired_pelvis_height = pelvis_height
    msg.angular_velocity_z = 0.0
    cmd_pub.publish(msg)

# --- walk phase: command forward velocity until the pelvis passes STOP_X ---
start = time.time()
while time.time() - start < max_walk_s and not state["fall"]:
    publish_cmd(vx)
    rclpy.spin_once(node, timeout_sec=0.04)
    print_feet()
    if state["final"] is not None and state["final"][0] > stop_x:
        state["stopped_at"] = state["final"][0]
        print(f"=== pelvis passed x={stop_x}: zeroing velocity command", flush=True)
        break

# --- stop phase: zero command, verify the robot settles on top ---
settle_end = time.time() + 8.0
while time.time() < settle_end and not state["fall"]:
    publish_cmd(0.0)
    rclpy.spin_once(node, timeout_sec=0.04)
    print_feet()

if state["final"] is None:
    print("VERDICT: NO_ODOM")
    sys.exit(1)
x, y, z, roll, pitch, yaw = state["final"]
if state["fall"]:
    print(f"VERDICT: FALL at x={x:.3f} z={z:.3f} roll={roll:.3f} pitch={pitch:.3f}")
    sys.exit(1)
climbed = z > expect_min_z and x > expect_min_x and state["max_x"] < max_x
print(f"VERDICT: {'SUCCESS' if climbed else 'INCOMPLETE'}"
      f" final x={x:.3f} y={y:.3f} z={z:.3f} (max x={state['max_x']:.3f} z={state['max_z']:.3f})")
sys.exit(0 if climbed else 1)
PYEOF
RESULT=$?

kill "$LAUNCH_PID" 2>/dev/null
sleep 4
pkill -f mujoco_ros2_control 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null

echo "=== terrain walk log lines:"
grep -E "terrain walk|TerrainFoothold|target mode" "$LOG" | tail -6
echo "=== gait transitions:"
grep -E "Increasing to gait|Decreasing to gait" "$LOG" | tail -6
echo "=== warnings/errors:"
grep -iE "error|crash|failed|threw|exception" "$LOG" | grep -viE "Parameter|error_gain|CppAdInterface" | tail -8
echo "=== done, full log at $LOG"
exit "$RESULT"
