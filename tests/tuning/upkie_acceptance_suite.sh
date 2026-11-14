#!/bin/bash
# M1 acceptance suite for Upkie (ground truth): roll+brake and push recovery, N repeats each.
# Usage: claude_upkie_suite.sh TAG N [extra harness args...]
TAG=$1; N=$2; shift 2  # results: /tmp/upkie_${TAG}_{roll,push}_${i}.*
cd /root/ocs2_ros2_ws && source install/setup.bash
H=src/legged_robot_mpc_controller/tests/wb_id_walk_test.py
for i in $(seq 1 $N); do
  for test in roll push; do
    P=/tmp/upkie_${TAG}_${test}_${i}
    if [ $test = roll ]; then
      ARGS="--axis vx --speed 0.2 --hold 5 --ramp 2 --settle 10 --rest 8"
    else
      ARGS="--axis vx --speed 0.05 --hold 1 --ramp 1 --settle 16 --rest 4 --wrench-sequence src/legged_robot_mpc_controller/config/tests/upkie_pitch_push.yaml --wrench-delay 2"
    fi
    timeout 900 python3 $H --robot-name upkie --source ground_truth_state --minimum-height 0.15 --startup-timeout 900 \
      --record-tracking $ARGS --prefix $P "$@" > $P.out 2>&1
    echo "SUITE $TAG $test $i $(grep -o '"verdict": "[A-Z_]*"' $P.out)"
  done
done
echo "SUITE DONE $TAG"
