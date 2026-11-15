#!/bin/bash
# Upkie rolls off a 5 cm and a 10 cm ledge at 0.3 m/s (docs/results/upkie_ledge_drop.md): once with the wheel KF
# driving the MPC, once with GT driving and the KF logged alongside. Each run starts from the kneeling stand-up.
# Run in the container from any directory; plot afterwards with tests/tuning/plot_upkie_ledge_drop.py OUT_DIR.
#   usage: upkie_ledge_drop.sh [OUT_DIR=/tmp/upkie_ledge]
set -u
OUT=${1:-/tmp/upkie_ledge}
R=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$OUT"
cd /root/ocs2_ros2_ws && source install/setup.bash
for h in 05 10; do
  python3 "$R/description/mujoco/terrains/generate_upkie_course.py" \
    --params "$R/config/terrains/upkie/ledge_${h}_params.yaml" --mjcf "$OUT/ledge_${h}.xml"
  python3 - "$R" "$OUT" "$h" <<'PY'
import sys
from pathlib import Path
root, out, h = sys.argv[1:]
sys.path.insert(0, f"{root}/tests")
from mujoco_scene import write_scene
scene = write_scene(Path(f"{out}/ledge_{h}_scene.xml"), "upkie", "upkie_course")
scene.write_text(scene.read_text().replace(f"{root}/description/mujoco/terrains/upkie_course.xml", f"{out}/ledge_{h}.xml"))
PY
  for src in est gt; do
    if [ $src = est ]; then S="--source state_estimator --estimator humanoid_linear_kf"
    else S="--source ground_truth_state --log-estimator --estimator humanoid_linear_kf"; fi
    # Stand up, hold 4 s, drive 0.3 m/s holding the spawn line (yaw rate from the driving source), stop at 30 s.
    python3 "$R/tests/wb_id_walk_test.py" --robot-name upkie --prefix "$OUT/upkie_ledge_${h}_${src}" \
      --mpc-rate 200 --record-joints --record-tracking --max-yaw-error 100 --minimum-height 0.05 \
      --startup-timeout 900 $S \
      --set "bringUp.standUpTrajectoryFile=\"$R/config/upkie/stand_up_trajectory.yaml\"" \
      --launch-arg "initialPoseFile:=$R/config/upkie/initial_pose_idle.yaml" \
      --axis vx --speed 0 --settle 4 --ramp 0 --hold 29 --rest 3 --scene "$OUT/ledge_${h}_scene.xml" \
      --command-sequence '[[4.0, 0.3, 0.0, 0.30, 1], [30.0, 0.0, 0.0, 0.30]]'
    echo "RUN ledge_${h}_${src} exit $?"
  done
done
