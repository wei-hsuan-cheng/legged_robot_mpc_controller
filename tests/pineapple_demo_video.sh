#!/bin/bash
# Pineapple sideways demo video: the planned stand-up, then sideways stepping (vy selects trot) left, right and
# diagonally, as one two-row video, RViz on top (estimated state, MPC optimized trajectory, future ghosts) and MuJoCo
# below, the same front three-quarter camera (config/rviz/pineapple_video.rviz), matched by sim time.
#
#   record [NAME]  in the ros_jazzy container, with an X display for RViz (DISPLAY, default :1): runs the demo
#                  headless with the wheel KF in control, recording a bag of what RViz shows, then steps RViz through
#                  the bag one video frame at a time (RViz's window shows on that display meanwhile: keep it uncovered).
#   render [NAME]  where MuJoCo 3.13, numpy, Pillow and ffmpeg are installed (PYTHON, default python3): replays the run
#                  in MuJoCo with the same camera under the RViz frames: NAME_clean.mp4, and NAME.gif: each row cropped
#                  to the robot (800x448 of 1280x716; the camera follows the base), 400 px wide, 10 fps, about 6 MB.
#
# Everything goes to <repo>/demo_videos/NAME/ (git-ignored, default NAME pineapple_sideways), as with
# tests/upkie_demo_video.sh.
CMD=${1:?usage: pineapple_demo_video.sh record|render [NAME]}
NAME=${2:-pineapple_sideways}
R=$(cd "$(dirname "$0")/.." && pwd)
D=$R/demo_videos/$NAME
mkdir -p "$D"
# After the MPC takeover: still for 1 s, 0.2 m/s left for 4 s, right for 4 s, diagonally forward-left for 3 s, stop.
COMMANDS='[{"time": 0, "vy": 0.0}, {"time": 1, "vy": 0.2}, {"time": 5, "vy": -0.2},
           {"time": 9, "vx": 0.15, "vy": 0.15}, {"time": 12, "vy": 0.0}]'
LEAD=1.5   # seconds of the stand-up shown before the takeover
case $CMD in
  record)
    source /root/ocs2_ros2_ws/install/setup.bash
    cd /root/ocs2_ros2_ws  # the CppAD cache path auto_generated/pineapple is relative to the workspace
    rm -rf "$D/bag" "$D/rviz"
    ros2 bag record -o "$D/bag" --use-sim-time \
      -e '/performance_visualization/.*|/tf|/tf_static|/robot_description|/clock' > "$D/bag.log" 2>&1 &
    BAG=$!
    python3 "$R/tests/wb_id_walk_test.py" --robot-name pineapple --mpc-rate 200 --source state_estimator \
      --estimator humanoid_linear_kf --minimum-height 0.15 --max-yaw-error 100 --record-joints --record-tracking \
      --axis vx --speed 0 --settle 0 --ramp 0 --hold 14 --rest 1 --command-sequence "$COMMANDS" \
      --prefix "$D/run" > "$D/run.out" 2>&1
    STATUS=$?
    sleep 2
    kill -TERM $BAG  # a background job ignores SIGINT
    wait $BAG
    python3 - "$D" "$LEAD" <<'PY' || exit 1
import json, sys
from pathlib import Path
d, lead = Path(sys.argv[1]), float(sys.argv[2])
result = json.loads((d / "run.json").read_text())
if result.get("takeover_time") is None:
    sys.exit(f"the run did not reach the MPC (see {d}/run.out)")
t0 = result["takeover_time"]
labels = dict(terrain="none", start=t0 - lead, end=t0 + 13.5,
              phases=[[t0 - lead, "Planned stand-up"], [t0, "MPC: sideways steps (trot)"]],
              subtitles=[[t0 + 1, "vy 0.2 m/s left"], [t0 + 5, "vy 0.2 m/s right"], [t0 + 9, "diagonal"],
                         [t0 + 12, "stop"]])
(d / "run_labels.json").write_text(json.dumps(labels, indent=1) + "\n")
print("verdict", result.get("verdict"), "window", labels["start"], labels["end"])
PY
    python3 "$R/tests/tuning/render_rviz_replay.py" "$D/bag" "$D/run_labels.json" "$D/rviz" \
      --rviz "$R/config/rviz/pineapple_video.rviz" --joints "$D/run_replay.json" --display "${DISPLAY:-:1}" || exit 1
    echo "recorded $D (run exit $STATUS); now: tests/pineapple_demo_video.sh render $NAME"
    exit $STATUS
    ;;
  render)
    PY=${PYTHON:-python3}
    "$PY" "$R/tests/tuning/render_upkie_video.py" "$D/run" --robot pineapple --rviz-frames "$D/rviz" --no-text \
      --azimuth 146 --elev -17 --dist 2.0 --follow-z -0.15 --out "$D/${NAME}_clean.mp4" || exit 1
    ffmpeg -y -loglevel error -i "$D/${NAME}_clean.mp4" \
      -vf "split[t][b];[t]crop=800:448:240:40[t2];[b]crop=800:448:240:816[b2];[t2][b2]vstack,fps=10,scale=400:-2:flags=lanczos,split[a][c];[a]palettegen=max_colors=96:stats_mode=diff[p];[c][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
      "$D/$NAME.gif"
    ls -la "$D/${NAME}_clean.mp4" "$D/$NAME.gif"
    ;;
  *)
    echo "usage: pineapple_demo_video.sh record|render [NAME]"
    exit 2
    ;;
esac
