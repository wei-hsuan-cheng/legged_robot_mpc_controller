#!/bin/bash
# Upkie demo video: the stand-up and course run (tests/upkie_demo.py full) as one two-row video, RViz on top (estimated
# state, MPC optimized trajectory, future ghosts) and MuJoCo below, same camera and field of view, matched by sim time.
#
#   record [NAME]  in the ros_jazzy container, with an X display for RViz (DISPLAY, default :1):
#                  1. runs the demo headless with the wheel KF in control, recording a bag of what RViz shows;
#                  2. steps RViz through the bag one video frame at a time and screenshots its render area
#                     (about 7 min; RViz's window shows on that display meanwhile: keep it uncovered).
#   render [NAME]  where MuJoCo 3.13, numpy, Pillow and ffmpeg are installed (PYTHON, default python3): replays the run in
#                  MuJoCo with the same camera and stacks it under the RViz frames: NAME.mp4 (captions), NAME_clean.mp4.
#
# Everything goes to <repo>/demo_videos/NAME/ (git-ignored, default NAME upkie_course): run* (harness outputs), bag/,
# rviz/ (frames, about 270 MB), the videos. The repository is the same folder on the host and in the container
# (~/src is mounted at /root/src), so record in the container and render on the host without copying anything.
CMD=${1:?usage: upkie_demo_video.sh record|render [NAME]}
NAME=${2:-upkie_course}
R=$(cd "$(dirname "$0")/.." && pwd)
D=$R/demo_videos/$NAME
mkdir -p "$D"
case $CMD in
  record)
    source /root/ocs2_ros2_ws/install/setup.bash
    cd /root/ocs2_ros2_ws  # the CppAD cache path auto_generated/upkie is relative to the workspace
    rm -rf "$D/bag" "$D/rviz"
    ros2 bag record -o "$D/bag" --use-sim-time \
      -e '/performance_visualization/.*|/tf|/tf_static|/robot_description|/clock' > "$D/bag.log" 2>&1 &
    BAG=$!
    python3 "$R/tests/upkie_demo.py" full --prefix "$D/run"
    STATUS=$?
    sleep 2
    kill -TERM $BAG  # a background job ignores SIGINT
    wait $BAG
    [ -f "$D/run_labels.json" ] || { echo "no run_labels.json: the run did not reach the MPC (see $D/run.out)"; exit 1; }
    python3 "$R/tests/tuning/render_rviz_replay.py" "$D/bag" "$D/run_labels.json" "$D/rviz" \
      --joints "$D/run_replay.json" --display "${DISPLAY:-:1}" || exit 1
    echo "recorded $D (demo exit $STATUS); now: tests/upkie_demo_video.sh render $NAME"
    exit $STATUS
    ;;
  render)
    PY=${PYTHON:-python3}
    "$PY" "$R/tests/tuning/render_upkie_video.py" "$D/run" --rviz-frames "$D/rviz" --out "$D/$NAME.mp4" &
    "$PY" "$R/tests/tuning/render_upkie_video.py" "$D/run" --rviz-frames "$D/rviz" --no-text --out "$D/${NAME}_clean.mp4"
    wait
    ls -la "$D"/*.mp4
    ;;
  *)
    echo "usage: upkie_demo_video.sh record|render [NAME]"
    exit 2
    ;;
esac
