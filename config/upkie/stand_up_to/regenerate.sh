#!/bin/bash
# Upkie contact-aware stand-up TO inside the ros_jazzy container (Ubuntu 24.04): offline plan, MuJoCo check, install.
#
#   setup        once per container: python3-venv (apt) if missing, a venv with the MuJoCo/CasADi/Pinocchio wheels
#                in WORK (default /root/upkie_stand_up, about 1.0 GB), the C++ Pinocchio->CasADi model and its
#                MuJoCo dynamics/CoM parity check.
#   plan MAX     solve with every contact phase capped at MAX seconds (handover to MPC at 2.6 MAX, MPC warm-up from
#                2.2 MAX): <repo>/demo_videos/upkie_stand_up_tune/candidate_MAX.yaml and WORK/solution_MAX.json.
#                MIN (default 0.5) is the lower phase bound; SEED replaces the warm start config/upkie/stand_up_to/seed.json.
#   check MAX    three synthetic-joystick stand/drive/sit cycles plus a forced fall with that candidate in MuJoCo
#                (shipped MPC, estimator and launch; ROS domain DOMAIN, default 169); prints the verdict and the
#                tilt in the first second of MPC per stand-up, to compare the handover with the shipped plan's.
#   install MAX  after a passing check: the candidate becomes config/upkie/stand_up_trajectory.yaml and its solution
#                the seed. Restart the launch (symlink install, no build); undo with git checkout of both files.
#
# setup and plan drop the ROS environment: Jazzy ships its own pinocchio, and its PYTHONPATH and LD_LIBRARY_PATH would
# shadow the venv's. Details: docs/upkie_stand_up_optimization.md, "Tuning the Duration and Regenerating".
set -e
CMD=${1:?usage: regenerate.sh setup | plan MAX | check MAX | install MAX}
MAX=$2
[ "$CMD" = setup ] || : "${MAX:?usage: regenerate.sh $CMD MAX (seconds per contact phase, shipped 1.5)}"
R=$(cd "$(dirname "$0")/../../.." && pwd)
WORK=${WORK:-/root/upkie_stand_up}
O=$R/demo_videos/upkie_stand_up_tune
to_env() {
  unset PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
  [ "$CMD" = setup ] || source "$WORK/env/bin/activate"
}
case $CMD in
  setup)
    /usr/bin/python3 -c "import ensurepip" 2>/dev/null || apt-get install -y --no-install-recommends python3-venv ||
      { apt-get update && apt-get install -y --no-install-recommends python3-venv; }
    to_env
    /usr/bin/python3 -m venv "$WORK/env"
    source "$WORK/env/bin/activate"
    pip install --no-cache-dir mujoco==3.13.0 casadi==3.8.2 pin==4.1.0 cmeel-eigen==3.4.1 numpy==2.5.5 \
      scipy==1.18.1 pyyaml matplotlib
    PREFIX=$(python -c 'import site; print(site.getsitepackages()[0])')
    cmake -S "$R/config/upkie/stand_up_to" -B "$WORK/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$PREFIX/cmeel.prefix;$PREFIX/casadi"
    cmake --build "$WORK/build" -j4
    "$WORK/build/upkie_stand_up_model" "$R/description/urdf/robots/upkie/upkie.urdf" "$WORK/upkie"
    python "$R/config/upkie/stand_up_to/plan.py" --models "$WORK/upkie" --check-model-only;;
  plan)
    to_env
    mkdir -p "$O"
    python "$R/config/upkie/stand_up_to/plan.py" --models "$WORK/upkie" --phase-seconds "${MIN:-0.5}" "$MAX" \
      ${SEED:+--seed "$SEED"} --solution "$WORK/solution_$MAX.json" --output "$O/candidate_$MAX.yaml"
    grep -E "^  (handover_time|mpc_start_time):" "$O/candidate_$MAX.yaml";;
  check)
    set +e  # ROS setup scripts are not written for errexit; the test status is kept below
    source /root/ocs2_ros2_ws/install/setup.bash
    cd /root/ocs2_ros2_ws  # the CppAD cache auto_generated/upkie is relative to the working directory
    STATUS=0
    ROS_DOMAIN_ID=${DOMAIN:-169} timeout 1800 /usr/bin/python3 "$R/tests/upkie_posture_cycle_test.py" \
      --cycles 3 --joystick --fall-test --trajectory "$O/candidate_$MAX.yaml" --prefix "$O/cycles_$MAX" \
      > "$O/cycles_$MAX.out" 2>&1 || STATUS=$?
    rm -f "$O/cycles_$MAX.csv.gz"
    /usr/bin/python3 - "$O/cycles_$MAX" <<'EOF'
import json, sys
prefix = sys.argv[1]
result = json.load(open(prefix + ".json"))
print(result["verdict"], result["cycles"], "cycles, fall latched", result.get("fall_latched"), result.get("error", ""))
# The largest tilt is at MPC takeover; compare it per stand-up with the shipped plan's (doc: Check).
segments, previous = [], None
for sample in json.load(open(prefix + ".replay.json")):
    if sample["phase"] == "locomotion" and previous != "locomotion":
        segments.append((sample["time"], []))
    if sample["phase"] == "locomotion" and sample["stage"] == "cycles":
        segments[-1][1].append((sample["time"] - segments[-1][0], sample["tilt"]))
    previous = sample["phase"]
segments = [s for _, s in segments if s]
early = [max([tilt for t, tilt in s if t < 1.0], default=0.0) for s in segments]
late = max([tilt for s in segments for t, tilt in s if t >= 1.0], default=0.0)
print("tilt in the first 1 s of MPC per stand-up:", " ".join(f"{v:.3f}" for v in early), f"rad; afterwards at most {late:.3f} rad")
EOF
    exit $STATUS;;
  install)
    /usr/bin/python3 -c "import json, sys; sys.exit(json.load(open(sys.argv[1]))['verdict'] != 'PASS')" \
      "$O/cycles_$MAX.json" 2>/dev/null || { [ -n "$FORCE" ] || { echo "run check $MAX first (FORCE=1 skips this)"; exit 1; }; }
    cp "$O/candidate_$MAX.yaml" "$R/config/upkie/stand_up_trajectory.yaml"
    /usr/bin/python3 -c "import json, sys; d = json.load(open(sys.argv[1])); \
json.dump({k: d[k] for k in ('x', 'v', 'a', 'force', 'torque', 'duration')}, open(sys.argv[2], 'w'))" \
      "$WORK/solution_$MAX.json" "$R/config/upkie/stand_up_to/seed.json"
    echo "installed the $MAX s plan; restart the launch";;
  *)
    echo "usage: regenerate.sh setup | plan MAX | check MAX | install MAX"; exit 2;;
esac
