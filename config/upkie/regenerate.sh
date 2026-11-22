#!/bin/bash
# Upkie offline assets inside the ros_jazzy container (Ubuntu 24.04): generate, check in MuJoCo, install.
#
#   setup            once per container: python3-venv (apt) if missing, a venv with the MuJoCo/CasADi/Pinocchio
#                    wheels in WORK (default /root/upkie_stand_up, about 1.0 GB), the C++ Pinocchio->CasADi stand-up
#                    model and its MuJoCo dynamics/CoM parity check.
#
# Contact-aware stand-up (config/upkie/stand_up_trajectory.yaml, bringUp.standUpTrajectoryFile):
#   plan MAX         solve with every contact phase capped at MAX seconds (handover to MPC at 2.6 MAX, MPC warm-up
#                    from 2.2 MAX): <repo>/demo_videos/upkie_stand_up_tune/candidate_MAX.yaml and
#                    WORK/solution_MAX.json. MIN (default 0.5) is the lower phase bound; SEED replaces the warm start
#                    config/upkie/stand_up_to/seed.json.
#   check MAX        three synthetic-joystick stand/drive/sit cycles plus a forced fall with that candidate in MuJoCo
#                    (shipped MPC, estimator and launch; ROS domain DOMAIN, default 169); prints the verdict and the
#                    tilt in the first second of MPC per stand-up, to compare the handover with the shipped plan's.
#   install MAX      after a passing check: the candidate becomes config/upkie/stand_up_trajectory.yaml and its
#                    solution the seed.
#
# Posture table (config/upkie/posture_table.yaml, ocs2.reference.postureTableFile):
#   posture [MIN MAX STEP]  balanced symmetric posture per base height (defaults 0.125 0.355 0.01 reproduce the
#                    shipped table): <repo>/demo_videos/upkie_posture_table/candidate.yaml; prints the change from
#                    the shipped table.
#   check-posture    the course demo with the candidate table: stand-up, height test standing still (max, min,
#                    default height, 2 s each), ramps, uneven field and height steps while driving; prints the verdict
#                    and, at the end of each height-test plateau, the height and hip/knee error from the table.
#   install-posture  after a passing check-posture: the candidate becomes config/upkie/posture_table.yaml.
#
# Installs take effect when the launch restarts (symlink install, no build); undo with git checkout of the files.
# setup, plan and posture drop the ROS environment: Jazzy ships its own pinocchio, and its PYTHONPATH and
# LD_LIBRARY_PATH would shadow the venv's. Details: docs/upkie_stand_up_optimization.md, "Tuning the Duration and
# Regenerating", and docs/upkie_posture_control.md, "Posture Table".
set -e
CMD=${1:?usage: regenerate.sh setup | plan|check|install MAX | posture [MIN MAX STEP] | check-posture | install-posture}
MAX=$2
case $CMD in
  plan|check|install) : "${MAX:?usage: regenerate.sh $CMD MAX (seconds per contact phase, shipped 1.5)}";;
esac
R=$(cd "$(dirname "$0")/../.." && pwd)
WORK=${WORK:-/root/upkie_stand_up}
O=$R/demo_videos/upkie_stand_up_tune
P=$R/demo_videos/upkie_posture_table
to_env() {
  unset PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
  [ "$CMD" = setup ] || source "$WORK/env/bin/activate"
}
ros_env() {
  set +e  # ROS setup scripts are not written for errexit; test statuses are kept explicitly
  source /root/ocs2_ros2_ws/install/setup.bash
  cd /root/ocs2_ros2_ws  # the CppAD cache auto_generated/upkie is relative to the working directory
}
passed() {  # passed RESULT.json VERDICT: the recorded verdict, unless FORCE is set
  /usr/bin/python3 -c "import json, sys; sys.exit(json.load(open(sys.argv[1]))['verdict'] != sys.argv[2])" \
    "$1" "$2" 2>/dev/null || [ -n "$FORCE" ] || { echo "run the check first (FORCE=1 skips this)"; exit 1; }
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
    ros_env
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
    passed "$O/cycles_$MAX.json" PASS
    cp "$O/candidate_$MAX.yaml" "$R/config/upkie/stand_up_trajectory.yaml"
    /usr/bin/python3 -c "import json, sys; d = json.load(open(sys.argv[1])); \
json.dump({k: d[k] for k in ('x', 'v', 'a', 'force', 'torque', 'duration')}, open(sys.argv[2], 'w'))" \
      "$WORK/solution_$MAX.json" "$R/config/upkie/stand_up_to/seed.json"
    echo "installed the $MAX s plan; restart the launch";;
  posture)
    to_env
    python "$R/config/upkie/compute_posture_table.py" ${2:+--min "$2"} ${3:+--max "$3"} ${4:+--step "$4"} \
      --output "$P/candidate.yaml"
    python - "$P/candidate.yaml" "$R/config/upkie/posture_table.yaml" <<'EOF'
import os, sys
import numpy as np
import yaml
if not os.path.exists(sys.argv[2]):
    print("no shipped table to compare with")
    sys.exit()
new, old = (yaml.safe_load(open(path))["posture_table"] for path in sys.argv[1:])
if new == old:
    print("identical to the shipped table")
else:
    heights = sorted(set(new["heights"]) & set(old["heights"]))
    rows = lambda table: {h: np.array(q) for h, q in zip(table["heights"], table["joint_states"])}
    change = max((float(np.max(abs(rows(new)[h] - rows(old)[h]))) for h in heights), default=float("nan"))
    print(f"heights {new['heights'][0]:.3f}-{new['heights'][-1]:.3f} m ({len(new['heights'])}), shipped "
          f"{old['heights'][0]:.3f}-{old['heights'][-1]:.3f} m ({len(old['heights'])}); "
          f"largest joint change at shared heights {change:.6f} rad")
EOF
    ;;
  check-posture)
    ros_env
    STATUS=0
    ROS_DOMAIN_ID=${DOMAIN:-169} timeout 1800 /usr/bin/python3 "$R/tests/upkie_demo.py" course --prefix "$P/course" \
      --set "ocs2.reference.postureTableFile=\"$P/candidate.yaml\"" || STATUS=$?
    /usr/bin/python3 - "$P/course" "$P/candidate.yaml" <<'EOF'
import csv, json, sys
import numpy as np
import yaml
prefix, table = sys.argv[1], yaml.safe_load(open(sys.argv[2]))["posture_table"]
heights, states = np.array(table["heights"]), np.array(table["joint_states"])
run = json.load(open(prefix + ".json"))
rows = list(csv.DictReader(open(prefix + "_tracking.csv")))
column = lambda key: np.array([float(r[key]) if r[key] not in ("", "nan") else np.nan for r in rows])
t = column("t") - next(float(r["t"]) for r in rows if r["test_phase"] == "settle")
z, hip, knee = column("obs_base/z"), column("meas_q/left_hip"), column("meas_q/left_knee")
commands = run["command_sequence"]
for (start, vx, wz, height, *_), (end, *_) in zip(commands, commands[1:]):
    if vx != 0 or wz != 0 or end - start < 1.5:
        break  # the standing height test is the first block of the course
    window = (t >= end - 0.5) & (t < end)
    reference = [np.interp(height, heights, states[:, i]) for i in (0, 1)]
    print(f"height test {height:.3f} m: measured {np.nanmean(z[window]):.3f} m, hip/knee - table "
          f"{np.nanmean(hip[window]) - reference[0]:+.3f}/{np.nanmean(knee[window]) - reference[1]:+.3f} rad")
EOF
    exit $STATUS;;
  install-posture)
    passed "$P/course.json" SUCCESS
    cp "$P/candidate.yaml" "$R/config/upkie/posture_table.yaml"
    echo "installed the posture table; restart the launch";;
  *)
    echo "usage: regenerate.sh setup | plan|check|install MAX | posture [MIN MAX STEP] | check-posture | install-posture"
    exit 2;;
esac
