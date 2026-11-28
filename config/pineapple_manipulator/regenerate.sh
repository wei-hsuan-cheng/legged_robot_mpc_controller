#!/bin/bash
# Pineapple offline assets: the contact-aware stand-up (and the posture table), generated, checked and installed.
#
#   setup            once: MuJoCo/CasADi/Pinocchio wheels into WORK/py (pip --target, no venv needed; about 1 GB),
#                    the C++ Pinocchio->CasADi model (stand_up_to/model.cpp) and its MuJoCo dynamics/CoM parity check.
#   plan MAX         solve with each contact phase capped at MAX seconds (1.5 manipulator, 2.5 arm-free):
#                    <repo>/demo_videos/ROBOT_stand_up_tune/candidate_MAX.yaml and WORK/solution_MAX.json.
#                    MIN (0.05 arm-free, 0.5 manipulator) is the lower phase bound; SEED overrides the warm start.
#   simulate MAX     replay the candidate in MuJoCo up to the MPC handover (stand_up_to/simulate.py), no ROS.
#   check MAX        three synthetic-joystick stand/drive/sit cycles plus a forced fall with the candidate in the full
#                    ROS stack (linear KF, MPC; tests/upkie_posture_cycle_test.py --robot-name ROBOT), ROS domain
#                    DOMAIN (default 171); run from a sourced workspace (WS, default ~/whcheng_ws).
#   install MAX      after a passing check: the candidate becomes config/ROBOT/stand_up_trajectory.yaml and its
#                    solution the seed. Takes effect at the next launch (symlink install).
#   posture [MIN MAX STEP]  regenerate config/ROBOT/posture_table.yaml (compute_posture_table.py).
#
# ROBOT defaults to pineapple_manipulator; config/pineapple/regenerate.sh selects the arm-free variant.
# The generators read config/ROBOT/geometry.yaml (tires and the base seat), shared with the MPC and estimator.
# setup, plan, simulate and posture drop the ROS environment: Jazzy's own pinocchio would shadow the pip one.
set -e
CMD=${1:?usage: regenerate.sh setup | plan|simulate|check|install MAX | posture [MIN MAX STEP]}
MAX=$2
case $CMD in
  plan|simulate|check|install) : "${MAX:?usage: regenerate.sh $CMD MAX (seconds per contact phase, shipped 1.5)}";;
esac
R=$(cd "$(dirname "$0")/../.." && pwd)
ROBOT=${ROBOT:-pineapple_manipulator}
case $ROBOT in
  pineapple|pineapple_manipulator);;
  *) echo "unsupported robot: $ROBOT"; exit 2;;
esac
DEFAULT_MIN=0.5
if [ "$ROBOT" = pineapple ]; then DEFAULT_MIN=0.05; fi
T=$R/config/pineapple_manipulator/stand_up_to
CFG=$R/config/$ROBOT
WORK=${WORK:-$HOME/${ROBOT}_stand_up}
O=$R/demo_videos/${ROBOT}_stand_up_tune
DEFAULT_SEED=$T/seed.json
if [ "$ROBOT" = pineapple ] && [ -f "$CFG/stand_up_seed.json" ]; then DEFAULT_SEED=$CFG/stand_up_seed.json; fi
to_env() {
  unset PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH
  PY=$WORK/py
  export PYTHONPATH=$PY:$(echo "$PY"/cmeel.prefix/lib/python3*/site-packages)
  export LD_LIBRARY_PATH=$PY/cmeel.prefix/lib:$PY/casadi
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
}
case $CMD in
  setup)
    to_env
    /usr/bin/python3 -m pip install --no-cache-dir --target "$WORK/py" mujoco==3.13.0 casadi==3.8.2 pin==4.1.0 \
      cmeel-eigen==3.4.1 scipy pyyaml
    cmake -S "$T" -B "$WORK/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$WORK/py/cmeel.prefix;$WORK/py/casadi"
    cmake --build "$WORK/build" -j4
    "$WORK/build/pineapple_manipulator_stand_up_model" "$R/description/urdf/robots/$ROBOT/$ROBOT.urdf" "$WORK/$ROBOT"
    /usr/bin/python3 "$T/plan.py" --robot "$ROBOT" --models "$WORK/$ROBOT" --check-model-only;;
  plan)
    to_env
    mkdir -p "$O"
    /usr/bin/python3 -u "$T/plan.py" --robot "$ROBOT" --models "$WORK/$ROBOT" --phase-seconds "${MIN:-$DEFAULT_MIN}" "$MAX" \
      --seed "${SEED:-$DEFAULT_SEED}" --solution "$WORK/solution_$MAX.json" --output "$O/candidate_$MAX.yaml"
    grep -E "^  (handover_time|mpc_start_time):" "$O/candidate_$MAX.yaml";;
  simulate)
    to_env
    /usr/bin/python3 "$T/simulate.py" --robot "$ROBOT" "$O/candidate_$MAX.yaml";;
  check)
    set +e  # ROS setup scripts are not written for errexit
    source "${WS:-$HOME/whcheng_ws}/install/setup.bash"
    cd "${WS:-$HOME/whcheng_ws}"  # CppAD caches are relative to the workspace, not the source tree.
    ROS_DOMAIN_ID=${DOMAIN:-171} timeout 1800 /usr/bin/python3 "$R/tests/upkie_posture_cycle_test.py" \
      --robot-name "$ROBOT" --cycles 3 --joystick --fall-test --trajectory "$O/candidate_$MAX.yaml" \
      --prefix "$O/cycles_$MAX" > "$O/cycles_$MAX.out" 2>&1
    STATUS=$?
    /usr/bin/python3 -c "import json, sys; r = json.load(open(sys.argv[1])); \
print(r['verdict'], r['cycles'], 'cycles, fall latched', r.get('fall_latched'), 'max locomotion tilt', \
r.get('max_locomotion_tilt'), r.get('error', ''))" "$O/cycles_$MAX.json"
    exit $STATUS;;
  install)
    /usr/bin/python3 -c "import hashlib, json, pathlib, sys; r = json.load(open(sys.argv[1])); \
sys.exit(not (r['verdict'] == 'PASS' and r['robot'] == sys.argv[3] and \
r['trajectory_sha256'] == hashlib.sha256(pathlib.Path(sys.argv[2]).read_bytes()).hexdigest()))" \
      "$O/cycles_$MAX.json" "$O/candidate_$MAX.yaml" "$ROBOT" 2>/dev/null || [ -n "$FORCE" ] || \
      { echo "run the check for this candidate first (FORCE=1 skips this)"; exit 1; }
    # Do not follow the arm-free variant's historical link into the manipulator's export.
    if [ -L "$CFG/stand_up_trajectory.yaml" ]; then rm "$CFG/stand_up_trajectory.yaml"; fi
    cp "$O/candidate_$MAX.yaml" "$CFG/stand_up_trajectory.yaml"
    SEED_OUT=$T/seed.json
    if [ "$ROBOT" = pineapple ]; then SEED_OUT=$CFG/stand_up_seed.json; fi
    /usr/bin/python3 -c "import json, sys; d = json.load(open(sys.argv[1])); \
json.dump(d, open(sys.argv[2], 'w'))" \
      "$WORK/solution_$MAX.json" "$SEED_OUT"
    echo "installed the $MAX s plan; restart the launch";;
  posture)
    to_env
    /usr/bin/python3 "$CFG/compute_posture_table.py" ${2:+--min "$2"} ${3:+--max "$3"} ${4:+--step "$4"};;
  *)
    echo "unknown command $CMD"; exit 2;;
esac
