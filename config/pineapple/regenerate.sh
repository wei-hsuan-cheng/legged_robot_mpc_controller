#!/bin/bash
# Pineapple offline assets: the contact-aware stand-up (and the posture table), generated, checked and installed.
#
#   setup            once: MuJoCo/CasADi/Pinocchio wheels into WORK/py (pip --target, no venv needed; about 1 GB),
#                    the C++ Pinocchio->CasADi model (stand_up_to/model.cpp) and its MuJoCo dynamics/CoM parity check.
#   plan MAX         solve with each of the two contact phases capped at MAX seconds (shipped 1.5):
#                    <repo>/demo_videos/pineapple_stand_up_tune/candidate_MAX.yaml and WORK/solution_MAX.json.
#                    MIN (default 0.5) is the lower phase bound; SEED replaces the warm start stand_up_to/seed.json.
#   simulate MAX     replay the candidate in MuJoCo up to the MPC handover (stand_up_to/simulate.py), no ROS.
#   check MAX        three synthetic-joystick stand/drive/sit cycles plus a forced fall with the candidate in the full
#                    ROS stack (linear KF, MPC; tests/upkie_posture_cycle_test.py --robot-name pineapple), ROS domain
#                    DOMAIN (default 171); run from a sourced workspace (WS, default ~/whcheng_ws).
#   install MAX      after a passing check: the candidate becomes config/pineapple/stand_up_trajectory.yaml and its
#                    solution the seed. Takes effect at the next launch (symlink install).
#   posture [MIN MAX STEP]  regenerate config/pineapple/posture_table.yaml (compute_posture_table.py).
#
# The generators read config/pineapple/geometry.yaml (tires and the base seat), shared with the MPC and the estimator.
# setup, plan, simulate and posture drop the ROS environment: Jazzy's own pinocchio would shadow the pip one.
set -e
CMD=${1:?usage: regenerate.sh setup | plan|simulate|check|install MAX | posture [MIN MAX STEP]}
MAX=$2
case $CMD in
  plan|simulate|check|install) : "${MAX:?usage: regenerate.sh $CMD MAX (seconds per contact phase, shipped 1.5)}";;
esac
R=$(cd "$(dirname "$0")/../.." && pwd)
T=$R/config/pineapple/stand_up_to
WORK=${WORK:-$HOME/pineapple_stand_up}
O=$R/demo_videos/pineapple_stand_up_tune
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
    /usr/bin/python3 -m pip install --no-cache-dir --target "$WORK/py" mujoco==3.3.0 casadi==3.8.2 pin==4.1.0 \
      cmeel-eigen==3.4.1 scipy pyyaml
    cmake -S "$T" -B "$WORK/build" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$WORK/py/cmeel.prefix;$WORK/py/casadi"
    cmake --build "$WORK/build" -j4
    "$WORK/build/pineapple_stand_up_model" "$R/description/urdf/robots/pineapple/pineapple.urdf" "$WORK/pineapple"
    /usr/bin/python3 "$T/plan.py" --models "$WORK/pineapple" --check-model-only;;
  plan)
    to_env
    mkdir -p "$O"
    /usr/bin/python3 -u "$T/plan.py" --models "$WORK/pineapple" --phase-seconds "${MIN:-0.5}" "$MAX" \
      ${SEED:+--seed "$SEED"} --solution "$WORK/solution_$MAX.json" --output "$O/candidate_$MAX.yaml"
    grep -E "^  (handover_time|mpc_start_time):" "$O/candidate_$MAX.yaml";;
  simulate)
    to_env
    /usr/bin/python3 "$T/simulate.py" "$O/candidate_$MAX.yaml";;
  check)
    set +e  # ROS setup scripts are not written for errexit
    source "${WS:-$HOME/whcheng_ws}/install/setup.bash"
    cd "${WS:-$HOME/whcheng_ws}"  # the CppAD cache auto_generated/pineapple is relative to the working directory
    ROS_DOMAIN_ID=${DOMAIN:-171} timeout 1800 /usr/bin/python3 "$R/tests/upkie_posture_cycle_test.py" \
      --robot-name pineapple --cycles 3 --joystick --fall-test --trajectory "$O/candidate_$MAX.yaml" \
      --prefix "$O/cycles_$MAX" > "$O/cycles_$MAX.out" 2>&1
    STATUS=$?
    /usr/bin/python3 -c "import json, sys; r = json.load(open(sys.argv[1])); \
print(r['verdict'], r['cycles'], 'cycles, fall latched', r.get('fall_latched'), 'max locomotion tilt', \
r.get('max_locomotion_tilt'), r.get('error', ''))" "$O/cycles_$MAX.json"
    exit $STATUS;;
  install)
    /usr/bin/python3 -c "import json, sys; sys.exit(json.load(open(sys.argv[1]))['verdict'] != 'PASS')" \
      "$O/cycles_$MAX.json" 2>/dev/null || [ -n "$FORCE" ] || { echo "run the check first (FORCE=1 skips this)"; exit 1; }
    cp "$O/candidate_$MAX.yaml" "$R/config/pineapple/stand_up_trajectory.yaml"
    /usr/bin/python3 -c "import json, sys; d = json.load(open(sys.argv[1])); \
json.dump({k: d[k] for k in ('x', 'v', 'a', 'force', 'torque', 'duration')}, open(sys.argv[2], 'w'))" \
      "$WORK/solution_$MAX.json" "$T/seed.json"
    echo "installed the $MAX s plan; restart the launch";;
  posture)
    to_env
    /usr/bin/python3 "$R/config/pineapple/compute_posture_table.py" ${2:+--min "$2"} ${3:+--max "$3"} ${4:+--step "$4"};;
  *)
    echo "unknown command $CMD"; exit 2;;
esac
