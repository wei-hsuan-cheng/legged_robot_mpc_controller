# Small-Wheel Terrain Trials

These profiles are for the 55 mm-radius Upkie wheels, not replacements for the humanoid ramp or uneven field.
The ramp starts at x = 0.6 m, rises at 2 degrees, and ends at x = 3.6 m with a vertical drop; stop before that drop.
The uneven field spans x = 0.5-4.5 m and y = -1-1 m, with 0.25 m tiles, a 0-24 mm height envelope and 3-8 mm edge-neighbor differences.
Its seed-7 realization has 0.67-21.92 mm tile heights and 3.83-6.85 mm neighbor differences.
Floor-to-field entry edges are not bounded by the neighbor-difference range.
Both use the existing stairs material.

From a sourced workspace, regenerate matching simulation/RViz modules:

```bash
cd src/legged_robot_mpc_controller
xacro description/mujoco/terrains/ramp.xml.xacro \
  ramp_params_file:="$PWD/config/terrains/upkie/ramp_params.yaml" \
  -o description/mujoco/terrains/upkie_ramp.xml
xacro description/urdf/terrains/ramp.urdf.xacro \
  ramp_params_file:="$PWD/config/terrains/upkie/ramp_params.yaml" \
  -o description/urdf/terrains/upkie_ramp.urdf.xacro
python3 description/mujoco/terrains/generate_uneven_terrain.py \
  --params config/terrains/upkie/uneven_terrain_params.yaml \
  --mjcf description/mujoco/terrains/upkie_uneven_terrain.xml \
  --urdf description/urdf/terrains/upkie_uneven_terrain.urdf.xacro
```

Keep the robot's single `scene.xml` unchanged and generate a temporary test scene:

```bash
cd /root/ocs2_ros2_ws
source install/setup.bash
python3 src/legged_robot_mpc_controller/tests/mujoco_scene.py \
  --robot-name upkie --terrain upkie_ramp --output /tmp/upkie_terrain_scene.xml
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=upkie \
  floatingBaseSource:=ground_truth_state floatingBaseEstimator:=none \
  mpcFreq:=200 mrtFreq:=1000 mujocoModelFile:=/tmp/upkie_terrain_scene.xml \
  rviz:=true mujoco_headless:=false baseCommandGui:=true
```

Choose `--terrain upkie_rough` for the uneven field, or `--terrain split_height --split-height 0.01` for a raised left-wheel strip starting at x = 0.6 m.
The split strip is 0.28 m wide, centered at y = 0.2 m, and ends at x = 3.4 m; the right wheel stays on the background floor.
Use only one simulator at a time.
RViz terrain geometry is separate; these commands do not alter your `description/urdf/env.urdf.xacro` selections.
The controller remains in `base_twist`, with no terrain planner, scene lookup, or preview of the upcoming heights/normals.
Simulation ground truth still supplies the floating-base state; this is not an estimated-state deployment test.
The ramp passed at a 0.2 m/s command and the uneven field passed at 0.3 m/s; the 0.15 m/s uneven trial stayed balanced but stalled at the entry.
Do not interpret that stall as successful traversal or assume that slower driving always crosses these vertical tile edges more easily.
The tire collision is a convex prism since 2026-11-14: MuJoCo's primitive cylinder never made a proper edge contact with a box step, so results before then are not comparable.
With the prism, the ramp, the 10 mm strip and the seed-7 field at 0.3 m/s pass; the 20 mm strip entry is climbed, but the run then fell when heading drift took the left tire off the strip's outer side.
Neither the 20 mm entry nor one-wheel lateral drops are a validated capability; see the handoff's tire-collision section.
Prioritize base pitch/roll and fall avoidance over exact height, straight-line position or heading tracking.
