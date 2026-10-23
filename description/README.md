# Robot and Environment Descriptions

Robot-specific assets live in [`robots/g1/`](robots/g1/): `urdf/` contains the MPC and ros2_control descriptions, `mujoco/` contains the robot MJCF, and `meshes/` contains their shared STL assets.
Reusable backgrounds, terrain, objects, materials and textures live in [`mujoco/`](mujoco/README.md), with `scene*.xml` as the top-level scene entry points.
The staircase's RViz xacro and shared YAML live next to its MuJoCo geometry in `mujoco/terrains/`.

`g1.launch.py` resolves these paths automatically, so commands such as `mujocoModelFile:=scene_flat.xml` remain unchanged.
Explicit paths supplied by external launch files must use the new layout.
