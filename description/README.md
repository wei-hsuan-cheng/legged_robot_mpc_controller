# Robot and Environment Descriptions

Descriptions are grouped by format, with semantic subdirectories such as `robots/`, `terrains/`, `objects/` and `common/` inside each format as needed.

```text
description/
  meshes/
    robots/g1/
  mujoco/
    scene*.xml
    mujoco_scene_creator.py
    common/backgrounds/
    common/materials/
    robots/g1/
    terrains/
    objects/
    textures/
  urdf/
    robots/g1/
    terrains/
config/
  terrains/stairs_params.yaml
```

[`urdf/robots/g1/`](urdf/robots/g1/) contains the MPC and ros2_control descriptions, [`mujoco/robots/g1/`](mujoco/robots/g1/) contains the robot MJCF, and [`meshes/robots/g1/`](meshes/robots/g1/) contains their shared STL assets.
Reusable MuJoCo environments live in [`mujoco/`](mujoco/README.md), with `scene*.xml` as the top-level scene entry points.
The RViz staircase is [`urdf/terrains/stairs.urdf.xacro`](urdf/terrains/stairs.urdf.xacro), while the simulator staircase is [`mujoco/terrains/stairs.xml`](mujoco/terrains/stairs.xml).
Both xacros read the same format-neutral geometry settings from [`config/terrains/stairs_params.yaml`](../config/terrains/stairs_params.yaml), so shared parameters and URDF assets do not live under MuJoCo.

`g1.launch.py` resolves these paths automatically, so commands such as `mujocoModelFile:=scene_flat.xml` remain unchanged.
Explicit paths supplied by external launch files must use the new layout.
