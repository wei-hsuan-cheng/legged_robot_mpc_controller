# Robot and Environment Descriptions

Descriptions are grouped by format, with semantic subdirectories such as `robots/`, `terrains/`, `objects/` and `common/` inside each format as needed.

```text
description/
  meshes/
    robots/g1/
    robots/h2/
    terrains/unit_wedge.stl
  mujoco/
    mujoco_scene_creator.py
    common/backgrounds/
    common/materials/
    robots/g1/
    robots/h2/
    terrains/
    objects/
    textures/
  urdf/
    robots/g1/
    robots/h2/
    terrains/
    objects/
config/
  terrains/stairs_params.yaml
  terrains/ramp_params.yaml
  objects/boxes_params.yaml
  terrains/stair_climbing/{g1,h2}.yaml
  terrains/terrain_walking/{g1,h2}.yaml
```

[`urdf/robots/g1/`](urdf/robots/g1/) contains the MPC and ros2_control descriptions, [`mujoco/robots/g1/`](mujoco/robots/g1/) contains the robot MJCF, and [`meshes/robots/g1/`](meshes/robots/g1/) contains their shared STL assets.
Reusable MuJoCo environments live in [`mujoco/`](mujoco/README.md), with one `robots/<robot>/scene.xml` entry point per robot.
H2 follows the same layout under `robots/h2/` in each format, with its pinned Unitree sources and integration changes documented in [`urdf/robots/h2/MODEL_SOURCE.md`](urdf/robots/h2/MODEL_SOURCE.md).
The RViz staircase is [`urdf/terrains/stairs.urdf.xacro`](urdf/terrains/stairs.urdf.xacro), while the simulator staircase is [`mujoco/terrains/stairs.xml`](mujoco/terrains/stairs.xml).
Both xacros read the same format-neutral geometry settings from [`config/terrains/stairs_params.yaml`](../config/terrains/stairs_params.yaml), so shared parameters and URDF assets do not live under MuJoCo.
The ramp and the movable boxes follow the same pattern: [`config/terrains/ramp_params.yaml`](../config/terrains/ramp_params.yaml) drives [`mujoco/terrains/ramp.xml.xacro`](mujoco/terrains/ramp.xml.xacro) and [`urdf/terrains/ramp.urdf.xacro`](urdf/terrains/ramp.urdf.xacro), and [`config/objects/boxes_params.yaml`](../config/objects/boxes_params.yaml) drives [`mujoco/objects/boxes.xml.xacro`](mujoco/objects/boxes.xml.xacro) and [`urdf/objects/boxes.urdf.xacro`](urdf/objects/boxes.urdf.xacro); the generated `ramp.xml` and `boxes.xml` are committed like `stairs.xml`.
The [shared terrain configuration directory](../config/terrains/README.md) also contains both robots' stair-climbing and terrain-walking planner variants; robot-specific height and foot geometry settings are preserved.

Box attributes are grouped under `boxes.box_1`, `boxes.box_2` and any additional entries; see [box configuration](../config/objects/README.md).
`robot.launch.py robotName:=g1` or `robotName:=h2` resolves the selected robot's paths automatically, so both use `mujocoModelFile:=scene.xml`.
Change that file's terrain/object includes instead of maintaining separate shipped scene variants; the launch still accepts any custom scene filename or absolute path.
The RViz counterpart is [`urdf/env.urdf.xacro`](urdf/env.urdf.xacro): it includes pieces from `urdf/terrains/` and `urdf/objects/` the same way, so comment or uncomment them to match the scene; the launch always publishes it.
Explicit paths supplied by external launch files must use the new layout.
