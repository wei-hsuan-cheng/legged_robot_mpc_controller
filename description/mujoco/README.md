# Modular MuJoCo Scenes

Each `scene*.xml` is a small composition of one robot, one background and optional terrain/object includes, plus its simulation timestep.

```text
description/
  robots/
    g1/
      meshes/
      mujoco/g1_29dof.xml
      urdf/
  mujoco/
    scene.xml
    scene_flat.xml
    scene_ramp.xml
    mujoco_scene_creator.py
    backgrounds/
    terrains/
    objects/
    materials/
    textures/
```

Robot-specific files live under `description/robots/<robot>/`, while scenes and reusable environment modules live under `description/mujoco/`.
The staircase's MuJoCo XML, RViz URDF xacro and shared parameters stay together in `terrains/` because they describe the environment, not G1.

| Module | Contents |
| --- | --- |
| `../robots/g1/mujoco/g1_29dof.xml` | Robot bodies, joints, actuators and sensors; meshes live in `../robots/g1/meshes/` |
| `backgrounds/background_day.xml` | Original daytime sky, lighting, camera and physical floor |
| `backgrounds/background_night.xml` | Studio lighting, camera, satin floor and robot finish |
| `materials/terrain_materials.xml` | Shared matte material using `textures/studio_terrain.png` |
| `terrains/stairs.xml` | Stair geometry generated from `stairs.xml.xacro` and `stairs_params.yaml` in the same directory |
| `terrains/stairs.urdf.xacro` | Matching RViz environment geometry using the same staircase parameters |
| `terrains/ramp.xml` | Flush-entry 3-degree ramp and its floor contact exclusion |
| `terrains/slope.xml` | Original separate 5-degree slope |
| `terrains/terrain_10x10.xml` | Textured rough tiles with a clear 0.5 x 0.5 m center patch using the background floor |
| `objects/boxes.xml` | Movable boxes with their original masses and colour tints |
| `objects/boxes_scattered_50.xml` | Alternative box layout with additional scattered cubes |

Both backgrounds include `../materials/terrain_materials.xml` once, so multiple terrains can coexist without duplicate material definitions.
Terrain modules assign `material="studio_terrain"` to their geoms; they do not copy lighting or texture assets.
The robot module contains no background or terrain definitions.

For example, this scene combines the night background, G1, stairs and boxes:

```xml
<mujoco model="g1_custom_scene">
  <include file="../robots/g1/mujoco/g1_29dof.xml"/>
  <include file="backgrounds/background_night.xml"/>
  <include file="terrains/stairs.xml"/>
  <include file="objects/boxes.xml"/>
  <option timestep="0.0001"/>
</mujoco>
```

Select exactly one background and include each terrain module at most once.
Keep scene entry points in `description/mujoco/` because MuJoCo resolves asset files and compiler directories from the top-level scene, while nested XML includes resolve from the including file.
The G1 module's compiler therefore uses `meshdir="../robots/g1/meshes"`, and shared texture assets use `file="textures/..."`.
These paths also work in the installed package because CMake installs the whole `description/` tree.
The floor's size, friction and priority are identical in both backgrounds.
Terrain dimensions, contact properties, inertias and actuator settings are independent of the background choice.

Entry points are `scene.xml` (night/configured terrain), `scene_flat.xml` (day/flat) and `scene_ramp.xml` (night/ramp).
Choose one through `g1.launch.py mujocoModelFile:=scene.xml`.
Switch day/night by changing the scene's background include; separate scene files for each appearance are unnecessary.

From the package root, the generator can compose scenes and regenerate night appearance assets:

Use `--terrain rough` to select `terrains/terrain_10x10.xml`, `--terrain scattered_boxes` for the alternative box layout, or `--terrain none` to use only the background floor.

```bash
python3 description/mujoco/mujoco_scene_creator.py
python3 description/mujoco/mujoco_scene_creator.py --reference scene_ramp.xml
python3 description/mujoco/mujoco_scene_creator.py --terrain stairs boxes -o scene_objects.xml
python3 description/mujoco/mujoco_scene_creator.py --background day --terrain ramp -o scene_day_ramp.xml
```

The default command updates `scene.xml` and the shared `backgrounds/background_night.xml`; every night scene immediately uses that appearance.
With `--reference`, the selected reference scene is updated unless `--output` is supplied.
Appearance overrides such as `--reflectance 0.02` write a background named after the output scene in `backgrounds/`, leaving the shared night preset intact.
Change the generator's `STYLES` and `STUDIO_LIGHTS` to tune the night appearance; edit `backgrounds/background_day.xml` for daytime settings and `materials/terrain_materials.xml` for terrain texture scale/specularity.
Regenerating stairs through xacro retains the material assignment, and all scenes immediately use the regenerated geometry without rerunning the scene generator.

Validate the layout from the package root with `python3 -m unittest discover -s tests -p test_mujoco_scene_layout.py -v`.
MuJoCo and xacro checks run when their Python packages are installed; the remaining resource-path checks need only the standard library.
