# Modular MuJoCo Scenes

Each `scene*.xml` is a small composition of one robot, one background and optional terrain/object includes, plus its simulation timestep.

| Module | Contents |
| --- | --- |
| `g1_29dof.xml` | Robot bodies, joints, meshes, actuators and sensors |
| `background_day.xml` | Original daytime sky, lighting, camera and physical floor |
| `background_night.xml` | Studio lighting, camera, satin floor and robot finish |
| `terrain_materials.xml` | Shared matte material using `textures/studio_terrain.png` |
| `stairs.xml` | Stair geometry generated from `stairs.xml.xacro` and `stairs_params.yaml` |
| `ramp.xml` | Flush-entry 3-degree ramp and its floor contact exclusion |
| `slope.xml` | Original separate 5-degree slope |
| `terrain_10x10.xml` | Textured rough tiles with a clear 0.5 x 0.5 m center patch using the background floor |
| `boxes.xml` | Movable boxes with their original masses and colour tints |

Both backgrounds include `terrain_materials.xml` once, so multiple terrains can coexist without duplicate material definitions.
Terrain modules assign `material="studio_terrain"` to their geoms; they do not copy lighting or texture assets.
The robot module remains usable independently of these backgrounds.

For example, this scene combines the night background, G1, stairs and boxes:

```xml
<mujoco model="g1_custom_scene">
  <include file="g1_29dof.xml"/>
  <include file="background_night.xml"/>
  <include file="stairs.xml"/>
  <include file="boxes.xml"/>
  <option timestep="0.0001"/>
</mujoco>
```

Select exactly one background and include each terrain module at most once.
Keep scene files in this directory so the robot mesh and texture paths resolve correctly.
The floor's size, friction and priority are identical in both backgrounds.
Terrain dimensions, contact properties, inertias and actuator settings are independent of the background choice.

Entry points are `scene.xml` (night/configured terrain), `scene_flat.xml` (day/flat) and `scene_ramp.xml` (night/ramp).
Choose one through `g1.launch.py mujocoModelFile:=scene.xml`.
Switch day/night by changing the scene's background include; separate scene files for each appearance are unnecessary.

From the package root, the generator can compose scenes and regenerate night appearance assets:

Use `--terrain rough` to select `terrain_10x10.xml`, or `--terrain none` to use only the background floor.

```bash
python3 description/g1/mujoco/mujoco_scene_creator.py
python3 description/g1/mujoco/mujoco_scene_creator.py --reference scene_ramp.xml
python3 description/g1/mujoco/mujoco_scene_creator.py --terrain stairs boxes -o scene_objects.xml
python3 description/g1/mujoco/mujoco_scene_creator.py --background day --terrain ramp -o scene_day_ramp.xml
```

The default command updates `scene.xml` and the shared `background_night.xml`; every night scene immediately uses that appearance.
With `--reference`, the selected reference scene is updated unless `--output` is supplied.
Appearance overrides such as `--reflectance 0.02` write a background named after the output scene, leaving the shared night preset intact.
Change the generator's `STYLES` and `STUDIO_LIGHTS` to tune the night appearance; edit `background_day.xml` for daytime settings and `terrain_materials.xml` for terrain texture scale/specularity.
Regenerating stairs through xacro retains the material assignment, and all scenes immediately use the regenerated geometry without rerunning the scene generator.
