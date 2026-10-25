# Box Configuration

[`boxes_params.yaml`](boxes_params.yaml) groups each box's attributes under `boxes.box_1`, `boxes.box_2` and any additional entry keys.
Each entry has its own `position`, `yaw`, full `size`, `mass`, principal `inertia`, RGBA `color` and MuJoCo `friction`.
The optional `name` is the simulator/RViz body name; omit it to use the entry key.
Existing names `purple_box` and `orange_box` and all physical values are preserved.
Add or remove an entry to change the number of boxes; there is no separate count or set of parallel lists.
Mass and inertia remain explicit and are not recomputed automatically when size changes.

Regenerate the MuJoCo module after editing the YAML:

```bash
xacro description/mujoco/objects/boxes.xml.xacro -o description/mujoco/objects/boxes.xml
```

Include `../../objects/boxes.xml` in the selected robot's `scene.xml` to add the boxes to physics.
To visualize their spawn poses in RViz, set `environmentDescription` to the full path of `description/urdf/objects/boxes.urdf.xacro`; it reads the same YAML at launch.
Selecting the boxes for RViz is independent of the MuJoCo scene, and RViz does not follow the boxes after they move in MuJoCo.
