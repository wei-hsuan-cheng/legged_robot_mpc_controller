# Pineapple Manipulator Model Source

Pineapple V3 is a wheel-legged robot: per leg a hip ab/ad (x axis), thigh and calf (y axis) and a driven wheel, plus a 6-DOF arm with a two-finger gripper on top; 14.92 kg in total.

| Item | Source |
| --- | --- |
| Simulation reference | Vendor MJCF `pineapple_v3/pineapple_v3.xml` (with `scene.xml`) |
| Meshes | The 37 OBJ and 3 STL files that MJCF references, copied unchanged to `description/meshes/robots/pineapple/` |
| Effort limits | The MJCF actuator `ctrlrange`s: 27 N m hip/thigh, 40 N m calf, 11 N m wheel, 27/7 N m arm |
| Velocity limits | Not in the vendor model. Legs: the no-load output speed at 24 V of the BOM motors (`pineapple_v3_bom.csv`), from third-party listings of Damiao's manuals, not yet checked against the manuals: hip DM-J4340P-2EC and thigh DM-J4340-2EC 5.5 rad/s (52.5 rpm), calf DM-J6248P-2EC 6.28 rad/s (60 rpm, motor output; the motor sits in the thigh). Wheel (DM-J6006-2EC) 60 rad/s and arm 10 rad/s are placeholders. All in `generate_pineapple_urdf.py`; MuJoCo does not limit joint speed, the MPC does |

There is no vendor URDF; [`pineapple_manipulator.urdf`](pineapple_manipulator.urdf) is generated from the project MJCF by [`generate_pineapple_urdf.py`](../pineapple/generate_pineapple_urdf.py), so every frame, axis, mass, CoM and inertia equals MuJoCo's (FK and CoM agree to 1e-15 over random states).
The gripper slides are fixed in the URDF (mass lumped into the gripper case); the arm joints stay revolute and the controller locks them (`ocs2.model.fixedJointNames`).

## Changes to the MJCF

[`pineapple_manipulator.xml`](../../../mujoco/robots/pineapple_manipulator/pineapple_manipulator.xml) is the vendor `pineapple_v3.xml` with these changes only; its mass matrix, CoM and tire positions equal the vendor model's to 5e-10 over random states:

| Change | Reason |
| --- | --- |
| `meshdir`/`texturedir` point at the repository folders; `autolimits` | Repository layout |
| `option integrator="implicitfast" noslip_iterations="4"` | As Upkie: wheel rolling without creep |
| Free joint named `floating_base_joint` | Distinct name for the plugin and tools |
| Tire collision: the vendor cylinder (r 0.0925 m, half width 0.0175 m) replaced by a 64-sided convex prism `tire_collision` of the same size, in the `wheel` class (condim 3, friction 0.8, priority 1) | MuJoCo's cylinder-box collision misses the edge contact of a tire meeting a step (see the Upkie notes); the vendor geoms used the `collision` class (condim 1, friction 0.4) although a `wheel` class was defined for them |
| Wheel body frames moved 9.7 mm along the axle to the tire centres (body pos, inertial and visual offsets compensated) | The MPC and the KF take one tire-centre offset for both wheels; the vendor tires sit at +-9.7 mm, mirrored. A shift along the spin axis changes nothing physically |

Joint armature (0.004), damping (0.02) and friction loss (0.1) are the vendor `pinappleV3` class defaults, unchanged; the per-joint classes (`hip`, `thigh`, `calf`) are defined but not applied in the vendor file, which is kept as is.

The arm-equipped assembly now composes `description/mujoco/common/pineapple/` (chassis/legs) and `common/pineapple_arm/` (arm/gripper), with one shared scene module.
The refactor preserves the original mass and dynamics; `pineapple` now names the arm-free chassis instead.
