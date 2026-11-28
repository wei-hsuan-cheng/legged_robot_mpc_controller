# Pineapple Chassis Model Source

`pineapple` is the arm-free Pineapple V3 chassis: hip ab/ad, thigh, calf and driven wheel per leg, eight actuators, 10.92 kg.
The original 14.919995 kg arm-equipped robot is now [`pineapple_manipulator`](../pineapple_manipulator/MODEL_SOURCE.md).
Both reuse the vendor meshes in `description/meshes/robots/pineapple/`.

The MuJoCo model is assembled from `description/mujoco/common/pineapple/`.
The arm subtree, its mass/inertia, collisions, sensors and actuators are absent; the original chassis and leg parameters are unchanged.
The arm-equipped variant adds `common/pineapple_arm/` at the original mounting transform.

[`pineapple.urdf`](pineapple.urdf) is generated from the assembled MJCF by [`generate_pineapple_urdf.py`](generate_pineapple_urdf.py).
The same generator supports `--robot pineapple_manipulator`, expanding shared include fragments before copying frames, axes, masses and inertias.
Effort/velocity limits, wheel approximations and provenance are detailed in the arm-equipped model source.

The first arm-free trial intentionally keeps the old numeric MPC/servo/gait settings and height/posture references.
Its stand-up file links to the original arm-equipped TO; no arm-free TO has been generated.
Only the stand-up/basic MPC entry was tested, not arm-free terrain or recovery performance.
See [Pineapple family](../../../../docs/pineapple.md) for commands, baseline scope and results.
