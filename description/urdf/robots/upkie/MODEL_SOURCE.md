# Upkie Model Source

The wheeled biped is Upkie: one hip, one knee and one driven wheel per side, six actuators in total.

| Item | Source |
| --- | --- |
| Simulation reference | [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie) `src/mjlab_upkie/robot/upkie/robot.xml` at `d7895789a601943f78b3b9bcdc4def7b444f9913`, Apache-2.0 (Marc Duclusaud) |
| Meshes | The 27 STL files that MJCF references, copied unchanged to `description/meshes/robots/upkie/` |
| Effort and velocity limits | [upkie_description](https://github.com/upkie/upkie_description) `urdf/upkie.urdf` at `94735fbe6137276a41de0ff4cc04d2e533fa9e33`, Apache-2.0 |

The upstream URDF and MJCF are different CAD exports: 5.33922 kg against 5.4605 kg, 50 mm against 55 mm tire radius, and different link inertias and joint ranges.
The MJCF is the simulation reference, so the MPC URDF is generated from it rather than reconciled by hand.


## Changes to the MJCF

[`upkie.xml`](../../../mujoco/robots/upkie/upkie.xml) is the upstream `robot.xml` with these changes only:

| Change | Reason |
| --- | --- |
| `meshdir` points at `description/meshes/robots/upkie`, `texturedir` at `description/mujoco` | Repository layout |
| Bodies renamed `base`, `{left,right}_femur`, `{left,right}_tibia`, `{left,right}_wheel_link`; free joint `floating_base_joint` | One name per side; distinct link and joint names for Pinocchio frame lookups |
| Position servos (legs) and velocity servos (wheels) replaced by unit-gear `motor`s, `ctrlrange` = `forcerange` = +/-16 N m hip/knee and +/-1.7 N m wheel | The controller commands torque. `effort_pd` writes PD + feed-forward to `ctrl` every physics step, so the cap applies to the summed command |
| Sensors `imu_quat`, `imu_gyro`, `imu_acc` at the upstream `imu` site | The names the ros2_control IMU sensor reads, as for G1/H2 |

Joint armature (0.005 kg m^2), friction loss (0.1 N m), joint ranges, inertias, the 55 mm x 44 mm tire cylinders and `noslip_iterations=4` are unchanged.
[`scene.xml`](../../../mujoco/robots/upkie/scene.xml) selects a 0.5 ms timestep, the [Upkie night background](../../../mujoco/common/backgrounds/background_night_upkie.xml) and the robot module, in the same composition style as H2.
The background provides the unchanged flat-floor physics and H2-style studio appearance with lights aimed at `base`; `upkie.xml` owns the implicit-fast integrator and four no-slip iterations.
Optional ramp/uneven includes remain commented out, so the default wheel-drive checkpoint is still flat.


## Generated URDF

[`upkie.urdf`](upkie.urdf) is written by [`generate_upkie_urdf.py`](generate_upkie_urdf.py) from the MJCF: link frames, joint axes, masses, centres of mass and inertia tensors are copied unchanged, visuals keep the MJCF mesh placements, and collisions are the MJCF primitives.
Rerun the script after any MJCF change.

- Wheels are scalar `revolute` joints with a +/-1e4 rad placeholder bound, so nq = nv = 12 with the MPC's Translation + SphericalZYX base. URDF `continuous` would give nq = 14.
- The MPC masks wheel angles out of position limits and posture costs; the placeholder never acts as a bound.
- Armature has no URDF field. The MPC sets it from `ocs2.model.jointArmature`, and Pinocchio's RNEA and CRBA include it.
- Joint friction is configured separately through `ocs2.model.jointFriction` and `jointFrictionVelocity`; the joint inverse-dynamics rows and execution fallback add smoothed Coulomb compensation, not MuJoCo's exact static-friction law.
- An `imu` link sits at the MJCF `imu` site.

Checked against MuJoCo over six random base poses and joint configurations, with armature 0.005 in the Pinocchio model:

| Quantity | Largest difference |
| --- | --- |
| Total mass | 0 (5.4605 kg both) |
| Wheel body positions | 2.3e-9 m |
| Joint gravity torques | 4.5e-9 N m |
| Joint block of the mass matrix, with armature | 1.3e-10 |


## Joint conventions

All hinge axes are the local z axes. In the upright zero pose:

| Joint | World axis | Positive rotation |
| --- | --- | --- |
| `left_hip`, `right_hip` | +y, -y | Swings the leg backward (left), forward (right) |
| `left_knee`, `right_knee` | -y, +y | Swings the lower leg forward (left), backward (right) |
| `left_wheel`, `right_wheel` | +y, -y | Rolls forward (left), backward (right) |

The posture tables use mirrored values: `[q_lh, q_lk, q_lw, -q_lh, -q_lk, -q_lw]` is symmetric.
