# Upkie Model Source

The wheeled-biped is Upkie: one hip, one knee and one driven wheel per side, six actuators in total.

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
| The base body spawns turned by pi about z (`quat="0 0 0 1"`) | The base +x points toward the knees, Upkie's back; turned, the front faces world +x and the test terrain. Commands use the `base_forward` control frame (`target.commandFrameYaw: pi`). The URDF generator ignores the root body's pose, so `upkie.urdf` is unchanged |
| Tire collision geoms `{left,right}_foot_collision` are a 64-sided convex prism mesh `tire_collision` (circumradius 0.055 m, half-width 0.022 m, same pose) instead of `cylinder` primitives | MuJoCo's cylinder-box collision never produces the edge contact of a tire meeting a step: see below |

Joint armature (0.005 kg m^2), friction loss (0.1 N m), joint ranges, inertias, the 55 mm x 44 mm tire size and `noslip_iterations=4` are unchanged.
[`scene.xml`](../../../mujoco/robots/upkie/scene.xml) selects a 0.5 ms timestep, the [Upkie night background](../../../mujoco/common/backgrounds/background_night_upkie.xml) and the robot module, in the same composition style as H2.
The background provides the unchanged flat-floor physics and H2-style studio appearance with lights aimed at `base`; `upkie.xml` owns the implicit-fast integrator and four no-slip iterations.
Optional ramp/uneven includes remain commented out, so the default wheel-drive checkpoint is still flat.


## Tire collision shape

MuJoCo 3.13.0 (the version `mujoco_ros2_control` links) handles a primitive `cylinder` against a `box` step with two contacts on the tire rims whose normal is the step's vertical face, (1, 0, 0).
The contact a real tire makes, at the step's top edge with a normal from the edge to the hub (about 50 degrees from vertical at 20 mm), is never generated.
The step therefore acts as a wall until the hub has passed the face; then the top face pushes the tire up by the whole step height at once.
A convex mesh prism of the same size gets the edge contact at every depth tested (0.5-15 mm), as a sphere does.
Its normal is the nearest facet's, so it is quantized to the 5.6-degree facet spacing: 1.5 degrees off the true normal at 10 mm and 2.7 degrees at 20 mm.

A 2.7 kg wheel on a slider (half an Upkie), held at 0.2 m/s by a force-limited servo, must be pushed with at least W tan(theta) to roll over a step undriven: 18.6 N at 10 mm and 32 N at 20 mm.

| Step | Primitive cylinder | 64-sided prism |
| --- | --- | --- |
| 10 mm | Climbs at 10 N, sinking 10.2 mm into the step | Blocked at 10 N, climbs at 20 N, 0.8 mm penetration |
| 20 mm | Blocked at 10 and 20 N; climbs at 40 N only by tunneling 20.6 mm into the step | Blocked at 10 and 20 N, climbs at 40 N, 1.1 mm penetration |

The prism matches the analytic threshold; the cylinder makes 10 mm steps easier and 20 mm steps harder than they are.
On flat ground each prism tire touches with four rim vertices at 0.1-0.2 mm depth; the rolling-radius ripple of a 64-gon is r (1 - cos(pi/64)) = 0.07 mm.
[`generate_upkie_urdf.py`](generate_upkie_urdf.py) writes the prism back as its 0.055 m x 0.044 m cylinder, so `upkie.urdf` is unchanged.


## Generated URDF

[`upkie.urdf`](upkie.urdf) is written by [`generate_upkie_urdf.py`](generate_upkie_urdf.py) from the MJCF: link frames, joint axes, masses, centres of mass and inertia tensors are copied unchanged, visuals keep the MJCF mesh placements, and collisions are the MJCF primitives (the tire prisms as cylinders).
Rerun the script after any MJCF change.
The quaternion-to-RPY conversion explicitly handles pitch at +/-90 degrees by choosing zero yaw and preserving the remaining roll/yaw combination.
The earlier conversion evaluated ambiguous zero-over-zero angle formulas at these orientations, misrotating six visual meshes and four collision primitives and making the leg parts appear disconnected in RViz.
The fix changes only those geometry rotations; all seven joint definitions and seven inertial blocks are exactly unchanged, as are the MJCF and controller parameters.
This is an asset-conversion issue, not a change to the MPC's runtime orientation representation.
The singular-angle convention is the same kind of nonunique-angle handling described in [SciPy's Euler conversion documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html); the generator itself uses only Python's standard library.
Regression coverage in [`test_upkie_urdf.py`](../../../../tests/test_upkie_urdf.py) checks exact/near-singular rotations, every source quaternion and all generated visual, collision, joint and IMU poses.

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
