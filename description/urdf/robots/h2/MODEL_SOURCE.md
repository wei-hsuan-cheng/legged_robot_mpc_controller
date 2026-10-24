# Unitree H2 Model Sources

The 31-DoF H2 description comes from Unitree's official repositories, not H1-2, H2 Plus or a third-party reconstruction.

| Asset | Official source and pinned revision |
| --- | --- |
| URDF and hand meshes | [unitree_ros H2](https://github.com/unitreerobotics/unitree_ros/tree/15ebe53f4cd77e7d745ae83b9d7ff90e685e8998/robots/h2_description) |
| MJCF and robot meshes | [unitree_mujoco H2](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/h2) |
| Licenses | `description/meshes/robots/h2/LICENSE.unitree_ros` and `LICENSE.unitree_mujoco` |


## Local Integration Changes

The URDF retains the official joint limits, masses, inertias and physical joint axes, with package-relative mesh URIs and added massless heel/toe estimator frames.
The ankle-pitch frames are translated along their own pitch axes by +0.0365 m on the left and -0.0365 m on the right, with opposite translations applied to their inertial and geometry origins in both formats; this is a coordinate reframe, not a geometry or joint-axis change.
It lets the existing symmetric MPC contact-frame translation represent both soles without a new controller abstraction.
The contact frame is `[0.035, 0, -0.058]` relative to each reframed ankle-pitch joint; the final leg joint is pitch, unlike G1's roll joint.

The official MJCF omits explicit pelvis inertia and therefore infers approximately 4.248 kg from the pelvis mesh, whereas the official URDF specifies 8.5477 kg.
The integrated MJCF uses the URDF pelvis mass, center of mass and full inertia tensor, giving approximately 75.58746 kg in both simulation and the OCP.
The existing MJCF wrist-yaw inertias include the fixed hands; they are not replaced with the smaller URDF wrist-only inertias.
The pelvis IMU site is placed at the official URDF `imu_in_pelvis` location, and its existing MuJoCo sensor names are mapped in the ros2_control xacro.
The floating base has zero artificial damping, friction and armature, while articulated joints retain Unitree's MJCF defaults; the integrator, contact solver settings, timestep and scene backgrounds follow G1.


## Configuration

H2 uses the existing WB-ID controller with 25 active joints: 12 leg, three waist, two head and eight shoulder/elbow joints.
The six wrist joints remain locked in the reduced MPC model, as for G1; all 31 joints are present in simulation and ros2_control.
Every inherited joint-dependent array is mapped by joint name, including the swapped ankle ordering, and both head joints use the existing Cartesian orientation/angular-velocity tracking cost on `head_yaw_link`.
There is no generic torso frame-tracking cost; `torsoTiltCost` remains enabled.

G1's deep crouch makes the H2 feet overlap because H2 has different hip-axis geometry, so the H2 model-dependent spawn and nominal standing postures are solved with level soles at y=+/-0.14 m and their midpoint aligned with the model's CoM in x.
That midpoint is approximately x=0.04961 m in the crouch and x=0.01771 m in the standing pose, relative to the pelvis; using the same x=0.04 m in both poses caused a settling failure and is not the retained configuration.
The initial pelvis heights are 0.882 m for spawning and 1.02 m for the standing reference; these are geometry-dependent initial settings, not a claim that G1's raw joint angles can be transplanted unchanged.
Control gains, solver settings, cost weights and gait timings start from G1 before any measured H2 tuning.
The retained H2 gain, velocity-weight, contact-moment and command-range changes are recorded in [H2 Integration and Validation](../../../../docs/h2_wb_id_mpc.md).
