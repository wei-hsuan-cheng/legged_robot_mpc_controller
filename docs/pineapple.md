# Pineapple Wheel-Legged Robot

Pineapple runs Upkie's WB-ID MPC with rolling-wheel contacts ([Wheeled-Biped WB-ID MPC](wheeled_biped_whole_body_inverse_dynamics_mpc.md)); the differences are the robot files and a hip ab/ad joint per leg.
Model provenance and MJCF changes: [MODEL_SOURCE.md](../description/urdf/robots/pineapple/MODEL_SOURCE.md).

| Item | Pineapple | Upkie |
| --- | --- | --- |
| MPC joints | `L/R_hip_joint` (ab/ad), `_thigh_joint`, `_calf_joint`, `_wheel_joint` (8) | hip, knee, wheel (6) |
| Arm | 6 joints locked in the MPC at zero (stowed, resting on the zero stops), held by the hardware PD (kp 60/20) | none |
| Tire | radius 0.0925 m, half width 0.0175 m, axle y, frame at the tire centre ([geometry.yaml](../config/pineapple/geometry.yaml)) | 0.08 m, axle z |
| Heights | default 0.36 m, command range 0.25-0.44 m, posture table 0.20-0.46 m | 0.245 m |
| Start | spawned standing at 0.36 m, MPC on the ground-truth base from the first tick (no stand-up plan) | kneeling, planned stand-up, linear KF |
| Front | base_link +x (no command frame) | base turned by pi |

The posture table ([posture_table.yaml](../config/pineapple/posture_table.yaml), from [compute_posture_table.py](../config/pineapple/compute_posture_table.py)) holds, per height, the level-trunk posture with hips not abducted, tires on the floor and the tire contact under the whole-robot CoM (arm stowed).
Costs, constraints and solver settings are Upkie's; the hip ab/ad posture weight is 10 (20 terminal), leg servo gains are kp 30, kd 1 (wheels kd 0.1).
`robot.launch.py robotName:=pineapple` defaults to `floatingBaseSource:=ground_truth_state`, with the linear KF logged alongside.
The KF does not drive control yet: the estimated-base bring-up needs a planned stand-up (Upkie's) or a resting start, which Pineapple does not have.

## Headless tests (2026-11-25, `jetpack_jazzy`, ground-truth base)

| Run | Result |
| --- | --- |
| Flat, 0.3 m/s forward for 10 s | SUCCESS: 0.307 m/s, 3.07 m for 3.0 expected, yaw drift 1.6 mrad, max pitch 0.005 rad |
| Flat, heights 0.44/0.25/0.36 m, 0.5 m/s, turns up to 3 rad/s, coupled, reverse (34 s) | SUCCESS: min height 0.249 m, max roll 0.004, pitch 0.010 rad |
| `scene_course.xml` at 0.3 m/s with the line hold: uneven field, 4 cm ramp, 1 cm steps, ramp down, 10 cm one-wheel ramp (35 s) | SUCCESS: max roll 0.003, pitch 0.020 rad; the legs take the terrain (calf -1.61 -> -1.89 rad on the 5 cm step; -2.16 left / -1.55 right on the one-wheel ramp) with the trunk level |

The height reference is the world height (`groundHeight.useEstimate: false`, as Upkie), so on raised terrain the legs fold to keep the base at the commanded world height.
Not tested: the KF closed loop, the joystick frontend (`legged_robot_teleop`'s launch lists only g1/h2/upkie), jump/walk gaits (only `wheel_drive` is configured), real hardware.

```bash
cd ~/whcheng_ws && source install/setup.bash
T=src/legged_robot_mpc_controller/tests/wb_id_walk_test.py
python3 $T --robot-name pineapple --axis vx --speed 0.3 --settle 4 --ramp 2 --hold 8 --rest 4 --minimum-height 0.2 --prefix /tmp/pineapple/flat_vx
python3 $T --robot-name pineapple --scene scene_course.xml --speed 0 --settle 2 --ramp 0 --hold 30 --rest 3 --minimum-height 0.2 \
  --command-sequence "[[2,0.3,0,0.36,1],[30,0,0,0.36]]" --prefix /tmp/pineapple/course
```
