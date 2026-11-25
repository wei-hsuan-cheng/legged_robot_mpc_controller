# Pineapple Wheel-Legged Robot

Pineapple runs Upkie's WB-ID MPC with rolling-wheel contacts ([Wheeled-Biped WB-ID MPC](wheeled_biped_whole_body_inverse_dynamics_mpc.md)) and Upkie's bring-up: it spawns seated, the wheel-aware linear KF initialises at rest, an offline contact-aware stand-up hands over to the MPC, and a planned sit-down returns it to its seat.
Model provenance and MJCF changes: [MODEL_SOURCE.md](../description/urdf/robots/pineapple/MODEL_SOURCE.md).

| Item | Pineapple | Upkie |
| --- | --- | --- |
| MPC joints | `L/R_hip_joint` (ab/ad), `_thigh_joint`, `_calf_joint`, `_wheel_joint` (8) | hip, knee, wheel (6) |
| Arm | 6 joints locked in the MPC at zero (stowed, resting on the zero stops), held by the hardware PD (kp 60/20) | none |
| Tire | radius 0.0925 m, half width 0.0175 m, axle y, frame at the tire centre ([geometry.yaml](../config/pineapple/geometry.yaml)) | 0.08 m, axle z |
| Heights | default 0.36 m, command range 0.25-0.44 m, posture table 0.14-0.46 m | 0.245 m |
| Idle support | base box (back bottom edge) and both tires | knee housings and both tires |
| Stand-up / sit-down | planned, 2 contact phases, 1.66 s to handover; sit-down = the stand-up reversed, 2.26 s | planned, 3 phases, 3.9 s; joint-space fold |
| Front | base_link +x (no command frame) | base turned by pi |

Costs, constraints and solver settings are Upkie's; the hip ab/ad posture weight is 10 (20 terminal), leg servo gains kp 30, kd 1 (wheels kd 0.1).
The posture table ([posture_table.yaml](../config/pineapple/posture_table.yaml), [compute_posture_table.py](../config/pineapple/compute_posture_table.py)) holds, per height, the level-trunk posture with hips not abducted, tires on the floor and the tire contact under the whole-robot CoM (arm stowed).

## Stand-up and sit-down

Upkie's method ([Upkie contact-aware stand-up](upkie_stand_up_optimization.md)) with Pineapple's supports, in [`config/pineapple/stand_up_to/`](../config/pineapple/stand_up_to):

| File | Role |
| --- | --- |
| `model.cpp` | Pinocchio RNEA, frames and CoM as CasADi functions (arm locked at zero, armature 0.004) |
| `plan.py` | IPOPT trajectory optimisation, and a MuJoCo inverse-dynamics/CoM parity check (5 states) before it |
| `feedback.py` | finite-horizon LQR tracking gains along the plan and the reversed plan; export |
| `rollout.py` | seated idle spawn (deepest fold: thigh 1.65, calf -3.15 rad), servo and state layout |
| `simulate.py` | MuJoCo replay of the export as the controller plays it (`--sit-down` for the reversed plan) |

The seated robot rests on the lowest base box's back bottom edge and both tires, its CoM between them.
The plan has two prescribed contact phases, each at most 1.5 s, the configurations, velocities, accelerations, torques, contact forces and durations optimised:

| Phase | Support | Purpose |
| --- | --- | --- |
| 1 | both tires, seat edge | roll the CoM over the tires; the seat force tapers to zero |
| 2 | both tires | rise to 0.30 m in the posture table's posture, at rest |

Every node satisfies the full inverse dynamics (3D, 8 joints, MJCF friction loss and damping); tires roll without slip; the seat edge holds while loaded; the corners of every thigh, calf and arm collision box stay 5 mm above the floor (without them the first solution folded the knees 18 cm through the floor); the seat clears by 30 mm after release; torques stay within 70 % of the actuator ranges.
The cost adds roll and left/right symmetry terms to Upkie's.
The MPC warm-up starts 0.25 of phase 2 after the release (1.13 s), the handover is 0.4 of phase 2 before the end (1.66 s).
The controller's support guard ([geometry.yaml](../config/pineapple/geometry.yaml) `knee_housings`) holds the MPC until the seat, as a disk of the box's half length along the base z axis, is 15 mm above the tires.

Sit-down: rigid-body dynamics and rolling contacts are time-reversible, so the stand-up played backwards (velocities negated, joint friction re-signed) is a dynamically consistent sit-down from rest at 0.30 m to the seat at rest, with its own LQR gains.
`feedback.py` exports it as a `sit_down` map next to `stand_up`; the controller (a new, optional feature) then has the MPC crouch to the map's first state (0.30 m, level, at rest, posture within 0.1 rad) and plays the map to its end, instead of Upkie's joint-space fold (which, from the 0.15 m crouch, dropped Pineapple 9 cm onto its seat and tipped it over).
Plans without a `sit_down` map (Upkie's) are unchanged.

```bash
R=~/src/wei-hsuan-cheng/MPC/legged_robot_mpc_controller   # in jetpack_jazzy, or the host path
bash $R/config/pineapple/regenerate.sh setup        # once: pip wheels in ~/pineapple_stand_up, C++ model, parity check
bash $R/config/pineapple/regenerate.sh plan 1.5     # about 20 min (IPOPT ends at its 1000-iteration limit, feasible)
bash $R/config/pineapple/regenerate.sh simulate 1.5 # MuJoCo replay, no ROS
bash $R/config/pineapple/regenerate.sh check 1.5    # 3 joystick stand/drive/sit cycles + forced fall, full ROS stack
bash $R/config/pineapple/regenerate.sh install 1.5  # after PASS
```

The shipped plan was solved on the host (MuJoCo 3.3.0, CasADi 3.8.2, Pinocchio 4.1.0) warm-started from a first solution without the clearance constraints; IPOPT stopped at its iteration limit and the retained iterate has 2.4e-7 constraint violation.
Offline replay: stand-up handover at 0.271 m, tilt 0.05 rad, millimetre tracking error, peak torque 42 % of the range, thighs and calves at least 6 mm off the floor; sit-down ends seated at rest (residual speed 0.002), maximum tilt 0.26 rad, seat touchdown peak 195 N (1.3 mg).
The stand-up ends about 0.2 m behind the spawn and turned by about 0.22 rad (the plan allows 0.25 m and 0.3 rad); the course test's line hold steers that out.

## Tests (2026-11-25, `jetpack_jazzy`, `~/whcheng_ws`)

| Run | Result |
| --- | --- |
| `upkie_posture_cycle_test.py --robot-name pineapple --cycles 3 --joystick --fall-test` (KF, C++ teleop with synthetic DualSense input) | PASS: 3 Square stand / L2 drive / Square planned sit-down cycles back to idle, a service stand-up, a 400 N shove latches `fallen` and every posture request is refused; max locomotion tilt 0.032 rad |
| Flat, KF, after the stand-up: heights 0.44/0.25/0.36 m, 0.5 m/s, turns up to 3 rad/s, coupled, reverse (34 s) | SUCCESS: min height 0.250 m, max roll 0.036, pitch 0.029 rad |
| `scene_course.xml`, KF, after the stand-up, 0.3 m/s with the line hold over the uneven field, ramps, 1 cm steps and the 10 cm one-wheel ramp (37 s) | SUCCESS: to x = 8.6 m, max roll 0.040, pitch 0.030 rad; the KF measures height from the tires, so the base rides 0.36 m above the local terrain |
| Ground-truth base, standing spawn (`initial_pose_standing.yaml`, no plan): flat 0.3 m/s; flat mixed; course | SUCCESS (before the stand-up existed): 0.307 m/s; max tilt 0.010 rad; course max pitch 0.020 rad |

```bash
cd ~/whcheng_ws && source install/setup.bash
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=pineapple mpcFreq:=200                      # GUI
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=pineapple mpcFreq:=200 baseCommandDevice:=joystick
# course: add mujocoModelFile:=scene_course.xml
T=src/legged_robot_mpc_controller/tests
python3 $T/upkie_posture_cycle_test.py --robot-name pineapple --cycles 3 --joystick --fall-test --prefix /tmp/pineapple/cycles
python3 $T/wb_id_walk_test.py --robot-name pineapple --scene scene_course.xml --source state_estimator \
  --estimator humanoid_linear_kf --speed 0 --settle 2 --ramp 0 --hold 32 --rest 3 --minimum-height 0.2 \
  --command-sequence "[[2,0.3,0,0.36,1],[32,0,0,0.36]]" --prefix /tmp/pineapple/course
```

Not tested: the physical DualSense on Pineapple (the teleop path is the same C++ node driven by synthetic `Joy`), jump/walk gaits (only `wheel_drive`), arm motion (stowed), model errors, real hardware.
Joint velocity limits are placeholders (the vendor model has none).
