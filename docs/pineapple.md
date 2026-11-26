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
| Gaits | `wheel_drive`, `jump`, `walk`, `trot`; vy selects `trot` (sideways steps with the hip ab/ad) | the same four; no vy |

Costs, constraints and solver settings are Upkie's; the hip ab/ad posture weight is 10 (20 terminal), leg servo gains kp 30, kd 1, wheels kd 2 (below).
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

## Robustness, gaits and sideways driving (2026-11-26)

[`tests/tuning/pineapple_suite.py`](../tests/tuning/pineapple_suite.py) runs 40 headless cases, each a
`wb_id_walk_test.py` run from the seated spawn and planned stand-up with the wheel KF driving the MPC, three in parallel
on their own ROS domains (`--list`, `--cases 'push_*'`, `--repeat N`, `--set KEY=VALUE` passed to every run;
results in `/tmp/pineapple_suite/TAG/summary.md`). Worst values are taken from 1.5 s after the takeover, so the stand-up
does not mask them; distances are in the robot's settled heading. Inputs: pushes
([`config/tests/pineapple/`](../config/tests/pineapple), 0.2 s at base_link; F N changes the speed by F / 74.6 m/s),
terrain ([`config/terrains/pineapple/`](../config/terrains/pineapple), generated at run time: head-on 20/30/40 mm
steps up and down, a 12-degree slope, a hard course with a rough field of 6-20 mm tile steps and a 24-34 mm entry edge,
20 mm up / 30 mm down steps and a 120 mm one-wheel ramp), the Upkie course, and full-range joystick play.

### Wheel drive: what limits it, and the tuning

Pushes were never the problem: 140 N forward/back (the robot thrown to 1.8 m/s), 80 N sideways, 20 N m pitch/roll/yaw
torques and diagonal pushes while driving at 0.5 m/s all recover with at most 0.14 rad tilt, before and after tuning.
Every failure was a tire meeting an edge head-on, and it has two parts:

1. **Physics.** On the edge of an h high step a tire of radius r = 92.5 mm touches with its normal tilted back by
   acos((r - h) / r): 38 degrees for 20 mm, 47 for 30 mm, 56 for 40 mm. With mu = 0.8 the friction cone reaches 38.7
   degrees from that normal, so above about 20 mm every force the edge can apply points backwards: no wheel torque
   climbs it, only momentum (30 mm costs m g h = 4.4 J; at 0.4 m/s the robot carries 1.2 J, at 0.8 m/s 4.8 J). The
   suite agrees: 20 mm passes at 0.4 m/s, 30 mm passes at 0.8 m/s and fails at 0.4, 40 mm fails slowly. The MPC has
   no terrain model and cannot see why the wheel stopped; stepping over is what the gaits are for.
2. **Wheel spin-up (fixed).** At the stall the tire slips on the edge or pops off it; a wheel has almost no inertia,
   so under the MPC's torque it reached 60-115 rad/s (6-10 m/s rim speed) within 0.1 s while the MPC still assumed
   rolling (its plan starts from the measured wheel speed), landed spinning and threw the robot to 2-4 m/s, and the
   legs folded. The wheel servo now damps towards the planned wheel speed with kd 2 (was 0.1). Swept against the MPC
   rolling-constraint gain (`tangentialVelocityGain` 50, 100), pitch/pitch-rate/vx/height weights and stiffer or softer
   leg posture, which all only moved failures around:

| wheel kd | step20 (0.4 m/s) | step30_fast | extreme_course_fast | extreme_course (0.4 m/s) |
| --- | --- | --- | --- | --- |
| 0.1 (before) | 5/6, and a near-fall at 0.43 rad | 1/1 | 4/6, and a near-fall at 0.67 rad | 4/6, every pass at 0.6-0.75 rad |
| 2.0 (shipped) | 4/4 at 0.01-0.02 rad | 3/3 at 0.02 rad | 4/4 at 0.01-0.03 rad | 2/4; passes at 0.02-0.04 rad |
| 3.0 | 2/2 | 2/2 | 2/2 | 1/2 |

The slow extreme course stays a coin toss: its rough field starts with a 24-34 mm edge at 0.4 m/s, the limit above.

### Gaits and vy

The `jump`, `walk` and `trot` templates are Upkie's, retimed for Pineapple (`ocs2.gait.templates`): requested on
`/humanoid/gait_command` or with the DualSense (legged_robot_teleop: Circle cycles walk -> trot -> wheel_drive,
Triangle jumps while held), they keep rolling while they step. With the hip ab/ad the body shifts over the stance
wheel, so the lifts are 0.25 s (Upkie 0.15 s). Trot's double support is 0.1 s: with 0.05 s a stance wheel stalled at
the Upkie course's 10 mm step fell 2 times in 4, with 0.1 s none in 4 (and 3/3 rolling on flat ground). The planner scales the swing height by
duration / `swingTimeScale` (0.4 s), hence the measured clearances.

Sideways: the tires cannot roll sideways, so `maxDisplacementVelocityY` (0.3 m/s) is served by stepping. A gait-ladder
rung can now carry its own `lateralVelocity` band (a new, optional `ocs2.gait.transitions.<rung>` entry; without it the
rung uses `linearVelocity` for vy as before, so G1/H2/Upkie are unchanged): `wheel_drive` serves any vx and yaw rate
but only |vy| < 0.05 m/s, and `trot` takes over until |vy| < 0.03 m/s. No cost changed: the MPC places the lifted wheel
sideways with the hip ab/ad by itself.

| case | result |
| --- | --- |
| `jump` in place / rolling 0.3 m/s / turning 1 rad/s | pass; both tires off 0.13-0.15 s, 17 mm clearance, 260-300 N landing per wheel, tilt <= 0.016 rad; in place it drifts 0.26 m forward over 8 s |
| `walk` in place / rolling | pass; one tire off 0.22-0.24 s, 30 mm, tilt <= 0.03 rad |
| `trot` in place / rolling | pass; one tire off 0.23-0.25 s, 24 mm, tilt <= 0.03 rad |
| vy 0.2 m/s left / right for 6 s | pass; 1.28-1.34 m sideways (1.2 m commanded plus the stop), 0.15-0.2 m forward drift, 0.05-0.08 rad yaw drift, roll <= 0.036 |
| vy with vx and with turning | pass, roll <= 0.07 rad |
| 50 N sideways pushes in trot, 60 N fore/aft in walk | pass, tilt <= 0.05 rad |
| walk, trot rolling 0.3 m/s over the Upkie course (ramps, 10 mm steps, 10 cm one-wheel ramp) | walk 3/3; trot 4/4 with 0.1 s double support (2/4 with 0.05 s); roll <= 0.05 rad |
| gait switching at 0.5 m/s (walk, trot, jump, wheel_drive, trot, wheel_drive within 9 s) | pass, roll 0.065 rad |
| walk/trot over the hard course's rough field, walk/jump onto the 40 mm step | fall |

The last row is the open part: the wheel KF measures the base height above the plane of the tires (both at z = 0)
and the swing planner lands on flat ground, so a tire put down on a 34-40 mm higher tile is a model error the gait
cannot absorb; stepping up needs terrain-aware height estimation (`reference.groundHeight.useEstimate` assumes an
estimator that does not pin the contacts to zero).

Final suite (`--tag final`, one run each): 32/38 pass; the 6 failures are the slow 40 mm step, the slow hard course,
and the four gait cases of the last row. In 3 of about 120 suite runs a robot held its height 3-5 cm low for a whole
run (first runs of a parallel batch); with the estimate logged, estimate and ground truth agree within 8 mm, and it
did not recur in a fresh batch: not diagnosed.

```bash
cd ~/whcheng_ws && source install/setup.bash
S=src/legged_robot_mpc_controller/tests/tuning/pineapple_suite.py
python3 $S --tag mine                                   # all 40 cases, about 10 min
python3 $S --tag gaits --cases 'jump_*' 'walk_*' 'trot_*' 'vy_*' --record-tracking
python3 src/legged_robot_mpc_controller/tests/tuning/analyze_upkie_flight.py /tmp/pineapple_suite/gaits/jump_place --robot pineapple
ros2 topic pub --once /humanoid/gait_command std_msgs/String "{data: trot}"   # interactive; "{data: ''}" releases
```

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

Not tested: the physical DualSense on Pineapple (the teleop path is the same C++ node driven by synthetic `Joy`), stepping onto higher ground (above), arm motion (stowed), model errors, real hardware.
Joint velocity limits are placeholders (the vendor model has none).
