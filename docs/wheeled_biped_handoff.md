# Wheeled-Biped (Upkie) WB-ID MPC: Handoff

Branch `feat/wheeled_biped`, based on `jazzy` at `ae134c0`.
The original [implementation checklist](design_notes/wheeled_biped/wheeled_biped_first_step.md) and [research survey](design_notes/wheeled_biped/wheeled_biped_wb_id_mpc_survey.md) are preserved as working notes under [design_notes/wheeled_biped](design_notes/wheeled_biped/README.md), separate from the formal implementation documentation.
Update this file at each milestone.
The implemented equations, current weights, rolling-contact equalities and execution limits are collected in [Wheeled-Biped WB-ID MPC](wheeled_biped_whole_body_inverse_dynamics_mpc.md).


## Demo Video Pipeline (2026-11-15, Claude)

`tests/upkie_demo.py full` is one run from the kneeling spawn: stand-up, drive along the course line over the uneven field and the full-width ramp, stop at the far edge of the middle step and spin across its 1 cm edge at 180 deg/s while the height goes max, min, max, min, default, turn back to the line, cross the 10 cm one-wheel ramp, stop.
`tests/upkie_demo_video.sh` turns it into a two-row video, RViz on top (estimated state, MPC optimized trajectory, future ghosts) and MuJoCo below, with the same camera and field of view and frames matched by sim time.
Nothing is filmed live: the run is headless and logged, and both rows are rendered afterwards.

1. `record`, in the container: the harness runs the demo with the wheel KF in control and writes the GT base pose and joint angles at 100 Hz (`run_replay.json`) and the captions (`run_labels.json`); a `ros2 bag` records `/tf`, `/tf_static`, `/robot_description` and the `/performance_visualization` markers alongside.
2. Still `record`: [`render_rviz_replay.py`](../tests/tuning/render_rviz_replay.py) starts RViz with [`upkie_video.rviz`](../config/rviz/upkie_video.rviz) (a side camera following `base`) and, for every video frame, publishes that sim time's `/clock`, base TF and markers, drives a local `robot_state_publisher` with the replay's joint angles, waits for RViz to draw and screenshots its 1280x720 render area with `xwd`.
   The recorded link TF is not used because `robot_state_publisher` throttles it to 20 Hz, which visibly lags the legs during the spin.
3. `render`, on the host: [`render_upkie_video.py`](../tests/tuning/render_upkie_video.py) `--rviz-frames` replays the run in MuJoCo 3.13 with the same camera (focal point 0.1 m below the base, 1.6 m, 14 degrees down, from world -y, vertical FoV 45 degrees as in RViz), stacks it under the RViz frame of the same index, and writes the captioned and the clean video.

Everything lands in the git-ignored `demo_videos/NAME/`; the repository is the same folder on the host and in the container.
Timing: run about 8 min, RViz capture about 7 min (its window shows on the VM desktop; keep it uncovered), render about 4 min.
The stop on the step is timed (`FULL_STOP_ON_STEP` in `upkie_demo.py`), not position-triggered; the robot's progress repeated within about 2 cm between runs.
Measured in the recorded run: 8 wheel crossings of the edge, 1.83 turns at 1.5 rad/s mean (3.28 turns at 2.75 rad/s on the flat step top), up to 0.5 m sideways drift, roll at most 0.048 rad, the second "min" height reached 0.27 m instead of 0.22 m while crossing the edge.

### Reproduce the demo and the video

One-time setup on the Mac, for the renderer (MuJoCo must be 3.13, the version the simulator links):

```bash
python3 -m venv ~/.venvs/upkie_render
~/.venvs/upkie_render/bin/pip install mujoco==3.13.0 numpy pillow
which ffmpeg || brew install ffmpeg
```

1. Record, in the VM (about 15 min). The RViz window appears on the VM desktop during the capture and the screenshots take whatever is on screen there, so keep it uncovered:

   ```bash
   docker exec -it ros_jazzy bash
   /root/src/mpc/legged_robot_mpc_controller/tests/upkie_demo_video.sh record upkie_course
   ```

2. Render, on the Mac (about 4 min):

   ```bash
   cd ~/src/mpc/legged_robot_mpc_controller
   PYTHON=~/.venvs/upkie_render/bin/python tests/upkie_demo_video.sh render upkie_course
   open demo_videos/upkie_course
   ```

`upkie_course` is the folder name under `demo_videos/`; use another to keep several takes.
It holds `run.json` (verdict and metrics), `run_replay.json`, `run_labels.json`, `run.out` (harness log), `bag/` and `bag.log`, `rviz/` (frames, about 270 MB, and `rviz.log`), `upkie_course.mp4` (captions) and `upkie_course_clean.mp4` (3D only).
`record` exits with the demo's status, nonzero when the robot fell; it still captures RViz as long as the MPC took over.

What to change:

| What | Where |
| --- | --- |
| Demo timing: stop at the step edge, seconds per height step, turn-back and finish durations | `FULL_STOP_ON_STEP` (17.0 s after the MPC takes over), `FULL_SPIN_HEIGHT_SECONDS`, `FULL_REALIGN_SECONDS`, `FULL_FINISH_SECONDS` in `tests/upkie_demo.py` |
| Course | `config/terrains/upkie/course_params.yaml`, then `python3 description/mujoco/terrains/generate_upkie_course.py` |
| Camera (must match in both) | `config/rviz/upkie_video.rviz` (Orbit Distance, Pitch, Focal Point Z) and `FOLLOW_Z`, `FOLLOW_DISTANCE`, `FOLLOW_ELEVATION` in `tests/tuning/render_upkie_video.py` |
| RViz content and colors | the displays and Background Color in `config/rviz/upkie_video.rviz` |

Known issue: RViz shows the robot model in plain white for the first frames (about 3 s in the 2026-11-15 take, frame 93) before its materials appear.
Trim it afterwards from the first colored frame, the same for both videos so the rows stay in sync:

```bash
ffmpeg -i demo_videos/upkie_course/upkie_course.mp4 -vf "trim=start_frame=93,setpts=PTS-STARTPTS" \
  -c:v libx264 -crf 20 -pix_fmt yuv420p demo_videos/upkie_course/upkie_course_trim.mp4
```


## Default Bring-up: Kneeling Spawn, Stand-up, Linear KF (2026-11-15, Claude)

A plain `ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=upkie mpcFreq:=200` now spawns Upkie kneeling, initialises the linear KF there, plays the planned stand-up and hands over to the MPC, with no ground truth in control.
`config/upkie/initial_pose.yaml` is the kneeling idle pose (the former `initial_pose_idle.yaml`; the standing pose file is gone, its joint values remain the YAML's `defaultJointState`), and the Upkie YAML sets `bringUp.standUpTrajectoryFile` to `$(find-pkg-share legged_robot_mpc_controller)/config/upkie/stand_up_trajectory.yaml`.
The launch file is unchanged; `floatingBaseSource` and `floatingBaseEstimator` already defaulted to `state_estimator` and `humanoid_linear_kf`.
Scripts no longer pass the stand-up arguments; `upkie_acceptance_suite.sh` now starts kneeling too, with `--minimum-height 0.05` for the catch at about 0.09 m.
Verified with a plain headless launch: kneeling spawn at 0.1346 m, idle, standing up, blend, locomotion, standing at 0.300 m on the estimate, MPC about 190 Hz; `tests/test_robot_launch.py` covers the defaults.


## Estimator Note and H2 Regression (2026-11-15, Claude)

[Wheeled-biped state estimation](wheeled_biped_state_estimation.md) derives the wheel-aware linear KF with code line references, its steady-state gains and its differences from Cheetah 3, legged_control and Hosseini et al.

H2 regression with the shared code of this branch: `humanoid_linear_kf` in control, the uneven field without the ramp, vx 0.2 m/s, the 2026-10-31 `nonuniform_1031/final_linear/balanced_rough_1` protocol and scene, unchanged H2 YAML.
The baseline predates H2's switch from the walk to the trot ladder (`ae134c0`), so the gait differs.

| run | verdict | distance | walk vx | min z | max roll / pitch | max yaw error | MPC rate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 2026-10-31 baseline (walk ladder) | success | 2.61 m | 0.280 m/s | 0.934 m | 0.156 / 0.099 | 0.088 | 94.1 Hz |
| HEAD, 3 runs (trot ladder) | success 3/3 | 2.60-2.76 m | 0.256-0.307 m/s | 0.956-0.997 m | 0.060-0.160 / 0.023-0.067 | 0.109-0.160 | 93.7 Hz |

Same performance: the walk entry is smoother than the baseline's (no 1 m/s forward spike or 6 cm height dip), and the worst mid-walk roll (run 3, 0.16 rad on the tiles) matches the baseline's peak.
Rendered frames at each run's worst moment show normal trotting with flat footholds and no leg crossing.

[Ledge drop](results/upkie_ledge_drop.md): Upkie drives off a 5 cm and a 10 cm edge at 0.3 m/s, wheel KF driving and GT driving, all 4 pass.
The KF hides the drop from the MPC (error -16/+7 mm, within 5 mm in 0.17 s, the same decay as when it is only logged); GT driving crouches by the platform height and reaches down at the edge.
Run with `tests/tuning/upkie_ledge_drop.sh`, plot with `tests/tuning/plot_upkie_ledge_drop.py`.


## Faster Commands and the Joystick Demo (2026-11-15, Claude)

### Command rate limits

At the user's request `velocityCommandRateLimits` went from `[0.5, 0.5, 0.1, 2.0]` to `[10, 10, 2, 40]` (20x: [ax, ay] m/s^2, height rate m/s, yaw acceleration rad/s^2), so GUI and joystick moves take effect within a few solves.
The first run with the 2 m/s height limit fell right after the stand-up handover: the handover restarts the height reference at the measured 0.09 m, and at 2 m/s its rise to 0.30 m is a near step (the same mechanism as the clamp-on-reference failure below).
`ocs2.reference.takeoverHeightRate` (Upkie 0.1 m/s, default 0 = off for G1/H2) now sets the rate of that rise only; the command limit applies once the reference reaches the command.
With it the stand-up and 0.3 m/s roll are unchanged: +2.06 of 2.10 m, 0.29 m reached 2.08 s after takeover, max pitch/roll after the catch 0.007/0.000 rad.

### Locomotion demo on a terrain course (pure wheel_drive)

- `--terrain upkie_course` (`config/terrains/upkie/course_params.yaml`, `generate_upkie_course.py`) along world +x from the spawn: an uneven field at 0.5..1.5 m; a ramp up to 40 mm at 1.6..2.4 m, a plateau with a 10 mm step up at 2.9 m and down at 3.5 m, and a ramp down at 4.0..4.8 m, all 6 m wide (both wheels); then a narrow 40 mm ramp up and down at 5.4..7.0 m, y = 0.04..0.34 m, under the +y wheel only (wheels at y = +-0.19 m), the other wheel on the floor.
  Flat floor behind the spawn and beyond 7.0 m.
- `tests/upkie_demo.py standup|course|scan` runs the demos with the wheel KF in closed loop from the kneeling stand-up (`--source ground_truth_state` for GT).
  `course`, pure `wheel_drive` (the user dropped jump, walk and trot from this test; their designs need more work): a height test standing still (0.33, 0.22, 0.30 m, 2 s each), a straight pass at 0.3 m/s over the field, the full-width ramp/plateau/step/ramp down and the one-wheel ramp, then 24 s of seeded random joystick play on the floor beyond (1.2-2.5 s segments of pure translation, pure rotation, coupled motion or none).
  Default seed 6: both turn directions, a reverse segment, 4 pure rotations, 3 pure translations, 4 coupled; `scan` lists others.
- The stand-up catch leaves the heading 0.18-0.21 rad off and varies by about 0.03 rad between runs, too much for an open-loop pass to hit a 0.3 m wide ramp 6 m away.
  The harness therefore steers the pass like an operator: `--command-sequence` entries with a fifth element 1 take their yaw rate from the robot's own estimated heading and lateral offset from the spawn line (`wb_id_walk_test.line_hold_yaw_rate`; GT only when GT drives the controller).
- `tests/tuning/render_upkie_video.py PREFIX` renders a run to MP4 with the captions the demo writes (`PREFIX_labels.json`; the ramp-pass captions come from where the robot actually was).
  It replays the logged states offscreen (the simulation runs headless); free camera following the base from the side, azimuth 90 deg, so world +x (Upkie's front at spawn) is on the right; MuJoCo's default 45 deg vertical field of view.
  The "<-- FRONT" marker of the earlier stand-up videos pointed the wrong way and was removed.

### Results (wheel KF in control, from the kneeling stand-up)

| run | verdict | notes |
| --- | --- | --- |
| `standup` | pass | catch: hips split up to 1.42 rad, right wheel off 0.18 s, both off 90 ms, rolls 24 cm back at up to 0.78 m/s; standing at 0.30 m 2.15 s after takeover |
| `course`, seeds 6 and 2 | pass, pass | height test 0.330 / 0.220 / 0.300 m measured; pass at 0.29 m/s within 0.05-0.08 m of the line; field, ramps, plateau, 10 mm step and one-wheel ramp with max roll 0.004 rad and pitch 0.015 rad; on the one-wheel ramp the +y wheel rose 40 mm while the base stayed level (roll 0.002 rad); play max roll/pitch 0.002/0.006 rad |
| `course`, seed 9 (back field still in place) | pass | |
| `course`, seed 6 (back uneven field 0.5 m behind the spawn) | fall | 2.1 s after takeover: the stand-up catch rolled back 1.1 m (usually 0.25-0.35 m) onto that field; the field was removed |

### Found on the way

- **Full-width 20 mm step head-on** (both wheels hit the edge together), holding the line: 0.15 m/s stalls at the edge, 0.2 and 0.3 m/s fall, also with GT, so not the estimator; 10 mm passes at 0.3 m/s.
  At 0.3 m/s the wheels stop at the edge (0.29 -> 0.12 m/s), then the robot surges to 0.58 m/s, pops up and collapses from 0.34 to 0.08 m.
  Oblique crossings of the 20 mm step passed (one wheel at a time), as does the 20 mm one-wheel strip; the course step is 10 mm until this is solved.
- **Jump gait** (out of this test for now): it drifts forward 0.3-0.5 m/s with a zero speed command; jumping while driving moves 1.3-3x the commanded speed; one fall on flat floor 0.5 s after switching from `jump` back to `wheel_drive` (surge to 1.57 m/s, base collapsed); 2 of 3 earlier course seeds fell when a landing straddled a terrain edge; above about 0.31 m the legs lack stroke (0.323 m: in place one 0.70 rad pitch bounce, with turning a fall).
  Jumps at 0.30 m on flat ground, driving + turning or spinning in place, pass.
- **Yaw tracking while driving** is about 50% of the command (driving speed about 100%).
- **Stand-up catch roll-back** varies run to run (0.25-1.1 m); keep the area behind a kneeling start flat.

## Forward Control Frame (2026-11-15, Claude, after 08:05)

The user reported front and back reversed: the knees point to Upkie's back, but the base frame's +x, and so the GUI's forward vx, pointed toward the knees.
Checked in the model: at the 0.30 m posture the knee joints sit at base x = +0.096 m, ahead of the hips at x = 0, and a positive vx command drove along base +x.
So the robot's true front is base -x, and the joints named `left_*` are on the robot's right when it faces its front.

### What was done, and why this way

Commands are now given in a control frame `base_forward`, the base turned by pi about z, whose +x points away from the knees (`target.commandFrame: base_forward`, `target.commandFrameYaw: pi`).
The rotation is applied once, where commands enter the controller ([`ros2_procedural_mpc_motion_manager.cpp`](../src/common/ros2_procedural_mpc_motion_manager.cpp)): twist (vx, vy) is rotated into the base frame, the yaw rate and height are unchanged, and a base-pose command gives the control frame's orientation.
The launch publishes `base_forward` as a static TF child of `base`, so RViz shows the true front.
The robot spawns turned by pi (`upkie.xml` base `quat="0 0 0 1"`), so its front faces world +x and the test terrain; the URDF generator ignores the root body's pose, so `upkie.urdf` is byte-identical.
The harness and `evaluate_upkie_run.py` measure forward along the control frame.

Planning and estimation stay in the URDF base frame, and need no change: a fixed turn about the vertical leaves the height and the zero pitch/roll references, the joint-space posture table and stand-up plan, the frame-free wheel equalities, the IMU mounting (relative to the base) and the base-frame KF as they are.
Re-rooting the model at the forward frame would regenerate the URDF and every CppAD library, re-derive the IMU mounting and re-export the stand-up plan for identical behaviour, and the `left_*` names would still be mirrored.
`commandFrameYaw` defaults to 0, so G1/H2 are unchanged (H2 drive smoke run passes).

Convention for reading older sections of this file: they call base +x "forward", which is the true back.
For example "pitched back 0.71 rad" at the stand-up handover means the trunk leaned 0.71 rad toward the true front, and the catch's "overshoots forward at 1-2 m/s" is rolling backward.
In true terms the stand-up is: kneel with the knees behind the wheels and the trunk leaning back, push so the trunk pivots forward over the axles while the robot rolls about 6 cm backward, catch while rolling backward, then rise while rolling forward.

### Results (same tests, after the change)

Every run below moves along the control frame's +x: positive forward displacement means travel away from the knees; in the 0.3 m/s roll the knee joints trail the base by 0.096-0.101 m along the direction of travel.

| run (estimator closed loop from the kneeling stand-up unless GT) | verdict | forward / yaw | max pitch / roll after the catch | other |
| --- | --- | --- | --- | --- |
| roll 0.3 m/s and brake | pass | +2.06 of 2.10 m | 0.007 / 0.000 | estimate vs GT: height RMS 0.1 mm, vx RMS 0.002 m/s |
| yaw 1.0 rad/s at 0.22 m | pass | 7.17 of 9.0 rad | 0.002 / 0.001 | height 0.220 m |
| pitch push sequence | pass | - | 0.085 / 0.001 | |
| 20 mm strip | pass | +2.32 of 2.40 m | 0.006 / 0.000 | |
| ramp | pass | +2.21 of 2.40 m | 0.005 / 0.002 | estimated height is above the wheels' plane, so it differs from world z on the slope (RMS 18 mm) |
| uneven field 0.3 m/s | pass | +3.11 of 3.30 m | 0.007 / 0.005 | same (RMS 13 mm) |
| `jump` x4 in place / x1 rolling | pass | rolling +1.46 of 1.40 m | 0.044 / 0.009 | 0.16-0.21 s flight |
| `walk` at rest / rolling | pass | rolling +1.42 of 1.40 m | 0.016 / 0.021 | 5 lifts each, 0.13-0.15 s |
| `trot` at rest / rolling | pass | rolling +1.65 of 1.40 m | 0.023 / 0.065 (at rest) | 13 and 15 lifts, 0.11-0.18 s |
| GT: 0.38 m and 0.18 m requests | heights clamped to 0.330 / 0.220 m (verdict scores the request) | +2.66 of 2.70 m; 7.18 of 9.0 rad | 0.007 / 0.001 | |
| GT: 20 mm strip, pushes | pass | +2.26 of 2.40 m | 0.013 / 0.006; push 0.080 | |
| H2 drive (shared code changed) | pass | | | |

`trot` at rest reached 0.065 rad roll here against 0.032 rad before; one run each, so not separable from run-to-run variation yet.
The stand-up videos were re-rendered with true front/back subtitles and a "FRONT" marker for the user (not in the repository).
Committed as ab68ac8, e3e0a40 and e05ffbd.


## Estimated State, Stand-up, Hop and Single Support (2026-11-15, Claude)

Milestones 5-8 of "Next steps" now pass in simulation (MPC 200 Hz, flat unless stated).
The MPC runs without ground truth: a wheel-aware linear KF drives it from the kneeling idle through the planned stand-up, rolling, terrain, pushes, jumps and alternating wheel lifts.
Caveat for all estimator results: the simulated IMU and encoders are noise-free, so these are best-case numbers.

### Wheel-aware linear KF

[`LinearKfFloatingBaseEstimator`](https://github.com/wei-hsuan-cheng/legged_robot_state_estimator/blob/main/src/linear_kf_floating_base_estimator.cpp) gained `contactModel: rolling_wheel`.
A tire's contact point moves with the robot, so it is no landmark; what stays at rest without slip is the tread material at the contact.
Each wheel therefore measures the base velocity (minus the velocity of that material point relative to the base, wheel spin included) and the base height (the lowest point of the tire circle on the plane z = 0), and the state is base position and velocity only.
This is the proprioceptive model the MPC already uses: height above the wheels' contact plane, flat ground at zero.
`imuExtrinsics: true` removes Upkie's rotated, offset IMU mounting (URDF frame `imu`) from attitude, rate and acceleration; the MPC's bring-up tilt check does the same through `floatingBase.imuFrame`.
G1/H2 keep `contactModel: point` and `imuExtrinsics: false`, so their filters are unchanged.
Equations, code lines, gains and the comparison with the papers: [wheeled-biped state estimation](wheeled_biped_state_estimation.md).

Logged against GT while GT drives the MPC:

| run | height error | roll/pitch error | forward velocity error | drift |
| --- | --- | --- | --- | --- |
| roll 0.3 m/s and brake | RMS 0.2 mm | < 0.0001 rad | RMS 1 mm/s, max 8 mm/s | x 3 mm over 2.1 m, yaw 0 |
| yaw 1.0 rad/s | RMS 0.2 mm | < 0.0001 rad | RMS 2 mm/s | yaw 0 |
| jumps (estimator driving) | RMS 1-2 mm, max 12 mm | < 0.003 rad | RMS 0.05-0.10 m/s, up to 0.7 m/s at touchdown | - |

The touchdown spikes are tires landing while spinning (slip) plus the estimate odometry's 100 Hz sampling; they did not destabilise any run.

### Stand-up and handover

The planned stand-up (Codex's offline plan, played back open loop) passes on GT and on the estimator: idle on the knee housings for 3 s (the estimator initialises there), fold and push for 0.7 s, then the MPC catches the robot at 0.09 m and pitch -0.7 rad and ramps the height to 0.30 m.
Two fixes were needed:

1. The height clamp of the previous checkpoint acted on the MPC reference, so the catch's height ramp (restarted at the measured 0.09 m) became a 12 cm step and the robot ran away at 1.3-2 m/s and fell; the clamp now acts on the commanded height, before the rate limit.
2. The posture table now starts at 0.10 m, so the posture reference during that ramp is the balanced posture of the current height instead of the 0.20 m one.

The catch crosses the legs (wheel gap 36-42 cm for about 0.7 s, then symmetric again) and leaves the robot turned by 0.2 rad, to either side on different runs.
That split is how the MPC catches the robot, not a side effect.
Straightening the body from -0.71 rad within about 0.15 s saturates both wheel torques (1.7 N m) at once and the base overshoots forward at 1-2 m/s; rolling alone cannot move the support under the body fast enough, so the MPC swings one leg forward and one back with the 16 N m hips, like a catching step.
A cost on the tires' fore/aft offset in the base frame (weights 300, 1000 and 3000, tested and removed) kept the split under 3 mm and made all three catches fall 0.6 s after the handover, while the unrestricted catch survived every run (about 20, including the estimator acceptance set).
The fix belongs in the handover: a gentler catch, for example a pitch reference ramped from the measured pitch as the height already is, so the wheels are not saturated.

### Estimator acceptance (no GT in control, each from the kneeling stand-up)

| case | result | max pitch / roll from 3 s after the catch |
| --- | --- | --- |
| roll 0.3 m/s and brake | pass | 0.006 / 0.001 |
| yaw 1.0 rad/s at 0.22 m | pass, 7.16 of 9.0 rad (80%, as on GT) | 0.002 / 0.001 |
| pitch push sequence | balanced; the verdict reads NO_TRACKING only because the 0.01 m/s probe drive covered 7 of 20 mm | 0.089 / 0.001 (GT 0.081) |
| 20 mm strip | pass | 0.020 / 0.005 |
| 2-degree ramp | pass | 0.005 / 0.004 |
| uneven field 0.3 m/s | pass | 0.010 / 0.005 |

### Gait names

Upkie's gaits are `wheel_drive`, `jump`, `walk` and `trot` (the user's names).
`wheel_drive` (both wheels down, rolling and turning) is the rest gait: the first rung of `ocs2.gait.transitions.ladder`.
The motion manager used to require that rung to be called `stance`; it now takes the first rung, whatever its name, so G1/H2 (still `stance`) are unchanged (H2 drive smoke run passes).
The others are requested on `/humanoid/gait_command` (held while requested, `""` returns to `wheel_drive`).
The jump/walk tables further down were recorded under the earlier names `hop` (now `jump`), `alternate` (now `walk`) and `lift_left`/`lift_right` (single lifts, now a short `walk` request).

### All gaits in closed loop on the estimator (final check under the new names)

No GT in control; each run starts from the kneeling idle and planned stand-up; GT is only logged for the benchmark columns.

| gait | runs | verdict | wheel(s) off the ground | clearance | max pitch / roll | estimate vs GT during the gait |
| --- | --- | --- | --- | --- | --- | --- |
| `jump` | 4 in place | pass | 0.16-0.18 s | 10-11 mm | 0.037 / 0.014 | height RMS 1.5 mm, vx RMS 0.10 m/s (touchdown spikes to 0.64) |
| `jump` | 1 while rolling 0.2 m/s | pass, 1.28 of 1.40 m | 0.15 s | 10 mm | 0.029 / 0.008 | height RMS 0.9 mm, vx RMS 0.013 m/s |
| `walk` | at rest, 5 lifts | pass | 0.13-0.15 s | 6.5 mm | 0.016 / 0.020 | height RMS 0.7 mm, vx RMS 0.010 m/s |
| `walk` | rolling 0.2 m/s, 5 lifts | pass, 1.30 of 1.40 m | 0.13-0.15 s | 6.5-6.8 mm | 0.016 / 0.021 | height RMS 0.7 mm, vx RMS 0.011 m/s |
| `trot` | at rest, 13 lifts | pass | 0.12-0.15 s | 6.5-6.9 mm | 0.015 / 0.032 | height RMS 1.1 mm, vx RMS 0.022 m/s |
| `trot` | rolling 0.2 m/s, 15 lifts | pass, 1.09 of 1.40 m | 0.12-0.16 s | 6.5-6.9 mm | 0.015 / 0.031 | height RMS 1.1 mm, vx RMS 0.026 m/s |

Roll/pitch estimate errors stay below 0.0015 rad RMS in every gait.
`trot` while rolling covers 78% of the commanded distance (the lifts cost forward speed); `walk` and `jump` 91-93%.
On GT the same gaits measure alike (`trot` 13 and 17 lifts, 0.014 / 0.031), and the H2 drive smoke run passes after the rest-gait change.

### The stand-up handover in detail

The cut-over is the plan's own `handover_time`, 0.697 s after the playback starts (knee lift-off in the planning rollout, where the body's divergent component is about zero); it is time-based, not detected, and needs a valid MPC policy (the MPC solves from the start of the playback), else the bring-up faults.
Then a 0.05 s blend from the plan's last sample into the MPC; in the estimator run the playback starts at 4.01 s, the blend at 4.70 s and the MPC alone at 4.75 s.
During the catch the MPC's schedule stays `wheel_drive`, yet the contact sensors show the right wheel off the ground for 0.18 s (4.85-5.03 s, up to 8 mm) while the legs split to 36 cm, then landing and rejoining by about 5.45 s.
So the catching step is emergent, chosen by the MPC under saturated wheel torque but not planned as a step; scheduling a single-wheel phase at the handover would let the MPC plan it and its landing (not done).
Videos of this run (real time and 0.25x) were rendered from the replay for the user and are not in the repository.

### Jump (milestone 7)

Template `jump` (`[STANCE, FLY]`, 0.4 s stance, 0.15 s flight, 30 mm swing height).
A request reaches the plan about 0.3 s ahead plus the template's stance, so hold it at least 0.9 s for one jump.

| run | flight measured (both contact sensors < 0.5 N) | tire clearance | base rise | landing peak | max pitch / roll |
| --- | --- | --- | --- | --- | --- |
| one jump, GT | 0.15 s | 11 mm | 34 mm | 136 N per wheel | 0.037 / 0.003 |
| five jumps, GT | 0.15 s each | 10-11 mm | 35-40 mm | 108-170 N | 0.049 / 0.009 |
| jump while rolling 0.2 m/s, GT | 0.16 s | 11 mm | 38 mm | 125-160 N | 0.036 / 0.002 |
| jump while rolling, estimator from stand-up | 0.16-0.17 s | 10-11 mm | 42 mm | 142 N | small |
| four jumps, estimator from stand-up | 0.15-0.21 s | 9-11 mm | 40-43 mm | 56-242 N | small |

Torque at the limits in under 1% of samples (wheels); weight is about 53 N, so landings peak at 2-4.5 body weights per wheel.

### Walk and trot: single and alternating support (milestone 8)

`walk` is `[STANCE, RF, STANCE, LF]` (0.15 s lifts, 0.45 s double support, 20 mm swing height); `trot` is `[RF, STANCE, LF, STANCE]` (0.15 s lifts, 0.05 s double support).
Upkie has no hip roll, so with one wheel up the body tips toward the lifted side; the lifts stay short for that reason.

| run | lifted wheel off the ground | clearance | max pitch / roll |
| --- | --- | --- | --- |
| lift left / lift right, GT | 0.14 s, the other wheel stays loaded | 7 mm | 0.014 / 0.022 |
| walk x 5, GT, at rest and at 0.2 m/s | 0.13-0.14 s each | 7 mm | 0.015 / 0.022 |
| walk x 5, estimator from stand-up, at rest and at 0.2 m/s | 0.13-0.18 s | 7-12 mm | 0.038 / 0.046 worst lift |
| trot x 13 at rest and x 17 at 0.2 m/s, GT | 0.14-0.17 s each | 7 mm | 0.014 / 0.031 |
| single 0.3 s lift, GT | 0.29 s (stance wheel briefly unloaded 0.06 s) | 15 mm | 0.045 / 0.073 |

The 0.3 s lift survives but roll grows as expected (CoM 0.15 m inside the stance wheel); longer single support would need a pre-lean with unequal leg lengths, which the MPC does not plan by itself at these weights.

### How to run

```bash
H=src/legged_robot_mpc_controller/tests/wb_id_walk_test.py
CFG=/root/ocs2_ros2_ws/install/legged_robot_mpc_controller/share/legged_robot_mpc_controller/config/upkie
# estimator driving the MPC, from the kneeling idle (the default spawn and stand-up): roll, jump while rolling, brake
python3 $H --robot-name upkie --mpc-rate 200 --record-joints --record-tracking --max-yaw-error 100 \
  --source state_estimator --estimator humanoid_linear_kf --minimum-height 0.05 --startup-timeout 900 \
  --axis vx --speed 0.2 --hold 6 --ramp 1 --settle 6 --rest 4 --gait-sequence '[[9,"jump"],[9.9,""]]' --prefix /tmp/upkie_jump_est
# GT driving, estimator logged alongside: add --source ground_truth_state --log-estimator --estimator humanoid_linear_kf
python3 src/legged_robot_mpc_controller/tests/tuning/compare_upkie_estimate.py /tmp/upkie_jump_est --plot
python3 src/legged_robot_mpc_controller/tests/tuning/analyze_upkie_flight.py /tmp/upkie_jump_est --plot
```

Interactive: the plain launch (`robotName:=upkie mpcFreq:=200`) spawns kneeling, stands up and runs on the linear KF; request a gait with `ros2 topic pub --once /humanoid/gait_command std_msgs/String "{data: jump}"` and release it with `"{data: ''}"`.
A standing spawn cannot run on the estimator: the robot cannot stand unbalanced while the filter initialises, so start from the kneeling idle.

### Open after this checkpoint

1. Leg crossing during the stand-up catch (above): it is the MPC's catching step under saturated wheel torque; make the catch gentler (pitch reference ramp) rather than forbid the split.
2. Sensor noise and biases: the filter has only seen perfect simulated sensors; add IMU/encoder noise in the simulator before trusting the tuning.
3. Touchdown slip: keep a landing wheel's rows inflated for a short settle time if the touchdown velocity spikes matter.
4. Longer single support (pre-lean), jumps with higher clearance, and repeat counts (N = 1-5 per case).


## Leg-Crossing Fix (2026-11-15, Claude)

Pushed as 3943180, 30b2410 and b7de5eb.
Shipped in `config/upkie/ros2_controllers.yaml`: posture by height (0.20-0.33 m table), commanded heights 0.22-0.33 m (`minBaseHeight`/`maxBaseHeight`), hip/knee posture 3/0.1 (terminal 6/0.2), `tangentialVelocityGain` 5; base height/pitch/roll weights unchanged at 3000/500/2000.
Every user observation is fixed in simulation (GT base, MPC 200 Hz) and the full flat/terrain regression passes; tables below.
Changed: [`TargetTrajectoriesCalculatorBase`](../include/legged_robot_mpc_controller/mpc/common/command/TargetTrajectoriesCalculatorBase.h) (optional posture-by-height table and height clamp), [`WbIdMpcTargetTrajectoriesCalculator.cpp`](../src/mpc/wb_id/command/WbIdMpcTargetTrajectoriesCalculator.cpp) (uses it), the WB-ID parameter schema and controller (`ocs2.reference.postureHeights` / `postureJointStates`), the table generator [`config/upkie/compute_posture_table.py`](../config/upkie/compute_posture_table.py) and the table in the Upkie YAML.
Without a table nothing changes, so H2/G1 behave as before.

### What shapes the leg posture: H2 versus Upkie (the user's question)

H2's leg posture comes mainly from its constraints, as the user expected.
In stance, `footMotionEquality` is a hard 6-DoF equality on each foot (still, height and tilt corrected), so with the base height/pitch/roll tracked a 6-DoF leg has no free motion left.
In swing, the foot height follows the swing path (hard) and the landing x/y comes from `captureFootPlacement` through `taskSpaceFootCost`.
H2's posture weights (hip pitch 3, knee 0.1) are regularizers on top; the hip one is commented as stopping "a persistent front/back stance".
Upkie has no equivalent: a rolling wheel constrains gap, tread slip, lateral slip and moment, but its spin is free, so each leg keeps one free sagittal motion (wheel fore/aft under the hip) and there is no foothold cost.
Balance pins the common mode (wheels under the CoM); the differential mode (one wheel ahead, one behind) is held by the joint-posture cost alone.

### Mechanism, from the reproduction runs

| Case (shipped weights, mesh tire) | Wheel fore/aft gap max | Hip asymmetry max | Tracking |
| --- | --- | --- | --- |
| 0.30 m, yaw 1.0 rad/s | 32 mm | 0.08 rad | yaw 70% |
| 0.22 m, yaw 1.0 rad/s | 379 mm | 1.20 rad | yaw 0% |
| 0.22 m, vx 0.3 m/s | 354 mm | 1.19 rad | vx 93% |
| 0.38 m request, vx 0.3 m/s | 30 mm | 0.48 rad | vx 92%; height -41 mm, knee straight, knee jitter 7-8 mrad |
| Ramp, vx 0.2 m/s | 224 mm | 0.71 rad | vx 93% |

At 0.22 m the legs first fold symmetrically, then split within a second during settling, before any velocity command (side-view frames confirm it).
At a fixed height each leg only sweeps fore/aft, and balance pins only the two wheels' mean; splitting by +/-d costs f(s+d) + f(s-d), which beats the symmetric f(s) when the posture cost f curves downward along the sweep.
Its curvature contains (q - q_ref) q'' terms, which become large and negative when the reference is far from the posture the height requires: at 0.22 m the balanced symmetric posture is hip -1.09 / knee -1.68 rad against the fixed reference -0.60 / -0.97.
On the ramp the GT base holds 0.30 m of absolute height while the ground rises, so the legs fold to about 0.23 m above the slope and split at 14 s the same way.
Yaw then fails geometrically: wheels 350 mm apart fore/aft have parallel axles that never meet, so without lateral slip a differential drive has no centre of rotation.
The 0.38 m GUI maximum is out of reach: straight legs give about 0.34 m, and the height cost (51-61% of the cost) pins the knees at the singularity, which is the jitter.

### Fix and results

Each height's balanced symmetric posture (level trunk, tires on the floor, tire contact under the whole-robot CoM) is precomputed by `compute_posture_table.py`; at 0.30 m it reproduces `defaultJointState` to six decimals.
The joint reference is interpolated at the target height, and commanded heights are clamped to `minBaseHeight`/`maxBaseHeight` (0.22-0.33 m; 0.33 m keeps a 0.5 rad knee bend), which the table must cover.

### Height range in the YAML

`ocs2.reference.minBaseHeight` / `maxBaseHeight` bound the commanded height in the WB-ID schema and every robot YAML.
They are absolute heights above the ground the model assumes, and `defaultBaseHeight` must lie between them.
They set the GUI slider ends and its published-command clamp, the clamp of position-command heights, and the clamp of the walking command's height in the MPC.
That clamp acts on the command before its rate limit, so the stand-up handover, which restarts the height ramp at the measured 0.10 m, still ramps up from there; a first version clamped the MPC reference itself, which turned that ramp into a 12 cm step and made the robot fall at the catch.
The controller refuses to start if a posture table does not cover the range, and the launch refuses a `refBaseHeight` outside it.
Upkie is 0.22-0.33 m, so the GUI no longer offers the unreachable 0.38 m.
H2 (0.62-1.42 m) and G1 (0.395-1.195 m) are their former `defaultBaseHeight +/- 0.4`, so their behaviour is unchanged.
Experiment 1 also set H2's posture ratio (hip 3 / knee 0.1, terminal 6 / 0.2), passed with `--set` and not yet in the YAML.

| Case | Shipped | Exp 1: hip 3 / knee 0.1 | Exp 2: + posture by height |
| --- | --- | --- | --- |
| 0.22 m, yaw 1.0 | 379 mm, yaw 0% | 204 mm, yaw 10% | 11 mm, yaw 81% |
| 0.22 m, vx 0.3 | 354 mm, vx 93% | 168 mm, vx 102% | 1 mm, vx 98% |
| 0.30 m, yaw 1.0 | 32 mm, yaw 70% | 17 mm, yaw 74% | 15 mm, yaw 74% |
| 0.38 m request, vx 0.3 | knee jitter 7-8 mrad | 7-8 mrad | clamped to 0.33 m: 0.4 mrad, vx 99% |
| Ramp | 224 mm | 2 mm | 1 mm, vx 96% |
| 20 mm strip | FALL (mesh tire) | not run | SUCCESS; 241 mm sweep while the wheel stalls at the edge |

Tilt stays at or below 0.003 rad on flat ground in experiment 2.
The high-height runs report HEIGHT_TRACKING_ERROR only because the harness scores against the requested 0.38 m.
On the 20 mm strip the remaining sweep is the slip-feedback mechanism of the tire-collision section again (planned left traction follows tread slip, correlation 0.95, up to 46 N, wheel torque at 1.66 N m).

### Weight balance (cause 3)

With 1/(acceptable error)^2, normalised to H2's base height 150 (2 cm, 0.02 rad tilt, 0.05 m/s, 0.1 rad/s yaw rate, 0.2 rad/s tilt rates, 0.2 rad posture, 2 rad/s joint speed), velocity, posture and joint-velocity weights are already balanced; only the base pose is 3-20x high (height 3000 vs 150, pitch 500 vs 150, roll 2000 vs 150).
The 30 input weights are regularizers (x 0.001), as in H2.

Both reductions were tested on top of experiment 2 and rejected on the no-fall priority:

| Experiment | Flat, ramp, roll and brake | Push max pitch | 20 mm strip |
| --- | --- | --- | --- |
| 2: height/pitch/roll 3000/500/2000 | pass | 0.080 (shipped) | pass, max roll 0.028 |
| 3: 150/150/150 | pass; ramp lets the body rise 15 mm | 0.114 | FALL, roll 0.44 |
| 3b: height 150 only | pass; ramp lets the body rise 15 mm | 0.080 | FALL, roll 0.31 |

The stiff height and tilt weights are what hold the body when one wheel stalls at a step; the crossing was the fixed posture reference, not the height weight.

### Slip feedback on a blocked wheel (experiment 4)

On the 20 mm strip, 2 runs per gain on top of experiment 2:

| `tangentialVelocityGain` | Stall at the edge | Max planned left traction | Max roll | Leg sweep (wheel gap / hip asymmetry) |
| --- | --- | --- | --- | --- |
| 50 | 1.7 s | 44-46 N | 0.028-0.043 | 241-278 mm / 0.71-0.83 rad |
| 10 | 1.1-1.65 s | 22-38 N | 0.007-0.065 | 183-334 mm / 0.55-0.98 rad |
| 5 | 1.2 s | 16-17 N | 0.007-0.008 | 191-203 mm / 0.57 rad |

Gain 5 was consistent and chosen; the leg still sweeps about 20 cm while the wheel waits at the edge, because the body keeps tracking the commanded speed.

### Final regression (experiment 2 + gain 5)

| Case | Result |
| --- | --- |
| Push (4 pulses) | pass, max pitch 0.079-0.083 rad (shipped 0.080) |
| Roll 0.2 m/s and brake | pass, vx 97%, max pitch 0.006 rad |
| Yaw 1.5 rad/s | pass, 83% (shipped 75%) |
| Height steps 0.26 / 0.34 (clamped 0.33) / 0.30 m | pass, steady knee jitter 0.01 mrad |
| 0.22 m yaw 1.0 / vx 0.3 | pass: yaw 80%, vx 98%; wheel gap 14 / 1 mm |
| 0.38 m request, vx 0.3 | clamped to 0.33 m: vx 99%, knee bend 0.50 rad, no jitter (reported HEIGHT_TRACKING_ERROR against 0.38 m) |
| Ramp 0.2 m/s | pass x 2, vx 96%, wheel gap 1 mm |
| 10 mm strip with height steps | pass; wheel gap 24-44 mm steady (length compensation; shipped -24 to +6) |
| Rough field 0.3 m/s | pass, vx 98% |
| Rough field 0.15 m/s (old stall) | pass: past the old 0.64-0.67 m stall, 1.53 m at 0.12 m/s |
| 20 mm strip | pass x 4 (final YAML x 2), max roll 0.010-0.018 |
| 20 mm, 1 m wide strip | pass, max roll 0.006 |
| 20 mm strip, driving off its far end (forward one-wheel drop) | pass, roll 0.006 rad after the drop |
| 20 mm strip rotated 0.1 rad (meant as a side drop) | pass, but no drop happened: the robot aligned with the strip and rode its edge |

Runs are `/tmp/claude_{rep,e1,e2,e3,e3b,cand,fin}_*` and `/tmp/claude_e4k*` in `ros_jazzy`.
Evaluate any run with `tests/tuning/evaluate_upkie_run.py PREFIX --plot` (statistics, per-dimension cost shares, cmd/plan/GT plot sheet) and render frames with `tests/tuning/render_upkie_replay.py SCENE PREFIX_replay.json OUT.png t...`.

### Other findings

At 0.30 m the 1.0 rad/s yaw reaches 70-74%: the wheel speeds imply 0.72 rad/s and the body turns 0.70, so there is no slip; the MPC plans only about 0.71 because about 0.15 N m per wheel of unmodelled drag remains.
A flat tire on a plane is loaded at both rims, 44 mm apart, and scrubs when turning: mu N 0.022 m per tire is about 0.9 N m of yaw drag, matching; the MPC models a thin disk and re-plans from the measured yaw rate.
Codex's slow-yaw undertracking has the same cause; not yet addressed.


## Tire Collision Fix and 20 mm Edge Analysis (2026-11-14, Claude, after 22:00)

Stopped at the user's request before the weekly usage limit; nothing is committed.
Changed files: [`upkie.xml`](../description/mujoco/robots/upkie/upkie.xml) (tire collision shape), [`generate_upkie_urdf.py`](../description/urdf/robots/upkie/generate_upkie_urdf.py), [`MODEL_SOURCE.md`](../description/urdf/robots/upkie/MODEL_SOURCE.md), the terrain [README](../config/terrains/upkie/README.md) and this file.
No C++ or controller parameter changed; `upkie.urdf` regenerates byte-identical and `tests/test_upkie_urdf.py` passes.

### Why the 20 mm one-sided entry fell (primitive-cylinder tire)

Reproduced first: `claude_s20_base` fell at 10.31 s against Codex's 10.22 s, same configuration.
From `codex_upkie_finalbalance_split20`:

1. The left tire stopped at the strip at 7.7 s and stayed at x = 0.558-0.575 m for 2 s (42 mm from the face is the touching distance for 20 mm).
2. Wheel torque never saturated while stalled (|tau| <= 1.0 N m until 9.6 s); the measured edge load was intermittent, 12 N mean, above 2 N in 42% of samples.
3. The left wheel spun up against the edge (17 to 56 rad/s), so the measured tread slip grew to 1-3 m/s.
4. The rolling row's slip feedback (`tangentialVelocityGain` 50) turned that slip into planned left traction: correlation 0.97 between measured slip and planned fx, which rose from 3 to 112 N, with planned fz up to 157 N against a 54 N robot weight.
5. Those planned wrenches drove the body forward past the stuck wheel: left hip -0.58 to +0.70 rad, left knee -0.97 to +0.05 rad (straight leg); then roll went from -0.02 to -0.30 rad in 0.9 s.

So the fall mechanism is the MPC treating a blocked, spinning wheel as a gripping one, not torque limits and not the z = 0 gap term (the tire rose only 1-4 mm).

### The simulator made the edge a wall

Replaying the stalled poses in MuJoCo 3.13.0 (the version `mujoco_ros2_control` links) showed the tire overlapping the step by about 15 mm, with the shin's wheel-motor housing (`left_tibia_collision2`) resting on the corner.
MuJoCo's primitive cylinder against a box gives two rim contacts with the step face's horizontal normal (1, 0, 0), never the edge contact a real tire makes; details and the analytic check are in [MODEL_SOURCE.md](../description/urdf/robots/upkie/MODEL_SOURCE.md#tire-collision-shape).
The tire collision is now a 64-sided convex prism of the same size, which reproduces the textbook push thresholds over 10 and 20 mm steps.
Every earlier terrain result (Codex's ramp, split10, rough, split20) was obtained with the primitive tire, which made 10 mm edges easier and 20 mm edges harder than they are.

### One parameter experiment (primitive tire, not adopted)

`claude_s20_kdt5` (`--set ocs2.constraints.wheelContact.tangentialVelocityGain=5.0`) did not fall in 23 s (max roll 0.033 rad) but stalled at the edge (final x 0.63 m, mean vx 0.064 m/s).
That confirms the slip feedback drives the fall; it was not run on the rest of the suite, so it is not adopted.

### Mesh-tire regression (shipped weights, GT base, MPC 200 Hz, Codex's exact arguments)

| Prefix under `/tmp/` | Result | Measured |
| --- | --- | --- |
| `claude_mesh_height` | SUCCESS, 23 s | max roll/pitch 0.0003/0.0035 rad |
| `claude_mesh_yaw` | SUCCESS, 17 s | 1.5 rad/s request, 1.127 rad/s measured; max roll/pitch 0.0016/0.0032 rad |
| `claude_mesh_push` | SUCCESS, 25 s | four original pulses; max pitch 0.080 rad |
| `claude_mesh_roll` | SUCCESS, 27 s | 0.2 m/s and brake, 0.195 m/s mean; max pitch 0.0053 rad |
| `claude_mesh_ramp` | SUCCESS, 23 s | 0.188 m/s mean; max roll/pitch 0.0016/0.0106 rad |
| `claude_mesh_split10` | SUCCESS, 25 s | 0.141 m/s mean with height steps; max roll/pitch 0.0025/0.0096 rad |
| `claude_mesh_rough` | SUCCESS, 22 s | seed-7 field at 0.3 m/s, 0.290 m/s mean, 3.19 m final x; max roll/pitch 0.0028/0.0057 rad |
| `claude_mesh_split20` | FALL at 16.37 s, after climbing | see below |

With the mesh tire the 20 mm entry is climbed: the left wheel stalled 1.9 s with the same slip build-up (planned fx up to 45 N), then rose onto the strip at 9.8 s and drove on it at 0.19 m/s.
The stall turned the heading by +0.2 rad, which relative-twist tracking deliberately does not restore, so the robot drove diagonally; at 14.2 s the left tire ran off the strip's outer side (tire y 0.37 m, strip edge 0.34 m), dropped 20 mm sideways, the yaw spun up, and it fell at 16.4 s.
This is a new failure mode, a one-wheel lateral drop, partly set up by the 0.28 m wide strip; the entry itself now passes once.

### Odometry/estimation sensitivity (user's question)

x, y and yaw are gauge states here, as for H2/G1:

- In `base_twist` the target is re-anchored at the current pose every solve and integrated from the body-frame command ([WbIdMpcTargetTrajectoriesCalculator.cpp](../src/mpc/wb_id/command/WbIdMpcTargetTrajectoriesCalculator.cpp)).
- `BaseMotionTrackingCost` in RelativeTwist mode drops world x/y/yaw errors and compares horizontal velocity in each frame's heading ([BaseMotionTrackingCost.cpp](../src/mpc/common/cost/BaseMotionTrackingCost.cpp)).
- The plain quadratic cost has a zero base block ([config builder](../src/controller/config_builder.cpp)).
- The rolling, lateral no-slip and friction-cone rows use velocities and the world-up normal only, so a yaw rotation or x/y shift leaves them unchanged.

Roll and pitch are regulated to absolute zero, which is fine because the IMU observes them.
Absolute height z is the sensitive one: the height target is world z above the plane z = 0, and each wheel's gap row has a position term `kp_n (n . p_c - 0)` with `kp_n` 50.
An estimator must therefore report z relative to the wheel contact plane (leg kinematics with the ground under the wheels at 0, as G1's `height.source: kinematic`), not integrated IMU z; a 1 cm z drift would read as both wheels 1 cm in the air.
Only the explicit `base_pose` target mode tracks absolute pose, by design.

### User's manual test notes (2026-11-15), the first tuning target

- Flat, base height raised toward the 0.38 m maximum: the knees jitter more as they approach full extension (singularity); just below that, vx and yaw-rate tracking stay good.
- Flat, base height lowered: the two legs cross (one wheel ahead, one behind) and the yaw rate essentially cannot track.
- Driving onto the 2 cm strip: the legs also cross; the robot should instead let the leg give (compliance) and rise rather than push hard.
- Driving up the ramp: fine at first, then the legs start to cross the higher it climbs.
- The user's reading: base height (and tilt) are weighted so strongly that the optimizer sacrifices a symmetric leg posture for them; with less sensitivity the legs should keep the same knee direction and drive smoothly.
- Expected: vx and yaw rate trackable at every commanded height, no crossing at low height, on the step or on the ramp.

Candidate causes (status after the 2026-11-15 tuning; see "Leg-Crossing Fix" above):

1. `defaultJointState` is the 0.30 m posture at every commanded height. Confirmed as the cause of crossing; fixed by the posture-by-height table.
2. The model stays proprioceptive with the ground at z = 0, like H2, by the user's decision: no ground-relative height or gap references. With the GT base the ramp still folds the legs (absolute height 0.30 m over rising ground), but the posture fix keeps them symmetric.
3. Base height/tilt weights 3000/500/2000 are 3-20x above a balanced weighting, but lowering them made the 20 mm step fall; kept.

### Next, in order (replaces item 1 of "Next steps" below)

1. A real one-wheel side drop: the rotated-strip test aligned with the strip instead; drive across a strip's side edge on purpose (for example a yaw command while one wheel is on a narrow strip).
2. Yaw scrub: the flat tire's two loaded rims cost about 0.9 N m of yaw drag the MPC does not model (yaw 75-83% of the command); decide whether the tire collision should be crowned (one contact line, as a rubber tire) or the MPC should model the scrub.
3. The remaining leg sweep while a wheel waits at a step edge (about 20 cm): the body keeps tracking the commanded speed; consider easing the speed target when a wheel is blocked.
4. Repeat counts are 1-4 per case; run N >= 5 on the step and push cases before calling it robust.
5. The rest of the original list below (push envelope, estimator, stand-up, hop, single support) is unchanged.

Reproduction: scenes come from `tests/mujoco_scene.py --terrain split_height --split-height 0.02` and the other terrain commands in the [terrain README](../config/terrains/upkie/README.md); each `/tmp/claude_mesh_*.json` holds its full arguments.


## Current Height/Terrain Checkpoint (2026-11-14)

Handover deadline: 22:00 Asia/Taipei on 2026-11-14; Claude takes over afterward.
These results predate the tire-collision fix above and used the primitive-cylinder tire.
The final selected configuration keeps the original hip/knee posture weights 1/2 and uses base height/pitch/roll 3000/500/2000 in Q and Qf.
This is the current YAML; all weak-posture candidates described below are rejected experiments, not the retained configuration.
`codex_upkie_finalbalance_yaw` passed the 1.5 rad/s abrupt turn at requested 200 Hz: measured mean 1.14565 rad/s, max pitch/roll 0.00388/0.00194 rad and 0.0656 m drift.
It retains a strong weighted preference for balance, not a mathematical hard priority or a guarantee against falls.

Final-configuration acceptance (all requested MPC 200 Hz, MRT 1 kHz, GT base state and no terrain map):

| Prefix under `/tmp/` | Result | Measured behavior |
| --- | --- | --- |
| `codex_upkie_finalbalance_height` | SUCCESS, 23 s | 0.26 / 0.34 / 0.30 m steps: settled mean errors 1.82 / 9.93 / 0.10 mm; max pitch/roll 0.00307/0.000283 rad |
| `codex_upkie_finalbalance_yaw` | SUCCESS, 17 s | Abrupt 1.5 rad/s request averaged 1.14565 rad/s; max pitch/roll 0.00388/0.00194 rad |
| `codex_upkie_finalbalance_ramp` | SUCCESS, 23 s | 2-degree ramp, 0.2 m/s request averaged 0.19321 m/s; max pitch/roll 0.01044/0.00179 rad |
| `codex_upkie_finalbalance_split10` | SUCCESS, 25 s | 10 mm unequal supports plus 0.34/0.30 m height commands; height errors 8.06/0.83 mm; max pitch/roll 0.00757/0.00261 rad |
| `codex_upkie_finalbalance_rough` | SUCCESS, 22 s | Seed-7 field, 0.3 m/s request averaged 0.29260 m/s; displacement 3.202 m; max pitch/roll 0.01068/0.00645 rad |
| `codex_upkie_finalbalance_push` | SUCCESS, 25 s | All four original pulses verified; max pitch/roll 0.07860/0.000440 rad; final recovery pitch <=0.00410 rad |
| `codex_upkie_finalbalance_split20` | FALL at 10.22 s | Final weights still do not handle the 20 mm one-sided vertical entry at a 0.2 m/s command; leave this as an explicit open issue |

The larger final height error is intentional relative to the rejected 3.55 mm-error candidate: balance, a stable leg-fold preference and turning are more important than exact height.
The old static 0.34 m request had 33.11 mm bias; the selected configuration reduces it to 9.93 mm without weakening the original posture cost.
All measured joints stayed below their effort limits in the final height, yaw, ramp and split10 tests.
Final mean logged rates were 190.25 Hz (height), 191.36 Hz (yaw), 192.41 Hz (ramp), 191.06 Hz (split10), and 191.42 Hz (rough).
Mean advance times for those runs were 1.67-1.94 ms, with p95 2.12-3.10 ms, but individual outliers reached 50.62 ms; there is no hard real-time or exact-200-Hz guarantee.
The Release install/build completed successfully in 4.63 s with no C++ changes or code-generation regeneration; 17 focused GUI, stand-up-data, terrain-generator and scene-compilation tests passed.
The plot asset linked below has been regenerated from these final-configuration runs, not the earlier weak-posture trials.

Reproduce the height profile from the sourced container workspace:

```bash
python3 src/legged_robot_mpc_controller/tests/wb_id_walk_test.py \
  --robot-name upkie --source ground_truth_state --mpc-rate 200 \
  --axis vx --speed 0 --height-sequence '[[4,0.26],[10,0.34],[16,0.30]]' \
  --hold 23 --ramp 0 --settle 0 --rest 0 --minimum-height 0.15 \
  --record-joints --record-tracking --prefix /tmp/upkie_height_mine
```

For exact terrain commands, each container prefix has its complete `.json` arguments and evaluated `.yaml` parameter snapshot.
Generate the selected temporary scene first using the commands in `config/terrains/upkie/README.md`; no extra installed robot `scene_*.xml` files are needed.
All simulations are intentionally stopped for the 22:00 handover; do not launch overlapping tests on the shared ROS topics.
No further test or controller change is scheduled after this checkpoint.


### Earlier tuning trials

At 21:39 an intermediate maximum-yaw regression exposed severe undertracking with the provisional 0.05/0.1 posture weights below: 1.5 rad/s requested, only 0.07866 rad/s measured, despite remaining upright.
Increasing yaw-rate weight from 10 to 100 did not fix it (0.15439 rad/s, 0.688 m translation drift).
Logged and measured right-knee motion crossed the nominal bending branch (about +0.978 to -0.537 rad); restoring the full posture weights recovered turning.
Do not treat the provisional height-only tuning as a complete yaw-compatible checkpoint.
The user clarified at about 21:42 that pitch/roll stability and fall avoidance outrank height precision, matching leg lengths, yaw drift and XY drift.
Do not pursue exact height/velocity tracking at the cost of losing balance or add new objective/constraint structures merely to improve a tracking metric.
Restoring full posture weights 1/2 with height weight 10000 recovered the 1.5 rad/s turn (1.14294 rad/s measured, max pitch/roll 0.00683/0.00292 rad) in `codex_upkie_heighttune_posture1_yaw`, but this high-height-weight candidate is not selected without terrain validation.
Another rejected candidate used height/pitch/roll 3000/500/2000, hip position 0.05/0.1 (Q/Qf), and the original knee position 1/2.
Its `codex_upkie_balancefirst_split20` still fell at 10.73 s on the 20 mm one-sided entry, and `codex_upkie_balancefirst_yaw` undertracked at 0.30974 rad/s with another knee-branch reversal; stronger tilt weights with weak hips were not sufficient.
The user has independently tested smooth driving/turning at requested MPC 200 Hz; use `mpcFreq:=200` in Upkie examples and keep MRT at 1 kHz and physics/PD at 2 kHz.
The shared launch still defaults to 100 Hz for other robots; the Upkie documentation and its acceptance suite select 200 explicitly.
New measurements here target 200 Hz and average about 190-192 Hz on the VM, not a guaranteed exact 200 Hz.
The historical 100 Hz results and earlier conservative rate advice below are superseded for the current manual-testing recommendation, not rewritten as 200 Hz evidence.

The first, now-rejected tuning changed these existing Q/Qf entries in `config/upkie/ros2_controllers.yaml`:

| Weight | Before | Rejected Intermediate | Reason for the trial |
| --- | --- | --- | --- |
| Base height, Q and Qf | 100 | 1000 | Reduce height bias while retaining a soft objective |
| Base pitch/roll, Q and Qf | 20 / 50 | 100 / 500 | Prefer trunk balance while the legs change length independently |
| Hip/knee position, Q | 1 | 0.05 | Stop the nominal crouch fighting a new height or unequal supports |
| Hip/knee position, Qf | 2 | 0.1 | Avoid forcing the nominal crouch at the horizon end |

Wheel-angle weights remain zero; no new cost, constraint, planner, runtime C++ change, PD gain, torque limit or friction value was added for this tuning.
The existing wheel equality gains remain 50/50/50 for normal position/normal velocity/tangential velocity.
Trials with normal-position gain 5 or 0, normal-velocity gain 5, forward-velocity weight 100, and roll weight 2000 combined with weak posture did not resolve the tested larger-edge failures; those trial combinations are not retained.
No new CppAD library generation is required for these YAML weight changes.
The table and early terrain results below describe the intermediate candidate; use the final regression table added at handover for current-parameter claims.

Flat static comparison at a 0.34 m request: the old weights settled at 0.30689 m (33.11 mm low); increasing height weight alone to 3000 reached 0.33027 m but increased pitch lean to about 0.032 rad.
The rejected weak-posture height/tilt combination reached 0.33664 m with max pitch 0.00143 rad in that trial.
The tiny 0.001 m/s static-test command failed its velocity-tracking verdict with those weights, but the height/attitude statistics are the intended comparison; do not label it a velocity success.

Earlier intermediate-parameter tests used `base_twist`, simulator base-state feedback, no terrain-map input and no stair/terrain-walking planner:

| Container prefix under `/tmp/` | Result | Evidence |
| --- | --- | --- |
| `codex_upkie_height_steps` | SUCCESS, 23 s | Height steps 0.30 -> 0.26 -> 0.34 -> 0.30 m; steady mean errors 0.10 / 0.80 / 3.55 / 0.44 mm after excluding the first 2 s of each step; max pitch/roll 0.00344/0.000193 rad |
| `codex_upkie_ramp_gain50` | SUCCESS, 23 s | 2-degree ramp, 0.2 m/s request averaged 0.19480 m/s; displacement 2.257 m; max pitch/roll 0.02225/0.00105 rad |
| `codex_upkie_split10_gain50` | SUCCESS, 25 s | Left tire on a 10 mm strip, right on floor; 0.15 m/s request averaged 0.14874 m/s; measured wheel-center difference about 10.02 mm while base roll averaged 0.00013 rad over x=1-2 m |
| `codex_upkie_split10_height` | SUCCESS, 25 s | Same unequal supports while commanding 0.34 m at 12 s and 0.30 m at 19 s; mean height errors 2.15 mm / 0.71 mm after settling; max pitch/roll 0.01052/0.00301 rad |
| `codex_upkie_rough_v30` | SUCCESS, 22 s | Seed-7 low uneven field; 0.3 m/s request averaged 0.29320 m/s; displacement 3.206 m; max pitch/roll 0.02362/0.00850 rad |
| `codex_upkie_rough_v30_repeat` | SUCCESS, 22 s | Independent repeat, 0.28995 m/s; displacement 3.189 m; max pitch/roll 0.03531/0.00943 rad |
| `codex_upkie_heighttune_push200` | SUCCESS, 25 s | All four original pulses verified; max pitch 0.12774 rad; final recovery pitch <=0.00584 rad |

The new profile files and regeneration/manual test commands are in `config/terrains/upkie/README.md`.
The ramp starts at x=0.6 m, rises 2 degrees over 3 m, and has an untested far-edge drop; the passing test stopped before it.
The uneven field spans x=0.5-4.5 m, y=-1-1 m with 0.25 m tiles, seeded 0.67-21.92 mm tops and 3.83-6.85 mm edge-neighbor jumps.
Floor-to-first-tile jumps are exempt from that neighbor range.
Do not overwrite the shared humanoid terrain parameters to regenerate this small-wheel variant.

Important unresolved limits:

- `codex_upkie_rough_gain50`, `rough_gain5_vx100` and `rough_compliant` remained upright but stalled near x=0.64-0.67 m at 0.15 m/s, rather than traversing the field.
  The first stalled trial had one wheel spinning against the entry while the other had climbed; weak world-plane height feedback alone was not the cure.
- `codex_upkie_split20_height` fell at 8.92 s on the 20 mm one-sided vertical entry at a 0.3 m/s command, before the scheduled height change.
  `codex_upkie_split20_roll2000` also fell at 13.38 s with roll weight 2000 and normal-position gain 5 at 0.2 m/s.
  Therefore the fall is not evidence that a height-command step caused it, and neither simply raising roll weight nor loosening normal position was sufficient.
  These are failed stress tests, not accepted capabilities.
- Height remains an absolute world-z base-origin target, not ground-relative clearance; the controller's support plane still assumes world z=0 with an upright normal.
  Small terrain errors are absorbed in receding-horizon feedback and finite SQP residuals, not represented by an identified per-wheel terrain plane.
  Ground truth supplies only the robot base state, not the upcoming terrain; estimated-state rolling control remains unimplemented.

Visual evidence: `docs/design_notes/wheeled_biped/assets/upkie_height_terrain.png` contains height steps, unequal measured wheel support levels and base pitch/roll traces.
It is reproducible with `tests/tuning/plot_upkie_height_terrain.py`; measured-state replay screenshots were also inspected at `/tmp/codex_upkie_terrain_visual.png` on the Mac, including the larger-step failure.
The test harness now supports `--height-sequence '[[seconds,height_m],...]'`, writes `_height.csv`, and reports per-step settled errors; production control is unchanged.
Eleven focused geometry/generator/scene-compilation tests passed after adding the small-wheel fixtures.
The intermediate user commit `b8baeee` already contains the terrain fixtures/docs/harness; the final checkpoint adds the selected YAML weights, current results, plot/reproduction script and open-issue handoff.


## Branch Checkpoint (2026-11-14)

The user requested committing and pushing the complete current Upkie checkpoint with a detailed commit message, retaining the current code and parameter values.
The checkpoint includes model assets and provenance, rolling-contact and inverse-dynamics extensions, wheel-drive configuration, optional stand-up playback, diagnostics, acceptance-test tooling and formulation documentation.
Research and initial implementation notes live under `docs/design_notes/wheeled_biped/`, not beside the formal formulation document.
The low-level PD execution was verified by source inspection: hardware `write()` runs every physics step using live MuJoCo joint state, while MRT updates the held references, feedforward and gains.
For the default Upkie scene this means 2 kHz PD with 1 kHz MRT and a requested 100 Hz MPC rate.
No new build or simulator test was run for the documentation and commit/push steps; the results below remain the prior recorded evidence, not fresh validation of the complete checkpoint.
Outstanding work remains the friction/damping comparison, repeated acceptance suite, full-stack stand-up, hop/single-support development, rolling-contact estimation and G1/H2 regressions.


## Scene Style Alignment (2026-11-14)

Upkie's `scene.xml` now follows H2's short composition layout: timestep, background, optional terrain/object includes, then robot.
The new `common/backgrounds/background_night_upkie.xml` reuses the humanoid night appearance, textures, lighting strengths and floor properties, but aims lights at `base` and uses smaller viewer framing.
The existing G1/H2 backgrounds are unchanged, and Upkie's original robot colors and material definitions remain intact.
Ramp and uneven-field includes are present as comments, so the default Upkie scene remains flat and does not imply newly validated terrain locomotion.
`implicitfast` moved into `upkie.xml` beside its existing four no-slip iterations, while the scene retains the 0.0005 s timestep and therefore 2 kHz physics-rate PD.
No controller code, gains, actuator caps, inertias, contact geometry or friction parameters were changed.
Local MuJoCo 4.0.1 compilation compared 38 physical arrays/options against the prior scene, all exactly unchanged, including joint/body parameters, contact settings and actuator limits.
A static render at the configured standing pose was visually inspected; the temporary preview is `/tmp/codex_upkie_night.png` on the Mac.
Updated scene-layout tests cover Upkie's assets, night-style/light targets, common floor physics, and flat/ramp/rough fixtures.
Four focused scene tests passed; the complete layout suite passed 14 of 15 tests, with the remaining pre-existing xacro ramp test failing on last-digit vertex-string formatting in the local environment.
The same ramp failure was reproduced from HEAD without the test changes; no terrain files were regenerated to hide it.
These checks compile/render scenes, not closed-loop MPC validation or a ROS build; no robot-controller simulation was launched.


## URDF Visualization Fix and Rate Headroom (2026-11-14)

The user reported correct MJCF rendering but disconnected leg meshes in RViz.
The generator's quaternion-to-RPY conversion lost the coupled roll/yaw rotation at exactly +/-90-degree pitch, producing rotation-matrix entry errors as large as 1.0 for ten geometries.
Added an explicit singular branch, regenerated the URDF and corrected six visual meshes plus four collision primitives.
All seven joint definitions and seven inertial blocks remain byte-for-byte identical as parsed XML; no MJCF, controller gains, contact equalities or MPC settings changed.
Three local regression tests pass: exact/near-singular rotations, all source quaternions and generated geometry/joint/IMU pose agreement.
An offline MuJoCo import/render of the old and corrected URDF reproduced the user's visual defect and confirmed that the corrected model matches the MJCF assembly.
The temporary three-panel comparison is `/tmp/codex_upkie_urdf_comparison.png` on the Mac; it is a static asset check, not an RViz or closed-loop runtime test.
No ROS nodes, builds or closed-loop tests were started, and the user's separate `description/urdf/env.urdf.xacro` edit was left untouched.
Relaunch the robot-state publisher and visualization to reload the corrected URDF; the generator fix does not require new CppAD libraries because the rigid-body model is unchanged.

The user's runtime logs request 100 Hz and measure about 97.7-98.0 Hz, with average MPC advances around 1.8-2.0 ms and observed interval peaks up to 4.9 ms.
The worker deliberately sleeps until its target period, so these logs do not imply a 100 Hz computation ceiling.
The measured advance excludes some post-solve diagnostics/publication and does not bound future worst-case latency.
Recommend 150 Hz as the next trial (6.67 ms period), or 125 Hz (8 ms) for more scheduling margin; neither rate has been validated here.
Do not claim stable 200 Hz from this evidence: its 5 ms budget leaves only about 0.1 ms beyond the observed 4.9 ms solve peak, before other work and scheduling jitter.
Keep the current controller parameters unchanged until the user requests a rate change or benchmark.
The reported zero solver exceptions and zero torque-window overruns are useful health evidence, but the logged equality residuals are not a physical stability guarantee.
Per-term maximum SSE is an unweighted sum over intermediate nodes in the diagnostic loop, whereas the aggregate solver metric uses its own accumulation; do not interpret their different numerical scales as a logging contradiction or a per-joint torque error.


## Formulation Documentation (2026-11-14)

Expanded the formulation with a source-to-code traceability table, exact paper sections/equations/pages and links next to the relevant formulas.
The notes distinguish world-aligned tire-center wrenches from body-axis or world-origin wrenches, and identify the six-wrench layout and shared lateral row as local adaptations.
Friction references support a differentiable model family, not the current numeric values: 0.1 Nm friction and 0.005 kg m^2 armature come from the MJCF, while 0.05 rad/s is an unoptimized local smoothing choice.
Documented the different tanh scale/sign conventions in Pyrhonen et al. Eq. (10), the actual partial-test evidence, and a proposed future calibration protocol clearly separated from completed tests.
No new tuning, runtime validation or controller changes were performed for the reference update.
Added the implementation-specific math document linked above after reading the current controller, configuration builder, contact kinematics, cost evaluators and OCS2 penalty functions.
It uses the existing humanoid document's dollar-delimited inline/display LaTeX and one-sentence-per-line prose, with two blank lines before headings.
It distinguishes the 24-state/30-input Euler-coordinate OCP, full-horizon RNEA, armature and smoothed Coulomb friction, eleven double-support wheel-contact rows, tracking costs, soft inequalities and final actuator clipping.
The document records the shared friction cone's default regularization of 25 N^2, whose zero-tangential-force nonnegative-residual threshold is about 8.33 N at coefficient 0.6.
It also records that the shared lateral equality is a signed average without axle-sign alignment, so it is not a proof of individual-wheel lateral no-slip; cancellation and equality rank need checking in future tests.
No controller code, parameters, model assets, build or simulation were changed or run for this documentation update.


## Rolling-contact Estimator Scope (2026-11-14)

Documented the need for rolling-contact estimation and its primary reference in [formulation Section 7.5](wheeled_biped_whole_body_inverse_dynamics_mpc.md#75-rolling-contact-state-estimation).
[Hosseini, Rodriguez and Behnke, State Estimation for Hybrid Locomotion of Driving-Stepping Quadrupeds, IRC 2022](https://www.ais.uni-bonn.de/~hosseini/se-dsq/IEEE_IRC_22_StateEstimationForDrivingSteppingQuadrupeds.pdf), Fig. 1 and its wheel-aware state-estimation formulation, is a direct reference for adapting legged Kalman-filter measurements to wheel geometry and rolling motion.
Source inspection confirms that our current linear KF assumes stationary planted-contact landmarks; selecting it unchanged is not a validated wheel-drive estimator.
The required main output remains floating-base pose and velocity, but tire-center motion must be distinguished from the approximately zero material tread velocity under no slip.
First adapt velocity and ground-relative height measurements using IMU, leg/wheel encoders and configured tire geometry, with contact/slip reliability gating.
Slip, effective radius, terrain normal and additional biases are optional extensions rather than automatically required extra states; contact forces can remain outside the estimator state.
Proprioception does not supply drift-free global XY/yaw, and arbitrary terrain elevation is not independently observable without assumptions or additional sensing.
Current Upkie runtime evidence still uses ground truth; no rolling estimator implementation, build or estimated-state closed-loop validation was added in this checkpoint.
The next estimator milestone is an estimated-versus-simulator comparison followed by closed-loop balance, drive, braking and push acceptance on estimated state.


## Yaw/Push Failure Fix and Validation (2026-11-14)

The user reported that forward/backward driving works but yaw-only commands and pushes immediately destabilize Upkie.
A baseline 0.3 rad/s yaw test at 100 Hz reproduced a fall 5.84 s after takeover, about 1.84 s after the yaw ramp began; artifacts are `/tmp/codex_upkie_yaw_baseline*` in `ros_jazzy`.
The original four-pulse push sequence also fell at 7.01 s, during recovery from the second pitch-torque pulse, before either force pulse; artifacts are `/tmp/codex_upkie_push_baseline*`.
Before the yaw fall, hip/knee position tracking errors stayed below roughly 0.010/0.006 rad, without actuator saturation, while planned and measured base behavior diverged.
Two implementation defects were found rather than treating this as missing turning capability or immediately retuning costs.

1. The mean lateral row averaged opposite signed wheel axles, canceling at the symmetric posture: its acceleration Jacobian norm was 1.52e-14 versus 1.04 after expressing both wheels in a common lateral basis.
   The corrected row uses the first wheel's lateral direction for every supporting wheel and includes the rotating-direction derivative; longitudinal slip now includes that derivative as well.
   This still constrains mean lateral slip, not independently both tires, and single-support/large-asymmetric-axle cases remain unvalidated.
   Generated wheel libraries use `_v2` names so existing `_v1` tapes cannot silently retain the old function.
2. MuJoCo ground-truth feedback mixed link-frame pose with inertial-frame/COM twist: `mj_objectVelocity(..., mjOBJ_BODY, ..., 1)` returns the principal-inertia frame, not the regular body frame.
   Upkie's base inertia axes are nearly 90 degrees from its link axes; pure physical yaw of 0.3 rad/s returned approximately `[0.01125, 0.29972, -0.00625]` in that wrong basis, which the controller interpreted as link-frame angular velocity.
   Both ros2_control body-state feedback and published odometry now use `mjOBJ_XBODY`, consistent with `xpos/xquat` and the advertised child frame.
   See [MuJoCo 3.3.2 source, mj_objectVelocity](https://github.com/google-deepmind/mujoco/blob/3.3.2/src/engine/engine_support.c#L1166); this simulator fix is in the separate `mujoco_ros2_control` repository and requires rebuilding it too.
   A synthetic rotated/off-center-inertia regression test matches link-origin Jacobians for yaw, pitch, translation and mixed motion, and verifies both feedback paths select XBODY.

The GUI height slider and published-command clamp now use `defaultBaseHeight +/- maxDeltaPelvisHeight` from the selected YAML instead of hard-coded 0.2-1.0 m (superseded on 2026-11-15 by absolute `minBaseHeight`/`maxBaseHeight`).
Upkie therefore spans 0.22-0.38 m with 0.30 m at the slider center, while H2 can command its configured heights above 1 m.
The launch passes the existing height delta to the GUI without adding a launch argument or controller parameter; four headless GUI mapping/clamping tests pass, including a fixed-height zero-delta configuration.
Both packages were rebuilt in Release in `ros_jazzy`; wheel `_v2` CppAD functions were generated on first launch and reused afterward.
No MPC cost, constraint gain, PD gain, actuator cap, friction value or command slew limit was retuned.
The following current-parameter tests used ground-truth feedback, an isolated flat scene, requested MPC 100 Hz and MRT 1 kHz, with tracking and joint logging.

| Test/artifact prefix in container `/tmp/` | Result | Key observation |
| --- | --- | --- |
| `codex_upkie_yaw_fixed_1` | Balanced for 19 s; tracking verdict NO_TRACKING | +0.3 rad/s command averaged +0.145 rad/s; max pitch 0.00551 rad, roll 0.00260 rad; slow yaw still undertracks |
| `codex_upkie_yaw_fixed_reverse` | SUCCESS, 19 s | -0.6 rad/s command averaged -0.359 rad/s; max pitch 0.00374 rad, roll 0.00328 rad |
| `codex_upkie_yaw_fixed_max` | SUCCESS, 17 s | Abrupt +1.5 rad/s command with harness ramp 0, controller slew unchanged; mean +1.121 rad/s, max pitch 0.01036 rad, roll 0.00282 rad |
| `codex_upkie_push_fixed_1` | SUCCESS, all four pulses verified | 25 s; max pitch 0.1853 rad, roll 0.000633 rad; final recovery pitch <=0.0103 rad |
| `upkie_codex_fixed_push_1`, `upkie_codex_fixed_push_2` | SUCCESS, both full sequences verified | Two repeats of the suite's 26 s push/drive test; max pitch 0.1848/0.1846 rad, roll <=0.000220 rad |
| `upkie_codex_fixed_roll_1`, `upkie_codex_fixed_roll_2` | SUCCESS | Two 27 s forward/brake tests; measured mean +0.2017/+0.2017 m/s for +0.2 m/s |
| `codex_upkie_reverse_drive` | SUCCESS, 16 s | Measured mean -0.1996 m/s for -0.2 m/s; max pitch 0.01359 rad, roll 0.000228 rad |
| `codex_h2_xbody_regression` | SUCCESS, 15 s | Shared simulator-frame change: H2 forward/brake smoke test, no fall, max pitch 0.02597 rad and roll 0.01472 rad; this is not a full H2 regression |
| `codex_g1_xbody_regression` | SUCCESS, 15 s | G1 forward/brake smoke test, no fall, mean +0.1778 m/s for +0.2 m/s; max pitch 0.00841 rad and roll 0.00586 rad |

Push disturbances were unchanged: +/-1.5 Nm pitch torque then +/-6 N forward force, each 0.2 s, with 3 s pauses.
In the first corrected push trial both contact flags stayed true, measured normal loads remained positive during recovery, and no joint reached the configured effort limits.
The fixed yaw trial's pre-fall-window rate-versus-pose-derivative RMS mismatch dropped from 0.0436 to 0.00230 rad/s for yaw, and from 0.0778 to 0.000719 rad/s for pitch.
The phase/sampling comparison is diagnostic rather than an exact trajectory-matched estimator benchmark.
Joint command/measurement plots show tight position tracking after the fixes; wheel velocities use the mirrored encoder sign convention, so both encoder speeds having the same sign during an in-place turn is correct.
Typical mean solves were 1.9-2.6 ms and p95 about 4.0-4.4 ms; actual logged rate was mostly 96-98 Hz at the requested 100 Hz.
Occasional wall-time outliers occurred, including a 263 ms sample in the first push run, so the earlier 4.9 ms peak is not a worst-case bound and no higher-rate guarantee is justified.
Retain 100 Hz for this tested checkpoint rather than changing frequency while validating the model fixes.
Slow-turn tracking, yaw deadband/friction effects, arbitrary push directions and larger disturbances remain tuning/validation work; the current result is not a guarantee of general push recovery.

Visual evidence on the Mac: `/tmp/codex_upkie_yaw_replay.png` compares measured-state replays of the original 0.3 rad/s fall with the corrected 1.5 rad/s turn, and `/tmp/codex_upkie_push_fixed_1_wheeled.png` shows attitude, velocity, slip, contact force and effort recovery curves.
The mapped Tk GUI was also captured and inspected at `/tmp/codex_upkie_gui_height.png`, with a centered 0.300 m slider and labeled 0.220/0.380 m endpoints; its preview callback publishes nothing.
The analyzer now writes `_wheeled_tracking.png` with planned/measured yaw rate, per-joint command errors and pose-derivative consistency, and includes these statistics in the JSON summary.
The analytic wheel geometry/basis derivative executable passed finite-difference regression checks; four headless GUI tests and two sourced ROS launch-resolution tests passed, as did the two simulator feedback-frame tests.
No simulator-frame compatibility switch was added: XBODY restores the already-advertised link-frame contract rather than offering a second measurement convention.


## Follow-up Model Audit (2026-11-14)

The frame correction is shared with H2/G1: their ground-truth twist now follows the link axes and origin that their pose and controller already assume.
It changes numerical feedback when the inertia axes or COM differ, but does not change those robots' MPC equations, weights, gains or estimator implementation.
H2 also passed `/tmp/codex_h2_xbody_yaw_audit` at requested 80 Hz: +0.3 rad/s averaged +0.29645 rad/s, max pitch/roll 0.01582/0.02635 rad, with no fall over 18 s.
That in-place turn drifted 0.495 m, so it is a turning smoke test rather than proof of unchanged full H2 performance.

The audit compared the generated RNEA and wheel equations against direct Pinocchio calculations, differentiated the material tread slip numerically along the double-integrator trajectory, checked every state/input Jacobian column, checked wrench virtual work, and checked double-support input-equality rank at 12 nominal/perturbed states.
`tests/wheel_model_consistency_test.cpp` passed: maximum equation error 1.07568e-10, maximum normalized AD Jacobian error 1.18102e-08, and minimum singular value of the 23-by-30 input-equality Jacobian 0.0715591.
Finite differences are used only for testing; production derivatives remain CppAD generated.
No further defect was found in these tested equations; this finite audit is not a proof for arbitrary tilt, changing ground normals, flight or single support.

The scan found and corrected two initialization/playback inconsistencies:

- `WbIdInitializer` now seeds each supporting wheel moment as `rho x force` and adds configured smoothed joint friction to its seed torque.
  The native audit checks its joint RNEA and tire-moment residuals at nonzero velocity and tilt; this is still an initial guess, not a projection onto all base/rolling equalities.
  Wheel-disabled, friction-empty G1/H2 configurations retain the old initializer calculation.
- The optional stand-up exporter used a pre-step lift-off time and omitted the actual lift sample, leaving handover after its last exported sample.
  The regenerated plan ends at its 0.6965 s handover; all earlier position/velocity/effort rows are unchanged and no search/tuning was performed.
  Loading now rejects nonfinite sample values and non-strict timestamps, and GT bring-up uses the base orientation for tilt/gravity hold rather than the rotated IMU-site orientation.
  Estimator-based bring-up retains its alignment assumption; full-stack stand-up remains unvalidated and disabled by default.

Six local Python tests pass (four GUI, two stand-up export/data), along with the two separate simulator frame tests.
The sourced ROS launch suite passes six of seven tests; its unrelated environment test expects at least one visual, but the user's currently edited `description/urdf/env.urdf.xacro` is empty.
That user edit is preserved and excluded from the commit rather than modifying their scene or weakening the test.

To rerun the native consistency audit from the sourced container workspace:

```bash
MAKEFLAGS=-j2 colcon build --symlink-install --packages-select legged_robot_mpc_controller \
  --cmake-target wheel_model_consistency_test \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
./build/legged_robot_mpc_controller/wheel_model_consistency_test \
  src/legged_robot_mpc_controller/description/urdf/robots/upkie/upkie.urdf \
  /root/ocs2_ros2_ws/auto_generated/upkie
```

Use the normal Release build with `BUILD_TESTING=OFF` afterward to install all targets.

The final complete Release build passed in 1 min 45 s, and the native equation/Jacobian audit passed again against the installed libraries.
The following tests were rerun after the initializer and GT bring-up corrections, without parameter changes:

| Container artifact prefix under `/tmp/` | Result | Measurements |
| --- | --- | --- |
| `codex_upkie_final_yaw` | SUCCESS, 17 s | Abrupt +1.5 rad/s request; mean +1.1199 rad/s; max pitch/roll 0.01207/0.00309 rad |
| `codex_upkie_final_push` | SUCCESS, 25 s, actual repository night scene | All four original pulses verified; max pitch 0.18804 rad; final recovery pitch <=0.00401 rad; no actuator saturation |
| `codex_upkie_final_drive` | SUCCESS, 21 s, actual repository night scene | +0.2 m/s request averaged +0.20146 m/s; max pitch/roll 0.01246/0.00076 rad; braking remained balanced |
| `codex_h2_xbody_final_yaw` | SUCCESS, 18 s at requested 80 Hz | +0.3 rad/s request averaged +0.29774 rad/s; max pitch/roll 0.02061/0.04042 rad; translation drift 0.487 m |

Logged actual rate means were 97.46 Hz for Upkie yaw, 97.58 Hz for push, 95.59 Hz for drive, and 77.48 Hz for H2 at its 80 Hz target.
These are VM wall-clock measurements with scheduling outliers, not guaranteed rates; Upkie drive had one 82.11 Hz reporting window.
All final Upkie health windows reported zero solver exceptions and torque-window overruns, but finite equality residuals remain because the controller uses one SQP iteration.
The updated measured-state replay `/tmp/codex_upkie_yaw_replay.png` compares the original fall with the final yaw trial; `/tmp/codex_upkie_final_push_wheeled.png` and `/tmp/codex_upkie_final_yaw_wheeled_tracking.png` were generated and inspected.
The push curves show decaying attitude/velocity responses, positive wheel loads and no saturation; the turn still shows periodic planned-rate ripple and command undertracking, not an exact yaw-tracking solution.
All owned simulations were stopped after these tests; full-stack stand-up and arbitrary-direction pushes remain future validation.
The required simulator fix was pushed as `mujoco_ros2_control` commit `44f395d` on `jazzy`; pull and rebuild it together with this controller checkpoint.


## Status (2026-11-14)

| Milestone | State |
| --- | --- |
| Model import and MJCF/URDF reconciliation | Done |
| Rolling-wheel WB-ID MPC, wheel drive, ground truth (standing spawn) | Works: balance, 0.2 m/s drive and braking in the first run |
| Unmodelled wheel friction (zero-command creep) | Diagnosed; friction model added; comparison against wheel PD started, only the friction-model variant ran (results below) |
| Activation segfault | Root cause found and fixed (hard-coded foot frame names) |
| Flat acceptance after yaw/push fixes | Passed repeated forward/brake, reverse/brake, four complete push sequences and turning tests without falls; final night-scene drive/push repeats also passed; slow +0.3 rad/s turn still undertracks |
| Kneeling idle -> planned stand-up -> MPC | Planned and implemented; never run in the full stack |
| Small symmetric hop | Not started (gait-request topic is ready) |
| Short single support, then trot | Not started |
| Estimator closed loop (linear KF adapted to rolling) | Not started; ground truth only so far |
| G1/H2 regression after the shared-code changes | Passed short flat forward/brake smoke tests and two H2 80 Hz turning tests; turning drift remains and broader regression is open |


## Terminology

Driving with both wheels in rolling contact is double support with no contact switches.
Ascento and wheeled ANYmal (Bjelonic et al.) call it **driving** (hybrid walking-driving when mixed with steps), Xin & Vijayakumar **rolling**, the user's documents **wheel drive**.
This branch uses **wheel drive**.


## Model

| Item | Choice |
| --- | --- |
| Robot | Upkie: hip, knee and wheel per side, six actuators, 5.4605 kg |
| Simulation reference | [mjlab_upkie](https://github.com/MarcDcls/mjlab_upkie) `d789578` MJCF: servos replaced by unit-gear torque motors capped at 16 N m (hip/knee) and 1.7 N m (wheel); [`upkie.xml`](../description/mujoco/robots/upkie/upkie.xml), [`scene.xml`](../description/mujoco/robots/upkie/scene.xml) |
| MPC model | [`upkie.urdf`](../description/urdf/robots/upkie/upkie.urdf), generated from the MJCF by [`generate_upkie_urdf.py`](../description/urdf/robots/upkie/generate_upkie_urdf.py) |
| Agreement | FK 2e-9 m, gravity torques 5e-9 N m, joint mass matrix (with armature) 1e-10, over random poses |
| Passive joint terms | MJCF armature 0.005 kg m^2 (16x the wheel's own spin inertia) and friction loss 0.1 N m on every joint, both now modelled in the MPC |
| Wheels | Scalar revolute joints (nq = nv = 12), excluded from position limits and posture costs; zero wheel position gain (enforced at configure) |
| ros2_control | [`upkie.ros2_control.xacro`](../description/urdf/robots/upkie/upkie.ros2_control.xacro): `effort_pd` on six joints, `base` GT body sensor, `base_imu`, simulation contact sensors on both wheel bodies |
| Nominal posture | Level trunk 0.30 m, knees forward, tire under the CoM: hip -0.6021, knee -0.9655 (right mirrored) |
| Joint signs | [MODEL_SOURCE.md](../description/urdf/robots/upkie/MODEL_SOURCE.md); positive left wheel rolls forward, positive right wheel backward |

The upstream URDF (5.339 kg, 50 mm tire, different inertias and joint ranges) is a different CAD export, so the MPC URDF is generated from the MJCF rather than reconciled by hand.


## Controller changes

### Rolling-wheel contact ([`WbIdWheelContactConstraintCppAd`](../src/mpc/wb_id/constraint/WbIdWheelContactConstraintCppAd.cpp))

The contact frames are the tire centres on the wheel links (`contactParentJointNames` = wheel joints, `frameTranslation` 31.2 mm along the axle); they spin with the wheel but their position does not.
The wrench `[f; m]` stays at the tire centre, so the RNEA rows are unchanged; `m = rho x f` makes `J_h^T [f; m] = J_c^T f`.
Shared kinematics (contact offset `rho`, its rate, material velocity/acceleration) are in [`WheelContactKinematics.h`](../include/legged_robot_mpc_controller/mpc/wb_id/model/WheelContactKinematics.h).

| Term | Rows | Active |
| --- | --- | --- |
| `<tire>_wheelRolling` | longitudinal tread slip (accel + velocity), gap (accel + velocity + position), `m - rho x f` (3) | wheel in contact |
| `wheelLateralNoSlip` | one lateral slip row, the contact-weighted mean over the wheels in contact | any wheel in contact |
| `<tire>_wheelSwingVertical` | tire bottom (centre z - r) follows the swing height spline | wheel in the air |
| `<tire>_zeroWrench` (existing) | zero wrench | wheel in the air |

The lateral row is shared because, with no hip roll, both wheels' lateral rows are the same function of the base motion in symmetric double support; two such equalities made the projected equality system rank deficient.
`wheelContact` replaces `footMotionEquality` and `contactMomentXY`, which must be off.
Friction cone and torque/velocity limits are the existing terms.

### Other changes

| Change | Where | Default (G1/H2 unaffected) |
| --- | --- | --- |
| Joint armature in the Pinocchio model (RNEA/CRBA include it) | `ocs2.model.jointArmature`, [createPinocchioModel.cpp](../src/mpc/common/model/createPinocchioModel.cpp) | empty |
| Coulomb joint friction `f_c tanh(v / v_s)` in the joint RNEA rows and the ID fallback | `ocs2.model.jointFriction`, `jointFrictionVelocity`, [WbIdRneaConstraintCppAd.cpp](../src/mpc/wb_id/constraint/WbIdRneaConstraintCppAd.cpp) | empty (separate CppAD library name when on) |
| Wheel joints get +/-inf position bounds and must have zero posture weight | config builder, factory | only with `wheelContact` |
| Arm-swing joints optional (robot without arms) | ModelSettings, builder | 4 names still required when arm swing is on |
| `computeJointTorques` takes the contact frame names (was hard-coded `foot_l/r_contact`: out-of-range read and intermittent segfault on Upkie) | DynamicsHelperFunctions | humanoid names |
| Planned stand-up bring-up for any floating-base source | `bringUp.standUpTrajectoryFile`, `idleSeconds`; controller | empty = humanoid bring-up |
| Height reference restarts at the measured height on the stand-up handover | `WalkingVelocityTarget::requestFilterReset(height)` | unchanged otherwise |
| Gait-request topic: hold a named template instead of the velocity ladder (hop/single-support tests) | `target.gaitTopic` (`/humanoid/gait_command`, `std_msgs/String`, empty = ladder) | subscribed, unused unless published |
| Tracking columns: wheel gap/slip at the contact point, measured wheel normal force and contact | `diagnostics.contactSensorNames`, `~/tracking` | none |
| Stand-up playback feed-forward clamped like the MPC's | `clamp_torque` | n/a |
| Upkie in the launch (`robotName:=upkie`, GT body `base`) and test harness (`--robot-name upkie`, `--launch-arg`) | [robot.launch.py](../launch/robot.launch.py), [wb_id_walk_test.py](../tests/wb_id_walk_test.py) | n/a |

The torque clamp is the same as H2's: the MPC feed-forward is clamped to `jointTorqueLimits` (URDF efforts 16/1.7 N m), the OCP penalises the same limits, and the MuJoCo motors' `ctrlrange` caps the summed PD + feed-forward.


## Results so far (ground truth, flat, standing spawn, MPC 100 Hz / MRT 1 kHz)

First run, before the friction model ([`tests/wb_id_walk_test.py`](../tests/wb_id_walk_test.py), analysed with [`analyze_wheeled_biped.py`](../tests/tuning/analyze_wheeled_biped.py)):

| Phase | Pitch max | Forward speed | Wheel normal force | Tread slip rms | Solve |
| --- | --- | --- | --- | --- | --- |
| Zero command, 10 s | 0.026 rad | -0.018 m/s (creep) | 26-27 N each | <= 0.04 m/s | 2.7 ms |
| Hold 0.2 m/s, 5 s | 0.021 rad | 0.223 m/s | 25.9-27.5 N | <= 0.005 m/s | 2.8 ms |
| Rest after braking, 8 s | 0.019 rad | +0.021 m/s (creep) | 26.2-27.4 N | <= 0.004 m/s | 2.9 ms |

Both wheels in contact throughout (measured flags 1.000), gap <= 0.3 mm, no effort saturation; verdict SUCCESS.
The spawn landing (now 0.5 mm instead of 2 mm) caused a 0.1 rad yaw change in the first 0.2 s; yaw angle is not tracked in `base_twist`.

### The zero-command creep and the friction model (answers to the user's questions)

**What the creep was.** At zero command the robot did not come to rest: it rolled at a steady 1.8 cm/s backwards (17 cm in 10 s, trunk leaning back 0.015 rad), and after braking at a steady 2 cm/s forwards.
It neither fell nor accelerated.
The MPC's planned velocity equalled the measured one, so it was not a tracking error: every solve planned a correction that did not happen.

**Why.** The MJCF has 0.1 N m Coulomb friction on every joint; the MPC model had none.
A wheeled pendulum balances near standstill with wheel torques of about 0.1 N m, the size of the friction: the MPC commanded +/-0.09 N m and the friction absorbed it.
The robot settled into lean + constant wheel speed + motor torque = friction torque, and an MPC without integral action replans the same thing every time.
With the wheel friction removed from a test scene the creep was 0.002 m/s and the wheel torques about 0.

**How friction enters the MPC.** The joint rows of the inverse-dynamics equality become `M a + h + f_c tanh(v / v_s) - tau - J^T W = 0` (`jointFriction` 0.1 N m, `jointFrictionVelocity` 0.05 rad/s), so the planned motor torque includes friction compensation and the policy feed-forward carries it.
`tanh(0) = 0`: static friction at standstill remains (a few mm of CoM deadband).

**H2 does not model it.** H2's MJCF has friction loss 0.2 N m, armature 0.01 and damping 0.05, but its MPC models none of them.
There it is negligible (hundreds of N m, planted feet, stiff kp 600 servo); on Upkie the wheels run kp 0 with a 1.7 N m budget and balance on ~0.1 N m.

**Should the low-level PD fix it, and do papers control wheel velocity only?**
The MPC sends the planned torque (feed-forward) and the planned joint positions and velocities, which `effort_pd` tracks at 2 kHz.
In the creep the planned wheel speed itself equalled the creep, because every solve restarts from the measured state and the base position is not tracked in `base_twist`; the PD therefore saw no error, and the root cause was the MPC model.
The wheel speed loop is still the right inner loop for errors between solves (friction, bumps, contact transients), so the plan is to use both: friction in the MPC and wheel speed damping.
In the literature the wheels are tracked by velocity (plus torque): wheeled ANYmal (Bjelonic et al.) tracks wheel speeds, Upkie's own MPC balancer commands wheel ground velocity with zero wheel position gain, Ascento's balancer feeds back wheel speed and body pitch.
The planned wheel angle is re-anchored to the measurement at every solve, so a wheel kp would only act within one 10 ms interval and adds stiffness against the rolling the base motion requires; holding position belongs to the MPC (a base position/velocity cost), not to wheel kp, which is why the user's document asks for zero wheel position gain (enforced at configure).

### Friction-model vs wheel-PD comparison (last test)

`tests/tuning/upkie_acceptance_suite.sh` was run (as `/tmp/claude_upkie_suite.sh`, results under `/tmp/claude_upkie_<variant>_{roll,push}_1.*` in the `ros_jazzy` container) for three variants:

| Variant | MPC friction | Wheel kd (N m s/rad) |
| --- | --- | --- |
| `model_only` | 0.1 N m | 0.05 |
| `pd_only` | 0 | 0.3 |
| `both` | 0.1 N m | 0.3 |

Stopped early on the user's request; only `model_only` ran:

| Run | Result |
| --- | --- |
| `model_only` roll+brake | SUCCESS (`/tmp/claude_upkie_model_only_roll_1*`; drift numbers not yet extracted, use `compare_wheeled_biped_runs.py`) |
| `model_only` push (4 pulses in the zero-command phase) | Survived all four pulses and finished every phase: base height 0.300 m throughout, pitch <= 0.01 rad, position within about 6 cm. Yaw swung by up to 0.24 rad during the pushes (open item). The verdict was lost to a harness bug (odometry written after its CSV was closed when a wrench sequence is used), now fixed in `wb_id_walk_test.py` |
| `pd_only`, `both` | Not run |

Servo structure in every variant: wheels kd only (kp 0, damping towards the planned wheel speed), hips and knees kp and kd around the planned trajectory, and the MPC torque feed-forward always on all six joints.


## Planned stand-up (kneeling idle -> MPC), not yet run in the stack

The user's start-up: an idle pose resting on the floor (kneeling on the knee-motor housings and both wheels, as in the Upkie photo), a planned feasible stand-up, then the MPC; the same phase machine as the H2 bring-up.

Kneeling, the CoM stays 5-16 cm ahead of the wheel contacts for every hip/knee configuration that keeps the knees down (knees carry 29-93% of the weight), so a quasi-static stand-up is impossible, and the wheel torque alone (2 x 1.7 N m) only drives the robot forward on its knees.
As in Ascento (an apply-torque phase until the body has the energy to reach upright, then the balancer) and Upkie (open loop until just before the body leaves the floor, then MPC), the stand-up is a planned dynamic push:

1. fold to hip -0.504 / knee -2.687 in 0.533 s (knees fully folded bring the CoM closest to the wheels);
2. push to hip -0.207 / knee -2.196 in 0.377 s with 1.361 N m forward wheel torque: the hips press the knee housings into the floor and the body pivots back about the axle.

The parameters minimise the pendulum's divergent component at knee lift-off, `xi = (x_com - x_wheel) + v_rel / omega`; with `xi ~ 0` the body coasts towards CoM-over-wheel.
Exported plan: handover (knee lift-off) at 0.696 s, xi +0.012 m, pitch -0.71 rad (nose up), pitch rate -0.67 rad/s, CoM height 0.104 m, largest servo command 0.80 of the limits.
[`plan_stand_up.py`](../config/upkie/plan_stand_up.py) (offline, MuJoCo Python) reproduces it and has a `--search` option; output [`stand_up_trajectory.yaml`](../config/upkie/stand_up_trajectory.yaml), idle spawn [`initial_pose.yaml`](../config/upkie/initial_pose.yaml).

Controller flow with `bringUp.standUpTrajectoryFile` set: Passive (idle hold with the plan's gains, `idleSeconds` 3 s) -> StandUp (playback of measured q, v and applied torque from the planning rollout; the MPC starts solving at its start) -> at `handover_time`, if a policy is valid, Blend (`blendSeconds` 0.05) -> Locomotion, with the height reference restarted at the measured height and ramped at 0.1 m/s to 0.30 m; no valid policy -> Fault.
`tiltLimit` is 1.2 rad for Upkie because the trunk leans back 0.71 rad at the handover.

To run (not yet done; minimum height must allow the low crouch):

```bash
python3 src/legged_robot_mpc_controller/tests/wb_id_walk_test.py --robot-name upkie --source ground_truth_state \
  --minimum-height 0.05 --startup-timeout 900 --record-tracking \
  --axis vx --speed 0.05 --hold 1 --ramp 1 --settle 10 --rest 4 \
  --prefix /tmp/upkie_standup
```

Risks to check first: the MPC catching a tucked, back-leaning, low robot (it is the first state the MPC sees with only wheel contacts); the idle settle in the full stack matching the planning rollout (gains 20/0.5, 3 s); the harness's 0.15 s fall detector during the rise.


## Test it yourself

In the `ros_jazzy` container, from `/root/ocs2_ros2_ws` (launch from there: the CppAD cache path `auto_generated/upkie` is relative), after the build below:

```bash
source install/setup.bash
# 1. Wheel drive with the GUI: kneeling spawn, planned stand-up, linear KF in control (all defaults), MuJoCo and
#    RViz windows. The MPC takes over about 5 s after the controller starts; GT is only published for comparison.
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=upkie mpcFreq:=200
#    The GUI's forward axis is normalised by maxDisplacementVelocityX (1.0 m/s); keep vy at 0
#    (a wheeled-biped cannot move sideways). Yaw rate turns by differential wheel speed.

# 2. The same from a script instead of the GUI (another terminal):
ros2 topic pub -r 20 /humanoid/walking_velocity_command ocs2_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.2, linear_velocity_y: 0.0, desired_pelvis_height: 0.30, angular_velocity_z: 0.0}"

# 3. Push the base from another terminal after MPC takeover.
SHARE="$(ros2 pkg prefix --share legged_robot_mpc_controller)"
ros2 run legged_robot_mpc_controller push_recovery_test.py \
  --sequence "$SHARE/config/tests/upkie_pitch_push.yaml" --delay 5

# 4. Scripted, headless, logged (one roll+brake and one push run, verdicts printed):
bash src/legged_robot_mpc_controller/tests/tuning/upkie_acceptance_suite.sh mine 1
python3 src/legged_robot_mpc_controller/tests/tuning/analyze_wheeled_biped.py /tmp/upkie_mine_roll_1 --plot

# 5. Kneeling idle -> planned stand-up -> MPC (not yet validated): see "Planned stand-up" below.
```

Do not run two simulations at once (shared topic names).
On the GUI the height slider is absolute; the Upkie nominal is 0.30 m and the range is `minBaseHeight`/`maxBaseHeight`, 0.22-0.33 m.


## How to build and test

Container `ros_jazzy` on the Parallels VM; sources from the Mac at `/root/src`; workspace `/root/ocs2_ros2_ws`.

```bash
cd /root/ocs2_ros2_ws && source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --packages-select legged_robot_mpc_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
# manual run (standing spawn, ground truth)
ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=upkie mpcFreq:=200 mrtFreq:=1000 \
  floatingBaseSource:=ground_truth_state floatingBaseEstimator:=none rviz:=true mujoco_headless:=false
# acceptance suite: TAG N [extra harness args]; results /tmp/upkie_<TAG>_{roll,push}_<i>.*
bash src/legged_robot_mpc_controller/tests/tuning/upkie_acceptance_suite.sh m1 3 \
  --launch-prefix "gdb -batch -ex run -ex bt --args"
```

Analyse on any machine with numpy/matplotlib: `tests/tuning/analyze_wheeled_biped.py PREFIX --plot` (per phase: pitch, speed, per-joint feed-forward vs applied effort and saturation, wheel normal force/contact, slip, gap, solver residuals) and `tests/tuning/compare_wheeled_biped_runs.py PREFIX...` (drift and push table).
The pitch push sequence is [`config/tests/upkie_pitch_push.yaml`](../config/tests/upkie_pitch_push.yaml): +/-1.5 N m pitch torque and +/-6 N base force pulses of 0.2 s.
A full rebuild takes about 11 minutes (ModelSettings/helper headers are included everywhere); the first launch after a CppAD-relevant change generates the libraries in `auto_generated/upkie/` (about a minute).


## Next steps, in order

Priority from the user: preventing a fall and controlling base pitch/roll comes first; accept height error, unequal leg lengths, yaw error and XY drift when needed.
These are proposed follow-up milestones, not permission to silently expand the control architecture.

| Order | Milestone | Current boundary and next evidence needed |
| --- | --- | --- |
| 1 | Larger asymmetric edges and slow terrain entries | Superseded by "Tire Collision Fix and 20 mm Edge Analysis" at the top: the fall mechanism is identified (slip feedback on a blocked wheel), the tire collision is fixed, and the mesh-tire 20 mm entry climbs but then falls off the strip's side |
| 2 | Broader no-map terrain acceptance | Still untested: more random seeds, reverse traversal, ramp descent/far drop, turns while on the ramp/field, and sudden stops at edges; preserve the passed small-terrain checkpoint |
| 3 | Push-recovery envelope | Only the original +/-1.5 Nm pitch and +/-6 N fore/aft pulses are validated; lateral/yaw pushes, combined disturbances and pushes on unequal-height terrain remain untested |
| 4 | Height envelope without sacrificing balance | Current tests cover 0.26-0.34 m and a height change on a 10 mm split; GUI endpoints 0.22/0.33 m, faster height changes and a range of support-height differences need separate testing |
| 5 | Rolling-contact estimated-state control | Passes in simulation (2026-11-15, top section): wheel-aware linear KF with IMU extrinsics drives the MPC through the full acceptance set; noise-free sensors only, slip gating not added |
| 6 | Kneeling stand-up and handover | Passes on GT and on the estimator (2026-11-15); the catch still crosses the legs for about 0.7 s |
| 7 | Flight/jump | Passes (2026-11-15): 0.15 s flight, 11 mm clearance, in place, repeated and while rolling, on GT and on the estimator |
| 8 | Single support and alternating support | Passes for 0.15 s lifts and alternating lifts at rest and rolling (2026-11-15); a 0.3 s lift reaches 0.073 rad roll |

For item 1, the current model continues to schedule both wheels as supporting even when measured contact is lost, and its contact height/normal is a fixed world plane.
The failing 20 mm trials show zero-load intervals and wheel torque saturation; those are observed symptoms, not a proven sole root cause.
A wheel-aware, per-contact local reference updated from robot pose/encoders is a possible implementation correction if parameter-only tests are exhausted, not an existing capability and not an excuse to supply a GT terrain map.
Do not simply enable the generic `groundHeight.useEstimate`: the current shared foot-height helper averages tire-center frames rather than tread contacts, introducing a tire-radius offset and still not representing separate left/right support heights.
Retain the corrected XBODY simulator feedback and compare failure windows before choosing that change.
The source-independent H2/G1 regression and broader wheel friction-versus-damping calibration also remain open, but do not change their parameters as part of the Upkie-only tuning.


## Traps found

- rclcpp rejects empty YAML lists (`fixedJointNames: []`): omit the key.
- `pkill -f <pattern>` inside `bash -c` matches its own command line (the workspace path contains `ros2`); kill by PID.
- Editing the parameter schema while a build runs leaves the generated header stale; rebuild after.
- `jointArmature` is baked into the generated CppAD libraries (set `codegen.recompileLibraries` or clear `auto_generated/upkie` after changing it); `jointFriction` and the wheel gains are parameters.
- Pass the gdb launch prefix as one argument: `--launch-prefix "gdb -batch -ex run -ex bt --args"`.
- Old `mujoco_ros2_control` builds returned principal-inertia/COM twist alongside link pose; rebuild the simulator with the XBODY fix, not only the controller, and sanity-check rates against pose differences.


## Operator Posture FSM (2026-11-21)

Requested: repeatable idle -> stand-up -> MPC -> sit-down -> idle, GUI buttons and DualSense R1 toggle, and a latched fall state without automatic fall recovery.
Upkie now uses `bringUp.autoStart: false`; the operator explicitly requests the first and each subsequent stand-up.
The first estimated-state 200 Hz trial stood up and drove successfully, but directly interpolating the legs from standing to the kneeling pose tipped the body past the 1.2 rad fall threshold.
That trial is a rejected implementation, not a successful sit-down: `/tmp/codex_upkie_fsm_est.*` in `ros_jazzy` contains its log, pose/joint replay and failure report.
The revised transition keeps MPC balancing while braking and lowering to a controller-only 0.10 m crouch before interpolating to the supported idle pose.
The normal GUI/joystick height range remains unchanged; the internal crouch override is not an operator command or a change to balance weights.
The worker acknowledges a pause before MRT policies are cleared; restarting uses current observations and a worker-side warm-start reset rather than an old cycle's plan.
Stand-up, sit-down and toggle requests are rejected in intermediate and terminal states rather than queued.
DualSense R1 is SDL button 10, edge-triggered, requires L2 released, neutral sticks and L1/D-pad/Circle released; holding R1 across startup or a blocked press never triggers later.
The teleop profile enables these requests only for a controller with a stand-up trajectory; existing humanoid motion mapping is unchanged.
All 80 teleop checks currently pass, including actual isolated DDS service requests and the L2/Center regression cases.
The final Release build passed in 1 min 49 s; only existing compiler warnings remain, and no CppAD model regeneration was needed.
The kneeling interpolation masks out zero-position-gain joints, so free wheel angles never become a wheel-speed return target.
Missing joint state or trunk attitude latches Fault; excessive trunk tilt latches Fallen even in planned idle, and both immediately disable operator commands, pause solves, zero position gains and feedforward, and retain velocity damping.
Lifecycle-inactive controllers refuse posture requests.

### Acceptance

| Trial | Result | Evidence |
| --- | --- | --- |
| Estimated state, real adapter with synthetic R1/L2, 200 Hz MPC target | Three full stand/drive/sit/idle cycles passed | `/tmp/codex_upkie_fsm_final.{out,log,json,replay.json}` in `ros_jazzy` |
| Deliberate lateral 150 N, 0.5 s pulse after those cycles | Fallen latched; stand_up, sit_down and toggle_stand all refused; no automatic retry | Same final trial |
| Ground-truth state, final binary, direct services | Two cycles passed | `/tmp/codex_upkie_fsm_gt_final.*` |
| GUI buttons, launch selection and GUI height mapping | 17 checks passed under isolated Xvfb/DDS | `tests/test_base_command_posture.py`, GUI/launch tests |
| Package layout, launch, startup/solver contracts, Upkie assets and GUI mapping | 40 passed, one optional Python-MuJoCo check skipped in the container | Focused pytest run |

The final estimated-state run's maximum locomotion-labelled tilt was 0.450 rad including the dynamic catches; braking stayed below 0.005 rad and MPC lowering below 0.064 rad.
Kneeling reached a maximum 0.914 rad trunk lean, below the unchanged 1.2 rad fall threshold, and settled on the supported idle pose.
The ground-truth comparison's maximum locomotion-labelled tilt was 0.286 rad.
The deliberately forced fall is excluded from these normal-cycle statistics.
The final estimated replay was rendered with MuJoCo 3.13 and visually inspected: `/tmp/codex_upkie_fsm_final.png` on the Mac shows balanced lowering, supported kneeling and successful re-stand.
Earlier direct-fold and intermediate build artifacts are retained only as diagnostics, not as accepted implementations.
Robot state and Joy are independent DDS topics; the controller's service-side phase check is authoritative even if a client's state display is one heartbeat behind.

### Run and Remaining Limits

See [Upkie posture control](./upkie_posture_control.md) for GUI, R1, services, parameters and repeatable test commands.
The interactive default now waits for Stand Up or R1; `wb_id_walk_test.py` explicitly opts automated locomotion tests into first-cycle auto-start.
No balance weights, contact equations, robot descriptions, simulator code or OCS2 stack were changed for this feature.
Sit-down has been validated on flat ground, not slopes, steps, unequal support heights or physical hardware.
The real PS5 device still needs user acceptance; synthetic Joy tests cover the actual teleop and service paths, not USB/Bluetooth transport.
There is no fall-recovery phase: manually restore supported idle before resetting the controller, or reset the simulation.
After validation, the user requested committing and pushing the MPC changes on `feat/wheeled_biped` and the companion teleop changes on `main`.
The final install refresh rebuilt both packages successfully in 3.18 s.
All 80 teleop checks passed again after adding held-R1 reconnect protection and validating heartbeat-before-edge ordering in the isolated service test.
All owned simulator/test processes were stopped; no robot launch was left running.

Suggested MPC commit message: `feat(upkie): add repeatable operator stand/sit FSM and latched fall handling`.
Suggested teleop commit message: `feat: add guarded DualSense R1 posture requests for Upkie`.
