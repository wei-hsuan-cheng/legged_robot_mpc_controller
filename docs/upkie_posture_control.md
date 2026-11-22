# Upkie Posture Control

Upkie starts kneeling in idle and waits for the operator.
Its MPC balance weights, rolling-contact model and 200 Hz default recommendation are unchanged.
The 80 mm wheels use a gentle, approximately 13 s contact-aware stand-up with both knees and both wheels modeled offline.
The [stand-up optimization note](upkie_stand_up_optimization.md) gives the equations, references, feedback/handover guards, regeneration commands and measured comparison.


## GUI

```bash
ros2 launch legged_robot_mpc_controller robot.launch.py \
  robotName:=upkie mpcFreq:=200 baseCommandDevice:=gui
```

Press **Stand Up**, wait for **locomotion**, then drive.
Press **Sit Down** to brake, lower and kneel.
The controls become available again in idle; another Stand Up repeats the cycle.
Buttons are disabled during a transition or after a fall/fault.


## Joystick

```bash
ros2 launch legged_robot_mpc_controller robot.launch.py \
  robotName:=upkie mpcFreq:=200 baseCommandDevice:=joystick
```

Release L2, center both sticks, release L1/D-pad, and press **Square** to stand up.
After handover, release and press L2 with centered sticks to enable driving.
Release L2 and press Square again to sit down.
Square is edge-triggered: holding it does not repeat the request, and a press blocked by L2 or another interlock is never queued.
Square held while connecting is ignored until released and pressed again.
Circle is unassigned and reserved for future use; release L2 to stop driving.
The Square mapping is `button_posture: 2` in the teleop adapter (SDL's standardized X / west face button).
R1 no longer requests stand-up or sit-down with the default mapping.
The adapter enables this feature from the selected controller YAML only when a stand-up trajectory is configured.


## State Machine

```mermaid
flowchart LR
    I[Idle] -->|Stand Up / Square| S[Stand-up playback]
    S --> B[Blend into MPC]
    B --> L[Locomotion]
    L -->|Sit Down / Square| P[MPC braking]
    P --> C[MPC crouch]
    C --> K[Kneeling transition]
    K --> I
    L -. excessive tilt .-> F[Fallen: damping]
    S -. excessive tilt .-> F
    K -. excessive tilt .-> F
```

The tilt check also applies during braking, lowering and blending.
Any active planned posture phase with invalid joint state or trunk orientation enters a latched fault.
No automatic fallen-to-idle or fallen-to-stand-up transition exists.
Reset the controller only after manually restoring a supported kneeling pose, or restart the simulator.

The controller owns the state machine; GUI and joystick only request transitions.
Stand Up is accepted only in idle; Sit Down only in locomotion; toggle only in either of those states.
Requests in intermediate states are refused rather than queued.
Operator twist, pose and gait commands are ignored outside locomotion.

| Parameter | Meaning |
| --- | --- |
| `bringUp.autoStart` | False by default; true automatically starts only the first planned cycle |
| `bringUp.idleSeconds` | Minimum supported-idle dwell before stand-up, currently 3 s |
| `bringUp.sitDownBrakeSeconds` | Minimum MPC braking interval, currently 1 s |
| `bringUp.sitDownVelocityMax` | Maximum horizontal speed before lowering, currently 0.1 m/s |
| `bringUp.sitDownHeight` | Internal balanced-crouch height, currently 0.125 m; must lie within the posture table |
| `bringUp.sitDownSeconds` | Smooth leg-joint transition from crouch to supported kneeling, currently 3 s |
| `bringUp.tiltLimit` | Maximum trunk tilt before latching Fallen, currently 1.2 rad |

Braking also requires angular-rate norm below 0.3 rad/s.
Lowering hands over to kneeling only within 0.02 m of the crouch target and with trunk tilt below 0.3 rad.
The final joint interpolation uses $s=\operatorname{clip}(t/T,0,1)$ and $\alpha=3s^2-2s^3$.
Idle arrival requires controlled-joint error below 0.15 rad and joint-speed norm below 1 rad/s.
Braking, lowering and kneeling have timeouts which latch a fault instead of starting the next phase blindly.
The MPC worker is paused before old policies are cleared, so repeated stand-up never intentionally reuses the previous cycle's plan.
During the contact-aware stand-up, it starts only after the planned wheel-only interval and sustained kinematic knee clearance, not while the knees support the robot.


## Services and State

```bash
CTRL=/humanoid_whole_body_inverse_dynamics_mpc_controller
ros2 service call "$CTRL/stand_up" std_srvs/srv/Trigger '{}'
ros2 service call "$CTRL/sit_down" std_srvs/srv/Trigger '{}'
ros2 service call "$CTRL/toggle_stand" std_srvs/srv/Trigger '{}'
ros2 topic echo "$CTRL/motion_state" --qos-reliability best_effort
```

The state topic is best effort, depth 1, at 10 Hz, published outside the control-update thread.
The joystick adapter stops accepting driving inputs when this heartbeat is lost for 0.5 s.
Service responses report request acceptance, not physical completion; use the state topic to observe completion.


## Automated Simulation

Run from the sourced workspace, with no other robot using the same ROS domain.
The script owns its launch process group and stops it after the trial.

```bash
ROS_DOMAIN_ID=169 python3 src/legged_robot_mpc_controller/tests/upkie_posture_cycle_test.py \
  --cycles 3 --joystick --fall-test --prefix /tmp/upkie_posture_test
```

Synthetic Joy exercises the real teleop adapter and Square service path; a physical gamepad is not required.
The trial drives between cycles and checks transition refusals, persistent idle and a deliberately induced latched fall.
Remove `--joystick` to test direct controller service requests.
Add `--yaw-command .2` to turn between cycles and check re-anchoring at another heading.
Use `--trajectory /path/to/candidate.yaml` to validate an offline candidate without replacing the default.
Use `--source ground_truth_state` for a simulator-state comparison; normal operation uses the wheel-aware linear KF.
Reports, pose/joint replays and launch logs share the requested prefix.
These posture transitions are intended for flat, supported ground; sit-down on slopes, obstacles, asymmetric support and physical hardware requires separate validation.
Before attempting a physical deployment, read the [real hardware validation handoff](./design_notes/wheeled_biped/upkie_hardware_validation_handoff.md), including the missing hardware backend, stop semantics and measured transition guards.
