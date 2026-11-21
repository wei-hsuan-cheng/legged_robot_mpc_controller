# Upkie Hardware Validation Handoff

Checkpoint: 2026-11-21, MPC commit `01e642b` on `feat/wheeled_biped`, teleop commit `81be82d` on `main`.
This is a hardware-readiness assessment and future work checklist, not a claim of hardware validation.
The operator FSM architecture is reusable, but the current launch is not a complete real-Upkie deployment.
No hardware adapter, safety implementation, controller tuning or physical robot test was added for this assessment.


## Proven Boundary

The implemented sequence is idle -> planned stand-up -> MPC locomotion -> MPC braking -> MPC crouch -> kneeling -> idle.
The GUI supplies Stand Up / Sit Down; DualSense R1 requests the same transitions with L2 released and neutral controls.
Fallen and Fault are latched damping states; neither implements automatic recovery.
Three estimated-state R1 cycles, two final ground-truth service cycles, deliberately induced fall detection and GUI/teleop checks passed in simulation.
The R1 trials used synthetic Joy through the actual adapter, not a physical USB/Bluetooth gamepad.
Sit-down acceptance covers flat ground only.
Simulation timing at a 200 Hz MPC target does not establish timing on the robot computer.
See [posture controls](../../upkie_posture_control.md) and [recorded simulation evidence](../../wheeled_biped_handoff.md#operator-posture-fsm-2026-11-21).


## Missing Pieces

| Priority | Area | Current limitation | Required work and evidence |
| --- | --- | --- | --- |
| P0 | Real hardware backend | Upkie's ros2_control description selects MuJoCo or fake hardware only; disabling both does not select a real driver | Integrate the actual Upkie actuator/IMU interface, export the claimed interfaces and validate read/write behavior before enabling motion |
| P0 | Stop, release and watchdog semantics | Software damping and zero velocity are not an independent actuator stop; process death cannot execute the software FSM | Define and validate hardware-backed stop behavior, communication watchdogs, actuator fault handling and controller lifecycle behavior |
| P0 | Complete actuator limits | Controller clipping does not independently bound the final PD-plus-feedforward torque on arbitrary hardware | Enforce the full command's actual motor limits, with correct joint units, direction, gear ratio and gain mapping |
| P1 | Fresh feedback | Finite joint/IMU values can remain frozen after communication failure | Expose acquisition age and health; reject stale feedback and test loss/freeze scenarios using controlled fault injection |
| P1 | MPC ready before push-off | Solver construction currently begins alongside playback; no usable policy at handover causes damping mid-motion | Prepare and exercise the solver while supported in idle; require an appropriate fresh policy before the dynamic push begins |
| P1 | Physical starting-state acceptance | Joint proximity, a dwell and estimator readiness do not prove the intended knee/wheel support configuration | Establish a validated supported-idle envelope using joint geometry, attitude, motion and available contact/load evidence |
| P1 | Measured stand-up handover | The exported playback hands over at a fixed 0.6965 s, not measured knee unloading | Validate the real motion envelope and add state/event acceptance and bounded timing/abort behavior |
| P1 | Supported sit-down completion | Height/tilt trigger the final interpolation; joint position/speed trigger idle without directly confirming support | Validate touchdown/support and settling before declaring idle and allowing another stand-up |
| P1 | Actual robot model and servos | Playback, friction, armature and PD settings were selected for this MuJoCo model | Measure or identify the real values and validate the resulting commands within an approved hardware envelope |
| P2 | Hardware acceptance envelope | No physical stand-up, balance, sit-down, gamepad or repeated-cycle acceptance exists | Complete staged physical validation and record limits, failures and recovery procedures before expanding beyond flat ground |


## Code Locations

- [Hardware selection and simulation-only sensors](../../../description/urdf/robots/upkie/upkie.ros2_control.xacro#L78): no real actuator plugin is selected in the current non-simulation, non-fake path.
- [Controller deactivation](../../../src/controller/controller.cpp#L552): zeros joint commands while deliberately retaining servo gains.
- [Command and gain writes](../../../src/controller/controller.cpp#L2159): the actuator backend must implement the position/velocity/effort plus kp/kd contract.
- [Planned idle and stand-up](../../../src/controller/controller.cpp#L982): joint-pose and speed checks, estimator readiness, solver request and playback.
- [Timed handover](../../../src/controller/controller.cpp#L1052): playback switches at `handoverTime` if the current policy passes its existing usability check.
- [Crouch-to-kneel and idle guards](../../../src/controller/controller.cpp#L1202): estimated height/tilt, interpolation, joint error and speed checks.
- [Trunk orientation validation](../../../src/controller/controller.cpp#L772): missing/nonfinite quaternion rejection and IMU mounting correction, but no acquisition-age check here.
- [Observation construction](../../../src/controller/controller.cpp#L1415): estimator validity and hardware-state handling.
- [Offline stand-up generator](../../../config/upkie/plan_stand_up.py): model-specific MuJoCo rollout and trajectory export.
- [Exported trajectory](../../../config/upkie/stand_up_trajectory.yaml): feedforward samples, playback gains and the 0.6965 s handover.
- [Current parameters](../../../config/upkie/ros2_controllers.yaml): operator settings, model compensation, servo gains and transition thresholds.

Line numbers identify the checkpoint and should be refreshed when these functions change.


## Critical Stop-Semantics Audit

Do not assume that controller deactivation, publishing zero velocity, damping, and a motor stop are equivalent.
The current deactivation path writes zero position/velocity/effort while leaving gain interfaces unchanged.
If a real backend continues to execute those targets with nonzero position gain, it could move joints toward zero rather than release them.
This is a backend-dependent hazard to resolve, not a demonstrated physical fault in an existing driver.
Document what happens on controller deactivation, lifecycle error, process termination, stale commands, lost actuator feedback and a hardware stop.
The chosen behavior must be checked on a supported test setup before unrestricted balance trials.
A software Fallen latch cannot execute if the host process or control thread has stopped.
L2 release and Circle currently stop commanded motion while retaining balance; neither should be presented as an independent hardware emergency stop.
The safety layer must remain authoritative over FSM, MPC, GUI and joystick requests.


## Transition Hardening

For idle -> stand-up, verify the calibrated starting joint branch, trunk attitude, sufficiently low body motion, supported pose, fresh sensors, actuator readiness and estimator readiness.
Warm up the MPC before push-off; do not rely on cold solver construction finishing within the fixed playback interval on the onboard computer.
During stand-up, compare measured attitude, angular rate, joint tracking and available load/contact evidence with an accepted envelope.
A timed open-loop phase is not inherently invalid, but the simulated handover time alone is not sufficient hardware acceptance.
Any measured handover logic must tolerate sensor noise, use bounded waiting, and define an appropriate abort action rather than wait indefinitely.
For MPC -> sit-down, verify braking and settling, not only a single low-speed sample.
Before disabling MPC for final kneeling, check the crouch state and motion are within a validated transition envelope.
Before declaring idle, require evidence of supported rest over a dwell, using available sensors or validated conservative proxies rather than assuming joint position implies contact.
Validate this first on flat, supported ground; slopes, steps and unequal wheel heights are separate acceptance cases.
Keep automatic fall recovery out of scope unless explicitly requested.


## Model and Timing Checks

Confirm actual joint zero offsets, direction, units, gear ratios, torque reporting and command saturation.
Confirm whether real servos use absolute gains or gain scales and map the existing kp/kd values accordingly.
Verify IMU mounting, orientation availability, gravity convention, gyro units and sample timing without using simulator ground truth.
The rolling estimator currently relies on its configured contact model; do not treat scheduled contact as proof of actual wheel support or no slip.
Validate physical wheel radius, knee-support geometry, mass distribution, joint friction and reflected inertia.
The current compensation arrays are explicitly matched to MJCF values, not identified from a physical Upkie.
Include actuator voltage, temperature and achievable torque/speed effects when defining the permitted test envelope.
Confirm how final PD-plus-feedforward torque is bounded by the hardware, including non-locomotion phases.
Measure control-loop jitter, solver deadlines, observation age, command age and first-policy latency on the intended computer.
Do not infer a hard real-time guarantee or an onboard 200 Hz capability from the VM simulation results.


## Validation Order

| Order | Milestone | Acceptance gate before proceeding |
| --- | --- | --- |
| 1 | Read-only hardware integration | Joint signs/offsets, IMU frames/units, feedback timestamps and fault reporting match independent checks |
| 2 | Actuator safety and lifecycle | Stop/watchdog/release and full command limits behave as documented on a suitable supported setup, including host failure |
| 3 | Supported low-energy servo checks | Position/velocity/feedforward/gain mapping is verified without unrestricted balancing |
| 4 | Estimator and MPC readiness | Fresh consistent estimates, bounded startup latency, valid pre-motion policies and measured timing margins |
| 5 | Restricted flat-ground stand-up and balance | Actual starting-state and handover envelopes are demonstrated, with an independently available stop and a suitable restraint setup |
| 6 | Restricted flat-ground sit-down and repeat cycles | Supported kneeling is confirmed; repeated requests, cancellation/fault paths and stale-policy rejection behave correctly |
| 7 | Hardware disturbance and operating envelope | Only after earlier gates: expand speeds, heights and disturbances within the reviewed setup; terrain is a separate milestone |

Use qualified hardware supervision and an independently accessible stop for physical trials.
Set numerical acceptance thresholds from the actual sensors, actuator limits and measured margins rather than copying simulation gains or inventing untested hardware thresholds.


## Evidence to Record

Record repository revisions, hardware revision, calibration, firmware, backend configuration, parameters, safety limits and test setup.
Log FSM phase and request acceptance/refusal alongside time-synchronized joint states, IMU samples, estimator state/health and command outputs.
Capture q/qd targets, feedforward, gains, limited actuator commands, saturation, actuator errors, voltage and temperatures where available.
Record policy timestamps, solver duration, deadline misses, feedback/command ages and the exact handover/settling criteria.
Keep independent video and a per-trial pass/fail report, including aborted trials and the reason for each abort.
Report validated hardware conditions separately from simulation conditions.
Do not mark this handoff complete until both normal operation and the specified stop/fault scenarios have hardware evidence.


## Upstream References

Upkie's [motion-control setup](https://upkie.github.io/upkie/motion-control-software.html) describes servo-limit configuration, the pi3hat spine and the operator emergency-stop path.
Its [developer notes](https://upkie.github.io/upkie/dev-notes.html#inter-process-communication) explain the agent/spine split and the spine's Stop, Reset, Idle, Act and Shutdown states.
Use that established actuator interface as the integration reference rather than treating the simulation ros2_control backend as a real driver.
The upstream spine FSM and this controller's posture FSM serve different layers and must not be conflated.
