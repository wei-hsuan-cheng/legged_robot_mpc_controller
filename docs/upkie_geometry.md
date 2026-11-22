# Upkie Geometry File

[`config/upkie/geometry.yaml`](../config/upkie/geometry.yaml) is the single source of Upkie's contact geometry.
Every consumer reads it by path, the same way the stand-up plan and the posture table are referenced; none keeps its own copy.

| Value | Meaning |
| --- | --- |
| `wheel.radius` | Tire radius, 0.08 m |
| `wheel.half_width` | Half the tire width along the axle, 0.022 m |
| `wheel.center_offset` | Tire centre in the wheel link frame, $[0,0,-0.0312]$ m |
| `wheel.axle_axis` | Axle direction in the wheel link frame, $[0,0,1]$ |
| `knee_housings` | The two motor housings at each knee as one cylinder along the femur frame's $z$ axis: frame, centre $[0,-0.17015,0.0700987]$ m, radius 0.05 m, half width 0.02685 m |

## Consumers

| Consumer | How it reads the file | What it takes |
| --- | --- | --- |
| WB-ID MPC controller | `paths.geometryFile` | `ocs2.constraints.wheelContact.radius`/`axleAxis` and `ocs2.model.contactGeometry.frameTranslation` (the tire-centre frames), plus the checks below |
| Wheel-aware linear KF (`legged_robot_state_estimator`) | `geometryFile` | `wheel.radius`, `wheel.centerOffset`, `wheel.axleAxis` |
| Stand-up TO (`config/upkie/stand_up_to/plan.py`, `--geometry`) | `config/upkie/upkie_geometry.py` | Wheel and knee-housing contacts; the model generator exports only frame origins and rotations, and the plan applies the offsets |
| Posture table (`config/upkie/compute_posture_table.py`) | `config/upkie/upkie_geometry.py` | Tire radius |
| `tests/tuning/analyze_upkie_flight.py` | reads the YAML | Tire radius |

Both runtime consumers refuse the same values in the parameter file next to the file (`... comes from paths.geometryFile` / `geometryFile; remove it from the parameter file`), so a second copy cannot silently disagree.
Without the path parameter, both still read their inline parameters, as the humanoids do.

The generated files record the geometry they were made for: the stand-up plan's `wheel_radius`, `wheel_half_width` and `knee_contacts`, and the posture table's `wheel_radius`.
The controller compares them with the geometry file at configure and refuses a stale file with `... was generated for other geometry than paths.geometryFile ...; regenerate it`.
The offline generators check the file against the simulated robot: the MJCF collision tire (`upkie.xml` `tire_collision`) must have the same radius and width within 1 µm.
The MJCF prism's circumradius is 0.0800006 m because of rounded vertex coordinates; the 0.08 m in the file is the value the MPC and the KF have always used.

## Changing the Geometry

1. Change the robot description (`description/`: URDF, MJCF and meshes) and `config/upkie/geometry.yaml` together.
2. In `ros_jazzy`, with `S=/root/src/mpc/legged_robot_mpc_controller/config/upkie/regenerate.sh`:

```bash
bash $S setup              # rebuilds the stand-up model from the URDF and checks the MJCF tire against the file
bash $S posture            # posture table for the new tire, then check-posture and install-posture
bash $S plan 1.5           # stand-up for the new geometry, then check 1.5 and install 1.5
```

3. Restart the launch; the controller loads the new values and accepts the regenerated files.
Heights in `ros2_controllers.yaml` (`defaultBaseHeight`, `minBaseHeight`, `maxBaseHeight`, `bringUp.sitDownHeight`) and the posture grid depend on the wheel too; they are tuning values, not geometry, and stay in their own files.
The wheel radius and axle are runtime parameters of the CppAD wheel-contact library, so changing them needs no code regeneration; a different tire centre moves the contact frames, so set `codegen.recompileLibraries` once.
