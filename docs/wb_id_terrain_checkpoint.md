# WB-ID Terrain Trial Checkpoint

**Archived proposal, not the current launch workflow.**
The scene-extraction hook and `terrainFromScene` argument were removed from the former `g1.launch.py` at the user's request; commands using that argument below describe the earlier checkpoint only.
The current entry point is [`robot.launch.py`](../launch/robot.launch.py), which does not expose that hook.
The standalone exporter remains unused by launch, and the mapped-planner extensions remain separate from ordinary `base_twist` walking.
For the tested ramp gains and current status, see [Codex.md](../Codex.md).

This is an initial geometry/reference checkpoint for the current ramp and box-tile field, not a claim of validated terrain walking.
The robot's model, actuator limits, no-slip equalities, contact rectangle and estimator implementation are unchanged.
The scene exporter is simulation-only ground truth and is not a real-world terrain perception system.


## Launch and Command

Build the updated package and source the workspace before launching.
Start with GT feedback to separate controller/terrain issues from the linear KF's flat-ground height assumption:

```bash
ros2 launch legged_robot_mpc_controller g1.launch.py \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller \
  mujocoModelFile:=scene_ramp.xml \
  terrainFromScene:=true \
  floatingBaseSource:=ground_truth_state \
  mpcFreq:=100 mrtFreq:=1000 \
  baseCommandGui:=false
```

After MPC takes over, select the terrain mode and start slowly:

```bash
ros2 topic pub --once /humanoid/target_mode std_msgs/msg/String "{data: terrain_walk}"
ros2 topic pub -r 25 /humanoid/walking_velocity_command \
  ocs2_msgs/msg/WalkingVelocityCommand \
  "{linear_velocity_x: 0.10, linear_velocity_y: 0.0, desired_pelvis_height: 0.795, angular_velocity_z: 0.0}"
```

The x/y/yaw fields are normalized commands; the current maximum x speed is 1 m/s, so x=0.10 requests 0.10 m/s.
Stop by replacing the velocity publication with zero x/y/yaw while retaining the same commanded height.
For the uneven field, include `terrains/uneven_terrain_10x10.xml` in your scene and launch that scene; the exporter follows the actual include tree, including edits to `scene.xml`.
No provided scene file was changed by this checkpoint.
`terrainFromScene` supplies geometry but does not switch modes automatically; ordinary `base_twist` remains the default until `terrain_walk` is selected.
For headless testing, additionally pass `rviz:=false mujoco_headless:=true`.


## Geometry and References

[terrain_scene_model.py](../launch/terrain_scene_model.py) exports static box tops and the common horizontal floor from the selected scene to a temporary YAML file.
At this archived checkpoint the former `g1.launch.py` used it for WB-ID MuJoCo launches when `terrainFromScene:=true`, retaining the planner settings from `terrainWalkingFile`; the current launch no longer does so.
Use `terrainFromScene:=false` to retain a manually configured terrain file or the legacy staircase geometry.
The exporter supports horizontal tiles, box tops, the upward faces of inline-vertex convex meshes such as the wedge ramp (`ramp_params.yaml`), nested static-body transforms, quaternion/Euler poses and included compiler angle units.
Dynamic bodies are excluded; unsupported static geometry, compound slopes with nonrectangular XY projections, tilted infinite planes and inconsistent floor levels fail explicitly.
Meshes, heightfields, moving platforms, riser collision avoidance and perception are not supplied by this exporter.

Each region stores a center $(x_0,y_0,z_0)$, rectangular XY footprint and upward unit normal $n$:

$$
h(x,y)=z_0-\frac{n_x(x-x_0)+n_y(y-y_0)}{n_z}.
$$

The height query takes the highest containing region or the background floor, so a tile below the existing floor does not create an artificial hole.
Footholds are projected into margin-shrunk rectangles, checked against the step-height limit, and for scene-derived terrain also checked against `max_step_reach: 0.22` m.
When no candidate passes those checks, the planner retains the previous foothold instead of bypassing the limits.
The old staircase centerline and step-up heuristics are retained only for legacy staircase geometry, not applied to a ramp or tile field.

The planner supplies one consistent normal to the WB-ID stance tilt equality, shared foot cost and friction cone.
The contact-moment inequalities already rotate wrenches into the foot frame and are unchanged.
Normals are blended during swing and held at the planned contact surface in stance.
The stance/swing CppAD constraint parameter count changes from 11 to 14, so their generated model names have a new `_v2` suffix to avoid loading stale libraries.
No manual cache deletion is needed; missing `_v2` libraries are generated on launch.

For anticipated support transfer over a swing, $s$ is normalized swing time and $b(s)=3s^2-2s^3$:

$$
p_{c,\mathrm{anticip}}=(1-b)p_{c,\mathrm{lift}}+bp_{c,\mathrm{land}},\qquad
\dot b=\frac{6s(1-s)}{T_{\mathrm{swing}}}.
$$

The base reference uses the mean anticipated support height plus the requested height above support, and the vertical-velocity target uses the derivative of that same support trajectory.
This anticipated path is a base-reference device, not the actual swing-foot clearance spline or a physical double-support claim.
References are sampled at 0.05 s spacing; interpolation between samples is approximate.
The separate vertical swing spline retains the planned lift-off and touchdown heights and the configured clearance.


## Initial Tuning

These are unvalidated terrain-trial settings in the WB-ID section of [ros2_controllers.yaml](../config/g1/ros2_controllers.yaml), not changes to the centroidal configuration.

| Setting | Previous | Terrain trial |
| --- | --- | --- |
| Running/terminal base height weight | 100 | 50 |
| Running/terminal vertical velocity weight | 10 | 5 |
| Foot tilt weights | 1000, 1000 | 300, 300 |
| Foot horizontal velocity weights | 10, 10 | 5, 5 |
| Extra `legTorqueCost` | enabled | disabled for ablation |
| Torso free tilt | 5 degrees | 10 degrees |
| Swing clearance | 0.08 m | 0.10 m |
| Maximum requested height above support | 0.70 m | 0.80 m |

The last cap avoids forcing the default 0.795 m standing command to crouch abruptly when entering terrain adaptation.
The input torque objective, joint torque/velocity penalties, pelvis pitch/roll weights, torso angular damping and contact stabilization gains remain unchanged.
Do not interpret softer posture costs as collision protection or a balance guarantee.


## Height Flags and Estimation

`useTerrainHeightEstimate` controls the ordinary walking fallback, not the explicit terrain-plan branch.
`false` uses world ground height zero; `true` estimates it from FK of the currently scheduled support feet and adjusts the base/swing references.
It does not supply terrain ahead or surface normals.
`maxTerrainHeightStep: 0.002` limits that height change to 2 mm per MPC update, not per step or per second, and is unused when the flag is false.

$$
h_{k+1}=\operatorname{clamp}(h_{\mathrm{FK}},h_k-0.002,h_k+0.002).
$$

The explicit terrain-plan branch uses mapped support heights and does not pass through this limiter.
Both flags remain unchanged in this checkpoint.
The current linear KF observes contact heights as zero, so its good flat-ground behavior does not establish correct world height on ramps or stairs; see [linear_kf_floating_base_estimator.cpp](../src/humanoid_state_estimation/linear_kf_floating_base_estimator.cpp).
The InEKF's kinematic/blended ground-height options can also impose a flat-ground assumption; merely selecting a different estimator is not a verified fix.
First isolate this checkpoint with `ground_truth_state`, then separately validate a terrain-compatible estimator height path.
The map and state must share a registered world/odometry frame; proprioceptive planar/yaw drift relative to a fixed map is not solved here.


## Verification

`tests/terrain_reference_test.py` checks scene extraction, the empty flat-scene terrain model, all 440 tile tops, ramp normals/heights against MuJoCo compilation and ray casts, unsupported geometry, support-height derivatives and normal/cache wiring.
These tests do not run the C++ planner in closed loop or establish ramp/uneven-field stability.
Build status and trial results are recorded in [Codex.md](../Codex.md).
