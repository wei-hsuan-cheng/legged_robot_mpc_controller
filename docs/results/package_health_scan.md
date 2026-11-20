# Package Health Scan

Checkpoint: 2026-11-20, `jazzy`, after `e2ad99e`.
This cleanup preserves the complete G1, H2, and Upkie controller/estimator YAML, robot descriptions, physics, and solver tuning.
No locomotion simulation was started; manual robot testing is left to the user.


## Fixed Findings

| Finding | Change | Validation |
| --- | --- | --- |
| The active-to-full joint map reserved capacity but indexed a size-zero vector | Allocate the required number of entries and use checked joint-name lookup | The new native test failed on the old G1 map size; rebuilt checks cover G1/H2/Upkie |
| Contact-frame indices had the same `reserve()`/indexing error | Allocate the two entries before indexing | Native checks compare both returned indices with Pinocchio frame IDs |
| Duplicate/unknown fixed joints could corrupt active-joint counts, including unsigned subtraction | Reject invalid fixed-joint lists before subtraction | Native checks cover unknown, duplicate, and all-fixed configurations |
| Two legacy headers had no include sites or consumers | Delete `ContactWrenchMapper.h` and `ImpactProximityFactorBase.h` | Package-wide local-include and header-reachability checks |
| An unused gait-list field and avoidable gait-map default construction remained | Remove the unused field and move the map directly in the initializer list | Source/static checks and existing gait regressions |
| The local `tl-expected` shim reported success without locating the actual dependency | Remove `cmake/` and use normal package discovery | CMake resolved `/usr/share/cmake/tl-expected` and the actual installed header |
| Direct dependencies were only provided transitively | Declare Boost, Eigen, `builtin_interfaces`, `tf2`, `controller_manager_msgs`, and Tkinter | Manifest checks and Release configure/build |
| Resource installation could copy source caches | Exclude Python/pytest caches and `.DS_Store` from installed resources | Source contract checks and fresh installation audit |

The vector bugs were pre-existing undefined behavior, not merely style warnings.
The joint-map regression checks vector sizes, joint-name/index correspondence, active/full angle round trips, and preservation of fixed-joint defaults.
The fixes do not intentionally change valid joint ordering or the optimization model.
They do not establish that the earlier G1 torque-window timing caveat is resolved.
The corrected joint map is in [ModelSettings.cpp](../../src/mpc/common/model/ModelSettings.cpp#L90), and the contact helper is in [DynamicsHelperFunctions.h](../../include/legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h#L211).
The new native checks are in [model_settings_test.cpp](../../tests/model_settings_test.cpp), and the package-wide checks are in [test_package_health.py](../../tests/test_package_health.py).


## Removed and Retained

Removed generated `__pycache__` and `.pytest_cache` directories from the code tree, and ignored future test caches and accidental source-tree `auto_generated/` output.
All remaining source modules have build coverage and all remaining public headers have local include sites.
The terrain/stair target modes and their scripts/configuration remain because they still have implemented consumers.
User tuning archives, demo recordings, `chore/`, and agent settings were not deleted.
No model meshes, textures, active scenes, or robot configuration folders were removed.


## Validation

- Release rebuild: passed in `ros_jazzy`, with two jobs and `BUILD_TESTING=ON` (9 minutes 16 seconds for the main build).
- Native regressions: all 8 passed against the installed build, including the new three-robot model-settings regression.
- Python regression suites: 136 passed, 10 optional checks skipped.
- Standalone parameter-contract script: passed.
- Cppcheck warning/performance/portability scan: passed with no remaining diagnostics in the selected checks.
- Python undefined-name and syntax-error lint (`F821,F822,F823,E9`): passed.
- All Python source syntax, XML resources, and YAML duplicate-key checks: passed.
- Local Markdown file links: passed.
- Full ament lint: Cppcheck, CMake lint, and XML lint passed; repository-wide Flake8, PEP257, and Uncrustify failed on existing style/docstring debt.
- The new Python health test passes Flake8 and PEP257, and the new native model-settings test passes Uncrustify.
- Fresh installed-file audit: passed; deleted headers and source caches are absent, and the controller has no missing linked libraries.
- Installed G1/H2/Upkie MuJoCo scene loading: passed through the simulator's C++ SDK, including meshes/textures, without stepping simulation.
- Installed G1/H2/Upkie ros2_control xacro expansion and the RViz environment xacro: passed.
- Installed launch-argument discovery: passed without starting nodes.

The optional Python skips are retained from existing tests; these are not claimed as passing MuJoCo Python checks.
These static/native checks are not a new locomotion, estimator, push-recovery, or real-time performance validation.
The installed MuJoCo scenes compiled to 29 actuators for G1, 31 for H2, and 6 for Upkie.
The complete installed package was refreshed and its colcon environment hooks regenerated, so an independent sourced shell resolves the new libraries correctly.
All owned build/check processes have finished, no robot processes are running, and the new installation is ready for manual testing.


## Remaining Caveats

The package manifest still has `TODO: License declaration` and no package-wide license decision was made during this cleanup.
Existing upstream/source license notices and model license files were preserved.
Compiler output also retains existing unused-parameter, signedness, and initialization-order warnings; this is not a warning-free or full-style-lint-clean claim.
The earlier G1 overrun/tail-latency finding remains documented in [the layout regression report](wb_id_layout_regression.md).
Manual simulation testing should start from the unchanged robot commands in the README after the rebuild is complete.
