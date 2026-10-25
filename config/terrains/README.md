# Shared Terrain Configuration

Terrain configuration is shared infrastructure for G1 and H2, not part of either robot's configuration directory.
The existing YAML loaders and launch argument names are unchanged.

```text
config/terrains/
  stairs_params.yaml
  ramp_params.yaml
  stair_climbing/
    g1.yaml
    h2.yaml
  terrain_walking/
    g1.yaml
    h2.yaml
```

[`stairs_params.yaml`](stairs_params.yaml) defines the staircase geometry and physical properties consumed by the MuJoCo and RViz staircase xacros.
[`ramp_params.yaml`](ramp_params.yaml) does the same for the wedge ramp: entry-edge position, yaw, horizontal length, incline angle and width.
The movable boxes are objects, not terrain: their parameters are in [`../objects/boxes_params.yaml`](../objects/boxes_params.yaml).
The planner YAML files contain matching staircase geometry plus gait/reference settings; keep that geometry synchronized when editing the environment.
Each launch automatically selects its robot's variant, and `stairClimbingFile` / `terrainWalkingFile` can still select another file explicitly.


## Robot-Specific Values

The files cannot be replaced by one identical planner configuration without changing the current behavior because the robots have different dimensions.
These five values are the only numerical differences between the corresponding G1 and H2 variants:

| Mode | Parameter | G1 | H2 |
| --- | --- | --- | --- |
| `stair_climbing` | `footholds.lateral_offset` | 0.115 m | 0.14 m |
| `stair_climbing` | `base.height_above_support` | 0.70 m | 0.90 m |
| `terrain_walk` | `hip_lateral_offset` | 0.115 m | 0.14 m |
| `terrain_walk` | `foot_margin_x` | 0.10 m | 0.12 m |
| `terrain_walk` | `max_base_height_above_support` | 0.80 m | 1.03 m |

All geometry, planner gains and mode timings were preserved during this path migration.
These optional mapped/fixed-sequence modes are not enabled by ordinary `base_twist`, and H2 terrain performance has not been validated.


## Path Migration

The former `config/g1/terrain/` and `config/h2/terrain/` directories have been removed.
Use `config/terrains/stair_climbing/<robot>.yaml` instead of the former `terrain/stair_climbing/stair_climbing.yaml` path, and `config/terrains/terrain_walking/<robot>.yaml` instead of `terrain/terrain_walking/terrain_walking.yaml`.
The package already installs the full `config/` tree, so no CMake installation change or new C++ loading mechanism is needed.
