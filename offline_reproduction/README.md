# Ranger Mini V3 Offline Navigation Reproduction

This directory provides a Python-only reproduction workflow for the Ranger Mini V3 wheeled mobile robot. It does not require ROS 2, a simulator, or a physical robot. The scripts load the real project occupancy map and point cloud, then generate an A* path figure, a point-cloud figure, a system pipeline diagram, and a simplified DWB-style navigation animation.

Every generated figure and metric is labeled **OFFLINE REPRODUCTION**. The outputs reproduce the retained method and data; they are not real-robot footage and do not claim to duplicate the Nav2 C++ plugins line by line.

## Data Requirements

The repository contains `map/scans5f2.yaml` and `nav2_params/nav2_params.yaml`, but the large raw PGM and PCD files are private and are not tracked by Git.

The `--source-zip` archive must preserve these original internal entries:

- `wzb/map/scans5f2.pgm`
- `wzb/map/gf/scans0120gf.pcd`

Here, `wzb/` is only the internal directory name retained by the private data archive. It is not expected to appear as a folder in this GitHub repository. The scripts read directly from the archive and do not extract over or modify the source data.

## Generate All Outputs

```bash
python -m pip install -r offline_reproduction/requirements.txt
python offline_reproduction/generate_all.py \
  --source-zip /path/to/private-project-data.zip \
  --data-root .
```

Windows PowerShell example:

```powershell
python -m pip install -r .\offline_reproduction\requirements.txt
python .\offline_reproduction\generate_all.py `
  --source-zip "C:\path\to\private-project-data.zip" `
  --data-root .
```

The tools can also be run separately:

```bash
python offline_reproduction/run_astar_demo.py --source-zip /path/to/private-project-data.zip
python offline_reproduction/visualize_pointcloud.py --source-zip /path/to/private-project-data.zip
python offline_reproduction/draw_pipeline.py
python offline_reproduction/run_navigation_demo.py --source-zip /path/to/private-project-data.zip
python offline_reproduction/validate_outputs.py --repo-root .
```

Instead of passing `--source-zip`, the archive path can be supplied through the `WZB_SOURCE_ZIP` environment variable for backward compatibility.

## Implementation

- `map_loader.py`: loads ROS trinary occupancy maps and converts between pixel and world coordinates.
- `inflation.py`: builds a conservative feasible region from the real footprint and local-costmap inflation radius.
- `astar.py`: implements 8-connected A* with an octile heuristic and diagonal corner-cut prevention.
- `dwb.py`: reads velocity, acceleration, sampling, horizon, and critic parameters, then scores trajectories using a planar nonholonomic model suitable for the wheeled base.
- `collision.py`: checks the complete rotating rectangular footprint instead of treating the robot as a point.
- `visualize_pointcloud.py`: streams binary PCD data directly from the ZIP archive and applies 0.20 m voxel downsampling.
- `draw_pipeline.py`: generates the mapping, localization, planning, and control architecture diagram.
- `validate_outputs.py`: checks artifacts, path collisions, navigation completion, and accidental inclusion of large raw data.

The start and goal are chosen deterministically from the largest safe connected region for offline testing. They are not presented as recorded field-test waypoints. Unknown occupancy cells are treated as non-traversable.

The simplified DWB scoring keeps the configured critic scales and converts path and goal distances from meters to costmap-cell units. Near the final goal, the implementation disables forward-scoring points that would extend beyond the goal to prevent an artificial stop caused by `forward_point_distance`.

## Outputs

- `assets/navigation/01_astar_global_path.png`
- `assets/navigation/02_slam_pointcloud.png`
- `assets/navigation/03_navigation_pipeline.png`
- `assets/navigation/04_offline_navigation_reproduction.gif`
- `results/offline_reproduction/astar_metrics.json`
- `results/offline_reproduction/pointcloud_metrics.json`
- `results/offline_reproduction/navigation_metrics.json`
- `results/offline_reproduction/parameters_used.json`

The original DWB configuration declares `20 × 5 × 10` samples for `(vx, vy, vtheta)`. The planar wheeled-base model has no lateral velocity degree of freedom, so the reproduction evaluates `20 × 10 = 200` `(v, ω)` trajectories per cycle while retaining `vy_samples: 5` in the parameter record.

All parameter values, units, YAML keys, and offline overrides are recorded in `parameters_used.json`. The retained Nav2 configuration contains an 8.0 m global inflation radius and a 0.40 m local inflation radius. The reproduction uses the local 0.40 m value and records the discrepancy for review.
