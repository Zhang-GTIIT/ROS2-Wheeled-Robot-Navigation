from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DATA_PROJECT_ROOT = Path(
    os.environ.get("OFFLINE_REPRO_PROJECT_ROOT", str(REPO_ROOT))
).resolve()

MAP_YAML_RELATIVE = Path("map/scans5f2.yaml")
NAV2_PARAMS_RELATIVE = Path("nav2_params/nav2_params.yaml")
MAP_ZIP_MEMBER = "wzb/map/scans5f2.pgm"
POINTCLOUD_ZIP_MEMBER = "wzb/map/gf/scans0120gf.pcd"

POINTCLOUD_VOXEL_SIZE_M = 0.20
POINTCLOUD_RENDER_LIMIT = 140_000
ENDPOINT_SELECTION_CLEARANCE_M = 0.80


def find_source_zip(explicit: str | Path | None = None) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env_path = os.environ.get("WZB_SOURCE_ZIP")
    if env_path:
        candidates.append(Path(env_path))
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    tried = "\n".join(f"  - {path}" for path in candidates)
    raise FileNotFoundError(
        "The original wzb.zip is required to regenerate the real-map figures. "
        "Pass --source-zip or set WZB_SOURCE_ZIP. Tried:\n" + tried
    )


def load_project_parameters(data_root: Path | None = None) -> dict[str, Any]:
    root = (data_root or DATA_PROJECT_ROOT).resolve()
    nav_path = root / NAV2_PARAMS_RELATIVE
    with nav_path.open("r", encoding="utf-8") as stream:
        nav = yaml.safe_load(stream)

    controller = nav["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    local_costmap = nav["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    planner = nav["planner_server"]["ros__parameters"]["GridBased"]

    footprint_raw = local_costmap["footprint"]
    footprint = yaml.safe_load(footprint_raw) if isinstance(footprint_raw, str) else footprint_raw

    return {
        "data_root": root,
        "nav2_path": nav_path,
        "map_yaml_path": root / MAP_YAML_RELATIVE,
        "controller_frequency": float(controller["controller_frequency"]),
        "goal_tolerance": float(controller["general_goal_checker"]["xy_goal_tolerance"]),
        "footprint": [[float(x), float(y)] for x, y in footprint],
        "costmap_resolution": float(local_costmap["resolution"]),
        "inflation_radius": float(local_costmap["inflation_layer"]["inflation_radius"]),
        "global_inflation_radius_detected": float(
            global_costmap["inflation_layer"]["inflation_radius"]
        ),
        "max_vel_x": float(follow["max_vel_x"]),
        "min_vel_x": float(follow["min_vel_x"]),
        "max_vel_theta": float(follow["max_vel_theta"]),
        "min_speed_xy": float(follow["min_speed_xy"]),
        "acc_lim_x": float(follow["acc_lim_x"]),
        "acc_lim_theta": float(follow["acc_lim_theta"]),
        "decel_lim_x": float(follow["decel_lim_x"]),
        "decel_lim_theta": float(follow["decel_lim_theta"]),
        "vx_samples": int(follow["vx_samples"]),
        "vy_samples": int(follow["vy_samples"]),
        "vtheta_samples": int(follow["vtheta_samples"]),
        "sim_time": float(follow["sim_time"]),
        "forward_point_distance": float(follow["PathAlign.forward_point_distance"]),
        "critics": {
            "BaseObstacle": float(follow["BaseObstacle.scale"]),
            "PathAlign": float(follow["PathAlign.scale"]),
            "PathDist": float(follow["PathDist.scale"]),
            "GoalAlign": float(follow["GoalAlign.scale"]),
            "GoalDist": float(follow["GoalDist.scale"]),
            "RotateToGoal": float(follow["RotateToGoal.scale"]),
        },
        "planner_use_astar": bool(planner["use_astar"]),
        "planner_allow_unknown": bool(planner["allow_unknown"]),
    }


def parameter_provenance(parameters: dict[str, Any], map_metadata: dict[str, Any]) -> dict[str, Any]:
    nav_source = NAV2_PARAMS_RELATIVE.as_posix()
    map_source = MAP_YAML_RELATIVE.as_posix()

    def item(value: Any, unit: str, source: str, key: str, **extra: Any) -> dict[str, Any]:
        record = {"value": value, "unit": unit, "source": source, "key": key}
        record.update(extra)
        return record

    result: dict[str, Any] = {
        "map_resolution": item(map_metadata["resolution"], "m/pixel", map_source, "resolution"),
        "map_origin": item(map_metadata["origin"], "m, m, rad", map_source, "origin"),
        "occupied_thresh": item(map_metadata["occupied_thresh"], "probability", map_source, "occupied_thresh"),
        "free_thresh": item(map_metadata["free_thresh"], "probability", map_source, "free_thresh"),
        "negate": item(map_metadata["negate"], "boolean flag", map_source, "negate"),
        "robot_footprint": item(
            parameters["footprint"],
            "m",
            nav_source,
            "local_costmap.local_costmap.ros__parameters.footprint",
        ),
        "inflation_radius": item(
            parameters["inflation_radius"],
            "m",
            nav_source,
            "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius",
        ),
        "global_inflation_radius_detected": item(
            parameters["global_inflation_radius_detected"],
            "m",
            nav_source,
            "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius",
            used_in_offline_reproduction=False,
            note="The retained file contains 8.0 m here; the offline reproduction uses the enabled 0.40 m local-costmap value and records this discrepancy for manual review.",
        ),
        "costmap_resolution": item(
            parameters["costmap_resolution"],
            "m/pixel",
            nav_source,
            "local_costmap.local_costmap.ros__parameters.resolution",
        ),
        "max_vel_x": item(parameters["max_vel_x"], "m/s", nav_source, "controller_server.ros__parameters.FollowPath.max_vel_x"),
        "min_vel_x": item(parameters["min_vel_x"], "m/s", nav_source, "controller_server.ros__parameters.FollowPath.min_vel_x"),
        "max_vel_theta": item(parameters["max_vel_theta"], "rad/s", nav_source, "controller_server.ros__parameters.FollowPath.max_vel_theta"),
        "acc_lim_x": item(parameters["acc_lim_x"], "m/s^2", nav_source, "controller_server.ros__parameters.FollowPath.acc_lim_x"),
        "acc_lim_theta": item(parameters["acc_lim_theta"], "rad/s^2", nav_source, "controller_server.ros__parameters.FollowPath.acc_lim_theta"),
        "decel_lim_x": item(parameters["decel_lim_x"], "m/s^2", nav_source, "controller_server.ros__parameters.FollowPath.decel_lim_x"),
        "decel_lim_theta": item(parameters["decel_lim_theta"], "rad/s^2", nav_source, "controller_server.ros__parameters.FollowPath.decel_lim_theta"),
        "controller_frequency": item(parameters["controller_frequency"], "Hz", nav_source, "controller_server.ros__parameters.controller_frequency"),
        "vx_samples": item(parameters["vx_samples"], "samples", nav_source, "controller_server.ros__parameters.FollowPath.vx_samples"),
        "vy_samples": item(
            parameters["vy_samples"],
            "samples",
            nav_source,
            "controller_server.ros__parameters.FollowPath.vy_samples",
            used_in_offline_reproduction=False,
            note="Retained for provenance. The Python unicycle model has no lateral velocity degree of freedom.",
        ),
        "vtheta_samples": item(parameters["vtheta_samples"], "samples", nav_source, "controller_server.ros__parameters.FollowPath.vtheta_samples"),
        "simulation_horizon": item(parameters["sim_time"], "s", nav_source, "controller_server.ros__parameters.FollowPath.sim_time"),
        "forward_point_distance": item(parameters["forward_point_distance"], "m", nav_source, "controller_server.ros__parameters.FollowPath.PathAlign.forward_point_distance"),
        "goal_tolerance": item(parameters["goal_tolerance"], "m", nav_source, "controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance"),
        "planner_use_astar": item(parameters["planner_use_astar"], "boolean", nav_source, "planner_server.ros__parameters.GridBased.use_astar"),
        "planner_allow_unknown": item(
            parameters["planner_allow_unknown"],
            "boolean",
            nav_source,
            "planner_server.ros__parameters.GridBased.allow_unknown",
            used_in_offline_reproduction=False,
            offline_value=False,
            note="Unknown cells are treated as non-traversable in the conservative offline reproduction.",
        ),
        "pointcloud_voxel_size": item(
            POINTCLOUD_VOXEL_SIZE_M,
            "m",
            "offline_reproduction/config.py",
            "POINTCLOUD_VOXEL_SIZE_M",
            source_kind="offline visualization setting",
        ),
    }
    for name, value in parameters["critics"].items():
        result[f"critic_{name}"] = item(
            value,
            "scale",
            nav_source,
            f"controller_server.ros__parameters.FollowPath.{name}.scale",
        )
    return result
