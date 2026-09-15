from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

from astar import AStarResult, astar_grid, select_far_endpoints
from config import (
    DATA_PROJECT_ROOT,
    ENDPOINT_SELECTION_CLEARANCE_M,
    MAP_ZIP_MEMBER,
    REPO_ROOT,
    find_source_zip,
    load_project_parameters,
    parameter_provenance,
)
from inflation import InflationResult, build_inflation
from map_loader import OccupancyMap, load_occupancy_map


@dataclass(frozen=True)
class NavigationBundle:
    occupancy_map: OccupancyMap
    inflation: InflationResult
    astar: AStarResult
    path_world: np.ndarray
    start_cell: tuple[int, int]
    goal_cell: tuple[int, int]
    component_size: int
    parameters: dict
    metrics: dict


def map_classes(occupancy_map: OccupancyMap, inflation: InflationResult) -> np.ndarray:
    classes = np.zeros(occupancy_map.pixels.shape, dtype=np.uint8)
    classes[occupancy_map.free] = 1
    classes[inflation.inflation_zone] = 2
    classes[occupancy_map.occupied] = 3
    return classes


def navigation_cmap() -> ListedColormap:
    return ListedColormap(["#aeb5bd", "#f7f8fa", "#f4b860", "#1c2530"])


def save_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def prepare_navigation_bundle(source_zip: Path, data_root: Path | None = None) -> NavigationBundle:
    parameters = load_project_parameters(data_root or DATA_PROJECT_ROOT)
    occupancy_map = load_occupancy_map(
        parameters["map_yaml_path"], source_zip, MAP_ZIP_MEMBER
    )
    inflation = build_inflation(
        occupancy_map, parameters["footprint"], parameters["inflation_radius"]
    )
    start_cell, goal_cell, component_size = select_far_endpoints(
        inflation.traversable,
        inflation.distance_to_nonfree_m,
        ENDPOINT_SELECTION_CLEARANCE_M,
    )
    result = astar_grid(inflation.traversable, start_cell, goal_cell)
    path_world = occupancy_map.grid_to_world(result.path_cells)
    start_world, goal_world = occupancy_map.grid_to_world(
        np.asarray([start_cell, goal_cell])
    )
    euclidean = float(np.linalg.norm(goal_world - start_world))
    path_length = float(result.path_length_pixels * occupancy_map.resolution)
    path_clearance = inflation.distance_to_nonfree_m[
        result.path_cells[:, 0], result.path_cells[:, 1]
    ]

    metrics = {
        "artifact": "assets/navigation/01_astar_global_path.png",
        "label": "OFFLINE REPRODUCTION",
        "source_map": {
            "yaml": "map/scans5f2.yaml",
            "image": MAP_ZIP_MEMBER,
            "source_archive": source_zip.name,
            **occupancy_map.metadata(),
        },
        "cell_counts": {
            "free": int(np.count_nonzero(occupancy_map.free)),
            "occupied": int(np.count_nonzero(occupancy_map.occupied)),
            "unknown": int(np.count_nonzero(occupancy_map.unknown)),
            "traversable_after_safety_margin": int(
                np.count_nonzero(inflation.traversable)
            ),
            "largest_connected_component": component_size,
        },
        "safety_model": {
            "footprint_m": parameters["footprint"],
            "footprint_circumscribed_radius_m": inflation.footprint_circumscribed_radius_m,
            "configured_local_inflation_radius_m": parameters["inflation_radius"],
            "center_safety_radius_m": inflation.center_safety_radius_m,
            "unknown_cells_traversable": False,
        },
        "endpoint_selection": {
            "method": "deterministic farthest pair in the largest safe connected component",
            "note": "Start and goal are selected for offline reproduction; they are not claimed as recorded mission waypoints.",
            "minimum_selection_clearance_m": ENDPOINT_SELECTION_CLEARANCE_M,
            "start_cell_row_col": list(start_cell),
            "goal_cell_row_col": list(goal_cell),
            "start_world_xy_m": [float(v) for v in start_world],
            "goal_world_xy_m": [float(v) for v in goal_world],
            "start_is_free": bool(occupancy_map.free[start_cell]),
            "goal_is_free": bool(occupancy_map.free[goal_cell]),
            "start_is_traversable": bool(inflation.traversable[start_cell]),
            "goal_is_traversable": bool(inflation.traversable[goal_cell]),
        },
        "astar": {
            "implementation": "offline_reproduction/astar.py (Python heap-based 8-connected A*)",
            "heuristic": "octile distance",
            "diagonal_corner_cutting": False,
            "expanded_nodes": result.expanded_nodes,
            "planning_time_s": result.planning_time_s,
            "path_points": int(len(result.path_cells)),
            "path_length_pixels": result.path_length_pixels,
            "path_length_m": path_length,
            "straight_line_distance_m": euclidean,
            "path_efficiency_ratio": path_length / euclidean,
            "minimum_center_clearance_m": float(path_clearance.min()),
            "all_path_cells_traversable": bool(
                np.all(inflation.traversable[result.path_cells[:, 0], result.path_cells[:, 1]])
            ),
        },
    }
    return NavigationBundle(
        occupancy_map=occupancy_map,
        inflation=inflation,
        astar=result,
        path_world=path_world,
        start_cell=start_cell,
        goal_cell=goal_cell,
        component_size=component_size,
        parameters=parameters,
        metrics=metrics,
    )


def render_astar(bundle: NavigationBundle, output_path: Path) -> None:
    occupancy_map = bundle.occupancy_map
    start_world, goal_world = occupancy_map.grid_to_world(
        np.asarray([bundle.start_cell, bundle.goal_cell])
    )
    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    ax.imshow(
        map_classes(occupancy_map, bundle.inflation),
        cmap=navigation_cmap(),
        vmin=0,
        vmax=3,
        origin="upper",
        extent=occupancy_map.extent,
        interpolation="nearest",
    )
    ax.plot(
        bundle.path_world[:, 0],
        bundle.path_world[:, 1],
        color="#087fce",
        linewidth=2.1,
        label="A* global path",
        zorder=3,
    )
    ax.scatter(*start_world, s=80, color="#179c52", edgecolor="white", linewidth=1.3, zorder=4, label="Start")
    ax.scatter(*goal_world, s=95, marker="*", color="#d43b3b", edgecolor="white", linewidth=1.0, zorder=4, label="Goal")
    ax.set_title("A* Global Path on the Real scans5f2 Occupancy Map", loc="left", weight="bold")
    ax.text(
        0.995,
        0.99,
        "OFFLINE REPRODUCTION",
        transform=ax.transAxes,
        ha="right",
        va="top",
        color="#9b2c2c",
        fontsize=10,
        weight="bold",
        bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "#9b2c2c", "pad": 4},
    )
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")
    ax.set_aspect("equal")
    ax.grid(False)
    ax.legend(loc="upper left", framealpha=0.92)
    ax.text(
        0.01,
        0.015,
        "Dark: occupied  |  Gray: unknown  |  White: free  |  Amber: local inflation zone",
        transform=ax.transAxes,
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "none", "pad": 3},
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def generate_astar_artifacts(
    source_zip: Path,
    repo_root: Path = REPO_ROOT,
    data_root: Path | None = None,
) -> NavigationBundle:
    bundle = prepare_navigation_bundle(source_zip, data_root)
    render_astar(bundle, repo_root / "assets/navigation/01_astar_global_path.png")
    save_json(
        repo_root / "results/offline_reproduction/astar_metrics.json",
        bundle.metrics,
    )
    save_json(
        repo_root / "results/offline_reproduction/parameters_used.json",
        parameter_provenance(bundle.parameters, bundle.occupancy_map.metadata()),
    )
    return bundle


def main() -> None:
    parser = argparse.ArgumentParser(description="Reproduce A* planning on the real scans5f2 map")
    parser.add_argument("--source-zip", type=Path, help="Path to the original wzb.zip")
    args = parser.parse_args()
    source_zip = find_source_zip(args.source_zip)
    bundle = generate_astar_artifacts(source_zip)
    print(json.dumps(bundle.metrics["astar"], indent=2))


if __name__ == "__main__":
    main()
