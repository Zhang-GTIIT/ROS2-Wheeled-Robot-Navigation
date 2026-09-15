from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image

from config import REPO_ROOT


EXPECTED_IMAGES = {
    "01_astar_global_path.png": 1,
    "02_slam_pointcloud.png": 1,
    "03_navigation_pipeline.png": 1,
    "04_offline_navigation_reproduction.gif": 2,
}


def validate_outputs(repo_root: Path = REPO_ROOT) -> dict:
    assets = repo_root / "assets/navigation"
    results = repo_root / "results/offline_reproduction"
    astar = json.loads((results / "astar_metrics.json").read_text(encoding="utf-8"))
    cloud = json.loads((results / "pointcloud_metrics.json").read_text(encoding="utf-8"))
    navigation = json.loads(
        (results / "navigation_metrics.json").read_text(encoding="utf-8")
    )
    json.loads((results / "parameters_used.json").read_text(encoding="utf-8"))

    if not astar["astar"]["all_path_cells_traversable"]:
        raise AssertionError("A* path validation failed")
    if not all(
        astar["endpoint_selection"][key]
        for key in (
            "start_is_free",
            "goal_is_free",
            "start_is_traversable",
            "goal_is_traversable",
        )
    ):
        raise AssertionError("A* start or goal is not legal free space")
    if not navigation["success"]:
        raise AssertionError("Offline navigation did not reach the goal")
    if not navigation["all_executed_poses_collision_free"]:
        raise AssertionError("Offline navigation contains a footprint collision")
    if cloud["downsampled_points"] >= cloud["finite_input_points"]:
        raise AssertionError("Voxel filtering did not reduce the point cloud")

    inspected = {}
    for name, minimum_frames in EXPECTED_IMAGES.items():
        path = assets / name
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            frames = int(getattr(image, "n_frames", 1))
            dimensions = list(image.size)
        if frames < minimum_frames:
            raise AssertionError(f"{name} does not contain the expected frames")
        if path.stat().st_size >= 100 * 1024 * 1024:
            raise AssertionError(f"{name} is unexpectedly larger than 100 MiB")
        inspected[name] = {
            "dimensions_px": dimensions,
            "frames": frames,
            "size_bytes": path.stat().st_size,
        }

    forbidden = [
        str(path.relative_to(repo_root))
        for path in (repo_root / "offline_reproduction").rglob("*")
        if path.is_file() and path.suffix.lower() in {".pgm", ".pcd", ".bag", ".db3"}
    ]
    if forbidden:
        raise AssertionError("Raw large data was copied into the reproduction folder")
    return {
        "status": "PASS",
        "astar_collision_free": True,
        "navigation_success": True,
        "navigation_collision_free": True,
        "raw_large_data_copied": False,
        "artifacts": inspected,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate generated offline reproduction outputs")
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    args = parser.parse_args()
    print(json.dumps(validate_outputs(args.repo_root.resolve()), indent=2))


if __name__ == "__main__":
    main()
