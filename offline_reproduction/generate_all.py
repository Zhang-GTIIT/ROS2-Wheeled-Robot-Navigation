from __future__ import annotations

import argparse
import json
from pathlib import Path

from config import DATA_PROJECT_ROOT, REPO_ROOT, find_source_zip
from draw_pipeline import generate_pipeline_artifact
from run_astar_demo import generate_astar_artifacts
from run_navigation_demo import generate_navigation_artifacts
from validate_outputs import validate_outputs
from visualize_pointcloud import generate_pointcloud_artifacts


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Regenerate every real-data offline navigation artifact"
    )
    parser.add_argument("--source-zip", type=Path, help="Path to the original wzb.zip")
    parser.add_argument(
        "--data-root",
        type=Path,
        default=DATA_PROJECT_ROOT,
        help="Project root containing map/ and nav2_params/ (default: repository root)",
    )
    args = parser.parse_args()
    source_zip = find_source_zip(args.source_zip)
    data_root = args.data_root.resolve()

    print(f"[1/4] Real occupancy map + A*: {source_zip}")
    bundle = generate_astar_artifacts(source_zip, REPO_ROOT, data_root)
    print("[2/4] Real PCD voxel downsampling")
    pointcloud = generate_pointcloud_artifacts(source_zip, REPO_ROOT)
    print("[3/4] Project navigation pipeline")
    pipeline = generate_pipeline_artifact(REPO_ROOT)
    print("[4/4] Simplified DWB-style offline simulation")
    navigation = generate_navigation_artifacts(bundle, REPO_ROOT)
    validation = validate_outputs(REPO_ROOT)

    summary = {
        "astar_path_length_m": bundle.metrics["astar"]["path_length_m"],
        "pointcloud_input_points": pointcloud["finite_input_points"],
        "pointcloud_downsampled_points": pointcloud["downsampled_points"],
        "navigation_success": navigation["success"],
        "navigation_steps": navigation["controller_steps"],
        "pipeline": str(pipeline),
        "validation": validation["status"],
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
