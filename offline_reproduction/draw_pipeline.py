from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from config import REPO_ROOT


def _box(ax, x, y, w, h, title, detail, face, edge="#28445f"):
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.012,rounding_size=0.015",
        linewidth=1.5,
        facecolor=face,
        edgecolor=edge,
        zorder=2,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h * 0.64, title, ha="center", va="center", fontsize=10.5, weight="bold", color="#162b3d", zorder=3)
    ax.text(x + w / 2, y + h * 0.30, detail, ha="center", va="center", fontsize=7.8, color="#334e63", linespacing=1.25, zorder=3)


def _arrow(ax, start, end, label="", color="#52738f", style="-", rad=0.0, label_offset=0.018):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=1.4,
        linestyle=style,
        color=color,
        connectionstyle=f"arc3,rad={rad}",
        zorder=1,
    )
    ax.add_patch(arrow)
    if label:
        x = (start[0] + end[0]) / 2
        y = (start[1] + end[1]) / 2
        ax.text(x, y + label_offset, label, ha="center", va="bottom", fontsize=7.2, color=color, backgroundcolor="white", zorder=4)


def render_pipeline(output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 9), constrained_layout=True)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_title("Ranger Mini V3 Mapping, Localization and Navigation Pipeline", loc="left", fontsize=18, weight="bold", pad=15)
    ax.text(0.0, 0.955, "Derived from retained launch scripts, nav2_params.yaml, behavior trees and ROS 2 packages", fontsize=9.5, color="#526b7d")

    blue = "#dcecf7"
    green = "#e1f2e6"
    amber = "#fff0cf"
    purple = "#ebe4f5"
    gray = "#edf0f2"

    # Mapping and localization lane.
    _box(ax, 0.02, 0.67, 0.15, 0.15, "Sensors", "Livox LiDAR + IMU\nOrbbec depth cloud", blue)
    _box(ax, 0.22, 0.67, 0.15, 0.15, "FAST-LIO2", "LiDAR-inertial mapping\n(upstream source reference)", green)
    _box(ax, 0.42, 0.67, 0.13, 0.15, "PCD Map", "scans0120gf.pcd\nreal SLAM output", green)
    _box(ax, 0.60, 0.67, 0.15, 0.15, "ICP Localization", "Live cloud + PCD map\npoint-to-plane matching", green)
    _box(ax, 0.80, 0.67, 0.16, 0.15, "Pose / TF", "map / odom / base_link\nlocalization feedback", gray)

    # Runtime navigation lane.
    _box(ax, 0.03, 0.34, 0.15, 0.16, "Map Server", "scans5f2.yaml + PGM\nstatic occupancy", green)
    _box(ax, 0.23, 0.34, 0.17, 0.16, "Nav2 Costmaps", "LiDAR/depth → STVL obstacles\n+ static + inflation\n0.8 × 0.6 m footprint", amber)
    _box(ax, 0.46, 0.34, 0.15, 0.16, "Global Planner", "Navfn GridBased\nuse_astar: true", amber)
    _box(ax, 0.67, 0.34, 0.15, 0.16, "Local Controller", "DWB trajectory sampling\n20 × 10 (v, ω)", amber)
    _box(ax, 0.87, 0.34, 0.11, 0.16, "Ranger Mini V3", "/cmd_vel\nwheeled base", gray)
    _box(ax, 0.46, 0.10, 0.15, 0.14, "Behavior Tree", "goals + replanning + recovery\nmulti-floor map switching", purple)

    _arrow(ax, (0.17, 0.745), (0.22, 0.745), "cloud + IMU")
    _arrow(ax, (0.37, 0.745), (0.42, 0.745))
    _arrow(ax, (0.55, 0.745), (0.60, 0.745), "reference map")
    _arrow(ax, (0.75, 0.745), (0.80, 0.745), "pose / TF")
    _arrow(ax, (0.18, 0.42), (0.23, 0.42), "occupancy")
    _arrow(ax, (0.40, 0.42), (0.46, 0.42), "global costmap")
    _arrow(ax, (0.61, 0.42), (0.67, 0.42), "global path")
    _arrow(ax, (0.82, 0.42), (0.87, 0.42), "/cmd_vel")
    _arrow(ax, (0.535, 0.24), (0.535, 0.34))
    _arrow(ax, (0.095, 0.67), (0.285, 0.50), "depth / live clouds", rad=-0.10, label_offset=-0.01)
    _arrow(ax, (0.88, 0.67), (0.755, 0.50), "localization", rad=-0.08, label_offset=-0.005)
    _arrow(ax, (0.40, 0.37), (0.67, 0.37), "local costmap", rad=0.22, label_offset=-0.045)
    _arrow(ax, (0.925, 0.50), (0.69, 0.67), "odometry feedback", color="#7a6a9a", style="--", rad=-0.20)

    offline = FancyBboxPatch(
        (0.012, 0.29), 0.825, 0.255,
        boxstyle="round,pad=0.012,rounding_size=0.02",
        linewidth=1.6,
        linestyle="--",
        facecolor="none",
        edgecolor="#b23a48",
        zorder=0,
    )
    ax.add_patch(offline)
    ax.text(0.25, 0.267, "OFFLINE REPRODUCTION SCOPE: real map → A* path → simplified DWB rollout", ha="center", va="top", fontsize=9, color="#9b2c2c", weight="bold")
    ax.text(
        0.01,
        0.04,
        "The offline scripts reproduce algorithmic behavior without ROS 2, hardware drivers or a live robot. "
        "FAST-LIO source in this repository is an upstream reference, not a byte-identical recovery of the robot workspace.",
        fontsize=8.2,
        color="#526b7d",
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def generate_pipeline_artifact(repo_root: Path = REPO_ROOT) -> Path:
    output = repo_root / "assets/navigation/03_navigation_pipeline.png"
    render_pipeline(output)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Draw the project navigation pipeline")
    parser.parse_args()
    print(generate_pipeline_artifact())


if __name__ == "__main__":
    main()
