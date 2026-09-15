from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import numpy as np

from collision import FootprintCollisionChecker
from config import DATA_PROJECT_ROOT, REPO_ROOT, find_source_zip
from dwb import DWBPlan, SimplifiedDWBPlanner
from robot_model import integrate_unicycle
from run_astar_demo import (
    NavigationBundle,
    generate_astar_artifacts,
    map_classes,
    navigation_cmap,
    save_json,
)


@dataclass(frozen=True)
class SimulationSnapshot:
    step: int
    pose: np.ndarray
    history_count: int
    plan: DWBPlan


@dataclass(frozen=True)
class SimulationResult:
    poses: np.ndarray
    velocities: np.ndarray
    omegas: np.ndarray
    snapshots: list[SimulationSnapshot]
    success: bool
    message: str
    compute_time_s: float


def simulate_navigation(bundle: NavigationBundle) -> tuple[SimulationResult, FootprintCollisionChecker]:
    p = bundle.parameters
    path = bundle.path_world
    initial_target = path[min(20, len(path) - 1)]
    pose = np.asarray(
        [
            path[0, 0],
            path[0, 1],
            math.atan2(initial_target[1] - path[0, 1], initial_target[0] - path[0, 0]),
        ],
        dtype=float,
    )
    checker = FootprintCollisionChecker(
        bundle.occupancy_map,
        p["footprint"],
        bundle.inflation.distance_to_nonfree_m,
    )
    if checker.collides(pose[None, :])[0]:
        raise RuntimeError("The selected starting footprint is in collision")
    planner = SimplifiedDWBPlanner(path, checker, p)
    controller_dt = 1.0 / float(p["controller_frequency"])
    nominal_duration = bundle.metrics["astar"]["path_length_m"] / max(p["max_vel_x"], 1e-6)
    max_steps = int(max(1200, nominal_duration / controller_dt * 2.2))
    snapshot_interval = max(1, max_steps // 72)

    poses = [pose.copy()]
    velocities: list[float] = []
    omegas: list[float] = []
    snapshots: list[SimulationSnapshot] = []
    velocity = 0.0
    omega = 0.0
    closest_distance = float(np.linalg.norm(pose[:2] - path[-1]))
    last_progress_step = 0
    started = time.perf_counter()
    message = "maximum simulation steps reached"
    success = False

    for step in range(max_steps):
        distance_to_goal = float(np.linalg.norm(pose[:2] - path[-1]))
        if distance_to_goal <= float(p["goal_tolerance"]):
            success = True
            message = "goal reached within configured xy tolerance"
            break
        plan = planner.plan(pose, velocity, omega, controller_dt)
        if step % snapshot_interval == 0:
            snapshots.append(
                SimulationSnapshot(step, pose.copy(), len(poses), plan)
            )
        next_pose = integrate_unicycle(
            pose, plan.velocity, plan.angular_velocity, controller_dt
        )
        if checker.collides(next_pose[None, :])[0]:
            message = "selected command would put the full footprint in collision"
            break
        pose = next_pose
        velocity = plan.velocity
        omega = plan.angular_velocity
        poses.append(pose.copy())
        velocities.append(velocity)
        omegas.append(omega)
        if distance_to_goal < closest_distance - 0.05:
            closest_distance = distance_to_goal
            last_progress_step = step
        if step - last_progress_step > int(20.0 / controller_dt):
            message = "controller made no measurable goal progress for 20 seconds"
            break

    # Always retain a final local-planner view for the animation.
    final_plan = planner.plan(pose, velocity, omega, controller_dt)
    snapshots.append(
        SimulationSnapshot(len(velocities), pose.copy(), len(poses), final_plan)
    )
    return (
        SimulationResult(
            poses=np.asarray(poses),
            velocities=np.asarray(velocities),
            omegas=np.asarray(omegas),
            snapshots=snapshots,
            success=success,
            message=message,
            compute_time_s=time.perf_counter() - started,
        ),
        checker,
    )


def _candidate_indices(plan: DWBPlan, maximum_each: int = 18) -> tuple[np.ndarray, np.ndarray]:
    safe = np.flatnonzero(~plan.collisions)
    rejected = np.flatnonzero(plan.collisions)
    if len(safe) > maximum_each:
        safe = safe[np.linspace(0, len(safe) - 1, maximum_each, dtype=int)]
    if len(rejected) > maximum_each:
        rejected = rejected[
            np.linspace(0, len(rejected) - 1, maximum_each, dtype=int)
        ]
    return safe, rejected


def render_navigation_gif(
    bundle: NavigationBundle,
    simulation: SimulationResult,
    checker: FootprintCollisionChecker,
    output_path: Path,
) -> None:
    occupancy_map = bundle.occupancy_map
    classes = map_classes(occupancy_map, bundle.inflation)
    fig, (overview, local) = plt.subplots(
        1, 2, figsize=(12, 6.4), gridspec_kw={"width_ratios": [1.15, 1]}, constrained_layout=True
    )
    for ax in (overview, local):
        ax.imshow(
            classes,
            cmap=navigation_cmap(),
            vmin=0,
            vmax=3,
            origin="upper",
            extent=occupancy_map.extent,
            interpolation="nearest",
        )
        ax.set_aspect("equal")
        ax.grid(False)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    overview.plot(bundle.path_world[:, 0], bundle.path_world[:, 1], color="#1388c9", linewidth=1.4, alpha=0.9)
    local.plot(bundle.path_world[:, 0], bundle.path_world[:, 1], color="#58a6c9", linewidth=1.0, alpha=0.8, linestyle="--")
    overview.set_title("Global progress", loc="left", weight="bold")
    local.set_title("Local DWB-style samples", loc="left", weight="bold")
    overview.scatter(bundle.path_world[0, 0], bundle.path_world[0, 1], s=45, color="#179c52", zorder=5)
    overview.scatter(bundle.path_world[-1, 0], bundle.path_world[-1, 1], s=70, marker="*", color="#d43b3b", zorder=5)
    local.scatter(bundle.path_world[-1, 0], bundle.path_world[-1, 1], s=70, marker="*", color="#d43b3b", zorder=5)
    fig.suptitle("Real-map A* + Simplified DWB Navigation", fontsize=15, weight="bold")
    fig.text(0.99, 0.985, "OFFLINE REPRODUCTION", ha="right", va="top", color="#9b2c2c", weight="bold")
    status = overview.text(
        0.02,
        0.98,
        "",
        transform=overview.transAxes,
        ha="left",
        va="top",
        fontsize=8.5,
        bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "none", "pad": 3},
        zorder=8,
    )
    local.text(
        0.02,
        0.02,
        "green: safe  |  red: collision\nblue: selected  |  purple: footprint",
        transform=local.transAxes,
        ha="left",
        va="bottom",
        fontsize=7.8,
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "none", "pad": 3},
        zorder=8,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = PillowWriter(fps=8, metadata={"title": "Offline navigation reproduction"})
    controller_dt = 1.0 / float(bundle.parameters["controller_frequency"])
    with writer.saving(fig, str(output_path), dpi=90):
        for snapshot in simulation.snapshots:
            removable = []
            history = simulation.poses[: snapshot.history_count]
            removable.extend(
                overview.plot(history[:, 0], history[:, 1], color="#e4572e", linewidth=2.0, zorder=4)
            )
            removable.extend(
                local.plot(history[:, 0], history[:, 1], color="#e4572e", linewidth=2.2, zorder=4)
            )
            safe, rejected = _candidate_indices(snapshot.plan)
            for index in safe:
                line, = local.plot(
                    snapshot.plan.trajectories[index, :, 0],
                    snapshot.plan.trajectories[index, :, 1],
                    color="#65a765",
                    linewidth=0.55,
                    alpha=0.50,
                    zorder=3,
                )
                removable.append(line)
            for index in rejected:
                line, = local.plot(
                    snapshot.plan.trajectories[index, :, 0],
                    snapshot.plan.trajectories[index, :, 1],
                    color="#cc5965",
                    linewidth=0.55,
                    alpha=0.52,
                    zorder=3,
                )
                removable.append(line)
            selected = snapshot.plan.trajectories[snapshot.plan.selected_index]
            line, = local.plot(selected[:, 0], selected[:, 1], color="#0067b1", linewidth=2.3, zorder=5)
            removable.append(line)
            footprint = checker.transformed_footprint(snapshot.pose)
            footprint = np.vstack((footprint, footprint[0]))
            for ax in (overview, local):
                line, = ax.plot(footprint[:, 0], footprint[:, 1], color="#702963", linewidth=2.0, zorder=6)
                removable.append(line)
            local.set_xlim(snapshot.pose[0] - 1.35, snapshot.pose[0] + 1.35)
            local.set_ylim(snapshot.pose[1] - 1.35, snapshot.pose[1] + 1.35)
            distance = float(np.linalg.norm(snapshot.pose[:2] - bundle.path_world[-1]))
            status.set_text(
                f"simulated t = {snapshot.step * controller_dt:6.1f} s\n"
                f"goal distance = {distance:5.2f} m"
            )
            writer.grab_frame(facecolor="white")
            for artist in removable:
                artist.remove()
    plt.close(fig)


def navigation_metrics(
    bundle: NavigationBundle,
    simulation: SimulationResult,
    checker: FootprintCollisionChecker,
) -> dict:
    controller_dt = 1.0 / float(bundle.parameters["controller_frequency"])
    traveled = float(np.linalg.norm(np.diff(simulation.poses[:, :2], axis=0), axis=1).sum())
    collisions = checker.collides(simulation.poses)
    clearances = checker.clearance_at(simulation.poses)
    final_distance = float(np.linalg.norm(simulation.poses[-1, :2] - bundle.path_world[-1]))
    return {
        "artifact": "assets/navigation/04_offline_navigation_reproduction.gif",
        "label": "OFFLINE REPRODUCTION",
        "scope": "Python-only real-map A* plus simplified DWB-style unicycle simulation; no ROS 2 graph and no hardware run",
        "source_map": "map/scans5f2.yaml + wzb/map/scans5f2.pgm",
        "global_path_source": "offline_reproduction/astar.py",
        "controller_source": "offline_reproduction/dwb.py",
        "success": simulation.success,
        "termination_message": simulation.message,
        "controller_steps": int(len(simulation.velocities)),
        "simulated_duration_s": float(len(simulation.velocities) * controller_dt),
        "compute_time_s": simulation.compute_time_s,
        "trajectory_length_m": traveled,
        "final_goal_distance_m": final_distance,
        "configured_goal_tolerance_m": float(bundle.parameters["goal_tolerance"]),
        "minimum_center_clearance_m": float(clearances.min()),
        "full_footprint_collision_count": int(np.count_nonzero(collisions)),
        "all_executed_poses_collision_free": bool(not np.any(collisions)),
        "candidate_trajectories_per_cycle": int(
            bundle.parameters["vx_samples"] * bundle.parameters["vtheta_samples"]
        ),
        "velocity_statistics": {
            "maximum_mps": float(simulation.velocities.max(initial=0.0)),
            "mean_mps": float(simulation.velocities.mean()) if len(simulation.velocities) else 0.0,
            "maximum_abs_angular_rps": float(np.abs(simulation.omegas).max(initial=0.0)),
        },
        "rendered_frames": int(len(simulation.snapshots)),
        "parameters_used": "results/offline_reproduction/parameters_used.json",
    }


def generate_navigation_artifacts(
    bundle: NavigationBundle,
    repo_root: Path = REPO_ROOT,
) -> dict:
    simulation, checker = simulate_navigation(bundle)
    metrics = navigation_metrics(bundle, simulation, checker)
    if not simulation.success:
        raise RuntimeError(
            "Offline controller did not reach the goal: " + simulation.message
        )
    if not metrics["all_executed_poses_collision_free"]:
        raise RuntimeError("Offline controller produced a footprint collision")
    render_navigation_gif(
        bundle,
        simulation,
        checker,
        repo_root / "assets/navigation/04_offline_navigation_reproduction.gif",
    )
    save_json(
        repo_root / "results/offline_reproduction/navigation_metrics.json", metrics
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the real-map offline navigation reproduction")
    parser.add_argument("--source-zip", type=Path, help="Path to the original wzb.zip")
    args = parser.parse_args()
    source_zip = find_source_zip(args.source_zip)
    bundle = generate_astar_artifacts(source_zip, data_root=DATA_PROJECT_ROOT)
    metrics = generate_navigation_artifacts(bundle)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
