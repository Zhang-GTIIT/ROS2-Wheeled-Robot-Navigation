from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from collision import FootprintCollisionChecker
from robot_model import rollout_unicycle, wrap_angle


@dataclass(frozen=True)
class DWBPlan:
    velocity: float
    angular_velocity: float
    selected_index: int
    trajectories: np.ndarray
    collisions: np.ndarray
    scores: np.ndarray
    minimum_clearance_m: np.ndarray


class SimplifiedDWBPlanner:
    """A small, transparent DWB-style trajectory sampler for offline reproduction."""

    def __init__(
        self,
        global_path_xy: np.ndarray,
        collision_checker: FootprintCollisionChecker,
        parameters: dict,
        integration_dt_s: float = 0.05,
    ) -> None:
        self.path = np.asarray(global_path_xy, dtype=float)
        self.path_tree = cKDTree(self.path)
        self.checker = collision_checker
        self.parameters = parameters
        self.integration_dt_s = float(integration_dt_s)
        path_steps = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.path_step_m = float(np.median(path_steps[path_steps > 0]))
        self.progress_index = 0

    def _local_path_tree(self, pose: np.ndarray) -> tuple[cKDTree, int]:
        # DWB scores against the forward section of the global plan. Restricting the
        # search prevents a nearby side of a hairpin from being mistaken for progress.
        update_low = max(0, self.progress_index - 20)
        update_high = min(
            len(self.path),
            self.progress_index + max(80, int(round(6.0 / self.path_step_m))),
        )
        update_segment = self.path[update_low:update_high]
        nearest = int(np.argmin(np.sum((update_segment - pose[:2]) ** 2, axis=1)))
        self.progress_index = max(self.progress_index, update_low + nearest)
        search_low = max(0, self.progress_index - 12)
        search_high = min(
            len(self.path),
            self.progress_index + max(120, int(round(10.0 / self.path_step_m))),
        )
        return cKDTree(self.path[search_low:search_high]), search_low

    def _polyline_distance(
        self, points: np.ndarray, nearest_indices: np.ndarray
    ) -> np.ndarray:
        points = np.asarray(points, dtype=float)
        indices = np.asarray(nearest_indices, dtype=int)

        def segment_distance(first: np.ndarray, second: np.ndarray) -> np.ndarray:
            delta = second - first
            denominator = np.sum(delta * delta, axis=1)
            fraction = np.divide(
                np.sum((points - first) * delta, axis=1),
                denominator,
                out=np.zeros(len(points), dtype=float),
                where=denominator > 1e-12,
            )
            fraction = np.clip(fraction, 0.0, 1.0)
            projected = first + fraction[:, None] * delta
            return np.linalg.norm(points - projected, axis=1)

        previous = np.maximum(indices - 1, 0)
        following = np.minimum(indices + 1, len(self.path) - 1)
        before = segment_distance(self.path[previous], self.path[indices])
        after = segment_distance(self.path[indices], self.path[following])
        return np.minimum(before, after)

    def _dynamic_window(
        self,
        current_velocity: float,
        current_omega: float,
        controller_dt_s: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        p = self.parameters
        v_low = max(float(p["min_vel_x"]), current_velocity + float(p["decel_lim_x"]) * controller_dt_s)
        v_high = min(float(p["max_vel_x"]), current_velocity + float(p["acc_lim_x"]) * controller_dt_s)
        if v_high < v_low:
            v_low = v_high
        w_low = max(-float(p["max_vel_theta"]), current_omega - float(p["acc_lim_theta"]) * controller_dt_s)
        w_high = min(float(p["max_vel_theta"]), current_omega + float(p["acc_lim_theta"]) * controller_dt_s)

        velocities = np.linspace(v_low, v_high, int(p["vx_samples"]), dtype=np.float32)
        omegas = np.linspace(w_low, w_high, int(p["vtheta_samples"]), dtype=np.float32)
        velocity_grid, omega_grid = np.meshgrid(velocities, omegas, indexing="ij")
        return velocity_grid.ravel(), omega_grid.ravel()

    def plan(
        self,
        pose: np.ndarray,
        current_velocity: float,
        current_omega: float,
        controller_dt_s: float,
    ) -> DWBPlan:
        velocities, omegas = self._dynamic_window(
            current_velocity, current_omega, controller_dt_s
        )
        trajectories = rollout_unicycle(
            pose,
            velocities,
            omegas,
            float(self.parameters["sim_time"]),
            self.integration_dt_s,
        )
        collisions = self.checker.trajectories_collide(trajectories)
        clearances = self.checker.clearance_at(trajectories.reshape(-1, 3)).reshape(
            trajectories.shape[:2]
        )
        minimum_clearance = np.min(clearances, axis=1)
        terminal = trajectories[:, -1]

        local_tree, local_offset = self._local_path_tree(pose)
        _, nearest_local_indices = local_tree.query(terminal[:, :2])
        nearest_indices = nearest_local_indices + local_offset
        path_distance = self._polyline_distance(terminal[:, :2], nearest_indices)
        forward_distance = float(self.parameters["forward_point_distance"])
        forward_points = terminal[:, :2] + forward_distance * np.column_stack(
            (np.cos(terminal[:, 2]), np.sin(terminal[:, 2]))
        )
        _, forward_local_indices = local_tree.query(forward_points)
        path_alignment = self._polyline_distance(
            forward_points, forward_local_indices + local_offset
        )

        lookahead_steps = max(1, int(round(max(0.8, forward_distance) / self.path_step_m)))
        lookahead_indices = np.minimum(nearest_indices + lookahead_steps, len(self.path) - 1)
        lookahead = self.path[lookahead_indices]
        desired_headings = np.arctan2(
            lookahead[:, 1] - terminal[:, 1], lookahead[:, 0] - terminal[:, 0]
        )
        heading_error = np.abs(wrap_angle(desired_headings - terminal[:, 2]))
        goal_distance = np.linalg.norm(terminal[:, :2] - self.path[-1], axis=1)
        distance_now = float(np.linalg.norm(np.asarray(pose[:2]) - self.path[-1]))
        # The forward scoring point extends 0.60 m in this project. Suppress that
        # critic in the final approach so it cannot create a false stop exactly
        # one forward-point distance before the goal.
        if distance_now < forward_distance + float(self.parameters["goal_tolerance"]):
            path_alignment = np.zeros_like(path_alignment)

        inflation_radius = max(float(self.parameters["inflation_radius"]), 1e-6)
        obstacle_cost = np.exp(-np.maximum(minimum_clearance, 0.0) / inflation_radius)
        critics = self.parameters["critics"]
        # Nav2's path/grid critics operate in costmap-cell units. Convert metric
        # distances before applying the scales retained in nav2_params.yaml.
        cells_per_metre = 1.0 / float(self.parameters["costmap_resolution"])
        scores = (
            float(critics["BaseObstacle"]) * obstacle_cost
            + float(critics["PathAlign"]) * path_alignment * cells_per_metre
            + float(critics["PathDist"]) * path_distance * cells_per_metre
            + float(critics["GoalAlign"]) * heading_error
            + float(critics["GoalDist"]) * goal_distance * cells_per_metre
        )

        rotate_range = max(0.30, 1.05 * float(self.parameters["goal_tolerance"]))
        if distance_now < rotate_range:
            goal_heading = np.arctan2(
                self.path[-1, 1] - terminal[:, 1],
                self.path[-1, 0] - terminal[:, 0],
            )
            rotate_error = np.abs(wrap_angle(goal_heading - terminal[:, 2]))
            scores += float(critics["RotateToGoal"]) * rotate_error
            scores += 10.0 * velocities

        # A light speed preference breaks equal-cost ties without replacing the configured critics.
        scores -= 0.10 * velocities / max(float(self.parameters["max_vel_x"]), 1e-6)
        scores[collisions] = np.inf
        selected = int(np.argmin(scores))
        if not np.isfinite(scores[selected]):
            raise RuntimeError("No collision-free DWB trajectory was found")
        return DWBPlan(
            velocity=float(velocities[selected]),
            angular_velocity=float(omegas[selected]),
            selected_index=selected,
            trajectories=trajectories,
            collisions=collisions,
            scores=scores,
            minimum_clearance_m=minimum_clearance,
        )
