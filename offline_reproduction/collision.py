from __future__ import annotations

import numpy as np

from map_loader import OccupancyMap


class FootprintCollisionChecker:
    """Conservative grid collision checks for an oriented polygon footprint."""

    def __init__(
        self,
        occupancy_map: OccupancyMap,
        footprint: list[list[float]],
        distance_to_nonfree_m: np.ndarray,
    ) -> None:
        self.map = occupancy_map
        self.footprint = np.asarray(footprint, dtype=float)
        self.distance = distance_to_nonfree_m
        self.circumscribed_radius = float(np.linalg.norm(self.footprint, axis=1).max())
        self.inscribed_radius = self._inscribed_radius(self.footprint)
        self.samples = self._interior_samples(self.footprint, max(0.08, occupancy_map.resolution))

    @staticmethod
    def _inscribed_radius(vertices: np.ndarray) -> float:
        distances = []
        for index, point in enumerate(vertices):
            nxt = vertices[(index + 1) % len(vertices)]
            edge = nxt - point
            cross = edge[0] * (-point[1]) - edge[1] * (-point[0])
            distances.append(abs(cross) / np.linalg.norm(edge))
        return float(min(distances))

    @staticmethod
    def _inside_polygon(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
        inside = np.zeros(len(points), dtype=bool)
        x, y = points[:, 0], points[:, 1]
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            crossing = ((yi > y) != (yj > y)) & (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            )
            inside ^= crossing
            j = i
        return inside

    @classmethod
    def _interior_samples(cls, footprint: np.ndarray, spacing: float) -> np.ndarray:
        xs = np.arange(footprint[:, 0].min(), footprint[:, 0].max() + spacing * 0.5, spacing)
        ys = np.arange(footprint[:, 1].min(), footprint[:, 1].max() + spacing * 0.5, spacing)
        grid = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
        interior = grid[cls._inside_polygon(grid, footprint)]
        return np.vstack((interior, footprint))

    def _center_indices(self, poses: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        points = poses[:, :2]
        cells = self.map.world_to_grid(points)
        rows, cols = cells[:, 0], cells[:, 1]
        inside = (
            (rows >= 0)
            & (rows < self.map.height)
            & (cols >= 0)
            & (cols < self.map.width)
        )
        return rows, cols, inside

    def clearance_at(self, poses: np.ndarray) -> np.ndarray:
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        rows, cols, inside = self._center_indices(poses)
        result = np.zeros(len(poses), dtype=np.float32)
        result[inside] = self.distance[rows[inside], cols[inside]]
        return result

    def collides(self, poses: np.ndarray) -> np.ndarray:
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        rows, cols, inside = self._center_indices(poses)
        collision = ~inside
        valid_indices = np.flatnonzero(inside)
        if not len(valid_indices):
            return collision

        clearance = self.distance[rows[valid_indices], cols[valid_indices]]
        definitely_colliding = clearance < self.inscribed_radius
        collision[valid_indices[definitely_colliding]] = True
        definitely_safe = clearance >= self.circumscribed_radius + self.map.resolution
        ambiguous_indices = valid_indices[~(definitely_colliding | definitely_safe)]
        if not len(ambiguous_indices):
            return collision

        ambiguous = poses[ambiguous_indices]
        cos_t = np.cos(ambiguous[:, 2])[:, None]
        sin_t = np.sin(ambiguous[:, 2])[:, None]
        sx = self.samples[:, 0][None, :]
        sy = self.samples[:, 1][None, :]
        world_x = ambiguous[:, 0, None] + cos_t * sx - sin_t * sy
        world_y = ambiguous[:, 1, None] + sin_t * sx + cos_t * sy
        ox, oy, _ = self.map.origin
        sample_cols = np.rint((world_x - ox) / self.map.resolution - 0.5).astype(int)
        sample_rows = np.rint(
            self.map.height - 0.5 - (world_y - oy) / self.map.resolution
        ).astype(int)
        sample_inside = (
            (sample_rows >= 0)
            & (sample_rows < self.map.height)
            & (sample_cols >= 0)
            & (sample_cols < self.map.width)
        )
        sample_collision = ~sample_inside
        good = sample_inside
        sample_collision[good] |= ~self.map.free[sample_rows[good], sample_cols[good]]
        collision[ambiguous_indices] = np.any(sample_collision, axis=1)
        return collision

    def trajectories_collide(self, trajectories: np.ndarray) -> np.ndarray:
        shape = trajectories.shape
        collisions = self.collides(trajectories.reshape(-1, 3)).reshape(shape[0], shape[1])
        return np.any(collisions, axis=1)

    def transformed_footprint(self, pose: np.ndarray) -> np.ndarray:
        x, y, theta = (float(v) for v in pose)
        rotation = np.asarray(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )
        return self.footprint @ rotation.T + np.asarray([x, y])
