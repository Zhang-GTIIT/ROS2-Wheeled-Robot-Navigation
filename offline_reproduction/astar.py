from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import label


SQRT2 = math.sqrt(2.0)
MOVES = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, SQRT2),
    (-1, 1, SQRT2),
    (1, -1, SQRT2),
    (1, 1, SQRT2),
)


@dataclass(frozen=True)
class AStarResult:
    path_cells: np.ndarray
    expanded_nodes: int
    planning_time_s: float
    path_length_pixels: float


def _octile(a: tuple[int, int], b: tuple[int, int]) -> float:
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def astar_grid(
    traversable: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> AStarResult:
    height, width = traversable.shape
    if not traversable[start] or not traversable[goal]:
        raise ValueError("Start and goal must both be traversable")

    started = time.perf_counter()
    start_id = start[0] * width + start[1]
    goal_id = goal[0] * width + goal[1]
    g_score = np.full(height * width, np.inf, dtype=np.float32)
    g_score[start_id] = 0.0
    parent: dict[int, int] = {}
    closed = np.zeros(height * width, dtype=bool)
    queue: list[tuple[float, float, int, int, int]] = []
    counter = 0
    heapq.heappush(queue, (_octile(start, goal), 0.0, counter, start[0], start[1]))
    expanded = 0

    while queue:
        _, current_g, _, row, col = heapq.heappop(queue)
        current_id = row * width + col
        if closed[current_id]:
            continue
        closed[current_id] = True
        expanded += 1
        if current_id == goal_id:
            break

        for dr, dc, step_cost in MOVES:
            nr, nc = row + dr, col + dc
            if nr < 0 or nr >= height or nc < 0 or nc >= width or not traversable[nr, nc]:
                continue
            if dr and dc and not (traversable[row + dr, col] and traversable[row, col + dc]):
                continue
            neighbor_id = nr * width + nc
            if closed[neighbor_id]:
                continue
            tentative = current_g + step_cost
            if tentative + 1e-7 < float(g_score[neighbor_id]):
                g_score[neighbor_id] = tentative
                parent[neighbor_id] = current_id
                counter += 1
                priority = tentative + _octile((nr, nc), goal)
                heapq.heappush(queue, (priority, tentative, counter, nr, nc))

    if not closed[goal_id]:
        raise RuntimeError("A* could not connect the selected start and goal")

    ids = [goal_id]
    while ids[-1] != start_id:
        ids.append(parent[ids[-1]])
    ids.reverse()
    path = np.asarray([(node // width, node % width) for node in ids], dtype=np.int32)
    deltas = np.diff(path.astype(float), axis=0)
    length_pixels = float(np.linalg.norm(deltas, axis=1).sum())
    return AStarResult(
        path_cells=path,
        expanded_nodes=expanded,
        planning_time_s=time.perf_counter() - started,
        path_length_pixels=length_pixels,
    )


def select_far_endpoints(
    traversable: np.ndarray,
    clearance_m: np.ndarray,
    selection_clearance_m: float,
) -> tuple[tuple[int, int], tuple[int, int], int]:
    labels, count = label(traversable, structure=np.ones((3, 3), dtype=np.uint8))
    if count == 0:
        raise RuntimeError("No traversable component remains after footprint inflation")
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    component_id = int(np.argmax(sizes))
    candidates = np.argwhere(
        (labels == component_id) & (clearance_m >= selection_clearance_m)
    )
    if len(candidates) < 2:
        candidates = np.argwhere(labels == component_id)

    first = candidates[len(candidates) // 2]
    for _ in range(3):
        distance = np.sum((candidates - first) ** 2, axis=1)
        second = candidates[int(np.argmax(distance))]
        distance = np.sum((candidates - second) ** 2, axis=1)
        first = candidates[int(np.argmax(distance))]

    # Present the route from the lower part of the map toward the upper part.
    if first[0] < second[0]:
        first, second = second, first
    return tuple(int(v) for v in first), tuple(int(v) for v in second), int(sizes[component_id])

