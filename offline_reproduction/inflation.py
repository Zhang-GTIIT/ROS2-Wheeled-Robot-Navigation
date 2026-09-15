from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt

from map_loader import OccupancyMap


@dataclass(frozen=True)
class InflationResult:
    distance_to_occupied_m: np.ndarray
    distance_to_nonfree_m: np.ndarray
    inflation_zone: np.ndarray
    traversable: np.ndarray
    footprint_circumscribed_radius_m: float
    center_safety_radius_m: float


def footprint_circumscribed_radius(footprint: list[list[float]]) -> float:
    vertices = np.asarray(footprint, dtype=float)
    return float(np.linalg.norm(vertices, axis=1).max())


def build_inflation(
    occupancy_map: OccupancyMap,
    footprint: list[list[float]],
    inflation_radius_m: float,
) -> InflationResult:
    distance_to_occupied_m = distance_transform_edt(~occupancy_map.occupied) * occupancy_map.resolution
    distance_to_nonfree_m = distance_transform_edt(occupancy_map.free) * occupancy_map.resolution
    circumscribed = footprint_circumscribed_radius(footprint)
    center_safety_radius = max(float(inflation_radius_m), circumscribed)
    inflation_zone = (
        occupancy_map.free
        & (distance_to_occupied_m <= float(inflation_radius_m))
    )
    traversable = occupancy_map.free & (distance_to_nonfree_m >= center_safety_radius)
    return InflationResult(
        distance_to_occupied_m=distance_to_occupied_m,
        distance_to_nonfree_m=distance_to_nonfree_m,
        inflation_zone=inflation_zone,
        traversable=traversable,
        footprint_circumscribed_radius_m=circumscribed,
        center_safety_radius_m=center_safety_radius,
    )

