from __future__ import annotations

import io
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image


@dataclass(frozen=True)
class OccupancyMap:
    pixels: np.ndarray
    occupied: np.ndarray
    free: np.ndarray
    unknown: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    occupied_thresh: float
    free_thresh: float
    negate: bool
    yaml_path: Path
    image_name: str
    zip_member: str

    @property
    def height(self) -> int:
        return int(self.pixels.shape[0])

    @property
    def width(self) -> int:
        return int(self.pixels.shape[1])

    @property
    def extent(self) -> tuple[float, float, float, float]:
        ox, oy, _ = self.origin
        return (
            ox,
            ox + self.width * self.resolution,
            oy,
            oy + self.height * self.resolution,
        )

    def grid_to_world(self, cells: np.ndarray) -> np.ndarray:
        cells = np.asarray(cells, dtype=float)
        rows = cells[..., 0]
        cols = cells[..., 1]
        ox, oy, _ = self.origin
        x = ox + (cols + 0.5) * self.resolution
        y = oy + (self.height - rows - 0.5) * self.resolution
        return np.stack((x, y), axis=-1)

    def world_to_grid(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=float)
        ox, oy, _ = self.origin
        cols = np.floor((points[..., 0] - ox) / self.resolution).astype(int)
        rows = self.height - 1 - np.floor((points[..., 1] - oy) / self.resolution).astype(int)
        return np.stack((rows, cols), axis=-1)

    def metadata(self) -> dict[str, Any]:
        return {
            "image": self.image_name,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": list(self.origin),
            "occupied_thresh": self.occupied_thresh,
            "free_thresh": self.free_thresh,
            "negate": int(self.negate),
            "mode": "trinary",
        }


def load_occupancy_map(yaml_path: Path, source_zip: Path, zip_member: str) -> OccupancyMap:
    with Path(yaml_path).open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)

    image_path = Path(yaml_path).parent / str(metadata["image"])
    if image_path.is_file():
        image = Image.open(image_path).convert("L")
    else:
        with zipfile.ZipFile(source_zip) as archive:
            raw = archive.read(zip_member)
        image = Image.open(io.BytesIO(raw)).convert("L")

    pixels = np.asarray(image, dtype=np.uint8)
    resolution = float(metadata["resolution"])
    origin = tuple(float(value) for value in metadata["origin"])
    occupied_thresh = float(metadata["occupied_thresh"])
    free_thresh = float(metadata["free_thresh"])
    negate = bool(int(metadata.get("negate", 0)))

    normalized = pixels.astype(np.float32) / 255.0
    occupancy_probability = normalized if negate else 1.0 - normalized
    occupied = occupancy_probability > occupied_thresh
    free = occupancy_probability < free_thresh
    unknown = ~(occupied | free)

    expected_width = metadata.get("width")
    expected_height = metadata.get("height")
    if expected_width is not None and int(expected_width) != pixels.shape[1]:
        raise ValueError("PGM width does not match the YAML width")
    if expected_height is not None and int(expected_height) != pixels.shape[0]:
        raise ValueError("PGM height does not match the YAML height")

    return OccupancyMap(
        pixels=pixels,
        occupied=occupied,
        free=free,
        unknown=unknown,
        resolution=resolution,
        origin=origin,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
        negate=negate,
        yaml_path=Path(yaml_path),
        image_name=str(metadata["image"]),
        zip_member=zip_member,
    )

