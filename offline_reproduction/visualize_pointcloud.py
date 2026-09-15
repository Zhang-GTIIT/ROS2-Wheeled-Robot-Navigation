from __future__ import annotations

import argparse
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

from config import (
    POINTCLOUD_RENDER_LIMIT,
    POINTCLOUD_VOXEL_SIZE_M,
    POINTCLOUD_ZIP_MEMBER,
    REPO_ROOT,
    find_source_zip,
)
from run_astar_demo import save_json


def _numpy_scalar_type(type_code: str, size: int) -> str:
    mapping = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "<i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "<u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    try:
        return mapping[(type_code.upper(), int(size))]
    except KeyError as exc:
        raise ValueError(f"Unsupported PCD field type/size: {type_code}{size}") from exc


def _read_header(stream) -> dict[str, list[str]]:
    header: dict[str, list[str]] = {}
    while True:
        raw = stream.readline()
        if not raw:
            raise ValueError("PCD DATA line was not found")
        line = raw.decode("ascii", errors="strict").strip()
        if not line or line.startswith("#"):
            continue
        key, *values = line.split()
        header[key.upper()] = values
        if key.upper() == "DATA":
            break
    return header


def _pcd_dtype(header: dict[str, list[str]]) -> np.dtype:
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    description = []
    for name, size, type_code, count in zip(fields, sizes, types, counts):
        scalar = _numpy_scalar_type(type_code, size)
        description.append((name, scalar) if count == 1 else (name, scalar, (count,)))
    return np.dtype(description)


def load_and_voxelize(
    source_zip: Path,
    voxel_size_m: float = POINTCLOUD_VOXEL_SIZE_M,
    chunk_points: int = 500_000,
) -> tuple[np.ndarray, dict]:
    reduced_chunks: list[np.ndarray] = []
    bbox_min = np.full(3, np.inf, dtype=float)
    bbox_max = np.full(3, -np.inf, dtype=float)
    finite_count = 0
    with zipfile.ZipFile(source_zip) as archive:
        info = archive.getinfo(POINTCLOUD_ZIP_MEMBER)
        with archive.open(info) as stream:
            header = _read_header(stream)
            if header["DATA"][0].lower() != "binary":
                raise ValueError(
                    f"Expected binary PCD for {POINTCLOUD_ZIP_MEMBER}; "
                    f"found {header['DATA'][0]}"
                )
            dtype = _pcd_dtype(header)
            point_count = int(header.get("POINTS", [header["WIDTH"][0]])[0])
            read_count = 0
            while read_count < point_count:
                request = min(chunk_points, point_count - read_count)
                raw = stream.read(request * dtype.itemsize)
                if len(raw) % dtype.itemsize:
                    raise ValueError("PCD binary payload ends inside a point record")
                records = np.frombuffer(raw, dtype=dtype)
                if not len(records):
                    break
                read_count += len(records)
                xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(
                    np.float32, copy=False
                )
                xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
                if not len(xyz):
                    continue
                finite_count += len(xyz)
                bbox_min = np.minimum(bbox_min, xyz.min(axis=0))
                bbox_max = np.maximum(bbox_max, xyz.max(axis=0))
                voxel = np.floor(xyz / voxel_size_m).astype(np.int32)
                _, representative = np.unique(voxel, axis=0, return_index=True)
                reduced_chunks.append(xyz[representative])
            if read_count != point_count:
                raise ValueError(
                    f"PCD header declares {point_count:,} points, read {read_count:,}"
                )

    first_pass = np.concatenate(reduced_chunks, axis=0)
    voxel = np.floor(first_pass / voxel_size_m).astype(np.int32)
    _, representative = np.unique(voxel, axis=0, return_index=True)
    downsampled = first_pass[representative]
    metadata = {
        "source_filename": Path(POINTCLOUD_ZIP_MEMBER).name,
        "source_member": POINTCLOUD_ZIP_MEMBER,
        "source_archive": source_zip.name,
        "pcd_data_encoding": header["DATA"][0],
        "pcd_fields": header["FIELDS"],
        "pcd_header_points": point_count,
        "finite_input_points": int(finite_count),
        "voxel_size_m": float(voxel_size_m),
        "downsampled_points": int(len(downsampled)),
        "retained_fraction": float(len(downsampled) / max(finite_count, 1)),
        "bounding_box_m": {
            "minimum_xyz": [float(v) for v in bbox_min],
            "maximum_xyz": [float(v) for v in bbox_max],
        },
        "compressed_member_size_bytes": int(info.compress_size),
        "uncompressed_member_size_bytes": int(info.file_size),
    }
    return downsampled, metadata


def render_pointcloud(points: np.ndarray, metadata: dict, output_path: Path) -> None:
    render_count = min(len(points), POINTCLOUD_RENDER_LIMIT)
    selection = np.linspace(0, len(points) - 1, render_count, dtype=int)
    shown = points[selection]

    # Trim only extreme plotting outliers so the building structure remains legible.
    low = np.percentile(shown, 0.2, axis=0)
    high = np.percentile(shown, 99.8, axis=0)
    visible = np.all((shown >= low) & (shown <= high), axis=1)
    shown = shown[visible]
    metadata["rendered_points"] = int(len(shown))
    metadata["render_percentile_window"] = [0.2, 99.8]

    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    marks = ax.scatter(
        shown[:, 0],
        shown[:, 1],
        shown[:, 2],
        c=shown[:, 2],
        cmap="viridis",
        s=0.35,
        alpha=0.78,
        linewidths=0,
        rasterized=True,
    )
    colorbar = fig.colorbar(marks, ax=ax, shrink=0.67, pad=0.03)
    colorbar.set_label("Height z (m)")
    ax.set_title("Voxel-downsampled Real SLAM Point Cloud: scans0120gf.pcd", loc="left", weight="bold")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.zaxis.set_major_locator(MaxNLocator(4))
    ax.view_init(elev=63, azim=-54)
    spans = np.maximum(high - low, 1e-3)
    ax.set_xlim(low[0], high[0])
    ax.set_ylim(low[1], high[1])
    ax.set_zlim(low[2], high[2])
    ax.set_box_aspect(spans)
    ax.text2D(
        0.99,
        0.98,
        "OFFLINE REPRODUCTION",
        transform=ax.transAxes,
        ha="right",
        va="top",
        color="#9b2c2c",
        weight="bold",
    )
    ax.text2D(
        0.01,
        0.04,
        "Real SLAM point-cloud map — visualization only",
        transform=ax.transAxes,
        fontsize=9,
        weight="bold",
    )
    ax.text2D(
        0.01,
        0.015,
        f"{metadata['finite_input_points']:,} source points  →  "
        f"{metadata['downsampled_points']:,} voxels at {metadata['voxel_size_m']:.2f} m",
        transform=ax.transAxes,
        fontsize=9,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def generate_pointcloud_artifacts(source_zip: Path, repo_root: Path = REPO_ROOT) -> dict:
    points, metrics = load_and_voxelize(source_zip)
    metrics.update(
        {
            "artifact": "assets/navigation/02_slam_pointcloud.png",
            "label": "OFFLINE REPRODUCTION",
            "method": "streaming NumPy voxel representative selection",
        }
    )
    output_path = repo_root / "assets/navigation/02_slam_pointcloud.png"
    render_pointcloud(points, metrics, output_path)
    save_json(
        repo_root / "results/offline_reproduction/pointcloud_metrics.json", metrics
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Render the real SLAM PCD with voxel downsampling")
    parser.add_argument("--source-zip", type=Path, help="Path to the original wzb.zip")
    args = parser.parse_args()
    metrics = generate_pointcloud_artifacts(find_source_zip(args.source_zip))
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
