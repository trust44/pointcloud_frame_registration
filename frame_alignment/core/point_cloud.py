"""Immutable source/reference cloud storage and file adapters."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Cloud:
    points: object
    colors: object = None

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.points.ndim != 2 or self.points.shape[1] != 3:
            raise ValueError("Point cloud must have shape (N, 3)")
        if self.colors is not None:
            self.colors = np.asarray(self.colors, dtype=np.float64)
            if self.colors.shape != self.points.shape:
                raise ValueError("Colors must have shape (N, 3)")

    def roi(self, center, radius):
        center = np.asarray(center, dtype=np.float64)
        keep = np.sum((self.points - center) ** 2, axis=1) <= float(radius) ** 2
        return Cloud(self.points[keep], None if self.colors is None else self.colors[keep])

    def voxel(self, size):
        size = float(size)
        if size <= 0 or len(self.points) == 0:
            return Cloud(self.points.copy(), None if self.colors is None else self.colors.copy())
        _, indices = np.unique(np.floor(self.points / size).astype(np.int64), axis=0, return_index=True)
        indices.sort()
        return Cloud(self.points[indices], None if self.colors is None else self.colors[indices])


def read_cloud(path):
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix in (".las", ".laz"):
        import laspy
        las = laspy.read(str(path))
        colors = None
        if all(hasattr(las, name) for name in ("red", "green", "blue")):
            colors = np.column_stack((las.red, las.green, las.blue)).astype(np.float64)
            if colors.size and colors.max() > 0:
                colors /= colors.max()
        return Cloud(np.column_stack((las.x, las.y, las.z)), colors)
    if suffix not in (".pcd", ".ply"):
        raise ValueError("Supported point clouds: PCD, PLY, LAS, LAZ")
    import open3d as o3d
    loaded = o3d.io.read_point_cloud(str(path))
    if loaded.is_empty():
        raise ValueError("Point cloud is empty: {}".format(path))
    colors = np.asarray(loaded.colors).copy() if loaded.has_colors() else None
    return Cloud(np.asarray(loaded.points).copy(), colors)

