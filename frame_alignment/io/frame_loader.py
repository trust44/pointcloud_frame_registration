"""Strict frame-id based loading with reusable global-map cache."""
from pathlib import Path

import numpy as np
import yaml

from ..contracts import FrameData
from .map_anchor import pose_in_map_coordinates
from .pose_parser import parse_tr_velo_to_map, validate_rigid_transform


def _require_non_empty(cloud, path):
    if cloud is None or not hasattr(cloud, "points") or len(cloud.points) == 0:
        raise ValueError("Point cloud is empty: {}".format(path))
    return cloud


class FrameLoader:
    def __init__(self, cloud_reader):
        self._cloud_reader = cloud_reader
        self._cached_global_path = None
        self._cached_global = None

    def load_frame(self, request):
        paths = request.resolve_existing_paths()
        if paths.global_map_file != self._cached_global_path:
            global_map = _require_non_empty(
                self._cloud_reader(paths.global_map_file), paths.global_map_file)
        else:
            global_map = self._cached_global
        source = _require_non_empty(
            self._cloud_reader(paths.frame_cloud_file), paths.frame_cloud_file)
        initial_pose = parse_tr_velo_to_map(paths.initial_pose_file)
        initial_pose = pose_in_map_coordinates(initial_pose, paths.global_map_file)
        if paths.global_map_file != self._cached_global_path:
            self._cached_global_path = paths.global_map_file
            self._cached_global = global_map
        return FrameData(
            paths.frame_id,
            paths.global_map_file,
            paths.frame_cloud_file,
            paths.initial_pose_file,
            global_map,
            source,
            initial_pose,
        )


def load_exported_corrected_pose(path):
    """Read ``corrected_T_map_lidar`` from this application's exported YAML."""
    path = Path(path).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8-sig") as handle:
            payload = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise ValueError("Invalid registered pose YAML: {}".format(path)) from exc
    if not isinstance(payload, dict) or "corrected_T_map_lidar" not in payload:
        raise ValueError("Registered pose YAML lacks corrected_T_map_lidar: {}".format(path))
    try:
        return validate_rigid_transform(np.asarray(payload["corrected_T_map_lidar"], dtype=np.float64))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid corrected_T_map_lidar in {}".format(path)) from exc


def robust_cloud_center(cloud):
    """Return the coordinate-wise median of a non-empty registered cloud."""
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or not np.all(np.isfinite(points)):
        raise ValueError("Registered cloud requires finite Nx3 points for fallback centre")
    return np.median(points, axis=0)


def fallback_review_pose(cloud):
    """Build an identity-orientation pose centred on the registered cloud median."""
    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = robust_cloud_center(cloud)
    return pose


class ReviewFrameLoader(FrameLoader):
    """Load already-registered map clouds with their exported corrected poses."""

    def load_frame(self, request):
        paths = request.resolve_existing_paths()
        if paths.global_map_file != self._cached_global_path:
            global_map = _require_non_empty(
                self._cloud_reader(paths.global_map_file), paths.global_map_file)
        else:
            global_map = self._cached_global
        source = _require_non_empty(
            self._cloud_reader(paths.registered_cloud_file), paths.registered_cloud_file)
        corrected_pose = (
            load_exported_corrected_pose(paths.registered_pose_file)
            if paths.registered_pose_file is not None else fallback_review_pose(source)
        )
        if paths.global_map_file != self._cached_global_path:
            self._cached_global_path = paths.global_map_file
            self._cached_global = global_map
        return FrameData(
            paths.frame_id,
            paths.global_map_file,
            paths.registered_cloud_file,
            paths.registered_pose_file,
            global_map,
            source,
            corrected_pose,
        )
