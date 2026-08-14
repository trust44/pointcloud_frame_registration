"""Atomic, frame-stem based YAML and optional adjusted-PCD export."""
import os
from dataclasses import dataclass
from pathlib import Path
import uuid

import numpy as np
import yaml


@dataclass(frozen=True)
class AlignmentExportState:
    frame_data: object
    manual_delta: object
    manual_transform: object
    corrected_pose: object
    adjusted_points: object
    quality: object = None
    adjusted_colors: object = None


@dataclass(frozen=True)
class ExportOutcome:
    yaml_path: Path
    pcd_path: object = None
    pcd_error: object = None


def _value(source, name):
    if isinstance(source, dict):
        return source[name]
    return getattr(source, name)


def _temporary_path(destination, suffix):
    return destination.parent / (".{}.{}.{}".format(destination.name, uuid.uuid4().hex, suffix))


def _stage_yaml(destination, payload):
    temporary = _temporary_path(destination, "tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as handle:
            yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)
            handle.flush()
            os.fsync(handle.fileno())
        return temporary
    except Exception:
        if temporary.exists():
            temporary.unlink()
        raise


def _default_cloud_writer(path, adjusted_points, adjusted_colors=None):
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(adjusted_points, dtype=np.float64))
    if adjusted_colors is not None:
        colors = np.asarray(adjusted_colors, dtype=np.float64)
        if colors.shape != np.asarray(adjusted_points).shape:
            raise ValueError("Adjusted colors must match adjusted points")
        cloud.colors = o3d.utility.Vector3dVector(colors)
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise IOError("Failed to write adjusted point cloud: {}".format(path))


def _call_cloud_writer(writer, path, points, colors):
    if writer is _default_cloud_writer:
        writer(path, points, colors)
    else:
        writer(path, points)


def _commit_staged(staged):
    """Commit staged files as a rollback-capable group."""
    backups = []
    committed = []
    try:
        for _, destination in staged:
            if destination.exists():
                backup = _temporary_path(destination, "bak")
                os.replace(str(destination), str(backup))
                backups.append((destination, backup))
        for temporary, destination in staged:
            os.replace(str(temporary), str(destination))
            committed.append(destination)
    except Exception:
        for destination in reversed(committed):
            if destination.exists():
                destination.unlink()
        for destination, backup in reversed(backups):
            if backup.exists():
                os.replace(str(backup), str(destination))
        raise
    finally:
        for temporary, _ in staged:
            if temporary.exists():
                temporary.unlink()
    for _, backup in backups:
        if backup.exists():
            backup.unlink()


def export_result(request, state, cloud_writer=None, overwrite=False):
    request = request.validate()
    frame = state.frame_data
    frame_cloud_path = Path(frame.frame_cloud_path).resolve()
    frame_stem = frame_cloud_path.stem
    yaml_path = Path(request.yaml_output_dir) / (frame_stem + ".yaml")
    requested_pcd_path = None
    if request.write_adjusted_pcd:
        requested_pcd_path = Path(request.pcd_output_dir) / (frame_stem + ".pcd")
    existing = [path for path in (yaml_path, requested_pcd_path) if path is not None and path.exists()]
    if existing and not overwrite:
        raise FileExistsError("Output already exists: {}".format(", ".join(map(str, existing))))

    writer = cloud_writer or _default_cloud_writer
    staged_pcd = None
    written_pcd_path = None
    pcd_error = None
    if requested_pcd_path is not None:
        staged_pcd = _temporary_path(requested_pcd_path, "tmp.pcd")
        try:
            _call_cloud_writer(writer, staged_pcd, state.adjusted_points, state.adjusted_colors)
            if not staged_pcd.is_file():
                raise IOError("PCD writer did not create {}".format(staged_pcd))
            written_pcd_path = requested_pcd_path
        except Exception as exc:
            pcd_error = str(exc)
            if staged_pcd.exists():
                staged_pcd.unlink()
            staged_pcd = None

    initial_pose = np.asarray(frame.initial_pose, dtype=np.float64)
    delta = state.manual_delta
    payload = {
        "frame_id": frame_stem,
        "input": {
            "global_map_path": str(Path(frame.global_map_path).resolve()),
            "frame_cloud_map_path": str(frame_cloud_path),
            "initial_pose_path": str(Path(frame.initial_pose_path).resolve()),
        },
        "initial_T_map_lidar": initial_pose.tolist(),
        "manual_delta_about_lidar_origin": {
            "translation_frame": "map",
            "rotation_axes_frame": "map",
            "pivot_initial_lidar_origin_map": initial_pose[:3, 3].tolist(),
            "dx_m": float(_value(delta, "dx_m")),
            "dy_m": float(_value(delta, "dy_m")),
            "dz_m": float(_value(delta, "dz_m")),
            "roll_deg": float(_value(delta, "roll_deg")),
            "pitch_deg": float(_value(delta, "pitch_deg")),
            "yaw_deg": float(_value(delta, "yaw_deg")),
        },
        "T_manual_map": np.asarray(state.manual_transform, dtype=np.float64).tolist(),
        "corrected_T_map_lidar": np.asarray(state.corrected_pose, dtype=np.float64).tolist(),
        "quality": state.quality or {},
        "output": {
            "adjusted_pcd_written": written_pcd_path is not None,
            "adjusted_pcd_path": None if written_pcd_path is None else str(written_pcd_path.resolve()),
        },
    }
    staged_yaml = None
    try:
        staged_yaml = _stage_yaml(yaml_path, payload)
        staged = [(staged_yaml, yaml_path)]
        if staged_pcd is not None:
            staged.append((staged_pcd, requested_pcd_path))
        _commit_staged(staged)
    except Exception:
        for temporary in (staged_yaml, staged_pcd):
            if temporary is not None and temporary.exists():
                temporary.unlink()
        raise
    return ExportOutcome(yaml_path, written_pcd_path, pcd_error)
