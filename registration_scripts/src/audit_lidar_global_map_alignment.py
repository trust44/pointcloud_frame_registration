#!/usr/bin/env python
"""Audit local lidar global exports against a source global point cloud."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pickle
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class RigidAlignmentResult:
    rotation: np.ndarray
    translation_centered_m: np.ndarray
    rmse_m: float


@dataclass(frozen=True)
class ValidDtmMask:
    mask_hw: np.ndarray
    origin_top_left_xy: tuple[float, float]
    resolution_xy: tuple[float, float]


def _as_xyz_array(name: str, value: np.ndarray) -> np.ndarray:
    xyz = np.asarray(value, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'{name} must have shape (N, 3), got {xyz.shape}')
    if not np.isfinite(xyz).all():
        raise ValueError(f'{name} contains NaN or Inf')
    return xyz


def read_ascii_pcd_xyz(path: Path) -> np.ndarray:
    """Read x/y/z columns from an ASCII PCD file."""
    path = Path(path)
    header: dict[str, list[str]] = {}
    data_line_index = None
    lines = path.read_text(encoding='utf-8').splitlines()
    for line_index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        parts = stripped.split()
        key = parts[0].upper()
        header[key] = parts[1:]
        if key == 'DATA':
            data_line_index = line_index + 1
            break
    if data_line_index is None:
        raise ValueError(f'{path} is missing DATA header')
    if header.get('DATA', [''])[0].lower() != 'ascii':
        raise ValueError(f'{path} must use DATA ascii')
    fields = header.get('FIELDS')
    if fields is None:
        raise ValueError(f'{path} is missing FIELDS header')
    for required in ('x', 'y', 'z'):
        if required not in fields:
            raise ValueError(f'{path} FIELDS must include x y z')
    xyz_indices = [fields.index(axis) for axis in ('x', 'y', 'z')]
    points = []
    for line in lines[data_line_index:]:
        stripped = line.strip()
        if not stripped:
            continue
        values = stripped.split()
        if len(values) < len(fields):
            raise ValueError(f'{path} data row has too few columns: {line}')
        points.append([float(values[index]) for index in xyz_indices])
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim == 1:
        xyz = xyz.reshape(0, 3)
    expected_points = header.get('POINTS')
    if expected_points is not None and int(expected_points[0]) != xyz.shape[0]:
        raise ValueError(
            f'{path} POINTS={expected_points[0]} but read {xyz.shape[0]}')
    return _as_xyz_array('pcd_xyz', xyz)


def write_ascii_pcd_xyz(path: Path, xyz: np.ndarray) -> None:
    """Write x/y/z points as an ASCII PCD file."""
    path = Path(path)
    xyz = _as_xyz_array('xyz', xyz)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {xyz.shape[0]}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {xyz.shape[0]}\n')
        f.write('DATA ascii\n')
        for point in xyz:
            f.write(' '.join(f'{float(v):.6f}' for v in point) + '\n')


def compute_centered_rigid_alignment(
    source_xyz: np.ndarray,
    target_xyz: np.ndarray,
    center_xyz: np.ndarray,
) -> RigidAlignmentResult:
    """Estimate target ~= R @ source + t after subtracting center_xyz.

    The transform is rigid: scale is fixed to 1. Source and target must have
    explicit point correspondence.
    """
    source_xyz = _as_xyz_array('source_xyz', source_xyz)
    target_xyz = _as_xyz_array('target_xyz', target_xyz)
    if source_xyz.shape != target_xyz.shape:
        raise ValueError(
            'source_xyz and target_xyz must have the same shape, got '
            f'{source_xyz.shape} and {target_xyz.shape}')
    if source_xyz.shape[0] < 3:
        raise ValueError('at least 3 points are required for rigid alignment')
    center_xyz = np.asarray(center_xyz, dtype=np.float64)
    if center_xyz.shape != (3, ):
        raise ValueError(
            f'center_xyz must have shape (3,), got {center_xyz.shape}')
    source_centered = source_xyz - center_xyz
    target_centered = target_xyz - center_xyz
    source_mean = source_centered.mean(axis=0)
    target_mean = target_centered.mean(axis=0)
    source_zero = source_centered - source_mean
    target_zero = target_centered - target_mean
    covariance = source_zero.T @ target_zero
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0:
        vt_matrix[-1, :] *= -1
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_mean - rotation @ source_mean
    aligned = (rotation @ source_centered.T).T + translation
    residual = np.linalg.norm(aligned - target_centered, axis=1)
    rmse = float(np.sqrt(np.mean(residual**2)))
    return RigidAlignmentResult(
        rotation=rotation.astype(np.float64, copy=False),
        translation_centered_m=translation.astype(np.float64, copy=False),
        rmse_m=rmse)


def compute_nearest_residual_stats(source_xyz: np.ndarray,
                                   target_xyz: np.ndarray) -> dict:
    """Compute nearest-neighbor residual distribution from source to target."""
    source_xyz = _as_xyz_array('source_xyz', source_xyz)
    target_xyz = _as_xyz_array('target_xyz', target_xyz)
    if source_xyz.shape[0] == 0 or target_xyz.shape[0] == 0:
        raise ValueError('source_xyz and target_xyz must be non-empty')
    distances, _ = cKDTree(target_xyz).query(source_xyz, k=1)
    distances = np.asarray(distances, dtype=np.float64)
    return {
        'count': int(distances.shape[0]),
        'mean_m': float(np.mean(distances)),
        'rmse_m': float(np.sqrt(np.mean(distances**2))),
        'p50_m': float(np.percentile(distances, 50)),
        'p95_m': float(np.percentile(distances, 95)),
        'max_m': float(np.max(distances)),
    }


def decompose_rotation_zyx_deg(rotation: np.ndarray) -> tuple[float, float,
                                                              float]:
    """Return yaw, pitch, roll in degrees for a ZYX rotation convention."""
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError(f'rotation must have shape (3, 3), got {rotation.shape}')
    yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    pitch = math.degrees(
        math.atan2(-rotation[2, 0],
                   math.sqrt(rotation[2, 1]**2 + rotation[2, 2]**2)))
    roll = math.degrees(math.atan2(rotation[2, 1], rotation[2, 2]))
    return float(yaw), float(pitch), float(roll)


def compute_global_rotation_from_local_correction(
        local_correction_rotation: np.ndarray,
        lidar2global_rotation: np.ndarray) -> np.ndarray:
    """Convert a local-frame correction into the global/map frame."""
    local_correction_rotation = np.asarray(
        local_correction_rotation, dtype=np.float64)
    lidar2global_rotation = np.asarray(lidar2global_rotation, dtype=np.float64)
    if local_correction_rotation.shape != (3, 3):
        raise ValueError(
            'local_correction_rotation must have shape (3, 3), got '
            f'{local_correction_rotation.shape}')
    if lidar2global_rotation.shape != (3, 3):
        raise ValueError(
            'lidar2global_rotation must have shape (3, 3), got '
            f'{lidar2global_rotation.shape}')
    return (lidar2global_rotation @ local_correction_rotation
            @ lidar2global_rotation.T)


def _rotation_from_zyx_deg(yaw_deg: float, pitch_deg: float,
                           roll_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
                  dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
                  dtype=np.float64)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
                  dtype=np.float64)
    return rz @ ry @ rx


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    trace_value = float(np.trace(rotation))
    cos_value = max(-1.0, min(1.0, (trace_value - 1.0) / 2.0))
    return float(math.degrees(math.acos(cos_value)))


def _load_manifest(path: Path) -> dict[str, dict]:
    with Path(path).open('r', encoding='utf-8', newline='') as f:
        return {str(row['sample_idx']): row for row in csv.DictReader(f)}


def _load_info_by_sample_idx(path: Path) -> dict[str, dict]:
    with Path(path).open('rb') as f:
        data = pickle.load(f)
    data_list = data.get('data_list') if isinstance(data, dict) else data
    if not isinstance(data_list, list):
        raise ValueError(f'{path} does not contain data_list')
    rows = {}
    for row in data_list:
        if row.get('sample_idx') is not None:
            rows[str(row['sample_idx'])] = row
    return rows


def _load_info_rows(path: Path) -> list[dict]:
    with Path(path).open('rb') as f:
        data = pickle.load(f)
    data_list = data.get('data_list') if isinstance(data, dict) else data
    if not isinstance(data_list, list):
        raise ValueError(f'{path} does not contain data_list')
    return data_list


def _matrix4(name: str, value) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f'{name} must have shape (4, 4), got {matrix.shape}')
    return matrix


def _voxel_down_sample_xyz(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0:
        return xyz
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError('Open3D is required for voxel downsampling') from exc
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    return np.asarray(pcd.voxel_down_sample(voxel_size).points,
                      dtype=np.float64)


def _crop_global_map_xyz(global_map_xyz: np.ndarray, source_xyz: np.ndarray,
                         crop_margin: float) -> np.ndarray:
    min_xyz = source_xyz.min(axis=0) - crop_margin
    max_xyz = source_xyz.max(axis=0) + crop_margin
    mask = np.all((global_map_xyz >= min_xyz) & (global_map_xyz <= max_xyz),
                  axis=1)
    return global_map_xyz[mask]


def _run_open3d_icp(source_xyz: np.ndarray, target_xyz: np.ndarray,
                    threshold: float) -> tuple[np.ndarray, float, float]:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError('Open3D is required for ICP audit') from exc
    source_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_xyz))
    target_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_xyz))
    reg = o3d.pipelines.registration.registration_icp(
        source_pcd,
        target_pcd,
        threshold,
        np.eye(4, dtype=np.float64),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))
    return (np.asarray(reg.transformation, dtype=np.float64),
            float(reg.fitness), float(reg.inlier_rmse))


def _apply_centered_transform(xyz: np.ndarray, center_xyz: np.ndarray,
                              transform: np.ndarray) -> np.ndarray:
    centered = xyz - center_xyz
    homog = np.ones((centered.shape[0], 4), dtype=np.float64)
    homog[:, :3] = centered
    transformed = (transform @ homog.T).T[:, :3]
    return transformed + center_xyz


def _yaw_from_rotation_deg(rotation: np.ndarray) -> float:
    return float(math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])))


def _identity_matrix4() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def _resolve_lidar_path(data_root: Path, lidar_path: str) -> Path:
    raw_path = Path(lidar_path)
    if raw_path.is_absolute():
        return raw_path
    candidates = [
        data_root / raw_path,
        data_root / 'training' / 'velodyne' / raw_path,
        data_root / 'velodyne' / raw_path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def load_lidar_points_from_info(data_root: Path, info_row: Mapping) -> np.ndarray:
    """Load a lidar bin referenced by a dataset info row.

    The returned array preserves all point features. Only the first three
    columns are interpreted as lidar-frame xyz by geometry helpers.
    """
    data_root = Path(data_root)
    lidar_info = info_row.get('lidar_points', {})
    lidar_path = (
        lidar_info.get('lidar_path') or lidar_info.get('filename')
        or info_row.get('lidar_path') or info_row.get('pts_filename'))
    if not lidar_path:
        raise KeyError('info_row does not contain a lidar path')
    num_features = int(
        lidar_info.get('num_pts_feats') or info_row.get('num_pts_feats') or 5)
    if num_features < 3:
        raise ValueError(f'num_pts_feats must be >= 3, got {num_features}')
    path = _resolve_lidar_path(data_root, str(lidar_path))
    points = np.fromfile(path, dtype=np.float32)
    if points.size % num_features != 0:
        raise ValueError(
            f'{path} contains {points.size} float32 values, not divisible by '
            f'num_pts_feats={num_features}')
    return points.reshape(-1, num_features).astype(np.float64, copy=False)


def transform_lidar_points_to_global(
    points_lidar: np.ndarray,
    info_row: Mapping,
    local_correction_rotation: np.ndarray | None = None,
) -> np.ndarray:
    """Transform lidar-frame points into the map/global frame."""
    points_lidar = np.asarray(points_lidar, dtype=np.float64)
    if points_lidar.ndim != 2 or points_lidar.shape[1] < 3:
        raise ValueError(
            'points_lidar must have shape (N, >=3), got '
            f'{points_lidar.shape}')
    if not np.isfinite(points_lidar[:, :3]).all():
        raise ValueError('points_lidar xyz contains NaN or Inf')
    lidar_info = info_row.get('lidar_points', {})
    lidar2ego = _matrix4('lidar2ego',
                         lidar_info.get('lidar2ego', _identity_matrix4()))
    ego2global = _matrix4('ego2global', info_row['ego2global'])
    xyz_lidar = points_lidar[:, :3]
    if local_correction_rotation is not None:
        local_correction_rotation = np.asarray(
            local_correction_rotation, dtype=np.float64)
        if local_correction_rotation.shape != (3, 3):
            raise ValueError(
                'local_correction_rotation must have shape (3, 3), got '
                f'{local_correction_rotation.shape}')
        xyz_lidar = (local_correction_rotation @ xyz_lidar.T).T
    homog = np.ones((points_lidar.shape[0], 4), dtype=np.float64)
    homog[:, :3] = xyz_lidar
    points_global_xyz = ((ego2global @ lidar2ego) @ homog.T).T[:, :3]
    output = points_lidar.copy()
    output[:, :3] = points_global_xyz
    return output


def apply_fixed_local_correction(
    source_xyz: np.ndarray,
    center_xyz: np.ndarray,
    local_correction_rotation: np.ndarray,
    lidar2global_rotation: np.ndarray,
) -> np.ndarray:
    """Apply a local-frame correction as a centered global/map rotation."""
    source_xyz = _as_xyz_array('source_xyz', source_xyz)
    center_xyz = np.asarray(center_xyz, dtype=np.float64)
    if center_xyz.shape != (3, ):
        raise ValueError(
            f'center_xyz must have shape (3,), got {center_xyz.shape}')
    global_correction = compute_global_rotation_from_local_correction(
        local_correction_rotation, lidar2global_rotation)
    return (global_correction @ (source_xyz - center_xyz).T).T + center_xyz


def _extract_ypr_deg(row: Mapping) -> np.ndarray:
    value = row['correction_local_yaw_pitch_roll_deg']
    if isinstance(value, Mapping):
        return np.asarray(
            [value['yaw_deg'], value['pitch_deg'], value['roll_deg']],
            dtype=np.float64)
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3, ):
        raise ValueError(
            'correction_local_yaw_pitch_roll_deg must have shape (3,), got '
            f'{array.shape}')
    return array


def summarize_candidate_correction(rows: Sequence[Mapping]) -> dict:
    """Summarize local yaw/pitch/roll corrections with robust statistics."""
    if not rows:
        return {
            'sample_count': 0,
            'robust_median_yaw_pitch_roll_deg': [],
            'mean_yaw_pitch_roll_deg': [],
            'std_yaw_pitch_roll_deg': [],
            'mad_yaw_pitch_roll_deg': [],
        }
    yprs = np.stack([_extract_ypr_deg(row) for row in rows], axis=0)
    median = np.median(yprs, axis=0)
    return {
        'sample_count': int(yprs.shape[0]),
        'robust_median_yaw_pitch_roll_deg': median.tolist(),
        'mean_yaw_pitch_roll_deg': yprs.mean(axis=0).tolist(),
        'std_yaw_pitch_roll_deg': yprs.std(axis=0).tolist(),
        'mad_yaw_pitch_roll_deg': np.median(np.abs(yprs - median),
                                           axis=0).tolist(),
    }


def make_structural_blind_angle_mask(
    points_lidar_xy: np.ndarray,
    structural_blind_angle_sectors_deg: Sequence[tuple[float, float]],
) -> np.ndarray:
    """Return True for lidar xy points inside known support-pillar blind sectors."""
    points_lidar_xy = np.asarray(points_lidar_xy, dtype=np.float64)
    if points_lidar_xy.ndim != 2 or points_lidar_xy.shape[1] != 2:
        raise ValueError(
            'points_lidar_xy must have shape (N, 2), got '
            f'{points_lidar_xy.shape}')
    angles = np.degrees(np.arctan2(points_lidar_xy[:, 1],
                                   points_lidar_xy[:, 0]))
    angles = np.mod(angles, 360.0)
    mask = np.zeros(points_lidar_xy.shape[0], dtype=bool)
    for start_deg, end_deg in structural_blind_angle_sectors_deg:
        start = float(start_deg) % 360.0
        end = float(end_deg) % 360.0
        if start <= end:
            mask |= (angles >= start) & (angles <= end)
        else:
            mask |= (angles >= start) | (angles <= end)
    return mask


def load_valid_dtm_mask(path: Path) -> ValidDtmMask:
    """Load an audited valid_dtm GeoTIFF mask and its map georeference."""
    try:
        import tifffile
    except ImportError as exc:
        raise RuntimeError('tifffile is required to read valid_dtm.tif') from exc
    path = Path(path)
    with tifffile.TiffFile(path) as tif:
        page = tif.pages[0]
        mask_hw = page.asarray().astype(bool)
        scale = page.tags['ModelPixelScaleTag'].value
        tiepoint = page.tags['ModelTiepointTag'].value
    if len(scale) < 2 or len(tiepoint) < 6:
        raise ValueError(f'{path} is missing GeoTIFF scale/tiepoint metadata')
    resolution_xy = (float(scale[0]), float(scale[1]))
    origin_top_left_xy = (float(tiepoint[3]), float(tiepoint[4]))
    return ValidDtmMask(
        mask_hw=mask_hw,
        origin_top_left_xy=origin_top_left_xy,
        resolution_xy=resolution_xy)


def _query_valid_dtm_points(points_global_xy: np.ndarray,
                            valid_dtm_mask: ValidDtmMask
                            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points_global_xy = np.asarray(points_global_xy, dtype=np.float64)
    if points_global_xy.ndim != 2 or points_global_xy.shape[1] != 2:
        raise ValueError(
            'points_global_xy must have shape (N, 2), got '
            f'{points_global_xy.shape}')
    origin_x, origin_y = valid_dtm_mask.origin_top_left_xy
    resolution_x, resolution_y = valid_dtm_mask.resolution_xy
    if resolution_x <= 0 or resolution_y <= 0:
        raise ValueError(
            f'valid_dtm resolution must be positive, got '
            f'{valid_dtm_mask.resolution_xy}')
    cols = np.floor((points_global_xy[:, 0] - origin_x) /
                    resolution_x).astype(np.int64)
    rows = np.floor((origin_y - points_global_xy[:, 1]) /
                    resolution_y).astype(np.int64)
    in_bounds = ((rows >= 0) & (rows < valid_dtm_mask.mask_hw.shape[0])
                 & (cols >= 0) & (cols < valid_dtm_mask.mask_hw.shape[1]))
    valid_flags = np.zeros(points_global_xy.shape[0], dtype=bool)
    valid_flags[in_bounds] = valid_dtm_mask.mask_hw[rows[in_bounds],
                                                    cols[in_bounds]]
    cell_indices = np.stack([rows, cols], axis=1)
    return valid_flags, in_bounds, cell_indices


def _ratio(numerator: int, denominator: int) -> float:
    if denominator <= 0:
        return 0.0
    return float(numerator) / float(denominator)


def compute_valid_dtm_coverage(
    points_global_xy: np.ndarray | None,
    valid_dtm_mask: ValidDtmMask | None,
    precomputed_valid_flags: np.ndarray | None = None,
    structural_blind_mask: np.ndarray | None = None,
) -> dict:
    """Measure how much of a local sample falls inside the manual valid DTM."""
    if precomputed_valid_flags is None:
        if points_global_xy is None or valid_dtm_mask is None:
            raise ValueError(
                'points_global_xy and valid_dtm_mask are required when '
                'precomputed_valid_flags is not provided')
        valid_flags, in_bounds, cell_indices = _query_valid_dtm_points(
            points_global_xy, valid_dtm_mask)
    else:
        valid_flags = np.asarray(precomputed_valid_flags, dtype=bool)
        in_bounds = np.ones(valid_flags.shape[0], dtype=bool)
        cell_indices = None
    if structural_blind_mask is None:
        structural_blind_mask = np.zeros(valid_flags.shape[0], dtype=bool)
    else:
        structural_blind_mask = np.asarray(structural_blind_mask, dtype=bool)
    if structural_blind_mask.shape != valid_flags.shape:
        raise ValueError(
            'structural_blind_mask must have the same shape as valid flags, '
            f'got {structural_blind_mask.shape} and {valid_flags.shape}')

    total_count = int(valid_flags.shape[0])
    valid_count = int(valid_flags.sum())
    invalid_count = total_count - valid_count
    blind_count = int(structural_blind_mask.sum())
    non_blind_mask = ~structural_blind_mask
    non_blind_count = int(non_blind_mask.sum())
    non_blind_valid_count = int((valid_flags & non_blind_mask).sum())

    if cell_indices is not None and total_count > 0:
        valid_cells = []
        blind_cells = []
        for cell, is_valid, is_in_bounds, is_blind in zip(
                cell_indices, valid_flags, in_bounds, structural_blind_mask):
            if not is_in_bounds:
                continue
            valid_cells.append((int(cell[0]), int(cell[1]), bool(is_valid)))
            blind_cells.append((int(cell[0]), int(cell[1]), bool(is_blind)))
        unique_cell_valid = {}
        for row, col, is_valid in valid_cells:
            unique_cell_valid[(row, col)] = (
                unique_cell_valid.get((row, col), False) or is_valid)
        unique_cell_blind = {}
        for row, col, is_blind in blind_cells:
            unique_cell_blind[(row, col)] = (
                unique_cell_blind.get((row, col), False) or is_blind)
        cell_total = len(unique_cell_valid)
        cell_valid = sum(1 for value in unique_cell_valid.values() if value)
        cell_blind = sum(1 for value in unique_cell_blind.values() if value)
    else:
        cell_total = total_count
        cell_valid = valid_count
        cell_blind = blind_count
    cell_non_blind = cell_total - cell_blind
    cell_non_blind_valid = max(0, cell_valid - cell_blind)

    return {
        'source_valid_dtm_point_count': valid_count,
        'source_invalid_dtm_point_count': invalid_count,
        'source_valid_dtm_point_ratio': _ratio(valid_count, total_count),
        'source_invalid_dtm_point_ratio': _ratio(invalid_count, total_count),
        'source_valid_dtm_point_ratio_non_blind': _ratio(
            non_blind_valid_count, non_blind_count),
        'source_structural_blind_point_ratio': _ratio(blind_count,
                                                     total_count),
        'bev_valid_dtm_cell_count': int(cell_valid),
        'bev_invalid_dtm_cell_count': int(cell_total - cell_valid),
        'bev_valid_dtm_cell_ratio': _ratio(cell_valid, cell_total),
        'bev_invalid_dtm_cell_ratio': _ratio(cell_total - cell_valid,
                                            cell_total),
        'bev_valid_dtm_cell_ratio_non_blind': _ratio(cell_non_blind_valid,
                                                     cell_non_blind),
        'bev_structural_blind_cell_ratio': _ratio(cell_blind, cell_total),
    }


def classify_alignment_sample_for_estimation(
    sample_metrics: Mapping,
    min_source_points: int = 3000,
    min_target_crop_points: int = 10000,
    min_downsampled_target_points: int = 3000,
    max_identity_p50_m: float = 0.30,
    min_icp_fitness: float = 0.90,
    max_correction_global_angle_deg: float = 3.0,
    max_correction_translation_centered_norm_m: float = 0.30,
    valid_dtm_min_source_point_ratio: float = 0.80,
    valid_dtm_review_source_point_ratio: float = 0.60,
) -> dict:
    """Classify whether a sample can estimate a global correction."""
    reject_reasons: list[str] = []
    review_category = ''
    valid_ratio = float(
        sample_metrics.get('source_valid_dtm_point_ratio_non_blind',
                           sample_metrics.get('source_valid_dtm_point_ratio',
                                              1.0)))
    if valid_ratio < valid_dtm_review_source_point_ratio:
        reject_reasons.append('outside_manual_valid_dtm')
        review_category = 'outside_manual_valid_dtm'
    elif valid_ratio < valid_dtm_min_source_point_ratio:
        reject_reasons.append('partial_manual_valid_dtm')
        review_category = 'partial_manual_valid_dtm'

    if int(sample_metrics.get('downsampled_source_points',
                              sample_metrics.get('source_points', 0))
           ) < min_source_points:
        reject_reasons.append('source_points_too_few')
    if int(sample_metrics.get('target_crop_points', 0)) < min_target_crop_points:
        reject_reasons.append('target_crop_points_too_few')
    if int(sample_metrics.get('downsampled_target_points',
                              sample_metrics.get('target_crop_points', 0))
           ) < min_downsampled_target_points:
        reject_reasons.append('downsampled_target_points_too_few')

    identity_p50 = float(
        sample_metrics.get('identity_nearest', {}).get('p50_m', math.inf))
    icp_fitness = float(sample_metrics.get('icp_fitness', 0.0))
    if identity_p50 > max_identity_p50_m and icp_fitness < min_icp_fitness:
        reject_reasons.append('low_overlap')

    if float(sample_metrics.get('correction_global_angle_deg',
                                0.0)) > max_correction_global_angle_deg:
        reject_reasons.append('icp_angle_over_threshold')
        if not review_category:
            review_category = 'icp_outlier'

    translation_norm = sample_metrics.get(
        'correction_translation_centered_norm_m')
    if translation_norm is None and sample_metrics.get(
            'correction_translation_centered_m') is not None:
        translation_norm = float(
            np.linalg.norm(
                np.asarray(sample_metrics['correction_translation_centered_m'],
                           dtype=np.float64)))
    if translation_norm is not None and float(
            translation_norm) > max_correction_translation_centered_norm_m:
        reject_reasons.append('icp_translation_over_threshold')
        if not review_category:
            review_category = 'icp_outlier'

    if not review_category and reject_reasons:
        review_category = reject_reasons[0]
    return {
        'usable_for_correction_estimation': not reject_reasons,
        'requires_manual_review': bool(reject_reasons),
        'review_category': review_category,
        'reject_reasons': reject_reasons,
    }


def audit_sample_alignment(
    sample_idx: str,
    manifest_row: Mapping,
    info_row: Mapping,
    global_map_xyz: np.ndarray,
    out_dir: Path,
    voxel_size: float,
    crop_margin: float,
    icp_threshold: float,
) -> dict:
    source_xyz = read_ascii_pcd_xyz(Path(manifest_row['pcd_path']))
    target_crop_xyz = _crop_global_map_xyz(global_map_xyz, source_xyz,
                                           crop_margin)
    if target_crop_xyz.shape[0] == 0:
        raise ValueError(f'sample {sample_idx} has empty target crop')
    center_xyz = source_xyz.mean(axis=0)
    source_centered_down = _voxel_down_sample_xyz(source_xyz - center_xyz,
                                                  voxel_size)
    target_centered_down = _voxel_down_sample_xyz(
        target_crop_xyz - center_xyz, voxel_size)
    source_down = source_centered_down + center_xyz
    target_down = target_centered_down + center_xyz
    identity_stats = compute_nearest_residual_stats(source_down, target_down)
    transform_centered, icp_fitness, icp_inlier_rmse = _run_open3d_icp(
        source_centered_down, target_centered_down, icp_threshold)
    icp_source_xyz = _apply_centered_transform(source_down, center_xyz,
                                               transform_centered)
    icp_stats = compute_nearest_residual_stats(icp_source_xyz, target_down)

    ego2global = _matrix4('ego2global', info_row['ego2global'])
    lidar_info = info_row.get('lidar_points', {})
    lidar2ego = _matrix4('lidar2ego',
                         lidar_info.get('lidar2ego', _identity_matrix4()))
    lidar2global = ego2global @ lidar2ego
    rotation_global = transform_centered[:3, :3]
    rotation_local = (
        lidar2global[:3, :3].T @ rotation_global @ lidar2global[:3, :3])
    global_ypr = decompose_rotation_zyx_deg(rotation_global)
    local_ypr = decompose_rotation_zyx_deg(rotation_local)

    sample_out_dir = out_dir / f'sample_{sample_idx}'
    sample_out_dir.mkdir(parents=True, exist_ok=True)
    write_ascii_pcd_xyz(sample_out_dir / 'source_downsampled_centered.pcd',
                        source_centered_down)
    write_ascii_pcd_xyz(sample_out_dir / 'target_crop_downsampled_centered.pcd',
                        target_centered_down)

    correction_global = {
        'yaw_deg': global_ypr[0],
        'pitch_deg': global_ypr[1],
        'roll_deg': global_ypr[2],
        'angle_deg': _rotation_angle_deg(rotation_global),
        'translation_centered_m': [
            float(v) for v in transform_centered[:3, 3]
        ],
        'matrix_centered': transform_centered.tolist(),
    }
    correction_local = {
        'yaw_deg': local_ypr[0],
        'pitch_deg': local_ypr[1],
        'roll_deg': local_ypr[2],
        'angle_deg': _rotation_angle_deg(rotation_local),
    }
    (sample_out_dir / 'icp_correction_global.json').write_text(
        json.dumps(correction_global, indent=2, sort_keys=True),
        encoding='utf-8')
    (sample_out_dir / 'icp_correction_local.json').write_text(
        json.dumps(correction_local, indent=2, sort_keys=True),
        encoding='utf-8')
    residual_histogram = {
        'identity_nearest': identity_stats,
        'icp_nearest': icp_stats,
    }
    (sample_out_dir / 'residual_histogram.json').write_text(
        json.dumps(residual_histogram, indent=2, sort_keys=True),
        encoding='utf-8')

    result = {
        'sample_idx': str(sample_idx),
        'token': str(manifest_row.get('token', info_row.get('token', ''))),
        'source_points': int(source_xyz.shape[0]),
        'target_crop_points': int(target_crop_xyz.shape[0]),
        'downsampled_source_points': int(source_down.shape[0]),
        'downsampled_target_points': int(target_down.shape[0]),
        'identity_nearest': identity_stats,
        'icp_nearest': icp_stats,
        'icp_fitness': icp_fitness,
        'icp_inlier_rmse': icp_inlier_rmse,
        'correction_global_yaw_pitch_roll_deg': {
            'yaw_deg': global_ypr[0],
            'pitch_deg': global_ypr[1],
            'roll_deg': global_ypr[2],
        },
        'correction_global_angle_deg': _rotation_angle_deg(rotation_global),
        'correction_local_yaw_pitch_roll_deg': {
            'yaw_deg': local_ypr[0],
            'pitch_deg': local_ypr[1],
            'roll_deg': local_ypr[2],
        },
        'correction_local_angle_deg': _rotation_angle_deg(rotation_local),
        'correction_translation_centered_m': [
            float(v) for v in transform_centered[:3, 3]
        ],
        'ego_yaw_deg': _yaw_from_rotation_deg(ego2global[:3, :3]),
        'lidar2ego_is_identity': bool(
            np.allclose(lidar2ego, _identity_matrix4(), atol=1e-6)),
        'lidar2global_rotation_matrix': lidar2global[:3, :3].tolist(),
        'sample_dir': str(sample_out_dir),
    }
    return result


def _build_sample_result(
    sample_idx: str,
    token: str,
    split_name: str,
    source_xyz: np.ndarray,
    source_lidar_xy: np.ndarray,
    info_row: Mapping,
    global_map_xyz: np.ndarray,
    out_dir: Path,
    voxel_size: float,
    crop_margin: float,
    icp_threshold: float,
    valid_dtm_mask: ValidDtmMask | None = None,
    structural_blind_angle_sectors_deg: Sequence[tuple[float, float]] = (),
    candidate_local_ypr_deg: Sequence[float] | None = None,
    write_per_sample_pcd: bool = True,
) -> dict:
    target_crop_xyz = _crop_global_map_xyz(global_map_xyz, source_xyz,
                                           crop_margin)
    if target_crop_xyz.shape[0] == 0:
        raise ValueError(f'sample {sample_idx} has empty target crop')
    center_xyz = source_xyz.mean(axis=0)
    source_centered_down = _voxel_down_sample_xyz(source_xyz - center_xyz,
                                                  voxel_size)
    target_centered_down = _voxel_down_sample_xyz(
        target_crop_xyz - center_xyz, voxel_size)
    source_down = source_centered_down + center_xyz
    target_down = target_centered_down + center_xyz
    identity_stats = compute_nearest_residual_stats(source_down, target_down)
    transform_centered, icp_fitness, icp_inlier_rmse = _run_open3d_icp(
        source_centered_down, target_centered_down, icp_threshold)
    icp_source_xyz = _apply_centered_transform(source_down, center_xyz,
                                               transform_centered)
    icp_stats = compute_nearest_residual_stats(icp_source_xyz, target_down)

    ego2global = _matrix4('ego2global', info_row['ego2global'])
    lidar_info = info_row.get('lidar_points', {})
    lidar2ego = _matrix4('lidar2ego',
                         lidar_info.get('lidar2ego', _identity_matrix4()))
    lidar2global = ego2global @ lidar2ego
    rotation_global = transform_centered[:3, :3]
    rotation_local = (
        lidar2global[:3, :3].T @ rotation_global @ lidar2global[:3, :3])
    global_ypr = decompose_rotation_zyx_deg(rotation_global)
    local_ypr = decompose_rotation_zyx_deg(rotation_local)

    sample_out_dir = out_dir / f'sample_{sample_idx}'
    if write_per_sample_pcd:
        sample_out_dir.mkdir(parents=True, exist_ok=True)
        write_ascii_pcd_xyz(sample_out_dir / 'source_downsampled_centered.pcd',
                            source_centered_down)
        write_ascii_pcd_xyz(
            sample_out_dir / 'target_crop_downsampled_centered.pcd',
            target_centered_down)
        residual_histogram = {
            'identity_nearest': identity_stats,
            'icp_nearest': icp_stats,
        }
        (sample_out_dir / 'residual_histogram.json').write_text(
            json.dumps(residual_histogram, indent=2, sort_keys=True),
            encoding='utf-8')

    coverage = {}
    if valid_dtm_mask is not None:
        source_blind_mask = make_structural_blind_angle_mask(
            source_lidar_xy, structural_blind_angle_sectors_deg)
        coverage = compute_valid_dtm_coverage(
            source_xyz[:, :2],
            valid_dtm_mask,
            structural_blind_mask=source_blind_mask)

    translation_centered_m = [float(v) for v in transform_centered[:3, 3]]
    result = {
        'sample_idx': str(sample_idx),
        'token': str(token),
        'split_name': str(split_name),
        'source_points': int(source_xyz.shape[0]),
        'target_crop_points': int(target_crop_xyz.shape[0]),
        'downsampled_source_points': int(source_down.shape[0]),
        'downsampled_target_points': int(target_down.shape[0]),
        'identity_nearest': identity_stats,
        'icp_nearest': icp_stats,
        'icp_fitness': icp_fitness,
        'icp_inlier_rmse': icp_inlier_rmse,
        'correction_global_yaw_pitch_roll_deg': {
            'yaw_deg': global_ypr[0],
            'pitch_deg': global_ypr[1],
            'roll_deg': global_ypr[2],
        },
        'correction_global_angle_deg': _rotation_angle_deg(rotation_global),
        'correction_local_yaw_pitch_roll_deg': {
            'yaw_deg': local_ypr[0],
            'pitch_deg': local_ypr[1],
            'roll_deg': local_ypr[2],
        },
        'correction_local_angle_deg': _rotation_angle_deg(rotation_local),
        'correction_translation_centered_m': translation_centered_m,
        'correction_translation_centered_norm_m': float(
            np.linalg.norm(transform_centered[:3, 3])),
        'ego_yaw_deg': _yaw_from_rotation_deg(ego2global[:3, :3]),
        'lidar2ego_is_identity': bool(
            np.allclose(lidar2ego, _identity_matrix4(), atol=1e-6)),
        'lidar2global_rotation_matrix': lidar2global[:3, :3].tolist(),
        'sample_dir': str(sample_out_dir),
    }
    result.update(coverage)
    if candidate_local_ypr_deg is not None:
        candidate_rotation = _rotation_from_zyx_deg(*candidate_local_ypr_deg)
        fixed_source_xyz = apply_fixed_local_correction(
            source_down,
            center_xyz=center_xyz,
            local_correction_rotation=candidate_rotation,
            lidar2global_rotation=lidar2global[:3, :3])
        result['candidate_local_yaw_pitch_roll_deg'] = [
            float(v) for v in candidate_local_ypr_deg
        ]
        result['candidate_fixed_local_nearest'] = (
            compute_nearest_residual_stats(fixed_source_xyz, target_down))
    return result


def audit_info_sample_alignment(
    sample_idx: str,
    info_row: Mapping,
    split_name: str,
    data_root: Path,
    global_map_xyz: np.ndarray,
    out_dir: Path,
    voxel_size: float,
    crop_margin: float,
    icp_threshold: float,
    valid_dtm_mask: ValidDtmMask | None = None,
    structural_blind_angle_sectors_deg: Sequence[tuple[float, float]] = (),
    candidate_local_ypr_deg: Sequence[float] | None = None,
    write_per_sample_pcd: bool = True,
) -> dict:
    points_lidar = load_lidar_points_from_info(data_root, info_row)
    points_global = transform_lidar_points_to_global(points_lidar, info_row)
    return _build_sample_result(
        sample_idx=sample_idx,
        token=str(info_row.get('token', '')),
        split_name=split_name,
        source_xyz=points_global[:, :3],
        source_lidar_xy=points_lidar[:, :2],
        info_row=info_row,
        global_map_xyz=global_map_xyz,
        out_dir=out_dir,
        voxel_size=voxel_size,
        crop_margin=crop_margin,
        icp_threshold=icp_threshold,
        valid_dtm_mask=valid_dtm_mask,
        structural_blind_angle_sectors_deg=structural_blind_angle_sectors_deg,
        candidate_local_ypr_deg=candidate_local_ypr_deg,
        write_per_sample_pcd=write_per_sample_pcd)


def _read_global_map_xyz(path: Path) -> np.ndarray:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError('Open3D is required to read source map PCD') from exc
    pcd = o3d.io.read_point_cloud(str(path))
    xyz = np.asarray(pcd.points, dtype=np.float64)
    return _as_xyz_array('global_map_xyz', xyz)


def _flatten_summary_row(row: Mapping) -> dict:
    flat = {
        'sample_idx': row['sample_idx'],
        'token': row['token'],
        'split_name': row.get('split_name', ''),
        'source_points': row['source_points'],
        'target_crop_points': row['target_crop_points'],
        'downsampled_source_points': row['downsampled_source_points'],
        'downsampled_target_points': row['downsampled_target_points'],
        'icp_fitness': row['icp_fitness'],
        'icp_inlier_rmse': row['icp_inlier_rmse'],
        'correction_global_angle_deg': row['correction_global_angle_deg'],
        'correction_local_angle_deg': row['correction_local_angle_deg'],
        'ego_yaw_deg': row['ego_yaw_deg'],
        'lidar2ego_is_identity': row['lidar2ego_is_identity'],
        'sample_dir': row['sample_dir'],
    }
    for prefix in ('identity_nearest', 'icp_nearest'):
        for metric_name, value in row[prefix].items():
            flat[f'{prefix}_{metric_name}'] = value
    if 'fixed_local_mean_nearest' in row:
        for metric_name, value in row['fixed_local_mean_nearest'].items():
            flat[f'fixed_local_mean_nearest_{metric_name}'] = value
    if 'candidate_fixed_local_nearest' in row:
        for metric_name, value in row['candidate_fixed_local_nearest'].items():
            flat[f'candidate_fixed_local_nearest_{metric_name}'] = value
    for metric_name in (
            'source_valid_dtm_point_count',
            'source_invalid_dtm_point_count',
            'source_valid_dtm_point_ratio',
            'source_invalid_dtm_point_ratio',
            'source_valid_dtm_point_ratio_non_blind',
            'source_structural_blind_point_ratio',
            'bev_valid_dtm_cell_count',
            'bev_invalid_dtm_cell_count',
            'bev_valid_dtm_cell_ratio',
            'bev_invalid_dtm_cell_ratio',
            'bev_valid_dtm_cell_ratio_non_blind',
            'bev_structural_blind_cell_ratio',
            'correction_translation_centered_norm_m',
            'usable_for_correction_estimation',
            'requires_manual_review',
            'review_category',
            'reject_reasons',
    ):
        if metric_name in row:
            value = row[metric_name]
            flat[metric_name] = (
                ';'.join(value) if isinstance(value, list) else value)
    for frame in ('global', 'local'):
        ypr = row[f'correction_{frame}_yaw_pitch_roll_deg']
        for axis in ('yaw_deg', 'pitch_deg', 'roll_deg'):
            flat[f'correction_{frame}_{axis}'] = ypr[axis]
    translation = row['correction_translation_centered_m']
    for axis, value in zip(('x', 'y', 'z'), translation):
        flat[f'correction_translation_centered_{axis}_m'] = value
    return flat


def _write_summary(out_dir: Path, rows: Sequence[dict],
                   args: argparse.Namespace) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    usable_rows = [
        row for row in rows
        if row.get('usable_for_correction_estimation', True)
    ]
    local_yprs = np.asarray(
        [_extract_ypr_deg(row) for row in usable_rows], dtype=np.float64)
    if local_yprs.size:
        local_mean = local_yprs.mean(axis=0)
        local_std = local_yprs.std(axis=0)
        local_mean_rotation = _rotation_from_zyx_deg(*local_mean)
        fixed_stats_by_sample = {}
        for row in usable_rows:
            sample_dir = Path(row['sample_dir'])
            source_centered_path = sample_dir / 'source_downsampled_centered.pcd'
            target_centered_path = (
                sample_dir / 'target_crop_downsampled_centered.pcd')
            if (not source_centered_path.exists()
                    or not target_centered_path.exists()):
                continue
            source_centered = read_ascii_pcd_xyz(source_centered_path)
            target_centered = read_ascii_pcd_xyz(target_centered_path)
            lidar2global_rotation = np.asarray(
                row['lidar2global_rotation_matrix'], dtype=np.float64)
            fixed_global_rotation = compute_global_rotation_from_local_correction(
                local_mean_rotation, lidar2global_rotation)
            fixed_source = (fixed_global_rotation @ source_centered.T).T
            fixed_stats = compute_nearest_residual_stats(
                fixed_source, target_centered)
            row['fixed_local_mean_nearest'] = fixed_stats
            fixed_stats_by_sample[str(row['sample_idx'])] = fixed_stats
    else:
        local_mean = np.asarray([], dtype=np.float64)
        local_std = np.asarray([], dtype=np.float64)
        fixed_stats_by_sample = {}

    summary_json = {
        'params': {
            'source_map_pcd': str(getattr(args, 'source_map_pcd', '')),
            'manifest': str(getattr(args, 'manifest', '')),
            'info': str(getattr(args, 'info', '')),
            'data_root': str(getattr(args, 'data_root', '')),
            'split_name': str(getattr(args, 'split_name', '')),
            'sample_idx': [
                str(v) for v in getattr(args, 'sample_idx', []) or []
            ],
            'sample_stride': getattr(args, 'sample_stride', None),
            'sample_offset': getattr(args, 'sample_offset', None),
            'max_samples': getattr(args, 'max_samples', None),
            'valid_dtm_path': str(getattr(args, 'valid_dtm_path', '')),
            'valid_dtm_min_source_point_ratio': getattr(
                args, 'valid_dtm_min_source_point_ratio', None),
            'valid_dtm_review_source_point_ratio': getattr(
                args, 'valid_dtm_review_source_point_ratio', None),
            'structural_blind_angle_sector': getattr(
                args, 'structural_blind_angle_sector', None),
            'candidate_local_ypr_deg': getattr(args,
                                               'candidate_local_ypr_deg',
                                               None),
            'voxel_size': getattr(args, 'voxel_size', None),
            'crop_margin': getattr(args, 'crop_margin', None),
            'icp_threshold': getattr(args, 'icp_threshold', None),
        },
        'rows': rows,
    }
    (out_dir / 'alignment_summary.json').write_text(
        json.dumps(summary_json, indent=2, sort_keys=True), encoding='utf-8')
    flat_rows = [_flatten_summary_row(row) for row in rows]
    if flat_rows:
        with (out_dir / 'alignment_summary.csv').open(
                'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(flat_rows[0].keys()))
            writer.writeheader()
            writer.writerows(flat_rows)
    if local_yprs.size:
        candidate = summarize_candidate_correction(usable_rows)
        candidate.update({
            'local_yaw_pitch_roll_deg_mean': local_mean.tolist(),
            'local_yaw_pitch_roll_deg_std': local_std.tolist(),
            'fixed_local_mean_nearest_by_sample': fixed_stats_by_sample,
            'usable_sample_count': len(usable_rows),
            'total_sample_count': len(rows),
        })
    else:
        candidate = {
            'local_yaw_pitch_roll_deg_mean': [],
            'local_yaw_pitch_roll_deg_std': [],
            'robust_median_yaw_pitch_roll_deg': [],
            'mad_yaw_pitch_roll_deg': [],
            'sample_count': 0,
            'usable_sample_count': 0,
            'total_sample_count': len(rows),
        }
    (out_dir / 'candidate_corrections.json').write_text(
        json.dumps(candidate, indent=2, sort_keys=True), encoding='utf-8')


def _write_review_csvs(out_dir: Path, rows: Sequence[dict]) -> None:
    flat_rows = [_flatten_summary_row(row) for row in rows]
    if not flat_rows:
        return
    fieldnames = sorted({key for row in flat_rows for key in row.keys()})
    rejected = [
        row for row in flat_rows
        if str(row.get('reject_reasons', '')) not in ('', '[]')
    ]
    outlier_rows = [
        row for row in flat_rows
        if row.get('review_category') == 'icp_outlier'
    ]
    valid_dtm_rows = [
        row for row in flat_rows
        if row.get('review_category') in ('outside_manual_valid_dtm',
                                          'partial_manual_valid_dtm')
    ]
    for filename, subset in (
        ('rejected_samples.csv', rejected),
        ('manual_review_icp_outliers.csv', outlier_rows),
        ('manual_valid_dtm_rejected_samples.csv', valid_dtm_rows),
    ):
        with (out_dir / filename).open('w', encoding='utf-8',
                                       newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(subset)


def _selected_info_rows(
    info_rows: Sequence[dict],
    sample_stride: int,
    sample_offset: int,
    max_samples: int | None,
    sample_idx: Sequence[str] | None,
    extra_sample_idx: Sequence[str] | None,
) -> list[tuple[str, dict]]:
    selected: list[tuple[str, dict]] = []
    seen: set[str] = set()
    if sample_stride <= 0:
        raise ValueError(f'sample_stride must be positive, got {sample_stride}')
    for row_index, row in enumerate(info_rows):
        row_sample_idx = str(row.get('sample_idx', row_index))
        if sample_idx:
            if row_sample_idx not in {str(value) for value in sample_idx}:
                continue
        elif (row_index - sample_offset) % sample_stride != 0:
            continue
        if row_sample_idx in seen:
            continue
        selected.append((row_sample_idx, row))
        seen.add(row_sample_idx)
        if max_samples is not None and len(selected) >= max_samples:
            break

    by_sample_idx = {
        str(row.get('sample_idx', row_index)): row
        for row_index, row in enumerate(info_rows)
    }
    for requested in extra_sample_idx or []:
        key = str(requested)
        if key not in by_sample_idx:
            raise KeyError(f'extra sample_idx {key} not found in info')
        if key not in seen:
            selected.append((key, by_sample_idx[key]))
            seen.add(key)
    return selected


def _classify_rows(rows: Sequence[dict], args: argparse.Namespace) -> None:
    for row in rows:
        decision = classify_alignment_sample_for_estimation(
            row,
            valid_dtm_min_source_point_ratio=getattr(
                args, 'valid_dtm_min_source_point_ratio', 0.80),
            valid_dtm_review_source_point_ratio=getattr(
                args, 'valid_dtm_review_source_point_ratio', 0.60))
        row.update(decision)


def _load_rows_from_summary_json(paths: Sequence[Path]) -> list[dict]:
    rows: list[dict] = []
    for path in paths:
        summary = json.loads(Path(path).read_text(encoding='utf-8'))
        rows.extend(summary.get('rows', []))
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-map-pcd', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--info', type=Path)
    parser.add_argument('--data-root', type=Path)
    parser.add_argument('--split-name', choices=('train', 'val', 'test'))
    parser.add_argument('--out-dir', required=True, type=Path)
    parser.add_argument('--sample-idx', nargs='+')
    parser.add_argument('--extra-sample-idx', nargs='*', default=[])
    parser.add_argument('--sample-stride', type=int, default=1)
    parser.add_argument('--sample-offset', type=int, default=0)
    parser.add_argument('--max-samples', type=int)
    parser.add_argument('--candidate-local-ypr-deg', nargs=3, type=float)
    parser.add_argument('--valid-dtm-path', type=Path)
    parser.add_argument('--valid-dtm-min-source-point-ratio',
                        type=float,
                        default=0.80)
    parser.add_argument('--valid-dtm-review-source-point-ratio',
                        type=float,
                        default=0.60)
    parser.add_argument('--structural-blind-angle-sector',
                        nargs=2,
                        action='append',
                        type=float,
                        default=[])
    parser.add_argument('--no-per-sample-pcd', action='store_true')
    parser.add_argument('--merge-summaries', action='store_true')
    parser.add_argument('--summary-json', nargs='+', type=Path)
    parser.add_argument('--voxel-size', type=float, default=0.20)
    parser.add_argument('--crop-margin', type=float, default=3.0)
    parser.add_argument('--icp-threshold', type=float, default=0.50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if args.merge_summaries:
        if not args.summary_json:
            raise ValueError('--summary-json is required with --merge-summaries')
        rows = _load_rows_from_summary_json(args.summary_json)
        _classify_rows(rows, args)
        _write_summary(args.out_dir, rows, args)
        _write_review_csvs(args.out_dir, rows)
        print(f'Merged {len(rows)} rows into {args.out_dir}')
        print(f'Summary: {args.out_dir / "alignment_summary.csv"}')
        return

    if args.source_map_pcd is None or args.info is None:
        raise ValueError('--source-map-pcd and --info are required')
    global_map_xyz = _read_global_map_xyz(args.source_map_pcd)
    rows = []
    valid_dtm_mask = (
        load_valid_dtm_mask(args.valid_dtm_path)
        if args.valid_dtm_path is not None else None)
    if args.manifest is not None:
        if not args.sample_idx:
            raise ValueError('--sample-idx is required in manifest mode')
        manifest_by_sample = _load_manifest(args.manifest)
        info_by_sample = _load_info_by_sample_idx(args.info)
        for sample_idx in [str(value) for value in args.sample_idx]:
            if sample_idx not in manifest_by_sample:
                raise KeyError(
                    f'sample_idx {sample_idx} not found in manifest')
            if sample_idx not in info_by_sample:
                raise KeyError(f'sample_idx {sample_idx} not found in info')
            row = audit_sample_alignment(
                sample_idx=sample_idx,
                manifest_row=manifest_by_sample[sample_idx],
                info_row=info_by_sample[sample_idx],
                global_map_xyz=global_map_xyz,
                out_dir=args.out_dir,
                voxel_size=args.voxel_size,
                crop_margin=args.crop_margin,
                icp_threshold=args.icp_threshold,
            )
            rows.append(row)
            print(
                f"sample {sample_idx}: identity_rmse="
                f"{row['identity_nearest']['rmse_m']:.4f} "
                f"icp_rmse={row['icp_nearest']['rmse_m']:.4f} "
                f"local_ypr={row['correction_local_yaw_pitch_roll_deg']}")
    else:
        if args.data_root is None:
            raise ValueError('--data-root is required in direct info mode')
        info_rows = _load_info_rows(args.info)
        selected_rows = _selected_info_rows(
            info_rows,
            sample_stride=args.sample_stride,
            sample_offset=args.sample_offset,
            max_samples=args.max_samples,
            sample_idx=args.sample_idx,
            extra_sample_idx=args.extra_sample_idx)
        print(f'Running direct info audit for {len(selected_rows)} samples')
        for sample_idx, info_row in selected_rows:
            row = audit_info_sample_alignment(
                sample_idx=sample_idx,
                info_row=info_row,
                split_name=args.split_name or '',
                data_root=args.data_root,
                global_map_xyz=global_map_xyz,
                out_dir=args.out_dir,
                voxel_size=args.voxel_size,
                crop_margin=args.crop_margin,
                icp_threshold=args.icp_threshold,
                valid_dtm_mask=valid_dtm_mask,
                structural_blind_angle_sectors_deg=[
                    tuple(value)
                    for value in args.structural_blind_angle_sector
                ],
                candidate_local_ypr_deg=args.candidate_local_ypr_deg,
                write_per_sample_pcd=not args.no_per_sample_pcd)
            rows.append(row)
            print(
                f"sample {sample_idx}: valid_nonblind="
                f"{row.get('source_valid_dtm_point_ratio_non_blind', -1):.3f} "
                f"identity_rmse={row['identity_nearest']['rmse_m']:.4f} "
                f"icp_rmse={row['icp_nearest']['rmse_m']:.4f} "
                f"local_ypr={row['correction_local_yaw_pitch_roll_deg']}")
    _classify_rows(rows, args)
    _write_summary(args.out_dir, rows, args)
    _write_review_csvs(args.out_dir, rows)
    print(f'Wrote {len(rows)} sample audits to {args.out_dir}')
    print(f'Summary: {args.out_dir / "alignment_summary.csv"}')


if __name__ == '__main__':
    main()
