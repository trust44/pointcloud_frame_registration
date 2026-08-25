#!/usr/bin/env python
"""Register raw LiDAR frames to a global map independently.

Input:
    --lidar-frame-in: dir contains lidar *.pcd
    --calib-frame-in: dir contains calib *.txt (same stem as pcd)
    --global-map: global map pcd/npy (map coordinate)
    --out-dir: output root dir (registration_out)
    --lidar_corrected_out: 0/1 whether save corrected lidar‑coord pcd to velodyne/
    --max-frame: optional, process first N frames
Output:
    registration_out/
        registration_summary.jsonl
        registration_summary.csv
        velodyne/          # corrected lidar‑coord binary pcd (5ch, optional)
        velodyne_map/      # corrected map‑coord binary pcd (5ch)
"""
from __future__ import annotations

import argparse
import csv
import json
import yaml
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from src.audit_lidar_global_map_alignment import (
    _crop_global_map_xyz,
    _run_open3d_icp,
    _voxel_down_sample_xyz,
    compute_nearest_residual_stats,
    decompose_rotation_zyx_deg,
    read_ascii_pcd_xyz,
)

PCD_DTYPE = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("intensity", "<f4"),
    ("time", "<f4"),
])


def output_suffix(output_format: str) -> str:
    if output_format not in ('pcd', 'las'):
        raise ValueError(f'unsupported output format: {output_format}')
    return f'.{output_format}'


def write_las(save_path: Path, points: np.ndarray) -> Path:
    """Write XYZ/intensity/time to LAS using x/y/z/intensity/gps_time."""
    try:
        import laspy
    except ImportError as exc:
        raise RuntimeError(
            'LAS output requires laspy; install it with: '
            'python -m pip install laspy') from exc
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 5:
        raise ValueError('LAS output requires points with shape (N, 5)')
    if not np.isfinite(points).all():
        raise ValueError('LAS output points contain NaN or Inf')
    header = laspy.LasHeader(point_format=1, version='1.2')
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = points[:, :3].min(axis=0) if points.shape[0] else np.zeros(3)
    las = laspy.LasData(header)
    if points.shape[0]:
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]
        las.intensity = np.clip(np.rint(points[:, 3]), 0, 65535).astype(np.uint16)
        las.gps_time = points[:, 4]
    save_path.parent.mkdir(parents=True, exist_ok=True)
    las.write(save_path)
    return save_path


def write_pointcloud(save_path: Path, points: np.ndarray,
                     output_format: str) -> Path:
    if output_format == 'pcd':
        write_binary_pcd(save_path, points)
        return save_path
    if output_format == 'las':
        return write_las(save_path, points)
    raise ValueError(f'unsupported output format: {output_format}')


def write_binary_pcd(save_path: Path, points: np.ndarray):
    """points(N,5):x,y,z,intensity,time，输出binary pcd"""
    data = np.empty(points.shape[0], dtype=PCD_DTYPE)
    data["x"] = points[:, 0]
    data["y"] = points[:, 1]
    data["z"] = points[:, 2]
    data["intensity"] = points[:, 3]
    data["time"] = points[:, 4]

    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity time
SIZE 4 4 4 4 4
TYPE F F F F F
COUNT 1 1 1 1 1
WIDTH {len(data)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(data)}
DATA binary
""".encode("ascii")
    with open(save_path, "wb") as f:
        f.write(header)
        data.tofile(f)
    return save_path.name, len(data)


def transform_points(points: np.ndarray, tr: np.ndarray, inverse: bool = False):
    """
    points: (N,5) x,y,z,intensity,time float32
    tr: 3×4 Tr_velo_to_map  R|t，lidar -> map
    inverse=False : lidar -> map
    inverse=True  : map -> lidar
    return (N,5) float32
    """
    xyz = points[:, :3].copy()
    intensity = points[:, 3:4]
    time = points[:, 4:5]

    # 提升到float64做矩阵运算，避免float32截断误差
    xyz = xyz.astype(np.float64)
    R = tr[:, :3].astype(np.float64)   # 3x3
    t = tr[:, 3:4].astype(np.float64)  # 3x1

    if not inverse:
        # lidar -> map
        xyz_map = (R @ xyz.T + t).T
        out_xyz = xyz_map
    else:
        # map -> lidar : P_lidar = R.T @ (P_map - t)
        xyz_lidar = (R.T @ (xyz.T - t)).T
        out_xyz = xyz_lidar

    out_xyz = out_xyz.astype(np.float32)
    out_pts = np.hstack([out_xyz, intensity, time])
    return out_pts


def parse_tr_velo_to_map(txt_path: Path) -> np.ndarray:
    """
    Parse Tr_velo_to_map line from txt file, return **3×4 R|t** matrix (lidar -> map).
    Line example:
    Tr_velo_to_map: 0.991080 -0.126949 0.040556 -60.940500 0.125258 0.991242 0.041825 115.085000 -0.045511 -0.036372 0.998302 8.101490
    """
    lines = txt_path.read_text(encoding="utf-8").splitlines()
    tr_line = None
    for line in lines:
        if line.strip().startswith("Tr_velo_to_map:"):
            tr_line = line.strip()
            break
    if tr_line is None:
        raise ValueError(f"{txt_path} missing Tr_velo_to_map line")
    nums_str = tr_line.split(":", 1)[1].strip().split()
    nums = np.array([float(x) for x in nums_str], dtype=np.float64)
    if nums.shape[0] != 12:
        raise ValueError(f"{txt_path} Tr_velo_to_map needs 12 numbers, got {nums.shape[0]}")
    tr34 = nums.reshape(3, 4)
    return tr34

def parse_map_anchor(yaml_path: Path) -> np.ndarray:
    """读取map_anchor.yaml 返回 map_translation_offset_xyz shape(3,) float64"""
    with open(yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    offset = cfg.get("map_translation_offset_xyz", [0.0, 0.0, 0.0])
    return np.array(offset, dtype=np.float64)

def load_binary_pcd_5ch(path: Path) -> np.ndarray:
    """
    Load binary pcd with 5 channels: x,y,z,intensity,time
    return (N,5) float32 array
    """
    content = path.read_bytes()
    header_end = content.find(b"\nDATA binary\n")
    if header_end == -1:
        raise RuntimeError(f"{path} is not binary pcd with DATA binary")
    header_size = header_end + len(b"\nDATA binary\n")
    raw_bin = content[header_size:]
    # 5 channel float32: x y z intensity time
    point_step = 5 * 4
    num_points = len(raw_bin) // point_step
    if num_points * point_step != len(raw_bin):
        raise RuntimeError(f"{path} binary size mismatch for 5×float32 points")
    arr = np.frombuffer(raw_bin, dtype=np.float32).reshape(num_points, 5)
    if not np.isfinite(arr[:, :3]).all():
        raise ValueError(f"{path} xyz contains nan/inf")
    return arr


def _load_map(path: Path) -> np.ndarray:
    if path.suffix == ".pcd":
        header_bytes = path.read_bytes()[:4096]
        if b"DATA binary" in header_bytes:
            import open3d as o3d
            cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=True, remove_infinite_points=True)
            points = np.asarray(cloud.points, dtype=np.float64)
            if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
                raise ValueError(f"could not read binary PCD points: {path}")
            return points
        return read_ascii_pcd_xyz(path)
    if path.suffix == ".npy":
        points = np.load(path)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("global map npy must have shape (N, >=3)")
        return np.asarray(points[:, :3], dtype=np.float64)
    raise ValueError("global map must be ASCII/Binary PCD or NPY")


def register_frame_custom(
    points_lidar: np.ndarray,
    tr_lidar2map_34: np.ndarray,
    frame_id: str,
    global_map_xyz: np.ndarray,
    map_offset_xyz: np.ndarray,
    out_velodyne: Path,
    out_velodyne_map: Path,
    save_lidar_corrected: bool,
    voxel_size: float,
    crop_margin: float,
    icp_threshold: float,
    output_format: str,
    enable_z_correction: bool,   # ========= NEW =========
) -> Dict[str, Any]:
    """
    points_lidar: (N,5) lidar‑coordinate 5‑ch point cloud float32
    tr_lidar2map_34: 3×4 R|t lidar -> map
    enable_z_correction: True=ICP R+tz(ignore tx/ty); False=only yaw/pitch/roll
    """
    # lidar -> raw_map（无offset）
    source_raw_pts = transform_points(points_lidar, tr_lidar2map_34, inverse=False)
    # ========== 配准前叠加offset，进入world坐标系，与global_map_xyz同一坐标系 ==========
    source_world_pts = source_raw_pts.copy()
    source_world_pts[:, :3] = (source_world_pts[:, :3].astype(np.float64) - map_offset_xyz).astype(np.float32)

    source_global = source_world_pts[:, :3].astype(np.float64)   # ICP source：带offset world坐标系
    target_crop = _crop_global_map_xyz(global_map_xyz, source_global, crop_margin)
    if target_crop.shape[0] == 0:
        raise ValueError(f"{frame_id}: target crop is empty")

    center = source_global.mean(axis=0)
    source_down = _voxel_down_sample_xyz(source_global - center, voxel_size) + center
    target_down = _voxel_down_sample_xyz(target_crop - center, voxel_size) + center

    identity = compute_nearest_residual_stats(source_down, target_down)

    # ==== 调用原有ICP函数，完全不动 _run_open3d_icp ====
    transform_centered, fitness, inlier_rmse = _run_open3d_icp(
        source_down - center,
        target_down - center,
        icp_threshold,
    )

    corrected_global = _apply_matrix_xyz(source_down - center, transform_centered) + center
    corrected = compute_nearest_residual_stats(corrected_global, target_down)

    rot_world = transform_centered[:3, :3]
    t_world = transform_centered[:3, 3]
    tz_world = t_world[2]

    R_lidar2map = tr_lidar2map_34[:, :3]
    rot_local = R_lidar2map.T @ rot_world @ R_lidar2map

    local_correction_44 = np.eye(4, dtype=np.float64)
    local_correction_44[:3, :3] = rot_local

    z_offset_out = 0.0
    if enable_z_correction:
        # ========= 开启z修正：只取world系tz，tx/ty置0，映射回lidar =========
        tz_only_world = np.array([0.0, 0.0, tz_world], dtype=np.float64)
        t_local = R_lidar2map.T @ tz_only_world
        local_correction_44[:3, 3] = t_local
        z_offset_out = float(tz_world)
        # 可选限幅防跳变，按需开启
        # z_offset_out = np.clip(z_offset_out, -0.5, 0.5)
    else:
        # ========= 不开启z修正：平移项保持0，仅姿态 =========
        local_correction_44[:3, 3] = np.array([0.,0.,0.])
        z_offset_out = 0.0

    # 对原始lidar点做局部修正
    corrected_lidar_xyz = _apply_matrix_xyz(points_lidar[:, :3].astype(np.float64), local_correction_44)
    corrected_local_points = points_lidar.copy()
    corrected_local_points[:, :3] = corrected_lidar_xyz.astype(np.float32)

    # 修正后lidar点 → raw_map → 再叠加offset得到最终world坐标系点云
    corrected_raw_pts = transform_points(corrected_local_points, tr_lidar2map_34, inverse=False)
    corrected_global_full_pts = corrected_raw_pts.copy()
    corrected_global_full_pts[:, :3] = (corrected_global_full_pts[:, :3].astype(np.float64) - map_offset_xyz).astype(np.float32)

    # 输出 map坐标系5ch binary pcd (始终输出)
    suffix = output_suffix(output_format)
    pcd_map_out = out_velodyne_map / f"{frame_id}{suffix}"
    write_pointcloud(pcd_map_out, corrected_global_full_pts, output_format)

    # 可选输出 lidar坐标系配准后5ch binary pcd
    pcd_lidar_out = out_velodyne / f"{frame_id}{suffix}"
    if save_lidar_corrected:
        write_pointcloud(pcd_lidar_out, corrected_local_points, output_format)

    ypr = decompose_rotation_zyx_deg(rot_local)

    return {
        "frame_id": frame_id,
        "status": "usable" if corrected["p95_m"] < identity["p95_m"] else "review",
        "source_points": int(points_lidar.shape[0]),
        "target_crop_points": int(target_crop.shape[0]),
        "downsampled_source_points": int(source_down.shape[0]),
        "downsampled_target_points": int(target_down.shape[0]),
        "identity": identity,
        "corrected": corrected,
        "icp_fitness": float(fitness),
        "icp_inlier_rmse": float(inlier_rmse),
        "correction_yaw_pitch_roll_deg": [float(v) for v in ypr],
        "z_offset_m": z_offset_out,
        "enable_z_correction": enable_z_correction,
        "coordinate_frame_local": "lidar",
        "coordinate_frame_global": "global/map",
        "correction_matrix_local": local_correction_44.tolist(),
        "map_translation_offset_xyz": map_offset_xyz.tolist(),
        "tr_lidar2map_34_init": tr_lidar2map_34.tolist(),
        "corrected_global_pcd": str(pcd_map_out),
        "corrected_lidar_pcd": str(pcd_lidar_out) if save_lidar_corrected else None,
    }


def _apply_matrix_xyz(points_xyz: np.ndarray, matrix44: np.ndarray) -> np.ndarray:
    """仅用于ICP内部残差计算，输入(N,3) xyz，4×4齐次矩阵，返回(N,3)"""
    homogeneous = np.ones((points_xyz.shape[0], 4), dtype=np.float64)
    homogeneous[:, :3] = points_xyz
    return (matrix44 @ homogeneous.T).T[:, :3]


def write_jsonl_summary(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True))
            stream.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lidar-frame-in", required=True, type=Path, help="lidar pcd input directory")
    parser.add_argument("--calib-frame-in", required=True, type=Path, help="calib txt input directory")
    parser.add_argument("--global-map", required=True, type=Path)
    parser.add_argument("--map-anchor", type=Path, default=None, help="map_anchor.yaml, read map_translation_offset_xyz; optional")
    parser.add_argument("--out-dir", required=True, type=Path, help="root output dir e.g. ./registration_out")
    parser.add_argument("--lidar-corrected-out", type=int, default=1, choices=[0,1], help="1: save corrected lidar‑coord pcd to velodyne; 0: skip")
    parser.add_argument("--voxel-size", type=float, default=0.20)
    parser.add_argument("--crop-margin", type=float, default=3.0)
    parser.add_argument("--icp-threshold", type=float, default=0.50)
    parser.add_argument("--max-frame", type=int, default=None, help="process first N frames; None for all")
    parser.add_argument("--output-format", choices=("pcd", "las"), default="pcd",
                        help="output point-cloud format; default: pcd")
    parser.add_argument("--z-offset", type=float, default=1,
                    help="Enable z-offset correction if supplied (value unused as threshold; just a switch flag). "
                         "When present: use ICP R+tz, discard ICP tx/ty; absent: only attitude correction.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    lidar_in_dir: Path = args.lidar_frame_in
    calib_in_dir: Path = args.calib_frame_in

    # 子输出目录
    out_root: Path = args.out_dir
    out_velodyne = out_root / "corrected_velodyne"
    out_velodyne_map = out_root / "corrected_velodyne_map"
    out_velodyne.mkdir(parents=True, exist_ok=True)
    out_velodyne_map.mkdir(parents=True, exist_ok=True)

    pcd_files = sorted(list(lidar_in_dir.glob("*.pcd")))
    if len(pcd_files) == 0:
        raise FileNotFoundError(f"No pcd found under {lidar_in_dir}")
    if args.max_frame is not None:
        if args.max_frame <= 0:
            raise ValueError("--max-frame must be positive")
        pcd_files = pcd_files[: args.max_frame]

    global_map = _load_map(args.global_map)
    map_offset_xyz = np.array([0.0,0.0,0.0], dtype=np.float64)
    if args.map_anchor is not None:
        map_offset_xyz = parse_map_anchor(args.map_anchor)
        print(f"Loaded map_translation_offset_xyz = {map_offset_xyz.tolist()}")
    enable_z_correction = args.z_offset is not None

    summaries = []
    total = len(pcd_files)
    save_lidar_corrected = bool(args.lidar_corrected_out)
    start_time = time.time()

    for idx, pcd_path in enumerate(pcd_files):
        frame_id = pcd_path.stem
        txt_path = calib_in_dir / f"{frame_id}.txt"

        # 每100帧打印进度
        if (idx + 1) % 100 == 0:
            elapsed = time.time() - start_time
            avg_per_frame = elapsed / (idx+1)
            print(f"===== Processed {idx+1}/{total} frames | elapsed: {elapsed:.2f}s | avg: {avg_per_frame:.3f} s/frame =====")

        try:
            points_lidar = load_binary_pcd_5ch(pcd_path)
            tr_lidar2map_34 = parse_tr_velo_to_map(txt_path)
            res = register_frame_custom(
                points_lidar=points_lidar,
                tr_lidar2map_34=tr_lidar2map_34,
                frame_id=frame_id,
                global_map_xyz=global_map,
                map_offset_xyz=map_offset_xyz,
                out_velodyne=out_velodyne,
                out_velodyne_map=out_velodyne_map,
                save_lidar_corrected=save_lidar_corrected,
                voxel_size=args.voxel_size,
                crop_margin=args.crop_margin,
                icp_threshold=args.icp_threshold,
                output_format=args.output_format,
                enable_z_correction=enable_z_correction,
            )
            summaries.append(res)
        except Exception as exc:
            summaries.append(
                {
                    "frame_id": frame_id,
                    "status": "failed",
                    "error": str(exc),
                }
            )

    write_jsonl_summary(out_root / "registration_summary.jsonl", summaries)
    fields = sorted({key for row in summaries for key in row})
    with (out_root / "registration_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in summaries:
            out_row = {}
            for k, v in row.items():
                if isinstance(v, (dict, list)):
                    out_row[k] = json.dumps(v)
                else:
                    out_row[k] = v
            writer.writerow(out_row)

    print(f"\nAll finished. Total {len(summaries)} frames. Output root: {out_root.resolve()}")


if __name__ == "__main__":
    main()
