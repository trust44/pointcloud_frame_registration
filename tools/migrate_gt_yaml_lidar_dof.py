"""Add manual_delta_lidar to exported GT YAML files."""
import argparse
from pathlib import Path
import shutil
import yaml
import numpy as np
from scipy.spatial.transform import Rotation

def convert(path, overwrite=False):
    data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    initial = np.asarray(data["initial_T_map_lidar"], dtype=float)
    corrected = np.asarray(data["corrected_T_map_lidar"], dtype=float)
    local = np.linalg.inv(initial) @ corrected
    yaw, pitch, roll = Rotation.from_matrix(local[:3, :3]).as_euler("ZYX", degrees=True)
    if "manual_delta_lidar" in data and not overwrite:
        return False
    data["manual_delta_lidar"] = {
        "coordinate_frame": "lidar", "direction": "original_to_corrected",
        "dx_m": float(local[0, 3]), "dy_m": float(local[1, 3]), "dz_m": float(local[2, 3]),
        "roll_deg": float(roll), "pitch_deg": float(pitch), "yaw_deg": float(yaw),
    }
    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy2(path, backup)
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    files = sorted(args.path.glob("*.yaml")) if args.path.is_dir() else [args.path]
    for path in files:
        try:
            if convert(path, args.overwrite): print("updated", path)
        except Exception as exc:
            print("error", path, exc)

if __name__ == "__main__":
    main()
