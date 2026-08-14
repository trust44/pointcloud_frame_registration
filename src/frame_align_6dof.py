"""Single-frame to global-map 6DoF profile alignment desktop tool.

The GUI can start empty, load the new directory-based YAML configuration, or
open the legacy ``--map --source --pose`` triplet. Source PCD points are
already in map coordinates and are never transformed by the initial pose.
"""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from frame_alignment.contracts import FrameData
from frame_alignment.core.point_cloud import Cloud, read_cloud
from frame_alignment.core.pose_model import Delta, PoseModel
from frame_alignment.io.pose_parser import parse_tr_velo_to_map, validate_rigid_transform


validate_pose = validate_rigid_transform
_CONFIG_PATH_KEYS = (
    "global_map_path",
    "frame_cloud_map_path",
    "initial_pose_path",
    "output_path_yaml",
    "output_path_pcd",
)


def _resolve_relative(value, base):
    if value is None or not str(value).strip():
        return ""
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = base / path
    return str(path.resolve())


def load_config(path):
    """Load configuration and resolve all relative paths beside the YAML."""
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8-sig") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("Configuration root must be a mapping: {}".format(config_path))
    data = dict(data)
    for key in _CONFIG_PATH_KEYS:
        if key in data:
            data[key] = _resolve_relative(data[key], config_path.parent)
    return data


def load_pose_file(path):
    """Load a strict Tr_velo_to_map text pose or a legacy YAML matrix."""
    pose_path = Path(path).expanduser().resolve()
    if pose_path.suffix.lower() == ".txt":
        return parse_tr_velo_to_map(pose_path)
    with pose_path.open("r", encoding="utf-8-sig") as handle:
        data = yaml.safe_load(handle)
    if isinstance(data, dict):
        for key in ("matrix", "T_map_lidar", "initial_T_map_lidar"):
            if key in data:
                data = data[key]
                break
    return validate_rigid_transform(data)


def _pose_from_config(config):
    value = config.get("initial_pose")
    if value is None:
        return None
    if isinstance(value, dict):
        value = value.get("matrix", value.get("T_map_lidar"))
    if value is None:
        return None
    return validate_rigid_transform(value)


def build_initial_frame(config, initial_pose=None, pose_path=None, cloud_reader=read_cloud):
    """Build the initially displayed frame for legacy file-based invocation."""
    map_path = Path(config.get("global_map_path", "")).expanduser().resolve()
    source_path = Path(config.get("frame_cloud_map_path", "")).expanduser().resolve()
    if not map_path.is_file():
        raise FileNotFoundError("Global map file does not exist: {}".format(map_path))
    if not source_path.is_file():
        raise FileNotFoundError("Source cloud file does not exist: {}".format(source_path))

    resolved_pose_path = None
    if pose_path is not None:
        resolved_pose_path = Path(pose_path).expanduser().resolve()
        pose = load_pose_file(resolved_pose_path)
    else:
        pose = initial_pose if initial_pose is not None else _pose_from_config(config)
        configured_pose = config.get("initial_pose_path", "")
        if configured_pose and Path(configured_pose).is_file():
            resolved_pose_path = Path(configured_pose).expanduser().resolve()
            if pose is None:
                pose = load_pose_file(resolved_pose_path)
    if pose is None:
        raise ValueError("Legacy file startup requires --pose or initial_pose.matrix")
    pose = validate_rigid_transform(pose)
    if resolved_pose_path is None:
        resolved_pose_path = (PROJECT_ROOT / "<inline-initial-pose>").resolve()

    global_map = cloud_reader(map_path)
    source_cloud = cloud_reader(source_path)
    if len(global_map.points) == 0:
        raise ValueError("Point cloud is empty: {}".format(map_path))
    if len(source_cloud.points) == 0:
        raise ValueError("Point cloud is empty: {}".format(source_path))
    return FrameData(
        source_path.stem,
        map_path,
        source_path,
        resolved_pose_path,
        global_map,
        source_cloud,
        pose,
    )


def run_self_test():
    initial = np.eye(4)
    initial[:3, 3] = (10.0, 20.0, 30.0)
    model = PoseModel(initial)
    np.testing.assert_allclose(model.transform_points([[10.0, 20.0, 30.0]]), [[10.0, 20.0, 30.0]])
    model.adjust("yaw_deg", 90.0)
    np.testing.assert_allclose(model.transform_points([[11.0, 20.0, 30.0]]), [[10.0, 21.0, 30.0]], atol=1e-10)
    np.testing.assert_allclose(model.corrected_pose, model.transform @ initial)
    print("Self-test passed")


def run_gui(config=None, initial_pose=None, output_dir=None, initial_frame=None):
    from PySide6 import QtWidgets
    from frame_alignment.ui.main_window import MainWindow

    config = dict(config or {})
    if output_dir:
        config["output_path_yaml"] = str(Path(output_dir).expanduser().resolve())
    if initial_frame is None and initial_pose is not None:
        initial_frame = build_initial_frame(config, initial_pose=initial_pose)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = MainWindow(config=config, initial_frame=initial_frame)
    window.show()
    return app.exec()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="YAML configuration; paths may be relative to the YAML file")
    parser.add_argument("--map", dest="map_path", help="Legacy global target cloud (PCD/PLY/LAS/LAZ)")
    parser.add_argument("--source", help="Legacy single-frame cloud already expressed in map coordinates")
    parser.add_argument("--pose", help="Legacy Tr_velo_to_map TXT or 4x4 pose YAML")
    parser.add_argument("--output-dir", help="Prefill the YAML output directory")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0

    config = load_config(args.config) if args.config else {}
    if args.output_dir:
        config["output_path_yaml"] = str(Path(args.output_dir).expanduser().resolve())

    legacy_values = (args.map_path, args.source, args.pose)
    if any(legacy_values) and not all(legacy_values):
        raise SystemExit("--map, --source, and --pose must be supplied together")
    initial_frame = None
    if all(legacy_values):
        legacy_output = Path(args.output_dir or "alignment_output").expanduser()
        if not legacy_output.is_absolute():
            legacy_output = Path.cwd() / legacy_output
        legacy_output = legacy_output.resolve()
        legacy_output.mkdir(parents=True, exist_ok=True)
        config["output_path_yaml"] = str(legacy_output)
        config["output_path_pcd"] = str(legacy_output)
        config.update({"global_map_path": args.map_path, "frame_cloud_map_path": args.source})
        initial_frame = build_initial_frame(config, pose_path=args.pose)
    elif config:
        source = Path(config.get("frame_cloud_map_path", ""))
        if source.is_file() and _pose_from_config(config) is not None:
            initial_frame = build_initial_frame(config)
    return run_gui(config=config, initial_frame=initial_frame)


if __name__ == "__main__":
    sys.exit(main())
