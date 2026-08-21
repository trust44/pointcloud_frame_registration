"""Request and state contracts shared by loader, UI, controller, and exporter."""
from dataclasses import dataclass
from pathlib import Path
import re


_FRAME_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def normalize_frame_id(value):
    frame_id = str(value).strip()
    lowered = frame_id.lower()
    if lowered.endswith(".pcd") or lowered.endswith(".txt"):
        frame_id = frame_id[:-4]
    if (not frame_id or frame_id in (".", "..") or
            not _FRAME_ID_RE.fullmatch(frame_id) or "/" in frame_id or "\\" in frame_id):
        raise ValueError("frame_id may contain only letters, digits, '_', '-', and '.'")
    return frame_id


def _required_file(path, label):
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError("{} does not exist: {}".format(label, resolved))
    return resolved


def _required_directory(path, label):
    if path is None or not str(path).strip():
        raise ValueError("{} is required".format(label))
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_dir():
        raise NotADirectoryError("{} does not exist: {}".format(label, resolved))
    return resolved


@dataclass(frozen=True)
class ResolvedFramePaths:
    frame_id: str
    global_map_file: Path
    frame_cloud_file: Path
    initial_pose_file: Path


@dataclass(frozen=True)
class LoadRequest:
    global_map_path: object
    frame_cloud_map_path: object
    initial_pose_path: object
    frame_id: str

    def resolve_existing_paths(self):
        frame_id = normalize_frame_id(self.frame_id)
        global_map_file = _required_file(self.global_map_path, "Global map file")
        frame_dir = _required_directory(self.frame_cloud_map_path, "Frame cloud directory")
        pose_dir = _required_directory(self.initial_pose_path, "Initial pose directory")
        frame_cloud_file = _required_file(frame_dir / (frame_id + ".pcd"), "Frame cloud file")
        initial_pose_file = _required_file(pose_dir / (frame_id + ".txt"), "Initial pose file")
        return ResolvedFramePaths(frame_id, global_map_file, frame_cloud_file, initial_pose_file)


@dataclass(frozen=True)
class ResolvedReviewFramePaths:
    """Resolved files for a read-only, already-registered frame."""

    frame_id: str
    global_map_file: Path
    registered_cloud_file: Path
    registered_pose_file: object = None


@dataclass(frozen=True)
class ReviewLoadRequest:
    """Request one map-coordinate PCD plus its exported alignment YAML."""

    global_map_path: object
    registered_cloud_path: object
    registered_pose_path: object
    frame_id: str

    def resolve_existing_paths(self):
        frame_id = normalize_frame_id(self.frame_id)
        global_map_file = _required_file(self.global_map_path, "Global map file")
        cloud_dir = _required_directory(self.registered_cloud_path, "Registered cloud directory")
        registered_cloud_file = _required_file(
            cloud_dir / (frame_id + ".pcd"), "Registered cloud file")
        registered_pose_file = None
        pose_directory = str(self.registered_pose_path).strip() if self.registered_pose_path is not None else ""
        if pose_directory:
            candidate = Path(pose_directory).expanduser().resolve() / (frame_id + ".yaml")
            if candidate.is_file():
                registered_pose_file = candidate
        return ResolvedReviewFramePaths(
            frame_id, global_map_file, registered_cloud_file, registered_pose_file)


@dataclass(frozen=True)
class ExportRequest:
    yaml_output_dir: object
    write_adjusted_pcd: bool = False
    pcd_output_dir: object = ""

    def validate(self):
        yaml_dir = _required_directory(self.yaml_output_dir, "YAML output directory")
        pcd_dir = None
        if self.write_adjusted_pcd:
            pcd_dir = _required_directory(self.pcd_output_dir, "PCD output directory")
        return ExportRequest(yaml_dir, bool(self.write_adjusted_pcd), pcd_dir)


@dataclass(frozen=True)
class FrameData:
    frame_id: str
    global_map_path: Path
    frame_cloud_path: Path
    initial_pose_path: Path
    global_map: object
    source_cloud: object
    initial_pose: object
