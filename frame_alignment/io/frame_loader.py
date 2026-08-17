"""Strict frame-id based loading with reusable global-map cache."""
from ..contracts import FrameData
from .map_anchor import pose_in_map_coordinates
from .pose_parser import parse_tr_velo_to_map


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
