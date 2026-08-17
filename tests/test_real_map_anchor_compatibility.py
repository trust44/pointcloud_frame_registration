"""Minimal real-data coverage for map-anchor-aware loading and profiles."""
import gc
from pathlib import Path
import unittest

import numpy as np

from frame_alignment.contracts import LoadRequest
from frame_alignment.core.point_cloud import read_cloud
from frame_alignment.core.profiles import default_profile_specs, extract_slice, profile_geometry
from frame_alignment.io.frame_loader import FrameLoader


DATA_ROOT = Path("D:/1_data/map_seg/global_map")
DATASETS = {
    "jinhua": {
        "map": DATA_ROOT / "jinhua/colored_map_global_voxel_blue_filled.pcd",
        "frame_dir": DATA_ROOT / "jinhua/velodyne_map",
        "pose_dir": DATA_ROOT / "jinhua/calib",
        "frame_id": "1781158324500077000",
        "origin": (8628.86, 9650.01, 106.995),
    },
    "yinxiu": {
        "map": DATA_ROOT / "yinxiu/yinxiu_colored_map_world_voxel_blue_filled.pcd",
        "frame_dir": DATA_ROOT / "yinxiu/velodyne_map",
        "pose_dir": DATA_ROOT / "yinxiu/calib",
        "frame_id": "1775716327400118000",
        "origin": (9871.832, 10003.86496, 48.5892),
    },
    "xiangxue": {
        "map": DATA_ROOT / "xiangxue/colored_map_world_voxel_blue_filled.pcd",
        "frame_dir": DATA_ROOT / "xiangxue/velodyne_map",
        "pose_dir": DATA_ROOT / "xiangxue/calib",
        "frame_id": "1774253268200090112",
        "origin": (419.2, 9837.7, 18.8704),
    },
}


class RealMapAnchorCompatibilityTests(unittest.TestCase):
    def _assert_dataset_loads_and_populates_profiles(self, name):
        dataset = DATASETS[name]
        required = (
            dataset["map"],
            dataset["frame_dir"] / (dataset["frame_id"] + ".pcd"),
            dataset["pose_dir"] / (dataset["frame_id"] + ".txt"),
        )
        if not all(path.is_file() for path in required):
            raise unittest.SkipTest("{} real sample is unavailable".format(name))

        frame = FrameLoader(read_cloud).load_frame(LoadRequest(
            dataset["map"], dataset["frame_dir"], dataset["pose_dir"],
            dataset["frame_id"],
        ))
        try:
            np.testing.assert_allclose(
                frame.initial_pose[:3, 3], dataset["origin"], atol=1e-9)
            roi = frame.global_map.roi(frame.initial_pose[:3, 3], 35.0)
            self.assertGreater(len(roi.points), 0, "{} map ROI".format(name))
            for spec in default_profile_specs():
                geometry = profile_geometry(
                    spec, frame.initial_pose[:3, 3], frame.initial_pose[:3, :3])
                reference = extract_slice(roi.points, geometry, 20.0, 0.2)
                source = extract_slice(
                    frame.source_cloud.points, geometry, 20.0, 0.2)
                self.assertGreater(
                    len(reference), 0, "{} reference {}".format(name, spec.name))
                self.assertGreater(
                    len(source), 0, "{} source {}".format(name, spec.name))
        finally:
            del frame
            gc.collect()

    def test_jinhua_loads_and_populates_profiles(self):
        self._assert_dataset_loads_and_populates_profiles("jinhua")

    def test_yinxiu_loads_and_populates_profiles(self):
        self._assert_dataset_loads_and_populates_profiles("yinxiu")

    def test_xiangxue_loads_and_populates_profiles(self):
        self._assert_dataset_loads_and_populates_profiles("xiangxue")


if __name__ == "__main__":
    unittest.main()
