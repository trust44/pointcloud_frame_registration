import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

from frame_alignment.contracts import LoadRequest


ENTRY = Path(__file__).parents[1] / "src" / "frame_align_6dof.py"


class FakeCloud:
    def __init__(self):
        self.points = np.zeros((10, 3), dtype=np.float64)


def load_entry():
    spec = importlib.util.spec_from_file_location("map_anchor_cli_entry", ENTRY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MapAnchorIntegrationTests(unittest.TestCase):
    def test_frame_loader_applies_sibling_map_anchor(self):
        from frame_alignment.io.frame_loader import FrameLoader

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame_dir = root / "frames"
            pose_dir = root / "poses"
            frame_dir.mkdir()
            pose_dir.mkdir()
            map_path = root / "map.pcd"
            map_path.touch()
            (frame_dir / "one.pcd").touch()
            (pose_dir / "one.txt").write_text(
                "Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30\n",
                encoding="utf-8",
            )
            (root / "map_anchor.yaml").write_text(
                "map_translation_offset_xyz: [-10000, -10000, 0]\n",
                encoding="utf-8",
            )

            frame = FrameLoader(lambda path: FakeCloud()).load_frame(
                LoadRequest(map_path, frame_dir, pose_dir, "one"))

        np.testing.assert_array_equal(
            frame.initial_pose[:3, 3], (10010.0, 10020.0, 30.0))

    def test_legacy_initial_frame_applies_sibling_map_anchor(self):
        module = load_entry()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_path = root / "map.pcd"
            source_path = root / "frame.pcd"
            pose_path = root / "frame.txt"
            map_path.touch()
            source_path.touch()
            pose_path.write_text(
                "Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30\n",
                encoding="utf-8",
            )
            (root / "map_anchor.yaml").write_text(
                "map_translation_offset_xyz: [50000, 10000, 0]\n",
                encoding="utf-8",
            )

            frame = module.build_initial_frame(
                {"global_map_path": str(map_path),
                 "frame_cloud_map_path": str(source_path)},
                pose_path=pose_path,
                cloud_reader=lambda path: module.Cloud([[1.0, 2.0, 3.0]]),
            )

        np.testing.assert_array_equal(
            frame.initial_pose[:3, 3], (-49990.0, -9980.0, 30.0))


if __name__ == "__main__":
    unittest.main()
