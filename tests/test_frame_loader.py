import tempfile
from pathlib import Path
import unittest

import numpy as np


class FakeCloud:
    def __init__(self, count):
        self.points = np.zeros((count, 3), dtype=float)


class FrameLoaderTests(unittest.TestCase):
    def make_files(self, root, frame_id):
        frame_dir, pose_dir = root / "frames", root / "poses"
        frame_dir.mkdir(exist_ok=True)
        pose_dir.mkdir(exist_ok=True)
        map_path = root / "map.pcd"
        map_path.write_bytes(b"map")
        (frame_dir / (frame_id + ".pcd")).write_bytes(b"frame")
        (pose_dir / (frame_id + ".txt")).write_text(
            "Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30", encoding="utf-8")
        return map_path, frame_dir, pose_dir

    def test_loads_exact_frame_and_reuses_unchanged_global_map(self):
        from frame_alignment.contracts import LoadRequest
        from frame_alignment.io.frame_loader import FrameLoader

        calls = []
        def read_cloud(path):
            calls.append(Path(path).name)
            return FakeCloud(100 if Path(path).name == "map.pcd" else 20)

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_path, frame_dir, pose_dir = self.make_files(root, "one")
            self.make_files(root, "two")
            loader = FrameLoader(read_cloud)
            first = loader.load_frame(LoadRequest(map_path, frame_dir, pose_dir, "one"))
            second = loader.load_frame(LoadRequest(map_path, frame_dir, pose_dir, "two.pcd"))

        self.assertEqual(calls, ["map.pcd", "one.pcd", "two.pcd"])
        self.assertIs(first.global_map, second.global_map)
        self.assertEqual(second.frame_id, "two")
        np.testing.assert_array_equal(second.initial_pose[:3, 3], (10, 20, 30))

    def test_rejects_empty_cloud(self):
        from frame_alignment.contracts import LoadRequest
        from frame_alignment.io.frame_loader import FrameLoader

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_path, frame_dir, pose_dir = self.make_files(root, "one")
            loader = FrameLoader(lambda path: FakeCloud(0))
            with self.assertRaisesRegex(ValueError, "empty"):
                loader.load_frame(LoadRequest(map_path, frame_dir, pose_dir, "one"))


if __name__ == "__main__":
    unittest.main()
