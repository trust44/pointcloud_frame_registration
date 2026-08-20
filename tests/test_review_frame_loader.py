import tempfile
from pathlib import Path
import unittest

import numpy as np
import yaml


class FakeCloud:
    def __init__(self, count=20):
        self.points = np.zeros((count, 3), dtype=np.float64)


class ReviewFrameLoaderTests(unittest.TestCase):
    def _files(self, root, frame_id="one", pose=None):
        cloud_dir = root / "registered"
        pose_dir = root / "poses"
        cloud_dir.mkdir(exist_ok=True)
        pose_dir.mkdir(exist_ok=True)
        map_path = root / "map.pcd"
        map_path.touch()
        (cloud_dir / (frame_id + ".pcd")).touch()
        pose = np.eye(4) if pose is None else pose
        (pose_dir / (frame_id + ".yaml")).write_text(
            yaml.safe_dump({"corrected_T_map_lidar": np.asarray(pose).tolist()}), encoding="utf-8")
        return map_path, cloud_dir, pose_dir

    def test_loads_registered_cloud_and_corrected_pose_with_map_cache(self):
        from frame_alignment.contracts import ReviewLoadRequest
        from frame_alignment.io.frame_loader import ReviewFrameLoader

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pose = np.eye(4)
            pose[:3, 3] = (10.0, 20.0, 3.0)
            map_path, cloud_dir, pose_dir = self._files(root, "one", pose)
            self._files(root, "two", pose)
            reads = []

            def reader(path):
                reads.append(Path(path).name)
                return FakeCloud()

            loader = ReviewFrameLoader(reader)
            first = loader.load_frame(ReviewLoadRequest(map_path, cloud_dir, pose_dir, "one"))
            second = loader.load_frame(ReviewLoadRequest(map_path, cloud_dir, pose_dir, "two"))

        self.assertEqual(reads, ["map.pcd", "one.pcd", "two.pcd"])
        self.assertIs(first.global_map, second.global_map)
        self.assertEqual(first.initial_pose_path.suffix, ".yaml")
        np.testing.assert_allclose(first.initial_pose, pose)

    def test_missing_pose_yaml_uses_registered_cloud_median(self):
        from frame_alignment.contracts import ReviewLoadRequest
        from frame_alignment.io.frame_loader import ReviewFrameLoader

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            map_path, cloud_dir, pose_dir = self._files(root)
            (pose_dir / "one.yaml").unlink()
            request = ReviewLoadRequest(map_path, cloud_dir, pose_dir, "one")
            cloud = FakeCloud()
            cloud.points = np.array([[1.0, 20.0, 3.0], [5.0, 24.0, 9.0], [100.0, 1.0, 0.0]])
            frame = ReviewFrameLoader(lambda path: cloud).load_frame(request)
            np.testing.assert_allclose(frame.initial_pose[:3, 3], (5.0, 20.0, 3.0))
            self.assertIsNone(frame.initial_pose_path)

    def test_rejects_existing_export_yaml_without_corrected_pose(self):
        from frame_alignment.contracts import ReviewLoadRequest
        from frame_alignment.io.frame_loader import ReviewFrameLoader

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            map_path, cloud_dir, pose_dir = self._files(root)
            (pose_dir / "one.yaml").write_text("frame_id: one\n", encoding="utf-8")
            request = ReviewLoadRequest(map_path, cloud_dir, pose_dir, "one")
            with self.assertRaisesRegex(ValueError, "corrected_T_map_lidar"):
                ReviewFrameLoader(lambda path: FakeCloud()).load_frame(request)

    def test_omitted_pose_directory_uses_registered_cloud_median(self):
        from frame_alignment.contracts import ReviewLoadRequest
        from frame_alignment.io.frame_loader import ReviewFrameLoader

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            map_path, cloud_dir, _ = self._files(root)
            cloud = FakeCloud()
            cloud.points = np.array([[0.0, 1.0, 2.0], [10.0, 11.0, 12.0]])
            frame = ReviewFrameLoader(lambda path: cloud).load_frame(
                ReviewLoadRequest(map_path, cloud_dir, "", "one"))
            np.testing.assert_allclose(frame.initial_pose[:3, 3], (5.0, 6.0, 7.0))
