import tempfile
from pathlib import Path
import unittest

import numpy as np


class MapAnchorTests(unittest.TestCase):
    def test_valid_anchor_subtracts_offset_without_mutating_input_pose(self):
        from frame_alignment.io.map_anchor import pose_in_map_coordinates

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_path = root / "map.pcd"
            map_path.touch()
            (root / "map_anchor.yaml").write_text(
                "map_translation_offset_xyz: [-10000, -10000, 0]\n",
                encoding="utf-8",
            )
            pose = np.eye(4)
            pose[:3, 3] = (10.0, 20.0, 30.0)

            corrected = pose_in_map_coordinates(pose, map_path)

        np.testing.assert_array_equal(corrected[:3, 3], (10010.0, 10020.0, 30.0))
        np.testing.assert_array_equal(pose[:3, 3], (10.0, 20.0, 30.0))

    def test_missing_anchor_returns_an_unchanged_pose_copy(self):
        from frame_alignment.io.map_anchor import pose_in_map_coordinates

        with tempfile.TemporaryDirectory() as temp:
            map_path = Path(temp) / "map.pcd"
            pose = np.eye(4)
            pose[:3, 3] = (10.0, 20.0, 30.0)

            corrected = pose_in_map_coordinates(pose, map_path)

        self.assertIsNot(corrected, pose)
        np.testing.assert_array_equal(corrected, pose)

    def test_malformed_anchor_reports_its_path(self):
        from frame_alignment.io.map_anchor import pose_in_map_coordinates

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_path = root / "map.pcd"
            anchor_path = root / "map_anchor.yaml"
            anchor_path.write_text(
                "map_translation_offset_xyz: [1, .nan]\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "map_anchor.yaml"):
                pose_in_map_coordinates(np.eye(4), map_path)


if __name__ == "__main__":
    unittest.main()
