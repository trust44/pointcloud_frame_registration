import tempfile
from pathlib import Path
import unittest

import numpy as np


VALID_LINE = (
    "Tr_velo_to_map: 0.807819 -0.589212 0.016066 8628.860000 "
    "0.588546 0.807800 0.032764 9650.010000 "
    "-0.032282 -0.017011 0.999334 106.995000"
)


class PoseParserTests(unittest.TestCase):
    def write_pose(self, root, text):
        path = Path(root) / "frame.txt"
        path.write_text(text, encoding="utf-8")
        return path

    def test_reads_only_unique_tr_velo_to_map(self):
        from frame_alignment.io.pose_parser import parse_tr_velo_to_map

        with tempfile.TemporaryDirectory() as temp:
            path = self.write_pose(temp, "P0: " + "0 " * 12 + "\n" + VALID_LINE + "\nP3: " + "9 " * 12)
            matrix = parse_tr_velo_to_map(path)

        expected = np.array([
            [0.807819, -0.589212, 0.016066, 8628.86],
            [0.588546, 0.807800, 0.032764, 9650.01],
            [-0.032282, -0.017011, 0.999334, 106.995],
            [0.0, 0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(matrix, expected)

    def test_rejects_missing_duplicate_and_wrong_count_with_file_path(self):
        from frame_alignment.io.pose_parser import PoseParseError, parse_tr_velo_to_map

        cases = {
            "missing": "P0: " + "0 " * 12,
            "duplicate": VALID_LINE + "\n" + VALID_LINE,
            "wrong count": "Tr_velo_to_map: " + "1 " * 11,
        }
        for reason, content in cases.items():
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as temp:
                path = self.write_pose(temp, content)
                with self.assertRaises(PoseParseError) as caught:
                    parse_tr_velo_to_map(path)
                self.assertIn(str(path.resolve()), str(caught.exception))
                self.assertIn(reason, str(caught.exception).lower())

    def test_rejects_non_finite_and_non_rigid_rotations(self):
        from frame_alignment.io.pose_parser import PoseParseError, parse_tr_velo_to_map

        invalid_lines = (
            "Tr_velo_to_map: 1 0 0 nan 0 1 0 0 0 0 1 0",
            "Tr_velo_to_map: 2 0 0 0 0 1 0 0 0 0 1 0",
        )
        for content in invalid_lines:
            with self.subTest(content=content), tempfile.TemporaryDirectory() as temp:
                with self.assertRaises(PoseParseError):
                    parse_tr_velo_to_map(self.write_pose(temp, content))


if __name__ == "__main__":
    unittest.main()
