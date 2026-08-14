from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock

import numpy as np

from frame_alignment.contracts import ExportRequest, FrameData
from frame_alignment.core.point_cloud import Cloud
from frame_alignment.io.exporter import AlignmentExportState, export_result


class ExportAtomicPairTests(unittest.TestCase):
    def test_yaml_staging_failure_leaves_existing_yaml_and_pcd_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            yaml_path = root / "frame.yaml"
            pcd_path = root / "frame.pcd"
            yaml_path.write_text("old-yaml", encoding="utf-8")
            pcd_path.write_text("old-pcd", encoding="utf-8")
            frame = FrameData(
                "frame", root / "map.pcd", root / "frame.pcd", root / "frame.txt",
                Cloud([[0, 0, 0]]), Cloud([[1, 2, 3]]), np.eye(4),
            )
            delta = SimpleNamespace(dx_m=0, dy_m=0, dz_m=0,
                                    roll_deg=0, pitch_deg=0, yaw_deg=0)
            state = AlignmentExportState(frame, delta, np.eye(4), np.eye(4), [[1, 2, 3]])

            def writer(path, points):
                Path(path).write_text("new-pcd", encoding="utf-8")

            with mock.patch("frame_alignment.io.exporter.yaml.safe_dump", side_effect=OSError("yaml failed")):
                with self.assertRaisesRegex(OSError, "yaml failed"):
                    export_result(ExportRequest(root, True, root), state,
                                  cloud_writer=writer, overwrite=True)

            self.assertEqual(yaml_path.read_text(encoding="utf-8"), "old-yaml")
            self.assertEqual(pcd_path.read_text(encoding="utf-8"), "old-pcd")
            self.assertEqual(list(root.glob(".*.tmp*")), [])


if __name__ == "__main__":
    unittest.main()
