from pathlib import Path
from types import SimpleNamespace
import os
import tempfile
import unittest
from unittest import mock

import numpy as np

from frame_alignment.contracts import ExportRequest, FrameData
from frame_alignment.core.point_cloud import Cloud
from frame_alignment.io.exporter import AlignmentExportState, export_result


class ExportCommitRollbackTests(unittest.TestCase):
    def test_second_file_commit_failure_rolls_back_both_outputs(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            yaml_path = root / "frame.yaml"
            pcd_path = root / "frame.pcd"
            yaml_path.write_text("old-yaml", encoding="utf-8")
            pcd_path.write_text("old-pcd", encoding="utf-8")
            frame = FrameData("frame", root / "map.pcd", pcd_path, root / "frame.txt",
                              Cloud([[0, 0, 0]]), Cloud([[1, 2, 3]]), np.eye(4))
            delta = SimpleNamespace(dx_m=0, dy_m=0, dz_m=0,
                                    roll_deg=0, pitch_deg=0, yaw_deg=0)
            state = AlignmentExportState(frame, delta, np.eye(4), np.eye(4), [[1, 2, 3]])

            def writer(path, points):
                Path(path).write_text("new-pcd", encoding="utf-8")

            real_replace = os.replace

            def fail_pcd_commit(source, destination):
                if Path(destination) == pcd_path and str(source).endswith(".tmp.pcd"):
                    raise OSError("commit interrupted")
                return real_replace(source, destination)

            with mock.patch("frame_alignment.io.exporter.os.replace", side_effect=fail_pcd_commit):
                with self.assertRaisesRegex(OSError, "commit interrupted"):
                    export_result(ExportRequest(root, True, root), state,
                                  cloud_writer=writer, overwrite=True)

            self.assertEqual(yaml_path.read_text(encoding="utf-8"), "old-yaml")
            self.assertEqual(pcd_path.read_text(encoding="utf-8"), "old-pcd")
            self.assertEqual(list(root.glob(".*.tmp*")), [])
            self.assertEqual(list(root.glob(".*.bak")), [])


if __name__ == "__main__":
    unittest.main()
