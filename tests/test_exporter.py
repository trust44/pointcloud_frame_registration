from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import numpy as np
import yaml


@dataclass
class DummyFrame:
    frame_id: str
    global_map_path: Path
    frame_cloud_path: Path
    initial_pose_path: Path
    initial_pose: object


class ExporterTests(unittest.TestCase):
    def state(self, root):
        from frame_alignment.io.exporter import AlignmentExportState

        frame = DummyFrame(
            "untrusted-user-text",
            root / "map.pcd",
            root / "actual-frame.pcd",
            root / "actual-frame.txt",
            np.eye(4),
        )
        delta = SimpleNamespace(dx_m=1.0, dy_m=2.0, dz_m=3.0,
                                roll_deg=0.1, pitch_deg=0.2, yaw_deg=0.3)
        adjusted = np.array([[1.0, 2.0, 3.0]])
        quality = {"rmse_m": 0.125, "sample_count": 42}
        return AlignmentExportState(frame, delta, np.eye(4), np.eye(4), adjusted, quality)

    def test_yaml_is_named_from_actual_loaded_pcd_stem(self):
        from frame_alignment.contracts import ExportRequest
        from frame_alignment.io.exporter import export_result

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            outcome = export_result(ExportRequest(root, False, ""), self.state(root))
            self.assertEqual(outcome.yaml_path, root / "actual-frame.yaml")
            self.assertIsNone(outcome.pcd_path)
            data = yaml.safe_load(outcome.yaml_path.read_text(encoding="utf-8"))
            self.assertEqual(data["frame_id"], "actual-frame")
            self.assertEqual(data["input"]["frame_cloud_map_path"], str((root / "actual-frame.pcd").resolve()))
            self.assertEqual(data["manual_delta_about_lidar_origin"]["pivot_initial_lidar_origin_map"], [0.0, 0.0, 0.0])
            self.assertFalse(data["output"]["adjusted_pcd_written"])
            self.assertEqual(list(root.glob("*.tmp*")), [])

    def test_optional_pcd_uses_adjusted_points_without_second_transform(self):
        from frame_alignment.contracts import ExportRequest
        from frame_alignment.io.exporter import export_result

        writes = []
        def writer(path, cloud):
            writes.append(np.asarray(cloud).copy())
            Path(path).write_text("pcd", encoding="ascii")

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            state = self.state(root)
            outcome = export_result(ExportRequest(root, True, root), state, cloud_writer=writer)
            np.testing.assert_array_equal(writes[0], state.adjusted_points)
            self.assertEqual(outcome.pcd_path, root / "actual-frame.pcd")
            data = yaml.safe_load(outcome.yaml_path.read_text(encoding="utf-8"))
            self.assertTrue(data["output"]["adjusted_pcd_written"])
            self.assertEqual(data["output"]["adjusted_pcd_path"], str(outcome.pcd_path.resolve()))

    def test_pcd_failure_still_writes_truthful_yaml(self):
        from frame_alignment.contracts import ExportRequest
        from frame_alignment.io.exporter import export_result

        def failing_writer(path, cloud):
            raise OSError("disk full")

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            outcome = export_result(ExportRequest(root, True, root), self.state(root), cloud_writer=failing_writer)
            self.assertIn("disk full", outcome.pcd_error)
            data = yaml.safe_load(outcome.yaml_path.read_text(encoding="utf-8"))
            self.assertFalse(data["output"]["adjusted_pcd_written"])
            self.assertIsNone(data["output"]["adjusted_pcd_path"])

    def test_existing_output_requires_explicit_overwrite(self):
        from frame_alignment.contracts import ExportRequest
        from frame_alignment.io.exporter import export_result

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "actual-frame.yaml").write_text("old", encoding="ascii")
            with self.assertRaises(FileExistsError):
                export_result(ExportRequest(root, False, ""), self.state(root))
            self.assertEqual((root / "actual-frame.yaml").read_text(encoding="ascii"), "old")


if __name__ == "__main__":
    unittest.main()
