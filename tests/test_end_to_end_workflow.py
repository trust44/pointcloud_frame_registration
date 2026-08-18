import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.core.point_cloud import Cloud
from frame_alignment.io.frame_loader import FrameLoader
from tests.test_main_window_integration import FakeProfile, FakeScene


class EndToEndWorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_exact_frame_load_adjust_export_and_map_cache(self):
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame_dir = root / "frames"
            pose_dir = root / "poses"
            output_dir = root / "yaml"
            frame_dir.mkdir()
            pose_dir.mkdir()
            output_dir.mkdir()
            map_path = root / "map.pcd"
            map_path.touch()
            for frame_id, x in (("000123", 10.0), ("000124", 11.0)):
                (frame_dir / (frame_id + ".pcd")).touch()
                (pose_dir / (frame_id + ".txt")).write_text(
                    "P0: ignored\n"
                    "Tr_velo_to_map: 1 0 0 {} 0 1 0 20 0 0 1 2\n".format(x),
                    encoding="utf-8",
                )

            reads = []

            def reader(path):
                path = Path(path).resolve()
                reads.append(path)
                if path == map_path.resolve():
                    return Cloud([[10, 20, 2], [11, 20, 2], [10, 21, 2]])
                return Cloud([[10, 20, 2], [10.5, 20, 2.1]])

            messages = []
            window = MainWindow(
                loader=FrameLoader(reader),
                scene=FakeScene(),
                profile_factory=FakeProfile,
                message_sink=lambda level, text: messages.append((level, text)),
            )
            panel = window.data_panel
            panel.global_map_edit.setText(str(map_path))
            panel.frame_dir_edit.setText(str(frame_dir))
            panel.pose_dir_edit.setText(str(pose_dir))
            panel.yaml_dir_edit.setText(str(output_dir))
            self.assertTrue(panel.scan_frames())
            panel.frame_id_edit.setText("000123.pcd")

            self.assertTrue(window.load_current_frame())
            self.assertEqual(panel.annotation_status_label.text(), "已标注 1/2")
            self.assertEqual(panel.annotation_count_label.text(), "标注量：0/2")
            window.nudge("dx_m", 1)
            outcome = window.export_current_frame()
            self.assertEqual(outcome.yaml_path, output_dir / "000123.yaml")
            self.assertEqual(panel.annotation_status_label.text(), "已标注 1/2")
            self.assertEqual(panel.annotation_count_label.text(), "标注量：1/2")
            exported = yaml.safe_load(outcome.yaml_path.read_text(encoding="utf-8"))
            self.assertAlmostEqual(exported["manual_delta_about_lidar_origin"]["dx_m"], 0.01)
            self.assertEqual(exported["input"]["frame_cloud_map_path"], str((frame_dir / "000123.pcd").resolve()))
            self.assertFalse(exported["output"]["adjusted_pcd_written"])

            panel.frame_id_edit.setText("000124")
            self.assertTrue(window.load_current_frame())
            self.assertEqual(window.controller.pose_model.delta.dx_m, 0.0)
            self.assertEqual(reads.count(map_path.resolve()), 1)
            self.assertEqual(messages[-1][0], "info")


if __name__ == "__main__":
    unittest.main()

