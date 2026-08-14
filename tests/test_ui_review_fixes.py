import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.contracts import FrameData
from frame_alignment.core.point_cloud import Cloud
from tests.test_main_window_integration import FakeProfile, FakeScene


class UiReviewFixTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_export_button_requires_existing_directories(self):
        from frame_alignment.ui.data_io_panel import DataIOPanel

        with tempfile.TemporaryDirectory() as temp:
            panel = DataIOPanel()
            panel.set_frame_loaded(True)
            panel.yaml_dir_edit.setText(str(Path(temp) / "missing"))
            self.assertFalse(panel.export_button.isEnabled())
            panel.yaml_dir_edit.setText(temp)
            self.assertTrue(panel.export_button.isEnabled())
            panel.export_pcd_check.setChecked(True)
            panel.pcd_dir_edit.setText(str(Path(temp) / "missing-pcd"))
            self.assertFalse(panel.export_button.isEnabled())

    def test_data_panel_is_collapsible(self):
        from frame_alignment.ui.data_io_panel import DataIOPanel

        panel = DataIOPanel()
        self.assertTrue(panel.isCheckable())
        self.assertTrue(panel.isChecked())
        panel.setChecked(False)
        self.assertTrue(panel.content_widget.isHidden())
        panel.setChecked(True)
        self.assertFalse(panel.content_widget.isHidden())

    def test_success_status_contains_actual_pose_file(self):
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            pose_path = root / "actual.txt"
            frame = FrameData(
                "actual", root / "map.pcd", root / "actual.pcd", pose_path,
                Cloud([[0, 0, 0]]), Cloud([[0, 0, 0]]), np.eye(4),
            )

            class Loader:
                def load_frame(self, request):
                    return frame

            window = MainWindow(loader=Loader(), scene=FakeScene(), profile_factory=FakeProfile,
                                message_sink=lambda level, text: None)
            self.assertTrue(window.load_current_frame())
            self.assertIn("pose={}".format(pose_path), window.statusBar().currentMessage())


if __name__ == "__main__":
    unittest.main()
