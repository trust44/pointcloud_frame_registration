import os
import tempfile
from pathlib import Path
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets


class DataIOPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_builds_load_request_from_editable_fields(self):
        from frame_alignment.ui.data_io_panel import DataIOPanel

        panel = DataIOPanel()
        panel.global_map_edit.setText("D:/map.pcd")
        panel.frame_dir_edit.setText("D:/frames")
        panel.pose_dir_edit.setText("D:/poses")
        panel.frame_id_edit.setText("frame-01.pcd")
        request = panel.get_load_request()

        self.assertEqual(request.global_map_path, Path("D:/map.pcd"))
        self.assertEqual(request.frame_cloud_map_path, Path("D:/frames"))
        self.assertEqual(request.initial_pose_path, Path("D:/poses"))
        self.assertEqual(request.frame_id, "frame-01.pcd")

    def test_pcd_output_controls_and_export_button_follow_state(self):
        from frame_alignment.ui.data_io_panel import DataIOPanel

        with tempfile.TemporaryDirectory() as temp:
            panel = DataIOPanel()
            self.assertFalse(panel.pcd_dir_edit.isEnabled())
            self.assertFalse(panel.export_button.isEnabled())

            panel.yaml_dir_edit.setText(temp)
            panel.set_frame_loaded(True)
            self.assertTrue(panel.export_button.isEnabled())

            panel.export_pcd_check.setChecked(True)
            self.assertTrue(panel.pcd_dir_edit.isEnabled())
            self.assertFalse(panel.export_button.isEnabled())

            panel.pcd_dir_edit.setText(temp)
            self.assertTrue(panel.export_button.isEnabled())
            request = panel.get_export_request()
            self.assertTrue(request.write_adjusted_pcd)
            self.assertEqual(request.pcd_output_dir, Path(temp))

    def test_load_button_does_not_load_on_path_edit(self):
        from frame_alignment.ui.data_io_panel import DataIOPanel

        panel = DataIOPanel()
        loads = []
        panel.load_requested.connect(lambda: loads.append(True))
        panel.global_map_edit.setText("D:/changed-map.pcd")
        self.app.processEvents()
        self.assertEqual(loads, [])
        panel.load_button.click()
        self.assertEqual(loads, [True])


if __name__ == "__main__":
    unittest.main()
