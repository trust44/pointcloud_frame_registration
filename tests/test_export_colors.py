import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.contracts import ExportRequest, FrameData
from frame_alignment.core.point_cloud import Cloud
from frame_alignment.core.pose_model import Delta
from frame_alignment.io.exporter import AlignmentExportState, export_result
from tests.test_main_window_integration import FakeProfile, FakeScene


class ExportColorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def frame(self, root):
        return FrameData(
            "frame", root / "map.pcd", root / "frame.pcd", root / "frame.txt",
            Cloud([[0, 0, 0]]),
            Cloud([[1, 2, 3]], [[0.1, 0.2, 0.3]]),
            np.eye(4),
        )

    def test_main_window_passes_source_colors_to_export_state(self):
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame = self.frame(root)

            class Loader:
                def load_frame(self, request):
                    return frame

            captured = {}

            def fake_export(request, state, overwrite=False):
                captured["state"] = state
                return SimpleNamespace(yaml_path=root / "frame.yaml", pcd_path=None, pcd_error=None)

            window = MainWindow(loader=Loader(), scene=FakeScene(), profile_factory=FakeProfile,
                                message_sink=lambda level, text: None)
            window.data_panel.yaml_dir_edit.setText(temp)
            window.load_current_frame()
            with mock.patch("frame_alignment.ui.main_window.export_result", side_effect=fake_export):
                window.export_current_frame()
            np.testing.assert_array_equal(captured["state"].adjusted_colors, frame.source_cloud.colors)

    def test_default_open3d_writer_attaches_colors(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame = self.frame(root)
            state = AlignmentExportState(
                frame, Delta(), np.eye(4), np.eye(4), frame.source_cloud.points,
                adjusted_colors=frame.source_cloud.colors,
            )
            recorded = {}

            class FakePointCloud:
                pass

            def vector(values):
                return np.asarray(values).copy()

            def write_point_cloud(path, cloud):
                recorded["cloud"] = cloud
                Path(path).write_text("pcd", encoding="ascii")
                return True

            fake_open3d = SimpleNamespace(
                geometry=SimpleNamespace(PointCloud=FakePointCloud),
                utility=SimpleNamespace(Vector3dVector=vector),
                io=SimpleNamespace(write_point_cloud=write_point_cloud),
            )
            with mock.patch.dict(sys.modules, {"open3d": fake_open3d}):
                export_result(ExportRequest(root, True, root), state)
            np.testing.assert_array_equal(recorded["cloud"].colors, frame.source_cloud.colors)


if __name__ == "__main__":
    unittest.main()
