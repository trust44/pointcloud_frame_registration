import os
from pathlib import Path
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

import numpy as np

from frame_alignment.contracts import FrameData
from frame_alignment.core.point_cloud import Cloud
from tests.test_main_window_integration import FakeProfile, FakeScene


class QualityInvalidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_every_manual_pose_mutation_invalidates_quality(self):
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame = FrameData(
                "actual", root / "map.pcd", root / "actual.pcd", root / "actual.txt",
                Cloud([[0, 0, 0]]), Cloud([[0, 0, 0]]), np.eye(4),
            )

            class Loader:
                def load_frame(self, request):
                    return frame

            window = MainWindow(loader=Loader(), scene=FakeScene(), profile_factory=FakeProfile,
                                message_sink=lambda level, text: None)
            window.load_current_frame()

            mutations = (
                lambda: window.nudge("dx_m", 1),
                lambda: window.set_field("dy_m", 0.25),
                window.reset,
                window.undo,
                window.redo,
            )
            for mutate in mutations:
                window.controller.quality.update({"icp_rmse_m": 0.1, "icp_fitness": 0.9})
                mutate()
                self.assertTrue(all(value is None for value in window.controller.quality.values()))


if __name__ == "__main__":
    unittest.main()
