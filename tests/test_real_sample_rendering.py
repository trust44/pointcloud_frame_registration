"""Environment-backed regression coverage for the supplied Jinhua sample."""
import os
from pathlib import Path
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.app.frame_catalog import FrameCatalog
from frame_alignment.core.point_cloud import read_cloud
from frame_alignment.core.profiles import extract_slice
from frame_alignment.ui.profile_view import ProfileView
from frame_alignment.ui.scene_3d import Scene3DView


MAP_PATH = Path(
    "D:/1_data/map_seg/global_map/jinhua/colored_map_global_voxel_blue_filled.pcd"
)
SOURCE_PATH = Path(
    "D:/1_data/map_seg/global_map/jinhua/velodyne_map/1781158324500077000.pcd"
)
POSE = np.array([
    [0.807819, -0.589212, 0.016066, 8628.860000],
    [0.588546, 0.807800, 0.032764, 9650.010000],
    [-0.032282, -0.017011, 0.999334, 106.995000],
    [0.0, 0.0, 0.0, 1.0],
])


@unittest.skipUnless(
    MAP_PATH.is_file() and SOURCE_PATH.is_file(),
    "Jinhua real-sample PCD files are not available",
)
class RealSampleRenderingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        # These PCD files are intentionally read once each for this test class.
        cls.global_map = read_cloud(MAP_PATH)
        cls.source = read_cloud(SOURCE_PATH)
        cls.c0 = POSE[:3, 3]
        cls.map_display = cls.global_map.roi(cls.c0, 35.0).voxel(0.05)
        cls.source_display = cls.source.voxel(0.05)

    def test_real_sample_populates_3d_roi_and_frame_catalog(self):
        self.assertGreater(len(self.global_map.points), 6_000_000)
        self.assertGreater(len(self.source.points), 90_000)
        self.assertGreater(len(self.global_map.roi(self.c0, 35.0).points), 800_000)
        self.assertGreater(len(self.map_display.points), 0)
        self.assertGreater(len(self.source_display.points), 0)

        catalog = FrameCatalog()
        self.assertIn(SOURCE_PATH.stem, catalog.scan(SOURCE_PATH.parent))

    def test_real_sample_populates_all_profiles_including_xz(self):
        profiles = {}
        for angle in (0.0, 90.0, 45.0, -45.0):
            reference = extract_slice(self.map_display.points, self.c0, angle, 20.0, 0.2)
            adjusted = extract_slice(self.source_display.points, self.c0, angle, 20.0, 0.2)
            self.assertGreater(len(reference), 0, "reference angle={}".format(angle))
            self.assertGreater(len(adjusted), 0, "adjusted angle={}".format(angle))
            profiles[angle] = (reference, adjusted)

        reference, adjusted = profiles[0.0]
        view = ProfileView("X-Z / 0\N{DEGREE SIGN}")
        view.set_profile_data(reference, adjusted, 20.0)
        self.assertGreater(len(view.reference_item.data["x"]), 0)
        self.assertGreater(len(view.adjusted_item.data["x"]), 0)
        view.close()

    def test_real_sample_camera_centers_on_lidar_origin(self):
        view = Scene3DView()
        view.set_reference(self.map_display.points)
        view.set_adjusted(self.source_display.points)
        view.focus_on(self.c0, 35.0)
        self.assertEqual(len(view.reference_item.pos), len(self.map_display.points))
        self.assertEqual(len(view.adjusted_item.pos), len(self.source_display.points))
        center = view.opts["center"]
        np.testing.assert_allclose([center.x(), center.y(), center.z()], self.c0)
        view.close()


if __name__ == "__main__":
    unittest.main()
