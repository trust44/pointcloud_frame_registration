import os
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets


class ViewGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_slice_rectangle_vertices_follow_center_length_width_and_angle(self):
        from frame_alignment.ui.scene_3d import slice_rectangle_vertices

        vertices = slice_rectangle_vertices(np.array((10.0, 20.0, 3.0)), 2.0, 1.0, 0.0)
        expected = np.array([
            [8.0, 19.5, 3.0],
            [12.0, 19.5, 3.0],
            [12.0, 20.5, 3.0],
            [8.0, 20.5, 3.0],
            [8.0, 19.5, 3.0],
        ])
        np.testing.assert_allclose(vertices, expected)

        diagonal = slice_rectangle_vertices(np.zeros(3), np.sqrt(2.0), 2.0, 45.0)
        np.testing.assert_allclose(diagonal[0], (1.0 / np.sqrt(2.0) - 1.0, -1.0 - 1.0 / np.sqrt(2.0), 0.0), atol=1e-12)

    def test_overlay_specs_have_required_names_and_colors(self):
        from frame_alignment.ui.scene_3d import SLICE_SPECS

        self.assertEqual([item.name for item in SLICE_SPECS], [
            "X-Z / 0\u00b0", "Y-Z / 90\u00b0", "Diag +45\u00b0", "Diag -45\u00b0"])
        self.assertEqual([item.color for item in SLICE_SPECS], [
            (1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0),
            (0.0, 0.45, 1.0, 1.0), (0.65, 0.2, 0.8, 1.0)])

    def test_focus_on_centers_camera_on_high_coordinate_origin(self):
        from frame_alignment.ui.scene_3d import Scene3DView

        view = Scene3DView()
        center = np.array([8628.86, 9650.01, 106.995])
        view.focus_on(center, 35.0)

        actual = view.opts["center"]
        np.testing.assert_allclose([actual.x(), actual.y(), actual.z()], center)
        self.assertGreaterEqual(view.opts["distance"], 70.0)

    def test_focus_on_ignores_malformed_center_or_radius(self):
        from frame_alignment.ui.scene_3d import Scene3DView

        view = Scene3DView()
        before = view.opts["center"]
        before_center = [before.x(), before.y(), before.z()]
        before_distance = view.opts["distance"]

        view.focus_on(("not", "a", "coordinate"), 35.0)
        view.focus_on(np.array([8628.86, 9650.01, 106.995]), None)

        actual = view.opts["center"]
        np.testing.assert_allclose([actual.x(), actual.y(), actual.z()], before_center)
        self.assertEqual(view.opts["distance"], before_distance)

    def test_focus_on_ignores_overflowing_numeric_input(self):
        """Extreme Python integers cannot escape the camera validation boundary."""
        from frame_alignment.ui.scene_3d import Scene3DView

        view = Scene3DView()
        before = view.opts["center"]
        before_center = [before.x(), before.y(), before.z()]
        before_distance = view.opts["distance"]

        view.focus_on([10 ** 10000, 0, 0], 35.0)

        actual = view.opts["center"]
        np.testing.assert_allclose([actual.x(), actual.y(), actual.z()], before_center)
        self.assertEqual(view.opts["distance"], before_distance)

    def test_profile_legend_is_stable_and_uses_independent_items(self):
        from frame_alignment.ui.profile_view import ProfileView

        view = ProfileView("X-Z / 0\u00b0")
        reference_item = view.reference_item
        adjusted_item = view.adjusted_item
        first_count = len(view.legend.items)
        view.set_reference_points(np.array([[0.0, 1.0], [1.0, 2.0]]))
        view.set_adjusted_points(np.array([[0.1, 1.1]]))
        view.set_adjusted_points(np.array([[0.2, 1.2], [0.3, 1.3]]))

        self.assertIsNot(reference_item, adjusted_item)
        self.assertIs(reference_item, view.reference_item)
        self.assertIs(adjusted_item, view.adjusted_item)
        self.assertEqual(first_count, 2)
        self.assertEqual(len(view.legend.items), 2)
        self.assertEqual([label.text for _, label in view.legend.items], [
            "Reference Map\uff08\u5168\u5c40\u70b9\u4e91\uff09",
            "Adjusted Frame\uff08\u5355\u5e27\u70b9\u4e91\uff09",
        ])


if __name__ == "__main__":
    unittest.main()
