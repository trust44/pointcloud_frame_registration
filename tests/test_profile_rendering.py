import os
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets


class ProfileRenderingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_set_profile_data_filters_nonfinite_points_and_uses_union_range(self):
        from frame_alignment.ui.profile_view import ProfileView

        view = ProfileView("X-Z / 0°")
        view.set_profile_data(
            np.array([[-2.0, -1.0], [0.0, np.nan], [2.0, 4.0]]),
            np.array([[-1.0, -3.0], [1.0, np.inf]]),
            20.0,
        )

        self.assertEqual(len(view.reference_item.data["x"]), 2)
        self.assertEqual(len(view.adjusted_item.data["x"]), 1)
        self.assertEqual(view.plotItem.viewRange()[0], [-20.0, 20.0])
        self.assertLess(view.plotItem.viewRange()[1][0], -3.0)
        self.assertGreater(view.plotItem.viewRange()[1][1], 4.0)
        self.assertFalse(view.empty_item.isVisible())

    def test_empty_state_and_legend_stay_stable_across_profile_updates(self):
        from frame_alignment.ui.profile_view import ProfileView

        view = ProfileView("X-Z / 0°")
        view.set_profile_data(np.empty((0, 2)), np.empty((0, 2)), 20.0)

        self.assertEqual(len(view.reference_item.data["x"]), 0)
        self.assertEqual(len(view.adjusted_item.data["x"]), 0)
        self.assertTrue(view.empty_item.isVisible())
        self.assertEqual(len(view.legend.items), 2)

        view.set_profile_data(np.array([[0.0, 1.0]]), np.empty((0, 2)), 20.0)
        self.assertFalse(view.empty_item.isVisible())
        self.assertEqual(len(view.legend.items), 2)

    def test_constant_z_points_keep_a_nonzero_visible_range(self):
        from frame_alignment.ui.profile_view import ProfileView

        view = ProfileView("X-Z / 0°")
        view.set_profile_data(np.array([[-1.0, 3.0], [1.0, 3.0]]), np.empty((0, 2)), 20.0)

        low, high = view.plotItem.viewRange()[1]
        self.assertLess(low, 3.0)
        self.assertGreater(high, 3.0)

    def test_title_can_change_without_recreating_legend_items(self):
        from frame_alignment.ui.profile_view import ProfileView

        view = ProfileView("Initial")
        legend_items = tuple(view.legend.items)
        view.set_title("Updated profile")
        self.assertEqual(view.plotItem.titleLabel.text, "Updated profile")
        self.assertEqual(tuple(view.legend.items), legend_items)
if __name__ == "__main__":
    unittest.main()
