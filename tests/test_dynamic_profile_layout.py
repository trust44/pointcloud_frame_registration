import os
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets


class FakeScene(QtWidgets.QWidget):
    def set_reference(self, points):
        pass

    def focus_on(self, center, roi_radius):
        pass

    def set_adjusted(self, points):
        pass

    def update_origin(self, center, rotation, axis_length=1.5):
        pass

    def update_slice_overlays(self, geometries, half_length):
        self.geometries = tuple(geometries)
        self.half_length = half_length


class FakeProfile(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        self.title = title

    def set_title(self, title):
        self.title = title

    def set_profile_data(self, reference_points, adjusted_points, half_length):
        self.reference = np.asarray(reference_points)
        self.adjusted = np.asarray(adjusted_points)
        self.half_length = half_length


class DynamicProfileLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def make_window(self, config=None):
        from frame_alignment.ui.main_window import MainWindow

        return MainWindow(
            config=config,
            scene=FakeScene(),
            profile_factory=FakeProfile,
            message_sink=lambda level, text: None,
        )

    def test_default_layout_and_half_length_bounds(self):
        window = self.make_window()

        self.assertEqual(len(window.profiles), 4)
        self.assertEqual(window.length_edit.minimum(), 10.0)
        self.assertEqual(window.length_edit.maximum(), 35.0)
        self.assertEqual(window.length_edit.value(), 20.0)
        self.assertEqual(window.profile_grid_positions(), {
            "xz": (0, 0),
            "yz": (1, 0),
            "diag_plus": (0, 1),
            "diag_minus": (1, 1),
        })
        self.assertEqual(window.profile_column_count, 2)
        scene_position = window.visualization_layout.getItemPosition(
            window.visualization_layout.indexOf(window.scene))
        self.assertEqual(scene_position, (0, 0, 1, 2))
        for spec in window.profile_specs:
            position = window.visualization_layout.getItemPosition(
                window.visualization_layout.indexOf(window._profile_widgets[spec.profile_id]))
            self.assertEqual(position[:2], (1 + spec.grid_row, spec.grid_column))

        window.close()

    def test_invalid_configured_half_length_falls_back_to_twenty(self):
        for value in (9.0, 36.0, float("nan"), "invalid"):
            with self.subTest(value=value):
                window = self.make_window({"display": {"slice_half_length_m": value}})
                self.assertEqual(window.slice_half_length, 20.0)
                self.assertEqual(window.length_edit.value(), 20.0)
                window.close()

    def test_extra_profiles_expand_to_three_columns_and_delete_back_to_two(self):
        window = self.make_window()
        window.profile_controls.add_profile()
        self.assertEqual(window.profile_grid_positions()["extra_1"], (0, 2))
        self.assertEqual(window.profile_column_count, 3)
        scene_position = window.visualization_layout.getItemPosition(
            window.visualization_layout.indexOf(window.scene))
        self.assertEqual(scene_position, (0, 0, 1, 3))

        window.profile_controls.add_profile()
        self.assertEqual(window.profile_grid_positions()["extra_2"], (1, 2))
        self.assertEqual(len(window.profiles), 6)

        window.profile_controls.select_profile("extra_1")
        window.profile_controls.delete_selected_profile()
        window.profile_controls.select_profile("extra_2")
        window.profile_controls.delete_selected_profile()
        self.assertEqual(len(window.profiles), 4)
        self.assertEqual(window.profile_column_count, 2)
        window.close()

    def test_profile_state_is_reset_for_each_new_window(self):
        first = self.make_window()
        first.profile_controls.add_profile()
        first.profile_controls.select_profile("diag_plus")
        first.profile_controls.angle_edit.setValue(12.0)

        second = self.make_window()
        self.assertEqual(len(second.profile_specs), 4)
        diagonal = next(spec for spec in second.profile_specs if spec.profile_id == "diag_plus")
        self.assertEqual(diagonal.angle_deg, 45.0)
        first.close()
        second.close()


if __name__ == "__main__":
    unittest.main()
