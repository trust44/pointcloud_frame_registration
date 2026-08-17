import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.core.profiles import ANGLE_MODE, PARALLEL_MODE


class ProfileControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_controls_start_with_four_defaults_and_limit_extras(self):
        from frame_alignment.ui.profile_controls import ProfileControls

        controls = ProfileControls()
        self.assertEqual(len(controls.profile_specs), 4)

        controls.add_profile()
        controls.add_profile()
        controls.add_profile()

        self.assertEqual(
            [(spec.grid_row, spec.grid_column) for spec in controls.profile_specs[-2:]],
            [(0, 2), (1, 2)],
        )
        self.assertFalse(controls.add_button.isEnabled())

    def test_only_extra_profiles_can_be_deleted(self):
        from frame_alignment.ui.profile_controls import ProfileControls

        controls = ProfileControls()
        controls.select_profile("diag_plus")
        self.assertFalse(controls.delete_button.isEnabled())

        controls.add_profile()
        controls.select_profile("extra_1")
        self.assertTrue(controls.delete_button.isEnabled())
        controls.delete_selected_profile()

        self.assertEqual(len(controls.profile_specs), 4)
        self.assertNotIn("extra_1", {spec.profile_id for spec in controls.profile_specs})

    def test_fixed_profiles_disable_position_editors(self):
        from frame_alignment.ui.profile_controls import ProfileControls

        controls = ProfileControls()
        controls.select_profile("xz")
        self.assertFalse(controls.mode_combo.isEnabled())
        self.assertFalse(controls.angle_edit.isEnabled())
        self.assertFalse(controls.reference_combo.isEnabled())
        self.assertFalse(controls.offset_edit.isEnabled())

        controls.select_profile("diag_plus")
        self.assertTrue(controls.mode_combo.isEnabled())
        self.assertTrue(controls.angle_edit.isEnabled())
        self.assertFalse(controls.reference_combo.isEnabled())

    def test_diagonal_can_switch_to_parallel_signed_offset(self):
        from frame_alignment.ui.profile_controls import ProfileControls

        controls = ProfileControls()
        snapshots = []
        controls.profiles_changed.connect(snapshots.append)
        controls.select_profile("diag_plus")
        controls.mode_combo.setCurrentIndex(controls.mode_combo.findData(PARALLEL_MODE))
        controls.reference_combo.setCurrentText("YZ")
        controls.offset_edit.setValue(-2.5)

        spec = next(spec for spec in controls.profile_specs if spec.profile_id == "diag_plus")
        self.assertEqual((spec.mode, spec.reference, spec.offset_m), (PARALLEL_MODE, "YZ", -2.5))
        self.assertFalse(controls.angle_edit.isEnabled())
        self.assertTrue(controls.reference_combo.isEnabled())
        self.assertTrue(controls.offset_edit.isEnabled())
        self.assertTrue(snapshots)
        self.assertIsInstance(snapshots[-1], tuple)

    def test_angle_editor_is_bounded_and_updates_selected_profile(self):
        from frame_alignment.ui.profile_controls import ProfileControls

        controls = ProfileControls()
        controls.select_profile("diag_minus")
        self.assertEqual((controls.angle_edit.minimum(), controls.angle_edit.maximum()), (-180.0, 180.0))
        controls.mode_combo.setCurrentIndex(controls.mode_combo.findData(ANGLE_MODE))
        controls.angle_edit.setValue(30.0)

        spec = next(spec for spec in controls.profile_specs if spec.profile_id == "diag_minus")
        self.assertEqual(spec.angle_deg, 30.0)


if __name__ == "__main__":
    unittest.main()
