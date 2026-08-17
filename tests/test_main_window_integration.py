import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.contracts import FrameData
from frame_alignment.core.point_cloud import Cloud


class FakeScene(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.reference_calls = 0
        self.adjusted_calls = 0
        self.overlay_calls = 0
        self.focus_calls = []

    def set_reference(self, points):
        self.reference_calls += 1

    def focus_on(self, center, roi_radius):
        self.focus_calls.append((np.asarray(center), roi_radius))

    def set_adjusted(self, points):
        self.adjusted_calls += 1

    def update_origin(self, center, rotation, axis_length=1.5):
        self.origin = np.asarray(center)

    def update_slice_overlays(self, geometries, half_length):
        self.overlay_calls += 1
        self.geometries = tuple(geometries)
        self.overlay_half_length = half_length


class FakeProfile(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        self.title = title
        self.reference = None
        self.adjusted = None

    def set_title(self, title):
        self.title = title

    def set_profile_data(self, reference_points, adjusted_points, half_length):
        self.reference = np.asarray(reference_points)
        self.adjusted = np.asarray(adjusted_points)
        self.half_length = half_length
        self.refresh_calls = getattr(self, "refresh_calls", 0) + 1


class MainWindowIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def frame(self, root):
        pose = np.eye(4)
        pose[:3, 3] = (10.0, 20.0, 2.0)
        points = np.array([[10.0, 20.0, 2.0], [11.0, 20.0, 2.1], [10.0, 21.0, 2.2]])
        return FrameData(
            "actual", root / "map.pcd", root / "actual.pcd", root / "actual.txt",
            Cloud(points + np.array((0.0, 0.0, 0.1))), Cloud(points), pose)

    def test_load_focuses_once_and_refreshes_do_not_refocus_camera(self):
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame = self.frame(root)
            class Loader:
                def load_frame(self, request):
                    return frame
            scene = FakeScene()
            window = MainWindow(loader=Loader(), scene=scene, profile_factory=FakeProfile,
                                message_sink=lambda level, text: None)
            window.data_panel.yaml_dir_edit.setText(temp)
            window.load_current_frame()

            self.assertEqual(window.controller.current_frame.frame_id, "actual")
            self.assertEqual(scene.reference_calls, 1)
            self.assertEqual(scene.adjusted_calls, 1)
            self.assertEqual(scene.overlay_calls, 1)
            self.assertEqual(
                [geometry.profile_id for geometry in scene.geometries],
                ["xz", "yz", "diag_plus", "diag_minus"],
            )
            np.testing.assert_allclose(scene.geometries[0].height_axis, (0.0, 0.0, 1.0))
            self.assertEqual(len(scene.focus_calls), 1)
            np.testing.assert_allclose(scene.focus_calls[0][0], (10.0, 20.0, 2.0))
            self.assertEqual(scene.focus_calls[0][1], window.roi_radius)
            self.assertTrue(window.data_panel.export_button.isEnabled())
            self.assertTrue(all(profile.reference is not None for profile in window.profiles))
            self.assertTrue(all(profile.adjusted is not None for profile in window.profiles))
            self.assertTrue(all(profile.refresh_calls == 1 for profile in window.profiles))

            window.nudge("dx_m", 1)
            self.assertEqual(scene.reference_calls, 1)
            self.assertEqual(scene.adjusted_calls, 2)
            self.assertEqual(scene.overlay_calls, 2)
            self.assertAlmostEqual(window.controller.pose_model.delta.dx_m, window.translation_step)

            window.undo()
            window.length_edit.setValue(window.slice_half_length + 1.0)
            window.thickness_edit.setValue(window.slice_thickness + 0.1)
            self.assertEqual(len(scene.focus_calls), 1)

            window.load_current_frame()
            self.assertEqual(len(scene.focus_calls), 2)

    def test_profile_edits_and_additions_refresh_resolved_geometries_without_reload(self):
        from frame_alignment.core.profiles import PARALLEL_MODE
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame = self.frame(root)

            class Loader:
                def load_frame(self, request):
                    return frame

            scene = FakeScene()
            window = MainWindow(
                loader=Loader(),
                scene=scene,
                profile_factory=FakeProfile,
                message_sink=lambda level, text: None,
            )
            window.data_panel.yaml_dir_edit.setText(temp)
            window.load_current_frame()
            self.assertEqual(scene.overlay_calls, 1)

            window.profile_controls.select_profile("diag_plus")
            window.profile_controls.angle_edit.setValue(12.0)
            diagonal = next(
                geometry for geometry in scene.geometries
                if geometry.profile_id == "diag_plus")
            radians = np.deg2rad(12.0)
            np.testing.assert_allclose(
                diagonal.along_axis,
                (np.cos(radians), np.sin(radians), 0.0),
                atol=1e-12,
            )

            window.profile_controls.mode_combo.setCurrentIndex(
                window.profile_controls.mode_combo.findData(PARALLEL_MODE))
            window.profile_controls.reference_combo.setCurrentText("YZ")
            window.profile_controls.offset_edit.setValue(-2.5)
            diagonal = next(
                geometry for geometry in scene.geometries
                if geometry.profile_id == "diag_plus")
            np.testing.assert_allclose(diagonal.center, (12.5, 20.0, 2.0))

            window.profile_controls.add_profile()
            self.assertEqual(len(scene.geometries), 5)
            self.assertEqual(scene.geometries[-1].profile_id, "extra_1")
            self.assertEqual(scene.reference_calls, 1)
            self.assertTrue(all(profile.reference is not None for profile in window.profiles))
            window.close()


    def test_failed_load_keeps_export_enabled_for_previous_frame(self):
        from frame_alignment.ui.main_window import MainWindow

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            outcomes = [self.frame(root), ValueError("bad pose file")]
            class Loader:
                def load_frame(self, request):
                    result = outcomes.pop(0)
                    if isinstance(result, Exception):
                        raise result
                    return result
            messages = []
            scene = FakeScene()
            window = MainWindow(loader=Loader(), scene=scene, profile_factory=FakeProfile,
                                message_sink=lambda level, text: messages.append((level, text)))
            window.data_panel.yaml_dir_edit.setText(temp)
            window.load_current_frame()
            previous = window.controller.current_frame
            window.load_current_frame()

            self.assertIs(window.controller.current_frame, previous)
            self.assertEqual(len(scene.focus_calls), 1)
            self.assertTrue(window.data_panel.export_button.isEnabled())
            self.assertIn("bad pose file", messages[-1][1])



def test_config_scan_failure_is_reported_with_directory_and_reason(tmp_path):
    """Startup scanning reports the selected directory and concrete failure reason."""
    from frame_alignment.ui.main_window import MainWindow

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    messages = []
    with mock.patch(
        "frame_alignment.ui.data_io_panel.FrameCatalog.scan",
        side_effect=PermissionError("access denied"),
    ):
        MainWindow(
            config={"frame_cloud_map_path": str(frame_dir)},
            scene=FakeScene(),
            profile_factory=FakeProfile,
            message_sink=lambda level, text: messages.append((level, text)),
        )

    assert messages
    assert messages[-1][0] == "error"
    assert str(frame_dir) in messages[-1][1]
    assert "access denied" in messages[-1][1]


def test_partial_export_refreshes_yaml_annotation_before_warning(tmp_path):
    """A successful YAML write updates status before reporting a PCD failure."""
    from frame_alignment.ui.main_window import MainWindow

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    frame = MainWindowIntegrationTests().frame(tmp_path)
    frame.frame_cloud_path.touch()
    observed_messages = []

    class Loader:
        def load_frame(self, request):
            return frame

    window = MainWindow(
        loader=Loader(),
        scene=FakeScene(),
        profile_factory=FakeProfile,
        message_sink=lambda level, text: observed_messages.append(
            (
                level,
                text,
                window.data_panel.annotation_status_label.text(),
                window.data_panel.annotation_count_label.text(),
            )
        ),
    )
    panel = window.data_panel
    panel.frame_dir_edit.setText(str(tmp_path))
    assert panel.scan_frames()
    panel.yaml_dir_edit.setText(str(tmp_path))
    pcd_dir = tmp_path / "pcd-output"
    pcd_dir.mkdir()
    panel.export_pcd_check.setChecked(True)
    panel.pcd_dir_edit.setText(str(pcd_dir))
    window.load_current_frame()
    assert panel.annotation_status_label.text() == "未标注"

    yaml_path = tmp_path / "actual.yaml"

    def fake_export(request, state, overwrite=False):
        yaml_path.write_text("frame_id: actual\n", encoding="utf-8")
        return SimpleNamespace(yaml_path=yaml_path, pcd_path=None, pcd_error="disk full")

    with mock.patch("frame_alignment.ui.main_window.export_result", side_effect=fake_export):
        window.export_current_frame()

    assert observed_messages[-1][0] == "warning"
    assert observed_messages[-1][2] == "已标注"
    assert observed_messages[-1][3] == "1/1（已标注/总量）"
    window.close()


def test_loaded_frame_with_empty_reference_roi_does_not_refocus_camera(tmp_path):
    """An empty map ROI preserves the current camera instead of focusing blindly."""
    from frame_alignment.ui.main_window import MainWindow

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pose = np.eye(4)
    pose[:3, 3] = (10.0, 20.0, 2.0)
    frame = FrameData(
        "empty-roi",
        tmp_path / "map.pcd",
        tmp_path / "empty-roi.pcd",
        tmp_path / "empty-roi.txt",
        Cloud([[1000.0, 1000.0, 1000.0]]),
        Cloud([[10.0, 20.0, 2.0]]),
        pose,
    )

    class Loader:
        def load_frame(self, request):
            return frame

    scene = FakeScene()
    window = MainWindow(
        loader=Loader(),
        scene=scene,
        profile_factory=FakeProfile,
        message_sink=lambda level, text: None,
    )
    window.load_current_frame()

    assert len(window.map_display.points) == 0
    assert scene.reference_calls == 1
    assert scene.focus_calls == []
    window.close()


if __name__ == "__main__":
    unittest.main()
