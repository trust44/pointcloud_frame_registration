import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets


def _app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_review_panel_uses_review_paths_and_auto_loads_navigation(tmp_path):
    from frame_alignment.contracts import ReviewLoadRequest
    from frame_alignment.ui.data_io_panel import DataIOPanel

    _app()
    map_path = tmp_path / "map.pcd"
    cloud_dir = tmp_path / "registered"
    pose_dir = tmp_path / "poses"
    map_path.touch()
    cloud_dir.mkdir()
    pose_dir.mkdir()
    for frame_id in ("2", "10"):
        (cloud_dir / (frame_id + ".pcd")).touch()
        (pose_dir / (frame_id + ".yaml")).touch()

    panel = DataIOPanel(mode="review")
    panel.apply_config({
        "global_map_path": str(map_path),
        "registered_cloud_path": str(cloud_dir),
        "registered_pose_path": str(pose_dir),
        "frame_id": "2",
    })
    assert panel.annotation_status_label.text() == "当前位置：1/2"
    loads = []
    panel.load_requested.connect(lambda: loads.append(panel.frame_combo.currentText()))

    assert isinstance(panel.get_load_request(), ReviewLoadRequest)
    assert panel.select_relative_frame(1)
    assert panel.frame_combo.currentText() == "10"
    assert loads == ["10"]
    assert panel.export_button.parent() is None


def test_review_window_is_read_only_and_starts_with_six_profiles():
    from frame_alignment.ui.main_window import MainWindow

    _app()
    window = MainWindow(config={"mode": "review"}, message_sink=lambda level, text: None)
    assert not window.allow_manual_adjustment
    assert len(window.profile_specs) == 6
    assert not window.edits["dx_m"].isEnabled()
    assert window.profile_controls.delete_button.isEnabled() is False
    window.profile_controls.select_profile("extra_1")
    assert window.profile_controls.delete_button.isEnabled()
    window.close()
