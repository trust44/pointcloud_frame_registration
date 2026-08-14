import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtWidgets

from frame_alignment.ui.data_io_panel import DataIOPanel


def _app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _frame_items(panel):
    return [panel.frame_combo.itemText(index) for index in range(panel.frame_combo.count())]


def test_scan_populates_naturally_sorted_frames_and_annotation_status(tmp_path):
    """A scan exposes discovered frames and their real YAML annotation state."""
    _app()
    frame_dir = tmp_path / "frames"
    yaml_dir = tmp_path / "yaml"
    frame_dir.mkdir()
    yaml_dir.mkdir()
    (frame_dir / "10.pcd").touch()
    (frame_dir / "2.pcd").touch()
    (yaml_dir / "2.yaml").touch()

    panel = DataIOPanel()
    panel.frame_dir_edit.setText(str(frame_dir))

    assert panel.scan_frames()
    assert _frame_items(panel) == ["2", "10"]

    panel.frame_combo.setCurrentText("2")
    panel.yaml_dir_edit.setText(str(yaml_dir))
    panel.refresh_annotation_status()

    assert panel.annotation_status_label.text() == "\u5df2\u6807\u6ce8"
    assert panel.annotation_count_label.text() == "1/2\uff08\u5df2\u6807\u6ce8/\u603b\u91cf\uff09"


def test_browsing_and_text_edits_never_request_a_load_until_load_is_clicked(tmp_path):
    """Frame selection stays explicit, while arbitrary manual IDs remain loadable."""
    app = _app()
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    (frame_dir / "2.pcd").touch()
    (frame_dir / "10.pcd").touch()
    panel = DataIOPanel()
    panel.frame_dir_edit.setText(str(frame_dir))
    assert panel.scan_frames()
    loads = []
    panel.load_requested.connect(lambda: loads.append(True))

    panel.frame_combo.setCurrentText("2")
    panel.previous_button.click()
    panel.next_button.click()
    panel.frame_id_edit.setText("manual-99")
    app.processEvents()

    assert loads == []
    assert panel.frame_combo.currentText() == "manual-99"
    assert panel.get_load_request().frame_id == "manual-99"
    panel.load_button.click()
    assert loads == [True]


def test_manual_frame_annotation_status_uses_current_yaml_file(tmp_path):
    """A manual ID is annotated whenever its YAML exists, even if it was not scanned."""
    _app()
    yaml_dir = tmp_path / "yaml"
    yaml_dir.mkdir()
    (yaml_dir / "manual-99.yaml").touch()
    panel = DataIOPanel()
    panel.frame_id_edit.setText("manual-99")

    panel.yaml_dir_edit.setText(str(yaml_dir))
    assert panel.annotation_status_label.text() == "\u5df2\u6807\u6ce8"

    panel.yaml_dir_edit.setText(str(tmp_path / "missing"))
    assert panel.annotation_status_label.text() == "\u672a\u6807\u6ce8"
    panel.yaml_dir_edit.setText("")
    assert panel.annotation_status_label.text() == "\u672a\u6807\u6ce8"

def test_config_scans_valid_directory_and_failed_rescan_preserves_frames(tmp_path):
    """Valid configured directories scan at startup; a bad replacement keeps the old list."""
    _app()
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    (frame_dir / "2.pcd").touch()
    (frame_dir / "10.pcd").touch()
    panel = DataIOPanel()
    panel.apply_config({"frame_cloud_map_path": str(frame_dir)})
    assert _frame_items(panel) == ["2", "10"]

    errors = []
    panel.scan_failed.connect(errors.append)
    invalid_dir = tmp_path / "missing"
    panel.frame_dir_edit.setText(str(invalid_dir))
    assert not panel.scan_frames()
    assert str(invalid_dir) in errors[-1]
    assert _frame_items(panel) == ["2", "10"]


def test_manual_annotation_lookup_normalizes_extension_and_rejects_unsafe_ids(tmp_path):
    """Status lookup uses the same normalized, traversal-safe Frame ID as loading."""
    _app()
    yaml_dir = tmp_path / "yaml"
    yaml_dir.mkdir()
    (yaml_dir / "frame-2.yaml").touch()
    panel = DataIOPanel()
    panel.yaml_dir_edit.setText(str(yaml_dir))

    panel.frame_id_edit.setText("frame-2.pcd")
    assert panel.annotation_status_label.text() == "已标注"

    (tmp_path / "frame-2.yaml").touch()
    panel.frame_id_edit.setText("../frame-2")
    assert panel.annotation_status_label.text() == "未标注"
