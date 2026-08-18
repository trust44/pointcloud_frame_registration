"""Editable, collapsible data and output selection panel."""
from pathlib import Path

from PySide6 import QtCore, QtWidgets

from ..app.frame_catalog import FrameCatalog
from ..contracts import ExportRequest, LoadRequest, normalize_frame_id


class DataIOPanel(QtWidgets.QGroupBox):
    load_requested = QtCore.Signal()
    export_requested = QtCore.Signal()
    scan_failed = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__("\u6570\u636e\u4e0e\u8f93\u51fa", parent)
        self._frame_loaded = False
        self.global_map_edit = QtWidgets.QLineEdit()
        self.frame_dir_edit = QtWidgets.QLineEdit()
        self.pose_dir_edit = QtWidgets.QLineEdit()
        self.catalog = FrameCatalog()
        self.frame_combo = QtWidgets.QComboBox()
        self.frame_combo.setEditable(True)
        self.frame_id_edit = self.frame_combo.lineEdit()
        self.yaml_dir_edit = QtWidgets.QLineEdit()
        self.export_pcd_check = QtWidgets.QCheckBox("\u5bfc\u51fa\u8c03\u6574\u540e PCD")
        self.pcd_dir_edit = QtWidgets.QLineEdit()
        self.previous_button = QtWidgets.QPushButton("上一帧")
        self.next_button = QtWidgets.QPushButton("下一帧")
        self.scan_button = QtWidgets.QPushButton("刷新帧列表")
        self.annotation_status_label = QtWidgets.QLabel("未标注")
        self.annotation_count_label = QtWidgets.QLabel("标注量：0/0")
        self.load_button = QtWidgets.QPushButton("\u52a0\u8f7d\u5f53\u524d\u5e27")
        self.export_button = QtWidgets.QPushButton("\u5bfc\u51fa\u5f53\u524d\u5e27")
        self.setCheckable(True)
        self.setChecked(True)
        self._build()
        self.toggled.connect(self._set_expanded)
        self.export_pcd_check.toggled.connect(self._on_pcd_toggled)
        self.yaml_dir_edit.textChanged.connect(self._refresh_export_enabled)
        self.yaml_dir_edit.textChanged.connect(self.refresh_annotation_status)
        self.pcd_dir_edit.textChanged.connect(self._refresh_export_enabled)
        self.frame_combo.currentTextChanged.connect(self.refresh_annotation_status)
        self.previous_button.clicked.connect(self._select_previous_frame)
        self.next_button.clicked.connect(self._select_next_frame)
        self.scan_button.clicked.connect(self.scan_frames)
        self.load_button.clicked.connect(self.load_requested)
        self.export_button.clicked.connect(self.export_requested)
        self._on_pcd_toggled(False)
        self._set_expanded(True)

    def _build(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(6, 8, 6, 6)
        self.content_widget = QtWidgets.QWidget()
        outer.addWidget(self.content_widget)
        form = QtWidgets.QGridLayout(self.content_widget)
        form.setContentsMargins(0, 0, 0, 0)
        rows = (
            ("\u5168\u5c40\u5730\u56fe\u6587\u4ef6", self.global_map_edit, self._browse_global_file),
            ("\u5355\u5e27\u70b9\u4e91\u76ee\u5f55", self.frame_dir_edit, self._browse_frame_directory),
            ("\u521d\u59cb\u4f4d\u59ff\u76ee\u5f55", self.pose_dir_edit, lambda: self._browse_directory(self.pose_dir_edit)),
            ("YAML \u8f93\u51fa\u76ee\u5f55", self.yaml_dir_edit, lambda: self._browse_directory(self.yaml_dir_edit)),
            ("PCD \u8f93\u51fa\u76ee\u5f55", self.pcd_dir_edit, lambda: self._browse_directory(self.pcd_dir_edit)),
        )
        for row, (label, edit, browse) in enumerate(rows):
            form.addWidget(QtWidgets.QLabel(label), row, 0)
            form.addWidget(edit, row, 1)
            button = QtWidgets.QPushButton("\u6d4f\u89c8")
            button.clicked.connect(browse)
            form.addWidget(button, row, 2)
        frame_row = len(rows)
        form.addWidget(QtWidgets.QLabel("Frame ID"), frame_row, 0)
        form.addWidget(self.frame_combo, frame_row, 1)
        form.addWidget(self.load_button, frame_row, 2)
        navigation = QtWidgets.QHBoxLayout()
        navigation.addWidget(self.previous_button)
        navigation.addWidget(self.next_button)
        navigation.addWidget(self.scan_button)
        form.addLayout(navigation, frame_row + 1, 0, 1, 3)
        annotation = QtWidgets.QHBoxLayout()
        annotation.addWidget(self.annotation_status_label)
        annotation.addWidget(self.annotation_count_label)
        annotation.addStretch(1)
        form.addLayout(annotation, frame_row + 2, 0, 1, 3)
        form.addWidget(self.export_pcd_check, frame_row + 3, 0, 1, 2)
        form.addWidget(self.export_button, frame_row + 3, 2)

    def _set_expanded(self, checked):
        self.content_widget.setVisible(bool(checked))
        self.setMaximumHeight(16777215 if checked else 28)

    def _browse_global_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "\u9009\u62e9\u5168\u5c40\u5730\u56fe", self.global_map_edit.text(),
            "Point clouds (*.pcd *.ply *.las *.laz)")
        if path:
            self.global_map_edit.setText(path)

    def _browse_directory(self, edit):
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "\u9009\u62e9\u76ee\u5f55", edit.text())
        if path:
            edit.setText(path)

    def _browse_frame_directory(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "閫夋嫨鐩綍", self.frame_dir_edit.text())
        if path:
            self.frame_dir_edit.setText(path)
            self.scan_frames()

    def scan_frames(self):
        current_frame_id = self.frame_combo.currentText()
        try:
            frame_ids = self.catalog.scan(self.frame_dir_edit.text().strip())
        except Exception as exc:
            self.scan_failed.emit(str(exc))
            return False
        self.frame_combo.blockSignals(True)
        self.frame_combo.clear()
        self.frame_combo.addItems(frame_ids)
        if current_frame_id:
            self.frame_combo.setCurrentText(current_frame_id)
        self.frame_combo.blockSignals(False)
        self.refresh_annotation_status()
        return True

    def refresh_annotation_status(self):
        yaml_directory = self.yaml_dir_edit.text().strip()
        self.catalog.refresh_annotations(yaml_directory)
        try:
            frame_id = normalize_frame_id(self.frame_combo.currentText())
        except (TypeError, ValueError):
            frame_id = ""
        is_annotated = (
            bool(frame_id) and self._is_directory(yaml_directory)
            and (Path(yaml_directory).expanduser() / (frame_id + ".yaml")).is_file())
        position = 0
        if frame_id in self.catalog.frame_ids:
            position = self.catalog.frame_ids.index(frame_id) + 1
        status = "已标注" if is_annotated else "未标注"
        self.annotation_status_label.setText("{} {}/{}".format(status, position, self.catalog.total_count))
        self.annotation_count_label.setText("标注量：{}/{}".format(
            self.catalog.annotated_count, self.catalog.total_count))
    def _select_previous_frame(self):
        frame_id = self.catalog.previous(self.frame_combo.currentText())
        if frame_id is not None:
            self.frame_combo.setCurrentText(frame_id)

    def _select_next_frame(self):
        frame_id = self.catalog.next(self.frame_combo.currentText())
        if frame_id is not None:
            self.frame_combo.setCurrentText(frame_id)

    def _on_pcd_toggled(self, checked):
        self.pcd_dir_edit.setEnabled(bool(checked))
        self._refresh_export_enabled()

    @staticmethod
    def _is_directory(value):
        return bool(value.strip()) and Path(value.strip()).expanduser().is_dir()

    def _refresh_export_enabled(self):
        ready = self._frame_loaded and self._is_directory(self.yaml_dir_edit.text())
        if self.export_pcd_check.isChecked():
            ready = ready and self._is_directory(self.pcd_dir_edit.text())
        self.export_button.setEnabled(ready)

    def set_frame_loaded(self, loaded):
        self._frame_loaded = bool(loaded)
        self._refresh_export_enabled()

    def get_load_request(self):
        return LoadRequest(
            Path(self.global_map_edit.text().strip()),
            Path(self.frame_dir_edit.text().strip()),
            Path(self.pose_dir_edit.text().strip()),
            self.frame_id_edit.text().strip(),
        )

    def get_export_request(self):
        yaml_value = self.yaml_dir_edit.text().strip()
        pcd_value = self.pcd_dir_edit.text().strip()
        return ExportRequest(
            Path(yaml_value) if yaml_value else "",
            self.export_pcd_check.isChecked(),
            Path(pcd_value) if pcd_value else "",
        )

    def apply_config(self, config):
        self.global_map_edit.setText(str(config.get("global_map_path", "")))
        self.frame_dir_edit.setText(str(config.get("frame_cloud_map_path", "")))
        self.pose_dir_edit.setText(str(config.get("initial_pose_path", "")))
        self.frame_id_edit.setText(str(config.get("frame_id", "")))
        self.yaml_dir_edit.setText(str(config.get("output_path_yaml", "")))
        self.pcd_dir_edit.setText(str(config.get("output_path_pcd", "")))
        if self._is_directory(self.frame_dir_edit.text()):
            self.scan_frames()

