"""Single-window GUI integrating data selection, 3D scene, profiles, and 6DoF controls."""
import time

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

from ..app.controller import AlignmentController
from ..contracts import ExportRequest
from ..core.point_cloud import Cloud, read_cloud
from ..core.pose_model import Delta, PoseModel
from ..core.profiles import extract_slice, profile_geometry
from ..core.registration import constrained_icp, matching_error
from ..io.exporter import AlignmentExportState, export_result
from ..io.frame_loader import FrameLoader, ReviewFrameLoader
from .data_io_panel import DataIOPanel
from .profile_controls import ProfileControls
from .profile_view import ProfileView
from .scene_3d import Scene3DView
from .cloud_rendering import render_cloud_colors
from .cloud_render_controls import CloudRenderControls


def _bounded_profile_half_length(value):
    try:
        requested = float(value)
    except (TypeError, ValueError, OverflowError):
        return 20.0
    if np.isfinite(requested) and 10.0 <= requested <= 35.0:
        return requested
    return 20.0


class MainWindow(QtWidgets.QMainWindow):
    pose_fields = (
        ("dx_m", "\u0394X map (m)", "A", "D"),
        ("dy_m", "\u0394Y map (m)", "W", "S"),
        ("dz_m", "\u0394Z map (m)", "Q", "E"),
        ("roll_deg", "\u6a2a\u6eda map (\u00b0)", "7", "9"),
        ("pitch_deg", "\u4fef\u4ef0 map (\u00b0)", "4", "6"),
        ("yaw_deg", "\u504f\u822a map (\u00b0)", "1", "3"),
    )

    def __init__(self, config=None, loader=None, scene=None, profile_factory=ProfileView,
                  message_sink=None, initial_frame=None, parent=None):
        super().__init__(parent)
        self.config = config or {}
        self.mode = str(self.config.get("mode", "register")).strip().lower()
        if self.mode not in {"register", "review"}:
            raise ValueError("mode must be 'register' or 'review'")
        display = self.config.get("display", {})
        interaction = self.config.get("interaction", {})
        self.allow_manual_adjustment = (
            self.mode == "register"
            and bool(interaction.get("allow_manual_adjustment", True))
        )
        self.cloud_render_settings = display.get("clouds", {}) if isinstance(display.get("clouds", {}), dict) else {}
        error = self.config.get("error", {})
        self.roi_radius = float(display.get("map_roi_radius_m", 35.0))
        self.display_voxel = float(display.get("display_voxel_m", 0.05))
        self.slice_half_length = _bounded_profile_half_length(display.get("slice_half_length_m", 20.0))
        self.slice_thickness = float(display.get("slice_thickness_m", 0.20))
        self.translation_step = float(interaction.get("translation_step_m", 0.01))
        self.translation_large_step = float(interaction.get("translation_large_step_m", 0.10))
        self.rotation_step = float(interaction.get("rotation_step_deg", 0.05))
        self.rotation_large_step = float(interaction.get("rotation_large_step_deg", 0.50))
        self.error_method = str(error.get("method", "nearest_neighbor"))
        self.error_radius = float(error.get("evaluation_radius_m", 25.0))
        self.error_threshold = float(error.get("match_threshold_m", 0.2))
        self._message_sink = message_sink or self._qt_message
        self._load_started = None
        self.map_display = None
        self.source_display = None
        self.edits = {}
        self.setWindowTitle(
            "6DoF \u5355\u5e27\u70b9\u4e91 / \u5168\u5c40\u5730\u56fe\u5256\u9762\u914d\u51c6"
            if self.mode == "register" else "\u5df2\u914d\u51c6\u70b9\u4e91\u5ba1\u6838")
        self.resize(1600, 980)

        self.data_panel = DataIOPanel(mode=self.mode)
        self.scene = scene or Scene3DView()
        self._profile_factory = profile_factory
        self.profile_controls = ProfileControls(include_extra_profiles=True)
        self.cloud_render_controls = CloudRenderControls(self.cloud_render_settings)
        self.profile_specs = self.profile_controls.profile_specs
        self._profile_widgets = {
            spec.profile_id: profile_factory(spec.name) for spec in self.profile_specs
        }
        self.profiles = [self._profile_widgets[spec.profile_id] for spec in self.profile_specs]
        self._build_ui()
        self.data_panel.scan_failed.connect(self._on_scan_failed)
        self.data_panel.apply_config(self.config)
        self.data_panel.load_requested.connect(self.load_current_frame)
        self.data_panel.export_requested.connect(self.export_current_frame)
        default_loader = ReviewFrameLoader(read_cloud) if self.mode == "review" else FrameLoader(read_cloud)
        self.controller = AlignmentController(
            loader or default_loader, PoseModel, self._on_frame_loaded, self._on_load_error)
        self.profile_controls.profiles_changed.connect(self._apply_profile_specs)
        self.cloud_render_controls.changed.connect(self._on_cloud_render_changed)
        self._install_navigation_shortcuts()
        self._set_manual_controls_enabled()
        if initial_frame is not None:
            original_loader = self.controller.loader
            self.controller.loader = _SingleFrameLoader(initial_frame)
            self.load_current_frame()
            self.controller.loader = original_loader

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root_layout = QtWidgets.QHBoxLayout(central)
        root_layout.setContentsMargins(6, 6, 6, 6)

        self.visualization_widget = QtWidgets.QWidget()
        self.visualization_layout = QtWidgets.QGridLayout(self.visualization_widget)
        self.visualization_layout.setContentsMargins(0, 0, 0, 0)
        self._rebuild_profile_layout()

        self.pose_panel = self._build_pose_panel()
        self.matrix_panel = self._build_matrix_panel()
        self.right_sidebar = QtWidgets.QWidget()
        sidebar_layout = QtWidgets.QVBoxLayout(self.right_sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_layout.addWidget(self.pose_panel)
        sidebar_layout.addWidget(self.profile_controls)
        sidebar_layout.addWidget(self.cloud_render_controls)
        sidebar_layout.addWidget(self.matrix_panel)
        sidebar_layout.addWidget(self.data_panel)
        sidebar_layout.addStretch(1)

        sidebar_scroll = QtWidgets.QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setMinimumWidth(420)
        sidebar_scroll.setWidget(self.right_sidebar)
        root_layout.addWidget(self.visualization_widget, 1)
        root_layout.addWidget(sidebar_scroll)
        self.setStatusBar(QtWidgets.QStatusBar())

    def _on_cloud_render_changed(self, layer, settings):
        self.cloud_render_settings[layer] = dict(settings)
        if self.map_display is not None and hasattr(self.scene, "set_reference_colors"):
            self.scene.set_reference_colors(render_cloud_colors(self.map_display.points, self.map_display.colors, self.cloud_render_settings.get("global", {}), self.controller.pose_model.current_origin))
        if self.source_display is not None and hasattr(self.scene, "set_adjusted_colors"):
            adjusted = self.controller.pose_model.transform_points(self.source_display.points)
            self.scene.set_adjusted_colors(render_cloud_colors(adjusted, self.source_display.colors, self.cloud_render_settings.get("source", {}), self.controller.pose_model.current_origin))

    def _apply_profile_specs(self, specs):
        specs = tuple(sorted(specs, key=lambda spec: (spec.grid_column, spec.grid_row)))
        profile_ids = [spec.profile_id for spec in specs]
        if len(profile_ids) != len(set(profile_ids)):
            raise ValueError("profile ids must be unique")

        active_ids = set(profile_ids)
        for profile_id in set(self._profile_widgets) - active_ids:
            widget = self._profile_widgets.pop(profile_id)
            self.visualization_layout.removeWidget(widget)
            widget.setParent(None)
            widget.deleteLater()

        for spec in specs:
            widget = self._profile_widgets.get(spec.profile_id)
            if widget is None:
                widget = self._profile_factory(spec.name)
                self._profile_widgets[spec.profile_id] = widget
            if hasattr(widget, "set_title"):
                widget.set_title(spec.name)

        self.profile_specs = specs
        self.profiles = [self._profile_widgets[spec.profile_id] for spec in specs]
        self._rebuild_profile_layout()
        if hasattr(self, "controller"):
            self.refresh_views()

    def _rebuild_profile_layout(self):
        self.visualization_layout.removeWidget(self.scene)
        for widget in self._profile_widgets.values():
            self.visualization_layout.removeWidget(widget)

        self.profile_column_count = 3 if any(
            spec.grid_column == 2 for spec in self.profile_specs) else 2
        self.visualization_layout.addWidget(
            self.scene, 0, 0, 1, self.profile_column_count)
        for spec in self.profile_specs:
            self.visualization_layout.addWidget(
                self._profile_widgets[spec.profile_id],
                1 + spec.grid_row,
                spec.grid_column,
            )
        for column in range(3):
            self.visualization_layout.setColumnStretch(
                column, 1 if column < self.profile_column_count else 0)
        self.visualization_layout.setRowStretch(0, 4)
        self.visualization_layout.setRowStretch(1, 2)
        self.visualization_layout.setRowStretch(2, 2)

    def profile_grid_positions(self):
        return {
            spec.profile_id: (spec.grid_row, spec.grid_column)
            for spec in self.profile_specs
        }

    def _build_pose_panel(self):
        panel = QtWidgets.QGroupBox("6DOF \u63a7\u5236\uff08Pivot: \u521d\u59cb LiDAR \u539f\u70b9 C0\uff09")
        grid = QtWidgets.QGridLayout(panel)
        self._manual_widgets = []
        for row, (field, title, negative, positive) in enumerate(self.pose_fields):
            grid.addWidget(QtWidgets.QLabel(title), row, 0)
            minus = QtWidgets.QPushButton("\u2212 " + negative)
            plus = QtWidgets.QPushButton("+ " + positive)
            edit = QtWidgets.QDoubleSpinBox()
            edit.setRange(-10000.0, 10000.0)
            edit.setDecimals(4)
            edit.valueChanged.connect(lambda value, name=field: self.set_field(name, value))
            minus.clicked.connect(lambda checked=False, name=field: self.nudge(name, -1))
            plus.clicked.connect(lambda checked=False, name=field: self.nudge(name, 1))
            grid.addWidget(minus, row, 1)
            grid.addWidget(edit, row, 2)
            grid.addWidget(plus, row, 3)
            self.edits[field] = edit
            self._manual_widgets.extend((minus, edit, plus))
        row = len(self.pose_fields)
        grid.addWidget(QtWidgets.QLabel("\u5256\u9762\u534a\u957f (m)"), row, 0)
        self.length_edit = QtWidgets.QDoubleSpinBox()
        self.length_edit.setRange(10.0, 35.0)
        self.length_edit.setValue(self.slice_half_length)
        self.length_edit.valueChanged.connect(self._set_slice_length)
        grid.addWidget(self.length_edit, row, 1, 1, 3)
        grid.addWidget(QtWidgets.QLabel("\u5256\u9762\u539a\u5ea6 (m)"), row + 1, 0)
        self.thickness_edit = QtWidgets.QDoubleSpinBox()
        self.thickness_edit.setDecimals(3)
        self.thickness_edit.setRange(0.001, 20.0)
        self.thickness_edit.setValue(self.slice_thickness)
        self.thickness_edit.valueChanged.connect(self._set_slice_thickness)
        grid.addWidget(self.thickness_edit, row + 1, 1, 1, 3)
        actions = QtWidgets.QHBoxLayout()
        for title, callback, mutates_pose in (("\u91cd\u7f6e (R)", self.reset, True), ("\u64a4\u9500", self.undo, True),
                                              ("\u91cd\u505a", self.redo, True), ("ICP (V)", self.icp, True),
                                              ("\u8ba1\u7b97\u8bef\u5dee", self.compute_error, False)):
            button = QtWidgets.QPushButton(title)
            button.clicked.connect(callback)
            actions.addWidget(button)
            if mutates_pose:
                self._manual_widgets.append(button)
        grid.addLayout(actions, row + 2, 0, 1, 4)
        return panel

    def _build_matrix_panel(self):
        panel = QtWidgets.QGroupBox("矩阵")
        layout = QtWidgets.QVBoxLayout(panel)
        self.matrix_text = QtWidgets.QPlainTextEdit()
        self.matrix_text.setReadOnly(True)
        self.matrix_text.setMinimumHeight(300)
        layout.addWidget(self.matrix_text)
        return panel
    def _qt_message(self, level, text):
        methods = {"error": QtWidgets.QMessageBox.critical,
                   "warning": QtWidgets.QMessageBox.warning,
                   "info": QtWidgets.QMessageBox.information}
        methods.get(level, QtWidgets.QMessageBox.information)(self, "6DoF Alignment", text)

    def _on_scan_failed(self, message):
        directory = self.data_panel.frame_dir_edit.text().strip() or "<empty>"
        self._message_sink(
            "error",
            "\u626b\u63cf\u5355\u5e27\u70b9\u4e91\u76ee\u5f55\u5931\u8d25\uff1a{}\n{}".format(directory, message),
        )

    def _on_load_error(self, message):
        self.data_panel.set_frame_loaded(self.controller.current_frame is not None)
        self._message_sink("error", message)

    def load_current_frame(self):
        self._load_started = time.perf_counter()
        return self.controller.load_current_frame(self.data_panel.get_load_request())

    def _install_navigation_shortcuts(self):
        self._navigation_shortcuts = []
        for sequence, amount in (("Left", -1), ("Right", 1),
                                 ("Shift+Left", -10), ("Shift+Right", 10)):
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(sequence), self)
            shortcut.setContext(QtCore.Qt.WindowShortcut)
            shortcut.activated.connect(lambda step=amount: self.navigate_frames(step))
            self._navigation_shortcuts.append(shortcut)

    def navigate_frames(self, amount):
        """Select a relative frame; review mode also loads it immediately."""
        return self.data_panel.select_relative_frame(amount)

    def _set_manual_controls_enabled(self):
        for widget in self._manual_widgets:
            widget.setEnabled(self.allow_manual_adjustment)

    def _on_frame_loaded(self, frame):
        model = self.controller.pose_model
        self.map_display = frame.global_map.roi(model.c0, self.roi_radius).voxel(self.display_voxel)
        self.source_display = frame.source_cloud.voxel(self.display_voxel)
        self.scene.set_reference(self.map_display.points)
        if hasattr(self.scene, "set_reference_colors"):
            self.scene.set_reference_colors(render_cloud_colors(
                self.map_display.points, self.map_display.colors, self.cloud_render_settings.get("global", {"mode": "uniform", "color": "#90A4AE"}), model.current_origin))
        if len(self.map_display.points) > 0:
            self.scene.focus_on(model.c0, self.roi_radius)
        self.data_panel.frame_id_edit.setText(frame.frame_id)
        self.data_panel.set_frame_loaded(True)
        self.refresh_views()
        elapsed = 0.0 if self._load_started is None else time.perf_counter() - self._load_started
        pose_description = (
            frame.initial_pose_path
            if frame.initial_pose_path is not None else "<registered cloud median fallback>"
        )
        self.statusBar().showMessage(
            "frame_id={} | map={} ({} pts) | source={} ({} pts) | pose={} | C0={} | {:.3f}s".format(
                frame.frame_id, frame.global_map_path, len(frame.global_map.points),
                frame.frame_cloud_path, len(frame.source_cloud.points),
                pose_description, np.array2string(model.c0, precision=3), elapsed))

    def refresh_views(self):
        if self.controller.current_frame is None:
            return
        model = self.controller.pose_model
        adjusted = model.transform_points(self.source_display.points)
        self.scene.set_adjusted(adjusted)
        if hasattr(self.scene, "set_adjusted_colors"):
            self.scene.set_adjusted_colors(render_cloud_colors(
                adjusted, self.source_display.colors, self.cloud_render_settings.get("source", {"mode": "uniform", "color": "#F6C445"}), model.current_origin))
        self.scene.update_origin(model.current_origin, model.corrected_pose[:3, :3])
        geometries = tuple(
            profile_geometry(spec, model.current_origin, model.corrected_pose[:3, :3])
            for spec in self.profile_specs
        )
        self.scene.update_slice_overlays(geometries, self.slice_half_length)
        for profile, geometry in zip(self.profiles, geometries):
            reference = extract_slice(
                self.map_display.points, geometry, self.slice_half_length, self.slice_thickness)
            adjusted_profile = extract_slice(
                adjusted, geometry, self.slice_half_length, self.slice_thickness)
            profile.set_profile_data(reference, adjusted_profile, self.slice_half_length)
        for name, edit in self.edits.items():
            edit.blockSignals(True)
            edit.setValue(getattr(model.delta, name))
            edit.blockSignals(False)
        quality_lines = []
        displayed_quality = {"initial_icp", "icp_error", "manual_error"}
        for key, value in self.controller.quality.items():
            if key not in displayed_quality:
                continue
            if isinstance(value, dict):
                quality_lines.append("{}:".format(key))
                for metric, metric_value in value.items():
                    shown = "{:.6f}".format(float(metric_value)) if isinstance(metric_value, (int, float)) else metric_value
                    quality_lines.append("  {}: {}".format(metric, shown))
            else:
                quality_lines.append("{}: {}".format(key, value))
        self.matrix_text.setPlainText(
            "C0 = {}\nC_current = {}\nquality:\n{}\nT_manual_map =\n{}\nT_corrected_map_lidar =\n{}".format(
                np.array2string(model.c0, precision=4), np.array2string(model.current_origin, precision=4),
                "\n".join(quality_lines), np.array2string(model.transform, precision=6),
                np.array2string(model.corrected_pose, precision=6)))
    def _step_for(self, field):
        large = bool(QtWidgets.QApplication.keyboardModifiers() & QtCore.Qt.ShiftModifier)
        if field.endswith("deg"):
            return self.rotation_large_step if large else self.rotation_step
        return self.translation_large_step if large else self.translation_step

    def nudge(self, field, sign):
        if not self.allow_manual_adjustment or self.controller.pose_model is None:
            return
        self.controller.pose_model.adjust(field, float(sign) * self._step_for(field))
        self.controller.invalidate_quality()
        self.refresh_views()

    def set_field(self, field, value):
        if not self.allow_manual_adjustment or self.controller.pose_model is None:
            return
        values = dict(self.controller.pose_model.delta.__dict__)
        values[field] = float(value)
        self.controller.pose_model.set_delta(Delta(**values))
        self.controller.invalidate_quality()
        self.refresh_views()

    def _set_slice_length(self, value):
        self.slice_half_length = float(value)
        self.refresh_views()

    def _set_slice_thickness(self, value):
        self.slice_thickness = float(value)
        self.refresh_views()

    def reset(self):
        if self.allow_manual_adjustment and self.controller.pose_model is not None:
            self.controller.pose_model.reset()
            self.controller.invalidate_quality()
            self.refresh_views()

    def undo(self):
        if self.allow_manual_adjustment and self.controller.pose_model is not None and self.controller.pose_model.undo():
            self.controller.invalidate_quality()
            self.refresh_views()

    def redo(self):
        if self.allow_manual_adjustment and self.controller.pose_model is not None and self.controller.pose_model.redo():
            self.controller.invalidate_quality()
            self.refresh_views()

    def icp(self):
        if not self.allow_manual_adjustment or self.controller.current_frame is None:
            return
        try:
            icp_stats = constrained_icp(
                self.controller.pose_model, self.controller.current_frame.source_cloud,
                self.controller.current_frame.global_map)
            self.controller.record_icp(icp_stats, self._matching_error_for_current_pose())
            self.refresh_views()
        except Exception as exc:
            self._message_sink("warning", str(exc))

    def _matching_error_for_current_pose(self):
        frame = self.controller.current_frame
        model = self.controller.pose_model
        source = Cloud(model.transform_points(frame.source_cloud.points)).roi(
            model.current_origin, self.error_radius)
        target = frame.global_map.roi(model.current_origin, self.error_radius)
        target_normals = None
        if self.error_method == "point_to_plane":
            import open3d as o3d
            target_cloud = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(target.points.copy()))
            target_cloud.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.24, max_nn=30))
            target_normals = np.asarray(target_cloud.normals).copy()
        return matching_error(
            source.points, target.points, method=self.error_method,
            threshold=self.error_threshold, target_normals=target_normals)

    def compute_error(self):
        if self.controller.current_frame is None:
            return
        try:
            stats = self._matching_error_for_current_pose()
            self.controller.record_manual_error(stats)
            self.refresh_views()
            self._message_sink("info", "\u8bef\u5dee\u8ba1\u7b97\u5b8c\u6210")
        except Exception as exc:
            self._message_sink("warning", str(exc))

    def export_current_frame(self):
        if self.mode != "register":
            return None
        if self.controller.current_frame is None:
            self._message_sink("error", "\u8bf7\u5148\u52a0\u8f7d\u6709\u6548\u5e27")
            return None
        try:
            request = self.data_panel.get_export_request().validate()
        except Exception as exc:
            self._message_sink("error", str(exc))
            return None
        frame = self.controller.current_frame
        model = self.controller.pose_model
        stem = frame.frame_cloud_path.stem
        destinations = [request.yaml_output_dir / (stem + ".yaml")]
        if request.write_adjusted_pcd:
            destinations.append(request.pcd_output_dir / (stem + ".pcd"))
        overwrite = False
        existing = [path for path in destinations if path.exists()]
        if existing:
            answer = QtWidgets.QMessageBox.question(
                self, "\u786e\u8ba4\u8986\u76d6",
                "\u4ee5\u4e0b\u6587\u4ef6\u5df2\u5b58\u5728\uff1a\n{}\n\u662f\u5426\u8986\u76d6\uff1f".format("\n".join(map(str, existing))))
            if answer != QtWidgets.QMessageBox.Yes:
                return None
            overwrite = True
        state = AlignmentExportState(
            frame, model.delta, model.transform, model.corrected_pose,
            model.transform_points(frame.source_cloud.points), self.controller.quality,
            frame.source_cloud.colors)
        try:
            outcome = export_result(request, state, overwrite=overwrite)
        except Exception as exc:
            self._message_sink("error", str(exc))
            return None
        if outcome.yaml_path.is_file():
            self.data_panel.refresh_annotation_status()
        if outcome.pcd_error:
            self._message_sink("warning", "YAML: {}\nPCD: {}".format(outcome.yaml_path, outcome.pcd_error))
        else:
            paths = [str(outcome.yaml_path)] + ([] if outcome.pcd_path is None else [str(outcome.pcd_path)])
            self._message_sink("info", "\u5bfc\u51fa\u5b8c\u6210\uff1a\n{}".format("\n".join(paths)))
        self.statusBar().showMessage("Exported: {}".format(outcome.yaml_path))
        return outcome

    def keyPressEvent(self, event):
        mapping = {"A": ("dx_m", -1), "D": ("dx_m", 1), "W": ("dy_m", 1), "S": ("dy_m", -1),
                   "Q": ("dz_m", 1), "E": ("dz_m", -1), "7": ("roll_deg", 1), "9": ("roll_deg", -1),
                   "4": ("pitch_deg", 1), "6": ("pitch_deg", -1), "1": ("yaw_deg", 1), "3": ("yaw_deg", -1)}
        key = event.text().upper()
        if key in mapping:
            self.nudge(*mapping[key])
            return
        if key == "R":
            self.reset()
            return
        if key == "G":
            self.export_current_frame()
            return
        if key == "C":
            self.compute_error()
            return
        if key == "V":
            self.icp()
            return
        if key == "Z":
            self.load_current_frame()
            return
        super().keyPressEvent(event)


class _SingleFrameLoader:
    def __init__(self, frame):
        self.frame = frame

    def load_frame(self, request):
        return self.frame




