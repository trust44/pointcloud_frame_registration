"""Session-only controls for selecting and positioning profile views."""
from dataclasses import replace

from PySide6 import QtCore, QtWidgets

from frame_alignment.core.profiles import (
    ANGLE_MODE,
    PARALLEL_MODE,
    REFERENCE_XZ,
    REFERENCE_YZ,
    default_profile_specs,
    extra_profile_spec,
)


class ProfileControls(QtWidgets.QGroupBox):
    """Own the current session's immutable profile definitions."""

    profiles_changed = QtCore.Signal(object)

    def __init__(self, parent=None):
        super().__init__("剖面设置", parent)
        self._specs = list(default_profile_specs())

        self.profile_selector = QtWidgets.QComboBox()
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItem("与 XZ 夹角", ANGLE_MODE)
        self.mode_combo.addItem("平行偏移", PARALLEL_MODE)

        self.angle_edit = QtWidgets.QDoubleSpinBox()
        self.angle_edit.setRange(-180.0, 180.0)
        self.angle_edit.setDecimals(1)
        self.angle_edit.setSuffix("°")

        self.reference_combo = QtWidgets.QComboBox()
        self.reference_combo.addItems((REFERENCE_XZ, REFERENCE_YZ))

        self.offset_edit = QtWidgets.QDoubleSpinBox()
        self.offset_edit.setRange(-1000000.0, 1000000.0)
        self.offset_edit.setDecimals(3)
        self.offset_edit.setSuffix(" m")

        self.add_button = QtWidgets.QPushButton("新增剖面")
        self.delete_button = QtWidgets.QPushButton("删除当前新增剖面")

        form = QtWidgets.QFormLayout()
        form.addRow("当前剖面", self.profile_selector)
        form.addRow("位置方式", self.mode_combo)
        form.addRow("与 XZ 夹角", self.angle_edit)
        form.addRow("平行参考", self.reference_combo)
        form.addRow("有符号距离", self.offset_edit)
        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.add_button)
        buttons.addWidget(self.delete_button)
        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(form)
        root.addLayout(buttons)

        self.profile_selector.currentIndexChanged.connect(self._selection_changed)
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.angle_edit.valueChanged.connect(
            lambda value: self._update_selected(angle_deg=float(value)))
        self.reference_combo.currentTextChanged.connect(
            lambda value: self._update_selected(reference=str(value)))
        self.offset_edit.valueChanged.connect(
            lambda value: self._update_selected(offset_m=float(value)))
        self.add_button.clicked.connect(self.add_profile)
        self.delete_button.clicked.connect(self.delete_selected_profile)

        self._reload_selector(self._specs[0].profile_id)

    @property
    def profile_specs(self):
        return tuple(sorted(self._specs, key=lambda spec: (spec.grid_column, spec.grid_row)))

    def _selected_spec(self):
        profile_id = self.profile_selector.currentData()
        return next((spec for spec in self._specs if spec.profile_id == profile_id), None)

    def _reload_selector(self, selected_id):
        blocker = QtCore.QSignalBlocker(self.profile_selector)
        self.profile_selector.clear()
        for spec in self.profile_specs:
            self.profile_selector.addItem(spec.name, spec.profile_id)
        index = self.profile_selector.findData(selected_id)
        self.profile_selector.setCurrentIndex(index if index >= 0 else 0)
        del blocker
        self._refresh_editors()

    def _selection_changed(self):
        self._refresh_editors()

    def _refresh_editors(self):
        spec = self._selected_spec()
        if spec is None:
            for editor in (
                    self.mode_combo, self.angle_edit,
                    self.reference_combo, self.offset_edit, self.delete_button):
                editor.setEnabled(False)
            self.add_button.setEnabled(len(self._specs) < 6)
            return

        widgets = (self.mode_combo, self.angle_edit, self.reference_combo, self.offset_edit)
        blockers = [QtCore.QSignalBlocker(widget) for widget in widgets]
        self.mode_combo.setCurrentIndex(self.mode_combo.findData(spec.mode))
        self.angle_edit.setValue(float(spec.angle_deg))
        self.reference_combo.setCurrentText(spec.reference)
        self.offset_edit.setValue(float(spec.offset_m))
        del blockers

        is_angle = spec.mode == ANGLE_MODE
        self.mode_combo.setEnabled(spec.editable)
        self.angle_edit.setEnabled(spec.editable and is_angle)
        self.reference_combo.setEnabled(spec.editable and not is_angle)
        self.offset_edit.setEnabled(spec.editable and not is_angle)
        self.delete_button.setEnabled(spec.deletable)
        self.add_button.setEnabled(len(self._specs) < 6)

    def _mode_changed(self):
        mode = self.mode_combo.currentData()
        if mode in (ANGLE_MODE, PARALLEL_MODE):
            self._update_selected(mode=mode)

    def _update_selected(self, **changes):
        spec = self._selected_spec()
        if spec is None or not spec.editable:
            return False
        index = self._specs.index(spec)
        self._specs[index] = replace(spec, **changes)
        self._refresh_editors()
        self.profiles_changed.emit(self.profile_specs)
        return True

    def select_profile(self, profile_id):
        index = self.profile_selector.findData(profile_id)
        if index < 0:
            raise ValueError("unknown profile id: {}".format(profile_id))
        self.profile_selector.setCurrentIndex(index)

    def add_profile(self):
        existing_ids = {spec.profile_id for spec in self._specs}
        slot = next(
            (candidate for candidate in (0, 1)
             if extra_profile_spec(candidate).profile_id not in existing_ids),
            None,
        )
        if slot is None:
            self.add_button.setEnabled(False)
            return False
        spec = extra_profile_spec(slot)
        self._specs.append(spec)
        self._reload_selector(spec.profile_id)
        self.profiles_changed.emit(self.profile_specs)
        return True

    def delete_selected_profile(self):
        spec = self._selected_spec()
        if spec is None or not spec.deletable:
            return False
        self._specs.remove(spec)
        self._reload_selector(self.profile_specs[0].profile_id)
        self.profiles_changed.emit(self.profile_specs)
        return True
