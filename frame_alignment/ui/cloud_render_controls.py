from PySide6 import QtCore, QtWidgets


class CloudRenderControls(QtWidgets.QGroupBox):
    """Runtime controls for independent global/source cloud coloring."""
    changed = QtCore.Signal(str, object)

    def __init__(self, settings=None, parent=None):
        super().__init__("鐐逛簯娓叉煋", parent)
        self.settings = settings if isinstance(settings, dict) else {}
        layout = QtWidgets.QFormLayout(self)
        self.layer = QtWidgets.QComboBox(); self.layer.addItems(["global", "source"])
        self.mode = QtWidgets.QComboBox(); self.mode.addItems(["uniform", "native", "cmap"])
        self.cmap = QtWidgets.QComboBox(); self.cmap.addItems(["viridis", "plasma", "inferno", "turbo", "gray"])
        self.color = QtWidgets.QLineEdit()
        layout.addRow("鍥惧眰", self.layer); layout.addRow("妯″紡", self.mode)
        layout.addRow("cmap", self.cmap); layout.addRow("鍗曡壊", self.color)
        for widget in (self.layer, self.mode, self.cmap): widget.currentTextChanged.connect(self._emit)
        self.color.editingFinished.connect(self._emit)
        self._load_layer()
        self.layer.currentTextChanged.connect(lambda _: self._load_layer())

    def _load_layer(self):
        data = self.settings.get(self.layer.currentText(), {})
        if not isinstance(data, dict): data = {}
        self.mode.setCurrentText(str(data.get("mode", "uniform")))
        self.cmap.setCurrentText(str(data.get("cmap", "viridis")))
        self.color.setText(str(data.get("color", "#90A4AE")))

    def _emit(self):
        layer = self.layer.currentText()
        data = dict(self.settings.get(layer, {}))
        data.update(mode=self.mode.currentText(), cmap=self.cmap.currentText(), color=self.color.text().strip())
        self.settings[layer] = data
        self.changed.emit(layer, data)

