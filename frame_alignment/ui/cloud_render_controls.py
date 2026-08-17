from PySide6 import QtCore, QtWidgets


class CloudRenderControls(QtWidgets.QGroupBox):
    """Runtime controls for independent global/source cloud coloring."""
    changed = QtCore.Signal(str, object)

    def __init__(self, settings=None, parent=None):
        super().__init__("点云渲染", parent)
        self.settings = settings if isinstance(settings, dict) else {}
        root = QtWidgets.QHBoxLayout(self)
        self._editors = {}
        for layer, title in (("global", "Global 全局点云"), ("source", "Source 单帧点云")):
            box = QtWidgets.QGroupBox(title)
            form = QtWidgets.QFormLayout(box)
            mode = QtWidgets.QComboBox(); mode.addItems(["uniform", "native", "cmap"])
            cmap = QtWidgets.QComboBox(); cmap.addItems(["viridis", "plasma", "inferno", "turbo", "gray"])
            color = QtWidgets.QComboBox(); color.setEditable(True)
            colors = [("蓝灰", "#90A4AE"), ("金黄", "#F6C445"), ("青色", "#00D4FF"), ("洋红", "#FF3CAC"), ("绿色", "#39FF88"), ("橙红", "#FF6B35"), ("白色", "#FFFFFF")]
            color.addItems([f"{n} {v}" for n, v in colors])
            color.setCurrentText(str(self.settings.get(layer, {}).get("color", colors[0][1])))
            form.addRow("模式", mode); form.addRow("cmap", cmap); form.addRow("单色", color)
            self._editors[layer] = (mode, cmap, color)
            for w in (mode, cmap, color):
                (w.currentTextChanged if isinstance(w, QtWidgets.QComboBox) else w.editingFinished).connect(lambda _=None, l=layer: self._emit(l))
            root.addWidget(box)
            self._load_layer(layer)

    def _load_layer(self, layer):
        data = self.settings.get(layer, {})
        if not isinstance(data, dict): data = {}
        mode, cmap, color = self._editors[layer]
        mode.setCurrentText(str(data.get("mode", "uniform")))
        cmap.setCurrentText(str(data.get("cmap", "viridis")))
        value = str(data.get("color", "#90A4AE"))
        matches = [color.itemText(i) for i in range(color.count()) if value.lower() in color.itemText(i).lower()]
        color.setCurrentText(matches[0] if matches else value)

    def _emit(self, layer):
        mode, cmap, color = self._editors[layer]
        raw = color.currentText().strip()
        value = raw.split()[-1] if raw.startswith(("蓝", "金", "青", "洋", "绿", "橙", "白")) else raw
        data = dict(self.settings.get(layer, {})); data.update(mode=mode.currentText(), cmap=cmap.currentText(), color=value)
        self.settings[layer] = data
        self.changed.emit(layer, data)
