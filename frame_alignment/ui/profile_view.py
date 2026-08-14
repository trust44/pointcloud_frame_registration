"""One profile plot with stable source-based legend entries."""
import numpy as np
import pyqtgraph as pg


REFERENCE_COLOR = (144, 164, 174, 255)
ADJUSTED_COLOR = (246, 196, 69, 255)


class ProfileView(pg.PlotWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent=parent, title=title)
        self.setBackground((22, 25, 29))
        self.showGrid(x=True, y=True, alpha=0.25)
        self.setLabel("bottom", "s from corrected LiDAR origin (m)")
        self.setLabel("left", "Z relative to origin (m)")
        self.legend = self.addLegend()
        self.legend.setBrush(pg.mkBrush(20, 24, 28, 175))
        self.legend.anchor((1, 0), (1, 0), offset=(-10, 10))
        self.reference_item = pg.ScatterPlotItem(
            size=3, pen=None, brush=pg.mkBrush(REFERENCE_COLOR),
            name="Reference Map\uff08\u5168\u5c40\u70b9\u4e91\uff09")
        self.adjusted_item = pg.ScatterPlotItem(
            size=4, pen=None, brush=pg.mkBrush(ADJUSTED_COLOR),
            name="Adjusted Frame\uff08\u5355\u5e27\u70b9\u4e91\uff09")
        self.addItem(self.reference_item)
        self.addItem(self.adjusted_item)
        self.addLine(x=0, pen=pg.mkPen((255, 202, 40), width=1))
        self.empty_item = pg.TextItem("\u5f53\u524d\u5256\u9762\u65e0\u70b9", anchor=(0.5, 0.5), color=(205, 210, 215))
        self.empty_item.setPos(0, 0)
        self.empty_item.hide()
        self.addItem(self.empty_item)

    @staticmethod
    def _xy(points):
        points = np.asarray(points, dtype=np.float64)
        if points.size == 0:
            return np.empty(0), np.empty(0)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("Profile points must have shape (N, 2)")
        return points[:, 0], points[:, 1]

    @classmethod
    def _finite_points(cls, points):
        x, y = cls._xy(points)
        if x.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        return np.column_stack((x, y))[np.isfinite(x) & np.isfinite(y)]

    def set_reference_points(self, points):
        x, y = self._xy(points)
        self.reference_item.setData(x=x, y=y)

    def set_adjusted_points(self, points):
        x, y = self._xy(points)
        self.adjusted_item.setData(x=x, y=y)

    def set_profile_range(self, half_length):
        self.setXRange(-float(half_length), float(half_length), padding=0)

    def set_profile_data(self, reference_points, adjusted_points, half_length):
        reference = self._finite_points(reference_points)
        adjusted = self._finite_points(adjusted_points)
        self.reference_item.setData(x=reference[:, 0], y=reference[:, 1])
        self.adjusted_item.setData(x=adjusted[:, 0], y=adjusted[:, 1])
        self.set_profile_range(half_length)

        y_values = np.concatenate((reference[:, 1], adjusted[:, 1]))
        if y_values.size == 0:
            self.setYRange(-1.0, 1.0, padding=0)
            self.empty_item.show()
            return
        low = float(np.min(y_values))
        high = float(np.max(y_values))
        padding = max(0.05, (high - low) * 0.05)
        self.setYRange(low - padding, high + padding, padding=0)
        self.empty_item.hide()
