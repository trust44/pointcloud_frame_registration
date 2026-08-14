"""Embedded pyqtgraph OpenGL scene with point clouds and slice overlays."""
from dataclasses import dataclass

import numpy as np
from PySide6.QtGui import QVector3D
import pyqtgraph.opengl as gl


REFERENCE_RGBA = (144 / 255.0, 164 / 255.0, 174 / 255.0, 0.72)
ADJUSTED_RGBA = (246 / 255.0, 196 / 255.0, 69 / 255.0, 0.96)


@dataclass(frozen=True)
class SliceSpec:
    angle_deg: float
    name: str
    color: tuple


SLICE_SPECS = (
    SliceSpec(0.0, "X-Z / 0\u00b0", (1.0, 0.0, 0.0, 1.0)),
    SliceSpec(90.0, "Y-Z / 90\u00b0", (0.0, 1.0, 0.0, 1.0)),
    SliceSpec(45.0, "Diag +45\u00b0", (0.0, 0.45, 1.0, 1.0)),
    SliceSpec(-45.0, "Diag -45\u00b0", (0.65, 0.2, 0.8, 1.0)),
)


def slice_rectangle_vertices(center, half_length, thickness, angle_deg):
    center = np.asarray(center, dtype=np.float64)
    theta = np.deg2rad(float(angle_deg))
    axis = np.array((np.cos(theta), np.sin(theta), 0.0))
    normal = np.array((-np.sin(theta), np.cos(theta), 0.0))
    along = float(half_length) * axis
    across = float(thickness) * 0.5 * normal
    corners = np.asarray((
        center - along - across,
        center + along - across,
        center + along + across,
        center - along + across,
    ))
    return np.vstack((corners, corners[0]))


class Scene3DView(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCameraPosition(distance=60.0, elevation=25.0, azimuth=-45.0)
        grid = gl.GLGridItem()
        grid.setSize(50, 50)
        grid.setSpacing(5, 5)
        self.addItem(grid)
        self.reference_item = gl.GLScatterPlotItem(
            pos=np.empty((0, 3)), color=REFERENCE_RGBA, size=1.5, pxMode=True)
        self.adjusted_item = gl.GLScatterPlotItem(
            pos=np.empty((0, 3)), color=ADJUSTED_RGBA, size=2.5, pxMode=True)
        self.origin_item = gl.GLScatterPlotItem(
            pos=np.empty((0, 3)), color=(1.0, 0.15, 0.15, 1.0), size=10, pxMode=True)
        for item in (self.reference_item, self.adjusted_item, self.origin_item):
            self.addItem(item)
        self.axis_items = []
        for color in ((1, 0, 0, 1), (0, 1, 0, 1), (0.15, 0.45, 1, 1)):
            item = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=color, width=3, antialias=True)
            self.axis_items.append(item)
            self.addItem(item)
        self.origin_label = gl.GLTextItem(pos=(0, 0, 0), text="LiDAR Origin", color=(255, 255, 255, 255))
        self.addItem(self.origin_label)
        self.slice_items = {}
        self.slice_labels = {}
        for spec in SLICE_SPECS:
            line = gl.GLLinePlotItem(pos=np.zeros((5, 3)), color=spec.color, width=2, antialias=True)
            label = gl.GLTextItem(pos=(0, 0, 0), text=spec.name, color=tuple(int(v * 255) for v in spec.color))
            self.slice_items[spec.angle_deg] = line
            self.slice_labels[spec.angle_deg] = label
            self.addItem(line)
            self.addItem(label)

    def focus_on(self, center, roi_radius):
        try:
            center = np.asarray(center, dtype=np.float64)
            radius = float(roi_radius)
        except (TypeError, ValueError, OverflowError):
            return
        if center.shape != (3,) or not np.all(np.isfinite(center)) or not np.isfinite(radius) or radius <= 0.0:
            return
        self.setCameraPosition(
            pos=QVector3D(float(center[0]), float(center[1]), float(center[2])),
            distance=max(5.0, radius * 2.0),
        )

    def set_reference(self, points):
        self.reference_item.setData(pos=np.asarray(points), color=REFERENCE_RGBA, size=1.5, pxMode=True)

    def set_adjusted(self, points):
        self.adjusted_item.setData(pos=np.asarray(points), color=ADJUSTED_RGBA, size=2.5, pxMode=True)

    def update_origin(self, center, rotation, axis_length=1.5):
        center = np.asarray(center, dtype=np.float64)
        rotation = np.asarray(rotation, dtype=np.float64)
        self.origin_item.setData(pos=center[None, :], color=(1.0, 0.15, 0.15, 1.0), size=10, pxMode=True)
        for index, item in enumerate(self.axis_items):
            item.setData(pos=np.vstack((center, center + axis_length * rotation[:, index])))
        self.origin_label.setData(pos=center + np.array((0.2, 0.2, 0.3)), text="LiDAR Origin")

    def update_slice_overlays(self, center, half_length, thickness):
        center = np.asarray(center, dtype=np.float64)
        for spec in SLICE_SPECS:
            vertices = slice_rectangle_vertices(center, half_length, thickness, spec.angle_deg)
            self.slice_items[spec.angle_deg].setData(pos=vertices, color=spec.color, width=2)
            theta = np.deg2rad(spec.angle_deg)
            label_position = center + np.array((np.cos(theta), np.sin(theta), 0.0)) * half_length
            label_position[2] += 0.25
            self.slice_labels[spec.angle_deg].setData(pos=label_position, text=spec.name)

