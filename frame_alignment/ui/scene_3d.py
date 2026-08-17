"""Embedded pyqtgraph OpenGL scene with point clouds and slice overlays."""
import numpy as np
from PySide6.QtGui import QVector3D
import pyqtgraph.opengl as gl

from frame_alignment.core.profiles import ProfileGeometry


REFERENCE_RGBA = (144 / 255.0, 164 / 255.0, 174 / 255.0, 0.72)
ADJUSTED_RGBA = (246 / 255.0, 196 / 255.0, 69 / 255.0, 0.96)


def slice_rectangle_vertices(geometry, half_length, vertical_half_length=None):
    """Return a closed vertical plane outline for one resolved profile."""
    if not isinstance(geometry, ProfileGeometry):
        raise TypeError("geometry must be a ProfileGeometry")
    half_length = float(half_length)
    if vertical_half_length is None:
        vertical_half_length = half_length
    vertical_half_length = float(vertical_half_length)
    if not np.isfinite(half_length) or half_length <= 0.0:
        raise ValueError("half_length must be positive and finite")
    if not np.isfinite(vertical_half_length) or vertical_half_length <= 0.0:
        raise ValueError("vertical_half_length must be positive and finite")
    along = half_length * geometry.along_axis
    vertical = vertical_half_length * geometry.height_axis
    corners = np.asarray((
        geometry.center - along - vertical,
        geometry.center + along - vertical,
        geometry.center + along + vertical,
        geometry.center - along + vertical,
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

    def update_slice_overlays(self, geometries, half_length):
        """Synchronize vertical profile outlines with the current profile set."""
        geometries = tuple(geometries)
        if any(not isinstance(geometry, ProfileGeometry) for geometry in geometries):
            raise TypeError("geometries must contain ProfileGeometry values")
        profile_ids = [geometry.profile_id for geometry in geometries]
        if len(profile_ids) != len(set(profile_ids)):
            raise ValueError("profile ids must be unique")

        active_ids = set(profile_ids)
        for profile_id in set(self.slice_items) - active_ids:
            self.removeItem(self.slice_items.pop(profile_id))
            self.removeItem(self.slice_labels.pop(profile_id))

        for geometry in geometries:
            profile_id = geometry.profile_id
            color = tuple(geometry.color)
            if profile_id not in self.slice_items:
                line = gl.GLLinePlotItem(
                    pos=np.zeros((5, 3)), color=color, width=2, antialias=True)
                label = gl.GLTextItem(
                    pos=(0, 0, 0), text=geometry.name,
                    color=tuple(int(component * 255) for component in color))
                self.slice_items[profile_id] = line
                self.slice_labels[profile_id] = label
                self.addItem(line)
                self.addItem(label)
            vertices = slice_rectangle_vertices(geometry, half_length)
            self.slice_items[profile_id].setData(pos=vertices, color=color, width=2)
            label_position = (
                geometry.center + geometry.along_axis * float(half_length)
                + geometry.height_axis * 0.25)
            self.slice_labels[profile_id].setData(pos=label_position, text=geometry.name)

