"""Global-coordinate layout checks for the right-hand controls."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtCore, QtWidgets

from frame_alignment.ui.main_window import MainWindow


def test_controls_are_ordered_in_right_sidebar_by_global_coordinates():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow(message_sink=lambda level, text: None)
    window.resize(1600, 980)
    window.show()
    app.processEvents()

    scene_pos = window.scene.mapTo(window, QtCore.QPoint(0, 0))
    profile_pos = window.profiles[0].mapTo(window, QtCore.QPoint(0, 0))
    sidebar_pos = window.right_sidebar.mapTo(window, QtCore.QPoint(0, 0))
    pose_pos = window.pose_panel.mapTo(window, QtCore.QPoint(0, 0))
    matrix_pos = window.matrix_panel.mapTo(window, QtCore.QPoint(0, 0))
    data_pos = window.data_panel.mapTo(window, QtCore.QPoint(0, 0))

    assert scene_pos.x() < sidebar_pos.x()
    assert profile_pos.x() < sidebar_pos.x()
    assert pose_pos.y() < matrix_pos.y() < data_pos.y()
    window.close()
