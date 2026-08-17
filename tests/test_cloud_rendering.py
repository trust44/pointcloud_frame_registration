import unittest

import numpy as np


class CloudRenderingTests(unittest.TestCase):
    def test_uniform_mode_returns_one_color_for_each_point(self):
        from frame_alignment.ui.cloud_rendering import render_cloud_colors

        colors = render_cloud_colors(
            np.zeros((3, 3)), None,
            {"mode": "uniform", "color": "#123456"}, np.zeros(3))

        self.assertEqual(colors.shape, (3, 4))
        np.testing.assert_allclose(colors[0], (0x12 / 255, 0x34 / 255, 0x56 / 255, 1.0))
        np.testing.assert_allclose(colors[0], colors[1])

    def test_native_mode_falls_back_to_uniform_without_native_colors(self):
        from frame_alignment.ui.cloud_rendering import render_cloud_colors

        colors = render_cloud_colors(
            np.zeros((2, 3)), None,
            {"mode": "native", "color": "#FF0000"}, np.zeros(3))

        np.testing.assert_allclose(colors, ((1.0, 0.0, 0.0, 1.0),) * 2)

    def test_cmap_height_mode_uses_shared_fixed_range(self):
        from frame_alignment.ui.cloud_rendering import render_cloud_colors

        points = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]])
        settings = {"mode": "cmap", "cmap": "viridis", "scalar": "z", "range": [0.0, 10.0]}
        colors = render_cloud_colors(points, None, settings, np.zeros(3))

        self.assertEqual(colors.shape, (2, 4))
        self.assertFalse(np.allclose(colors[0], colors[1]))
        self.assertTrue(np.all((colors >= 0.0) & (colors <= 1.0)))

    def test_distance_scalar_uses_origin(self):
        from frame_alignment.ui.cloud_rendering import render_cloud_colors

        points = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        settings = {"mode": "cmap", "cmap": "gray", "scalar": "distance", "range": [0.0, 2.0]}
        colors = render_cloud_colors(points, None, settings, np.zeros(3))

        self.assertLess(colors[0, 0], colors[1, 0])


if __name__ == "__main__":
    unittest.main()
