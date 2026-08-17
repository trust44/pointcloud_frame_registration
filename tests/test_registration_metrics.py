import unittest

import numpy as np


class RegistrationMetricTests(unittest.TestCase):
    def test_nearest_neighbor_returns_statistics_and_threshold_ratio(self):
        from frame_alignment.core.registration import matching_error

        source = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.4, 0.0, 0.0]])
        target = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        stats = matching_error(source, target, method="nearest_neighbor", threshold=0.2)

        self.assertAlmostEqual(stats["median_m"], 0.1)
        self.assertAlmostEqual(stats["mean_m"], (0.0 + 0.1 + 0.4) / 3.0)
        self.assertAlmostEqual(stats["rmse_m"], np.sqrt((0.0 + 0.01 + 0.16) / 3.0))
        self.assertAlmostEqual(stats["p95_m"], 0.37)
        self.assertAlmostEqual(stats["match_ratio"], 2.0 / 3.0)

    def test_symmetric_chamfer_combines_both_directions(self):
        from frame_alignment.core.registration import matching_error

        source = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        target = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        stats = matching_error(source, target, method="symmetric_chamfer", threshold=0.2)

        self.assertAlmostEqual(stats["mean_m"], 0.25)
        self.assertAlmostEqual(stats["match_ratio"], 0.75)

    def test_point_to_plane_uses_supplied_target_normals(self):
        from frame_alignment.core.registration import matching_error

        source = np.array([[0.0, 0.0, 0.1], [0.0, 0.0, 0.3]])
        target = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        normals = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])

        stats = matching_error(
            source, target, method="point_to_plane", threshold=0.2,
            target_normals=normals)

        self.assertAlmostEqual(stats["median_m"], 0.2)
        self.assertAlmostEqual(stats["mean_m"], 0.2)
        self.assertAlmostEqual(stats["rmse_m"], np.sqrt(0.05))
        self.assertAlmostEqual(stats["match_ratio"], 0.5)

    def test_empty_inputs_return_empty_statistics(self):
        from frame_alignment.core.registration import matching_error

        stats = matching_error(np.empty((0, 3)), np.zeros((1, 3)))

        self.assertEqual(stats, {
            "median_m": None, "mean_m": None, "rmse_m": None,
            "p95_m": None, "match_ratio": None,
        })


if __name__ == "__main__":
    unittest.main()
