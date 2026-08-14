import unittest

import numpy as np


class CoreBehaviorTests(unittest.TestCase):
    def test_rotation_uses_initial_lidar_origin_as_fixed_pivot(self):
        from frame_alignment.core.pose_model import Delta, PoseModel

        initial = np.eye(4)
        initial[:3, 3] = (100.0, 200.0, 10.0)
        model = PoseModel(initial)
        model.set_delta(Delta(dx_m=1.0, dy_m=-2.0, dz_m=0.5, yaw_deg=90.0))

        np.testing.assert_allclose(model.corrected_pose[:3, 3], (101.0, 198.0, 10.5), atol=1e-12)
        transformed_pivot = model.transform_points(np.array([[100.0, 200.0, 10.0]]))[0]
        np.testing.assert_allclose(transformed_pivot, (101.0, 198.0, 10.5), atol=1e-12)

    def test_source_points_are_not_multiplied_by_initial_pose(self):
        from frame_alignment.core.pose_model import PoseModel

        initial = np.eye(4)
        initial[:3, 3] = (1000.0, 2000.0, 100.0)
        source_in_map = np.array([[1001.0, 2002.0, 103.0]])
        model = PoseModel(initial)
        np.testing.assert_array_equal(model.transform_points(source_in_map), source_in_map)

    def test_new_initial_pose_clears_delta_and_history(self):
        from frame_alignment.core.pose_model import Delta, PoseModel

        model = PoseModel(np.eye(4))
        model.set_delta(Delta(dx_m=1.0))
        model.set_initial_pose(np.eye(4))
        self.assertEqual(model.delta, Delta())
        self.assertFalse(model.undo())


if __name__ == "__main__":
    unittest.main()
