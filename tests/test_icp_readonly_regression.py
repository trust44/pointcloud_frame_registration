import unittest

import numpy as np


class IcpReadonlyRegressionTests(unittest.TestCase):
    def test_constrained_icp_accepts_open3d_readonly_transformation(self):
        from frame_alignment.core.point_cloud import Cloud
        from frame_alignment.core.pose_model import PoseModel
        from frame_alignment.core.registration import constrained_icp

        points = np.random.default_rng(0).random((100, 3))
        stats = constrained_icp(PoseModel(np.eye(4)), Cloud(points), Cloud(points))

        self.assertIn("icp_rmse_m", stats)
        self.assertIn("icp_fitness", stats)


if __name__ == "__main__":
    unittest.main()
