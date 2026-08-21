import unittest

import numpy as np


class CurrentPoseIcpTests(unittest.TestCase):
    def test_icp_source_uses_the_existing_manual_transform(self):
        from frame_alignment.core.point_cloud import Cloud
        from frame_alignment.core.pose_model import PoseModel
        from frame_alignment.core.registration import current_pose_source

        initial = np.eye(4)
        initial[:3, 3] = (100.0, 200.0, 0.0)
        model = PoseModel(initial)
        model.adjust("dx_m", 1.25)
        model.adjust("yaw_deg", 90.0)
        source = Cloud(np.array([[101.0, 200.0, 0.0]]))

        prepared = current_pose_source(model, source)

        np.testing.assert_allclose(prepared.points, model.transform_points(source.points))
        np.testing.assert_allclose(prepared.points, [[101.25, 201.0, 0.0]])

