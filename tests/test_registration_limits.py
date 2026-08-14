import unittest

import numpy as np
from scipy.spatial.transform import Rotation


class RegistrationLimitTests(unittest.TestCase):
    def test_rotation_about_high_coordinate_origin_has_zero_origin_displacement(self):
        from frame_alignment.core.registration import correction_magnitudes_about_point

        center = np.array([8628.86, 9650.01, 106.995])
        rotation = Rotation.from_euler("z", 1.0, degrees=True).as_matrix()
        increment = np.eye(4)
        increment[:3, :3] = rotation
        increment[:3, 3] = center - rotation @ center

        translation_m, rotation_deg = correction_magnitudes_about_point(increment, center)
        self.assertAlmostEqual(translation_m, 0.0, places=9)
        self.assertAlmostEqual(rotation_deg, 1.0, places=9)
        self.assertGreater(np.linalg.norm(increment[:3, 3]), 100.0)

    def test_actual_origin_translation_is_measured(self):
        from frame_alignment.core.registration import correction_magnitudes_about_point

        center = np.array([8628.86, 9650.01, 106.995])
        increment = np.eye(4)
        increment[:3, 3] = [0.3, -0.4, 0.0]
        translation_m, rotation_deg = correction_magnitudes_about_point(increment, center)
        self.assertAlmostEqual(translation_m, 0.5)
        self.assertAlmostEqual(rotation_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
