import unittest
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation


class DynamicProfileGeometryTests(unittest.TestCase):
    def test_default_profiles_and_extra_slots_have_required_positions(self):
        from frame_alignment.core.profiles import default_profile_specs, extra_profile_spec

        defaults = default_profile_specs()

        self.assertEqual(
            [(item.profile_id, item.grid_row, item.grid_column, item.angle_deg) for item in defaults],
            [
                ("xz", 0, 0, 0.0),
                ("yz", 1, 0, 90.0),
                ("xz_offset", 0, 1, 0.0),
                ("yz_offset", 1, 1, 90.0),
            ],
        )
        self.assertEqual([item.editable for item in defaults], [False, False, True, True])
        self.assertFalse(any(item.deletable for item in defaults))
        self.assertEqual(
            [(extra_profile_spec(slot).grid_row, extra_profile_spec(slot).grid_column,
              extra_profile_spec(slot).angle_deg) for slot in (0, 1)],
            [(0, 2, 30.0), (1, 2, -60.0)],
        )
        self.assertTrue(all(extra_profile_spec(slot).deletable for slot in (0, 1)))

    def test_xz_geometry_follows_corrected_yaw_and_keeps_map_z(self):
        from frame_alignment.core.profiles import extra_profile_spec, profile_geometry

        rotation = Rotation.from_euler("ZYX", (30.0, 20.0, 15.0), degrees=True).as_matrix()

        geometry = profile_geometry(default_profile_specs()[0], np.zeros(3), rotation)

        angle = np.deg2rad(30.0)
        np.testing.assert_allclose(
            geometry.along_axis, (np.cos(angle), np.sin(angle), 0.0), atol=1e-12)
        np.testing.assert_allclose(
            geometry.across_axis, (-np.sin(angle), np.cos(angle), 0.0), atol=1e-12)
        np.testing.assert_array_equal(geometry.height_axis, (0.0, 0.0, 1.0))

    def test_profile_angle_is_relative_to_corrected_lidar_heading(self):
        from frame_alignment.core.profiles import extra_profile_spec, profile_geometry

        rotation = Rotation.from_euler("z", 30.0, degrees=True).as_matrix()

        geometry = profile_geometry(extra_profile_spec(0), np.zeros(3), rotation)

        angle = np.deg2rad(60.0)
        np.testing.assert_allclose(
            geometry.along_axis, (np.cos(angle), np.sin(angle), 0.0), atol=1e-12)

    def test_parallel_offsets_use_signed_reference_normals(self):
        from frame_alignment.core.profiles import default_profile_specs, profile_geometry

        origin = np.array((10.0, 20.0, 3.0))
        source = default_profile_specs()[2]
        xz_parallel = replace(source, mode="parallel", reference="XZ", offset_m=2.0)
        yz_parallel = replace(source, mode="parallel", reference="YZ", offset_m=2.0)

        xz_geometry = profile_geometry(xz_parallel, origin, np.eye(3))
        yz_geometry = profile_geometry(yz_parallel, origin, np.eye(3))

        np.testing.assert_allclose(xz_geometry.center, (10.0, 22.0, 3.0))
        np.testing.assert_allclose(xz_geometry.along_axis, (1.0, 0.0, 0.0), atol=1e-12)
        np.testing.assert_allclose(yz_geometry.center, (8.0, 20.0, 3.0), atol=1e-12)
        np.testing.assert_allclose(yz_geometry.along_axis, (0.0, 1.0, 0.0), atol=1e-12)

    def test_extract_slice_projects_along_axis_and_relative_map_height(self):
        from frame_alignment.core.profiles import (
            default_profile_specs,
            extract_slice,
            profile_geometry,
        )

        geometry = profile_geometry(default_profile_specs()[0], np.array((0.0, 0.0, 1.0)), np.eye(3))
        points = np.array([
            (2.0, 0.05, 3.0),
            (2.0, 0.11, 4.0),
            (11.0, 0.0, 5.0),
        ])

        result = extract_slice(points, geometry, half_length=10.0, thickness=0.2)

        np.testing.assert_allclose(result, ((2.0, 2.0),))

    def test_invalid_angle_and_extra_slot_are_rejected(self):
        from frame_alignment.core.profiles import default_profile_specs, extra_profile_spec, profile_geometry

        with self.assertRaises(ValueError):
            profile_geometry(replace(extra_profile_spec(0), angle_deg=181.0), np.zeros(3), np.eye(3))
        with self.assertRaises(ValueError):
            extra_profile_spec(2)


if __name__ == "__main__":
    unittest.main()
