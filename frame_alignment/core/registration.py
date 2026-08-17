"""Constrained point-to-plane ICP refinement and nearest-neighbour quality."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .point_cloud import Cloud


def residuals(source, target):
    if len(source) == 0 or len(target) == 0:
        return {"nn_residual_median_m": None, "nn_residual_p95_m": None}
    distances = cKDTree(target).query(source, k=1)[0]
    return {"nn_residual_median_m": float(np.median(distances)),
            "nn_residual_p95_m": float(np.percentile(distances, 95))}


def correction_magnitudes_about_point(increment, point):
    """Return displacement at ``point`` and rotation angle for an ICP increment."""
    increment = np.asarray(increment, dtype=np.float64)
    point = np.asarray(point, dtype=np.float64)
    rotation = increment[:3, :3]
    moved_point = rotation @ point + increment[:3, 3]
    distance = float(np.linalg.norm(moved_point - point))
    angle = float(np.degrees(Rotation.from_matrix(rotation).magnitude()))
    return distance, angle


def constrained_icp(model, source, target, radius=25.0, voxel=0.08, correspondence=0.35,
                    max_translation=0.5, max_rotation=3.0):
    import open3d as o3d
    center = model.current_origin
    source_roi = Cloud(model.transform_points(source.points), source.colors).roi(center, radius).voxel(voxel)
    target_roi = target.roi(center, radius).voxel(voxel)
    if len(source_roi.points) < 10 or len(target_roi.points) < 10:
        raise ValueError("ICP ROI requires at least 10 points in each cloud")
    source_3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_roi.points))
    target_3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_roi.points))
    target_3d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(0.1, voxel * 3), max_nn=30))
    outcome = o3d.pipelines.registration.registration_icp(
        source_3d, target_3d, correspondence, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
    increment = np.asarray(outcome.transformation, dtype=np.float64).copy()
    distance, angle = correction_magnitudes_about_point(increment, center)
    if distance > max_translation or angle > max_rotation:
        raise ValueError("Rejected ICP correction ({:.3f} m, {:.3f} deg)".format(distance, angle))
    model.set_transform(increment @ model.transform)
    stats = residuals(model.transform_points(source.points), target.roi(model.current_origin, radius).points)
    stats.update({"icp_rmse_m": float(outcome.inlier_rmse), "icp_fitness": float(outcome.fitness)})
    return stats
