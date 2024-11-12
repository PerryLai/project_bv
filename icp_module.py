import open3d as o3d
import numpy as np

def icp_registration(source, target, threshold=0.5):
    """使用 ICP 進行點雲對齊"""
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return icp_result.transformation, icp_result.fitness, icp_result.inlier_rmse