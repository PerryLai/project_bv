import numpy as np
import EPnP
import open3d as o3d

class CameraPoseEstimator:
    def __init__(self, intrinsic_matrix):
        self.epnp = EPnP.EPnP()
        self.A = intrinsic_matrix  # 相機內參矩陣

    def estimate_pose(self, image_points, world_points):
        """使用 EPnP 演算法計算相機姿勢 (R, t)"""
        error, Rt, Cc, Xc = self.epnp.efficient_pnp(world_points, image_points, self.A)
        R = Rt[:3, :3]
        t = Rt[:3, 3]
        return R, t, error