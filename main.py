import cv2
import numpy as np
import open3d as o3d
from epnp_module import CameraPoseEstimator
from object_detection import ObjectDetector
from icp_module import icp_registration

# 讀取點雲城市地圖
city_map = o3d.io.read_point_cloud('input/city_map.ply')

# 讀取輸入影像
image = cv2.imread('input/image.jpg')

# 相機內參矩陣（假設已知）
intrinsic_matrix = np.array([[1000, 0, 640],
                             [0, 1000, 360],
                             [0, 0, 1]])

# 初始化 EPnP 和物體檢測
pose_estimator = CameraPoseEstimator(intrinsic_matrix)
detector = ObjectDetector()

# Step 1: 使用物體檢測模型檢測影像中的人與車
labels, coords = detector.detect_objects(image)

# Step 2: 將影像中的 2D 座標轉換為 3D 座標
world_points = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])  # 示例控制點
image_points = np.array([[300, 400], [500, 600], [700, 800]])  # 示例影像點
R, t, error = pose_estimator.estimate_pose(image_points, world_points)
print("Estimated Pose:\nR:", R, "\nt:", t, "\nError:", error)

# Step 3: 將檢測到的人與車像素座標轉換到 3D 空間
for label, coord in zip(labels, coords):
    x_min, y_min, x_max, y_max, conf = coord
    if label == 0:  # 人
        print(f"Detected Person at [{x_min}, {y_min}]")
    elif label == 2:  # 車
        print(f"Detected Car at [{x_min}, {y_min}]")

# Step 4: 更新點雲資料並進行 ICP 對齊
# 假設新的物體點雲已經整合
new_points = np.random.rand(100, 3)  # 模擬新的物體點雲
new_cloud = o3d.geometry.PointCloud()
new_cloud.points = o3d.utility.Vector3dVector(new_points)
transformation, fitness, rmse = icp_registration(city_map, new_cloud)
print(f"ICP fitness: {fitness}, RMSE: {rmse}")

# Step 5: 方圓 1 公里內的點雲範圍
center = np.array([0, 0, 0])  # 假設相機座標為原點
filtered_points = np.asarray(city_map.points)
filtered_points = filtered_points[np.linalg.norm(filtered_points - center, axis=1) < 1000]
filtered_cloud = o3d.geometry.PointCloud()
filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)

# 顯示結果
o3d.visualization.draw_geometries([filtered_cloud])