import carla
import os
import numpy as np
from carlaAPI.image_converter import depth_to_local_point_cloud
import open3d as o3d

def extract_building_point_cloud(world, dirname="building_point_clouds"):
    """
    提取CARLA世界中的所有建築物點雲並儲存為 .ply 文件
    
    Parameters:
        world (carla.World): CARLA 世界實例
        dirname (str): 存儲生成的點雲文件的目錄名稱
    """
    
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    # 找出所有建築物藍圖
    building_blueprints = [bp for bp in world.get_blueprint_library().filter("*building*")]

    # 初始化變數來存儲所有建築物的點雲
    all_buildings_point_cloud = o3d.geometry.PointCloud()

    for i, blueprint in enumerate(building_blueprints):
        building_spawn_point = world.get_map().get_spawn_points()[i % len(world.get_map().get_spawn_points())]
        building = world.try_spawn_actor(blueprint, building_spawn_point)

        if building is None:
            continue

        # 設置深度相機以獲取建築物的點雲
        depth_camera_bp = world.get_blueprint_library().find("sensor.camera.depth")
        depth_camera_bp.set_attribute("image_size_x", "1920")
        depth_camera_bp.set_attribute("image_size_y", "1080")
        depth_camera_bp.set_attribute("fov", "90")

        depth_camera_transform = carla.Transform(carla.Location(x=0, z=50), carla.Rotation(pitch=-90))
        depth_camera = world.spawn_actor(depth_camera_bp, depth_camera_transform, attach_to=building, attachment_type=carla.AttachmentType.Rigid)

        # 收集深度資料
        point_cloud = None
        def process_depth_image(image):
            nonlocal point_cloud
            point_cloud = depth_to_local_point_cloud(image, None, max_depth=0.1)

        depth_camera.listen(lambda image: process_depth_image(image))
        world.tick()

        # 等待深度資料生成
        while point_cloud is None:
            world.tick()
        
        # 儲存點雲並將其添加到所有建築物的點雲
        building_point_cloud_file = os.path.join(dirname, f"building_{i}.ply")
        point_cloud.save_to_disk(building_point_cloud_file)
        building_point_cloud = o3d.io.read_point_cloud(building_point_cloud_file)
        all_buildings_point_cloud += building_point_cloud

        depth_camera.stop()
        depth_camera.destroy()
        building.destroy()
    
    # 儲存完整的所有建築物的點雲
    o3d.io.write_point_cloud(os.path.join(dirname, "all_buildings_point_cloud.ply"), all_buildings_point_cloud)
    print("已成功提取所有建築物的點雲")

# 使用範例
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

extract_building_point_cloud(world)
