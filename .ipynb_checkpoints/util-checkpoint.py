from carlaAPI.image_converter import depth_to_local_point_cloud, to_rgb_array
import numpy as np
import carla
import os
import math
import copy
import open3d as o3d

def icp(source, target, voxel_size = 0.5, threshold = 0.2):
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)

    result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
    trans_fast = result_ransac.transformation

    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_fast,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    
    return reg_p2l.transformation



def install_camera_set(world, ego_vehicle, dirname, high=2):
    
    camera_list = []
    
    cam_bp = None
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x",str(1920))
    cam_bp.set_attribute("image_size_y",str(1080))
    cam_location = carla.Location(0,0,80)
    cam_rotation = carla.Rotation(pitch=-90)
    cam_transform = carla.Transform(cam_location,cam_rotation)
    ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    ego_cam.listen(lambda image: image.save_to_disk(dirname+'/rgb.jpg') )
    camera_list.append(cam_bp)
    camera_list.append(ego_cam)
        
    depth1_cam = None
    depth1_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth1_bp.set_attribute("fov", str(90))
    depth1_location = carla.Location(0,0,high)
    depth1_rotation = carla.Rotation(0,0,0)
    depth1_transform = carla.Transform(depth1_location,depth1_rotation)
    depth1_cam = world.spawn_actor(depth1_bp,depth1_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler1(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/1.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '1.ply')
        )

    depth1_cam.listen(handler1)
    camera_list.append(depth1_bp)
    camera_list.append(depth1_cam)

    depth2_cam = None
    depth2_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth2_bp.set_attribute("fov", str(90))
    depth2_location = carla.Location(0,0,high)
    depth2_rotation = carla.Rotation(0,180,0)
    depth2_transform = carla.Transform(depth2_location,depth2_rotation)
    depth2_cam = world.spawn_actor(depth2_bp,depth2_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler2(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/2.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '2.ply')
        )

    depth2_cam.listen(handler2)
    camera_list.append(depth2_bp)
    camera_list.append(depth2_cam)

    depth3_cam = None
    depth3_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth3_bp.set_attribute("fov", str(90))
    depth3_location = carla.Location(0,0,high)
    depth3_rotation = carla.Rotation(0,90,0)
    depth3_transform = carla.Transform(depth3_location,depth3_rotation)
    depth3_cam = world.spawn_actor(depth3_bp,depth3_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler3(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/3.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '3.ply')
        )

    depth3_cam.listen(handler3)
    camera_list.append(depth3_bp)
    camera_list.append(depth3_cam)

    depth4_cam = None
    depth4_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth4_bp.set_attribute("fov", str(90))
    depth4_location = carla.Location(0,0,high)
    depth4_rotation = carla.Rotation(0,270,0)
    depth4_transform = carla.Transform(depth4_location,depth4_rotation)
    depth4_cam = world.spawn_actor(depth4_bp,depth4_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler4(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/4.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '4.ply')
        )

    depth4_cam.listen(handler4)
    camera_list.append(depth4_bp)
    camera_list.append(depth4_cam)
    
    return camera_list

def install_camera_set_for_car_model(world, ego_vehicle, dirname, high=2):
    
    camera_list = []
    
    depth1_cam = None
    depth1_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth1_bp.set_attribute("fov", str(90))
    depth1_location = carla.Location(-8,0,high)
    depth1_rotation = carla.Rotation(0,0,0)
    depth1_transform = carla.Transform(depth1_location,depth1_rotation)
    depth1_cam = world.spawn_actor(depth1_bp,depth1_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler1(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/1.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '1.ply')
        )

    depth1_cam.listen(handler1)
    camera_list.append(depth1_bp)
    camera_list.append(depth1_cam)

    depth2_cam = None
    depth2_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth2_bp.set_attribute("fov", str(90))
    depth2_location = carla.Location(8,0,high)
    depth2_rotation = carla.Rotation(0,180,0)
    depth2_transform = carla.Transform(depth2_location,depth2_rotation)
    depth2_cam = world.spawn_actor(depth2_bp,depth2_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler2(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/2.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '2.ply')
        )

    depth2_cam.listen(handler2)
    camera_list.append(depth2_bp)
    camera_list.append(depth2_cam)

    depth3_cam = None
    depth3_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth3_bp.set_attribute("fov", str(90))
    depth3_location = carla.Location(0,-5,high)
    depth3_rotation = carla.Rotation(0,90,0)
    depth3_transform = carla.Transform(depth3_location,depth3_rotation)
    depth3_cam = world.spawn_actor(depth3_bp,depth3_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler3(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/3.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '3.ply')
        )

    depth3_cam.listen(handler3)
    camera_list.append(depth3_bp)
    camera_list.append(depth3_cam)

    depth4_cam = None
    depth4_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth4_bp.set_attribute("fov", str(90))
    depth4_location = carla.Location(0,5,high)
    depth4_rotation = carla.Rotation(0,270,0)
    depth4_transform = carla.Transform(depth4_location,depth4_rotation)
    depth4_cam = world.spawn_actor(depth4_bp,depth4_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler4(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/4.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '4.ply')
        )

    depth4_cam.listen(handler4)
    camera_list.append(depth4_bp)
    camera_list.append(depth4_cam)
    
    return camera_list

def mergeply(path, ego, thresh):
    plypath = os.path.join(path, ego)

    mesh1 = o3d.io.read_point_cloud(os.path.join(plypath,"1.ply"))
    mesh2 = o3d.io.read_point_cloud(os.path.join(plypath,"2.ply"))
    mesh3 = o3d.io.read_point_cloud(os.path.join(plypath,"3.ply"))
    mesh4 = o3d.io.read_point_cloud(os.path.join(plypath,"4.ply"))



    point1 = np.asarray(mesh1.points)
    point2 = np.asarray(mesh2.points)
    point3 = np.asarray(mesh3.points)
    point4 = np.asarray(mesh4.points)
    point2[:, 1] = -point2[:, 1]
    point2[:, 0] = -point2[:, 0]

    point3[:, [0,1]] = point3[:, [1,0]]
    point3[:, 1] = -point3[:, 1]

    point4[:, [0,1]] = point4[:, [1,0]]
    point4[:, 0] = -point4[:, 0]

    points = np.concatenate((point1, point2, point3, point4), axis=0)
    points = points[np.where(points[:,2] > thresh)]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(os.path.join(path, "data{}.ply".format(ego)), pcd)

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, 0.2951, 0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
    
def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source, target, voxel_size):
    # print(":: Load two point clouds and disturb initial pose.")

    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def rotate_source(source, angle):
    rotation_matrix = np.array([[math.cos(angle), -math.sin(angle), 0],
                              [math.sin(angle), math.cos(angle), 0],
                                [0, 0, 1]])
    center = np.array([0, 0, 0])
    
    source.rotate(rotation_matrix, center)
    return source

def count_walker(points, walkers, rotation_matrix, ego_center, range=50):
    center_x = ego_center[0]
    center_y = ego_center[1]
    total_walkers = []
    found_walkers = []
    for walker in walkers:
        walker_x = walker['x']
        walker_y = walker['y']
        if (walker_x - center_x)**2 + (walker_y - center_y)**2 < range**2:

            total_walkers.append(walker)

            original_x = walker_x - center_x
            original_y = walker_y - center_y
            original_diff = np.array([original_x, original_y])

            rotated_diff = np.matmul(rotation_matrix, original_diff)

            xy_bool_array = np.logical_and(abs(points[:,0] + rotated_diff[0]) < 0.5, abs(points[:,1] + rotated_diff[1]) < 0.5)

            found_points = points[xy_bool_array]
            found_points = found_points[found_points > -1.8]

            if found_points.shape[0] > 10:
                found_walkers.append(walker)
    
    return total_walkers, found_walkers

def count_cyclist(points, cyclists, rotation_matrix, ego_center, range=50):
    center_x = ego_center[0]
    center_y = ego_center[1]
    total_cyclists = []
    found_cyclists = []
    for cyclist in cyclists:
        cyclist_x = cyclist['x']
        cyclist_y = cyclist['y']
        if (cyclist_x - center_x)**2 + (cyclist_y - center_y)**2 < range**2:

            total_cyclists.append(cyclist)

            original_x = cyclist_x - center_x
            original_y = cyclist_y - center_y
            original_diff = np.array([original_x, original_y])

            rotated_diff = np.matmul(rotation_matrix, original_diff)

            xy_bool_array = np.logical_and(abs(points[:,0] + rotated_diff[0]) < 1, abs(points[:,1] + rotated_diff[1]) < 1)

            found_points = points[xy_bool_array]
            found_points = found_points[found_points > -1.8]

            if found_points.shape[0] > 10:
                found_cyclists.append(cyclist)
    
    return total_cyclists, found_cyclists

def count_vehicle(points, vehicles, rotation_matrix, ego_center, ego_yaw, range=50):
    center_x = ego_center[0]
    center_y = ego_center[1]
    total_vehicles = []
    found_vehicles = []
    for vehicle in vehicles:
        vehicle_x = vehicle['x']
        vehicle_y = vehicle['y']
        vehicle_yaw = vehicle['yaw'] if vehicle['yaw'] > 0 else (vehicle['yaw'] + 360)
        if (vehicle_x - center_x)**2 + (vehicle_y - center_y)**2 < range**2:

            total_vehicles.append(vehicle)

            original_x = vehicle_x - center_x
            original_y = vehicle_y - center_y
            original_diff = np.array([original_x, original_y])

            rotated_diff = np.matmul(rotation_matrix, original_diff)
            
            if abs(ego_yaw - vehicle_yaw) < 3:
                x_err = 1.5
                y_err = 4
            else:
                x_err = 4
                y_err = 1.5

            xy_bool_array = np.logical_and(abs(points[:,0] + rotated_diff[0]) < x_err, abs(points[:,1] + rotated_diff[1]) < y_err)

            found_points = points[xy_bool_array]
            found_points = found_points[found_points > -1.8]

            if found_points.shape[0] > 20:
                found_vehicles.append(vehicle)
    
    return total_vehicles, found_vehicles

