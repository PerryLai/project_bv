import open3d as o3d
import os
import json
import random
import math
import numpy as np
from util import mergeply, draw_registration_result, prepare_dataset, execute_global_registration, rotate_source, icp, count_walker, count_cyclist, count_vehicle
from algorithms import golden_sample, resolution, roi, accumulated_roi, covisible_roi, filter_occlusion_points, filter_target_points, new_filter_occlusion_points

# ALG = "Base"
ALG = "ROI"
path = "data_network_SSMCO"
threshold = 0.2
points_density = 10
reduced_rate_target = 0.25


for i in range(100):
    occlusion_size_list = []
    reduced_size_list = []
    estimated_target_list = []
    print("Processing: ", i)
    file_path = os.path.join(path, str(i))
    cov_path = os.path.join(file_path, 'COV')
    occ_path = os.path.join(file_path, 'OCC')
    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))

    i = 1
    while i < 20:
        filename = os.path.join(file_path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        source = o3d.io.read_point_cloud(os.path.join(file_path, 'data{}.ply'.format(chr(ord('A')+i))))
        egoB = json.load(open(filename))

        egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
        egoByaw = egoB['yaw'] if egoB['yaw'] > 0 else (egoB['yaw'] + 360)
        rotation_angle = (egoAyaw - egoByaw)*math.pi/180

        source = rotate_source(source, rotation_angle)
        
        w = egoB['box_y']/2
        l = egoB['box_x']/2
        corner = np.array([[w, l, 0],[-w, l, 0],[-w, -l, 0],[w, -l, 0]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(corner)
        corner = rotate_source(pcd, rotation_angle)

        yaw = egoAyaw*math.pi/180 - math.pi/2
        original_x = egoB['x'] - egoA['x']
        original_y = egoB['y'] - egoA['y']
        z = egoB['box_z'] - egoA['box_z']
        original_diff = np.array([original_x, original_y])

        rotation_matrix = np.array([[math.cos(yaw), math.sin(yaw)],
                                  [math.sin(yaw), -math.cos(yaw)]])
        rotated_diff = np.matmul(rotation_matrix, original_diff)
        x = rotated_diff[0]
        y = rotated_diff[1]
        
        # ## Visual
        trans_init = np.asarray([[1, 0, 0, -x], [0, 1, 0, -y], [0, 0, 1, z], [0, 0, 0, 1]])
        # draw_registration_result(source, target, trans_init)
        # draw_registration_result(corner, target, trans_init)
        
        ## Preprocessing Algorithm
        original_size = len(source.points)
        occlusion_rate = 0

        estimated_target_position = [x + random.gauss(0, 1), y + random.gauss(0, 1), -z+random.gauss(0, 0.2)]
        # print('target_position:', [x, y, -1])
        # print('estimated_target_position', estimated_target_position)
        estimated_target_list.append(estimated_target_position)
        
        occlusion_points = new_filter_occlusion_points(source, estimated_target_position, corner)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(occlusion_points)
        # draw_registration_result(pcd, target, trans_init)
        
        o3d.io.write_point_cloud(os.path.join(occ_path, "data{}.ply").format(chr(ord('A')+i)), pcd)
        
        occlusion_rate = len(pcd.points)/original_size
        occlusion_size_list.append(occlusion_rate)
        print("occlusion rate:", occlusion_rate)

        source = covisible_roi(source, estimated_target_position, density=points_density, reduced_rate_target=reduced_rate_target)

        reduced_size = len(source.points)
        rate = reduced_size/original_size
        reduced_size_list.append(rate)
        print("reduced rate:", rate)

        pcd.points = o3d.utility.Vector3dVector(source.points)
        
        # draw_registration_result(pcd, target, trans_init)
        o3d.io.write_point_cloud(os.path.join(cov_path, "data{}.ply").format(chr(ord('A')+i)), pcd)
        i=i+1
    
    with open(os.path.join(occ_path,"rate.json"), "w") as outfile:
        json_object = json.dumps(occlusion_size_list)
        outfile.write(json_object)
        
    with open(os.path.join(cov_path,"rate.json"), "w") as outfile:
        json_object = json.dumps(reduced_size_list)
        outfile.write(json_object)
        
    with open(os.path.join(cov_path,"estimated_target.json"), "w") as outfile:
        json_object = json.dumps(estimated_target_list)
        outfile.write(json_object)