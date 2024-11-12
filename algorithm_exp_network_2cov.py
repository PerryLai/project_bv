import open3d as o3d
import os
import json
import random
import math
import numpy as np
from util import mergeply, draw_registration_result, prepare_dataset, execute_global_registration, rotate_source, icp, count_walker, count_cyclist, count_vehicle
from algorithms import golden_sample, resolution, roi, accumulated_roi, covisible_roi, filter_occlusion_points

# ALG = "Base"
ALG = "ROI"
path = "data_network_SSMCO"
threshold = 0.2
correct = 0
total_error = 0
error_list = []
total_occlusion_reduced_rate = 0
total_covisible_reduced_rate = 0
occlusion_size_list = []
reduced_size_list = []
total_estimated_position_error = 0
estimated_position_error_list = []
points_density = 10
reduced_rate_target = 0.15

correct = 0
total = 0
correctlist = [0 for i in range(20)]
totallist = [0 for i in range(20)]


for n in range(0,100):
    print("Processing: ", n)
    file_path = os.path.join(path, str(n))
    cov_path = os.path.join(path, str(n), 'COV')

    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    result_points = target.points
    
    i = 1
    while i < 20:
        filename = os.path.join(file_path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        source = o3d.io.read_point_cloud(os.path.join(cov_path, 'data{}.ply'.format(chr(ord('A')+i))))
        egoB = json.load(open(filename))

        # ## Visual
        # trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # draw_registration_result(source, target, trans_init)

        egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
        egoByaw = egoB['yaw'] if egoB['yaw'] > 0 else (egoB['yaw'] + 360)
        rotation_angle = (egoAyaw - egoByaw)*math.pi/180

        yaw = egoAyaw*math.pi/180 - math.pi/2
        original_x = egoB['x'] - egoA['x']
        original_y = egoB['y'] - egoA['y']
        origianl_z = egoB['z'] - egoA['z']
        original_diff = np.array([original_x, original_y])

        rotation_matrix = np.array([[math.cos(yaw), math.sin(yaw)],
                                  [math.sin(yaw), -math.cos(yaw)]])
        rotated_diff = np.matmul(rotation_matrix, original_diff)
        x = rotated_diff[0]
        y = rotated_diff[1]

        ## End Preprocessing algorithm



        trans_final = icp(source, target)
        # draw_registration_result(source, target, trans_final)

        t_x = np.array(trans_final)[0, 3]
        t_y = np.array(trans_final)[1, 3]
        print("Translation final: ", trans_final)


        error = math.sqrt((t_x+x)**2+(t_y+y)**2)
        print("error: ", error)

        total += 1
        totallist[i] += 1
        if error < threshold:
            correct += 1
            correctlist[i] += 1
        
        i = i+1
    
    
#     pcd = o3d.geometry.PointCloud()
#     trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#     pcd.points = o3d.utility.Vector3dVector(result_points)
#     draw_registration_result(pcd, pcd, trans_init)
    
    print("Correct: ", correct)
    print("Total: ", total)
    print("CorrectList: ", correctlist)
    print("TotalList: ", totallist)
