import open3d as o3d
import os
import json
import math
import numpy as np
from util import mergeply, draw_registration_result, prepare_dataset, execute_global_registration, rotate_source, icp

path = "data"
threshold = 0.2
correct = 0
total_error = 0
error_list = []

for i in range(100):
    print("Processing: ", i)
    file_path = os.path.join(path, str(i))

    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    source = o3d.io.read_point_cloud(os.path.join(file_path, 'dataB.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    egoB = json.load(open(os.path.join(file_path, 'ego2.json')))
    
    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
    egoByaw = egoB['yaw'] if egoB['yaw'] > 0 else (-egoB['yaw'] + 360)
    rotation_angle = (egoByaw - egoAyaw)*math.pi/180
    
    source = rotate_source(source, rotation_angle)
    
    yaw = egoAyaw*math.pi/180 - math.pi/2
    original_x = egoB['x'] - egoA['x']
    original_y = egoB['y'] - egoA['y']
    original_diff = np.array([original_x, original_y])
    print(original_diff)
    
    rotation_matrix = np.array([[math.cos(yaw), math.sin(yaw)],
                              [math.sin(yaw), -math.cos(yaw)]])
    rotated_diff = np.matmul(rotation_matrix, original_diff)
    x = rotated_diff[0]
    y = rotated_diff[1]
    
    ## Preprocessing Algorithm
    
    # trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # draw_registration_result(source, target, trans_init)
    trans_final = icp(source, target)
    # draw_registration_result(source, target, trans_final)
    
    t_x = np.array(trans_final)[0, 3]
    t_y = np.array(trans_final)[1, 3]
    print(trans_final)
    print('t_x', t_x)
    print('t_y', t_y)
    print('x', x)
    print('y', y)
    
    error = math.sqrt((t_x+x)**2+(t_y+y)**2)
    print("error", error)
    
    total_error += error
    error_list.append(error)
    if error < threshold:
        correct += 1
    print("corret", correct)
    
print("Final correct:", correct)
print("Final Total error:", total_error/100)
print("Error list:", error_list)