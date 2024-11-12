import open3d as o3d
import os
import math
import random
import numpy as np
import argparse
from util import mergeply, draw_registration_result, prepare_dataset, execute_global_registration
import json
from json import JSONEncoder
  
class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)

path = "data"

for i in range(100):
    print(i)
    file_path = os.path.join(path, str(i))
    
    jsonA = os.path.join(file_path, 'ego.json')
    jsonB = os.path.join(file_path, 'ego2.json')
    # Opening JSON file
    f = open(jsonA)
    egoA = json.load(f)
    print(egoA)
    f = open(jsonB)
    egoB = json.load(f)
    print(egoB)
    
    rotation_angle = (egoA['yaw'] - egoB['yaw'])*math.pi/180
    
    pathA = os.path.join(file_path, 'dataA.ply')
    pathB = os.path.join(file_path, 'dataB.ply')
    if not os.path.exists(pathA):
        mergeply(file_path, 'A')
    if not os.path.exists(pathB):
        mergeply(file_path, 'B')
    target = o3d.io.read_point_cloud(pathA)
    source = o3d.io.read_point_cloud(pathB)
    rotation_matrix = np.array([[math.cos(rotation_angle), -math.sin(rotation_angle), 0],
                              [math.sin(rotation_angle), math.cos(rotation_angle), 0],
                                [0, 0, 1]])
    source.rotate(rotation_matrix)
    
    trans_init = np.asarray([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0], [0.0, 0.0, 0.0, 1.0]])

    source_points = source.points

    init_angle = math.atan2( egoB['y']-egoA['y'], egoB['x']-egoA['x'] )*360/(2*math.pi)
    if init_angle < 0:
        init_angle += 360
    start_init = (init_angle - 90)
    end_init = (init_angle + 90)
    
    transformation_result = []
    
    for j in range(2):
        print("Test ROI", j)
        filter_points = []
        start = start_init + j*180
        end = end_init + j*180
        
        if start > 360 and end > 360:
            start -= 360
            end -= 360
        
        print(start)
        print(end)
        
        for point in source_points:
            x = point[0]
            y = point[1]
            z = point[2]
            temp_angle = math.atan2( y, x )
            if end > 360:
                angle = temp_angle + 2*math.pi
            else:
                angle = temp_angle if temp_angle > 0 else temp_angle + 2*math.pi
            if angle > start*math.pi/180 and angle < end*math.pi/180:
                filter_points.append(point)

        if len(filter_points) == 0:
            filter_points.append([random.randint(-100, 100), random.randint(-100, 100), random.randint(-100, 100)])
        filter_points = np.asarray(filter_points)
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(filter_points)

        # draw_registration_result(source, target, trans_init)

        voxel_size = 0.5  # means 50cm for this dataset
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)

        result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
        trans_fast = result_ransac.transformation
        
        threshold = 0.2
        print("Apply point-to-point ICP")
        reg_p2l = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_fast,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        print("Transformation is:")
        print(reg_p2l.transformation)
        
        transformation_result.append(reg_p2l.transformation)
        # draw_registration_result(source_down, target_down, result_ransac.transformation)
        
    with open(os.path.join(file_path, "transformation_result.json"), "w") as f:
        json.dump(transformation_result, f, cls=NumpyArrayEncoder)