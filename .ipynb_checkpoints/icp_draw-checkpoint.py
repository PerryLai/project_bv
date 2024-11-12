import open3d as o3d
import numpy as np
import copy
import os
import json
import math
import random
import argparse
from util import icp, rotate_source
from algorithms import golden_sample, resolution, roi, accumulated_roi, filter_non_sign_points

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

parser = argparse.ArgumentParser()
parser.add_argument("--path",
                    help="path")
parser.add_argument("--Afile",
                    help="file path")
parser.add_argument("--Bfile",
                    help="file path")
args = parser.parse_args()

path = args.path
Afile = args.Afile
Bfile = args.Bfile

target = o3d.io.read_point_cloud(os.path.join(path, 'dataA.ply') if Afile == None else Afile)
source = o3d.io.read_point_cloud(os.path.join(path, 'dataB.ply') if Bfile == None else Bfile)

egoA = json.load(open(os.path.join(path, 'ego.json')))
egoB = json.load(open(os.path.join(path, 'ego2.json')))

egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
egoByaw = egoB['yaw'] if egoB['yaw'] > 0 else (egoB['yaw'] + 360)
rotation_angle = (egoByaw - egoAyaw)*math.pi/180

source = rotate_source(source, rotation_angle)

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
print(rotated_diff)

## Preprocessing Algorithm
original_size = len(source.points)
# source = golden_sample(source, target, rotated_diff)
# source = resolution(source, 0.05)

estimated_target_position = [x + random.gauss(0, 0), y + random.gauss(0, 0), -1+random.gauss(0, 0)]
error = math.sqrt((estimated_target_position[0]+x)**2 + (estimated_target_position[1]+y)**2 + (estimated_target_position[2]+1)**2)

print(estimated_target_position)
points = filter_non_sign_points(source, estimated_target_position)
print(points.shape)

# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)

source = accumulated_roi(source, estimated_target_position, density=25, reduced_rate_target=0.25)

# pcd = o3d.geometry.PointCloud()

trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# draw_registration_result(source, target, trans_init)
trans_final = icp(source, target)

points = np.concatenate((points, np.array(source.points)), axis=0)
source.points = o3d.utility.Vector3dVector(points)

draw_registration_result(source, target, trans_final)
