import open3d as o3d
import numpy as np
import copy
import json
import os
import argparse
from util import icp

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

# egoA = json.load(open(os.path.join(path, 'ego.json')))
# egoB = json.load(open(os.path.join(path, 'ego2.json')))

# x = egoB['x'] - egoA['x']
# y = egoB['y'] - egoA['y']

# target = o3d.io.read_point_cloud(os.path.join(path, 'dataA.ply') if Afile == None else Afile)
# source = o3d.io.read_point_cloud(os.path.join(path, 'dataB.ply') if Bfile == None else Bfile)
egoB = o3d.io.read_point_cloud('egoB.ply')

# trans_final = icp(source, target)

# target_points = np.array(target.points)
egoB_points = np.array(egoB.points)

for point in egoB_points:
    point[1] *= -1

# new_points = np.array(new_points)



pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(egoB_points)
o3d.io.write_point_cloud('newegoB.ply', pcd)