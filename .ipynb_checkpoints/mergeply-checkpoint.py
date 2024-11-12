import open3d as o3d
import os
import json
import argparse
from util import mergeply


# parser = argparse.ArgumentParser()
# parser.add_argument("--ego",
#                     help="A or B")
# parser.add_argument("--path",
#                     help="path")
# args = parser.parse_args()

# path = args.path
# ego = args.ego

for i in range(100):
    path = os.path.join('data_network_SSMCO_highway', str(i))
    
    f = open(os.path.join(path, 'ego.json'))
    data = json.load(f)
    mergeply(path, 'A', -data['box_z']-0.56)
    
    i = 1
    while i < 20:
        filename = os.path.join(path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        f = open(filename)
        data = json.load(f)
        mergeply(path, chr(ord('A')+i), -data['box_z']-0.56)
        i=i+1
    
#     f = open(os.path.join(path, 'ego3.json'))
#     data = json.load(f)
#     mergeply(path, 'C', -data['box_z']-0.56)
    
#     f = open(os.path.join(path, 'ego4.json'))
#     data = json.load(f)
#     mergeply(path, 'D', -data['box_z']-0.56)
    
#     f = open(os.path.join(path, 'ego5.json'))
#     data = json.load(f)
#     mergeply(path, 'E', -data['box_z']-0.56)
    
#     f = open(os.path.join(path, 'ego6.json'))
#     data = json.load(f)
#     mergeply(path, 'F', -data['box_z']-0.56)

    
# for i in range(100):
#     path = os.path.join('data_network', str(i))
#     source = o3d.io.read_point_cloud(os.path.join(path, 'data{}.ply'.format('A')))
#     source.voxel_down_sample(0.5)
#     source = o3d.io.read_point_cloud(os.path.join(path, 'data{}.ply'.format('B')))
#     source.voxel_down_sample(0.5)
#     source = o3d.io.read_point_cloud(os.path.join(path, 'data{}.ply'.format('C')))
#     source.voxel_down_sample(0.5)
#     source = o3d.io.read_point_cloud(os.path.join(path, 'data{}.ply'.format('D')))
#     source.voxel_down_sample(0.5)
#     source = o3d.io.read_point_cloud(os.path.join(path, 'data{}.ply'.format('E')))
#     source.voxel_down_sample(0.5)
