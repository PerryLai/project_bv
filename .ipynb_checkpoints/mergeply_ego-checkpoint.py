import open3d as o3d
import os
import json
import argparse
from util import mergeply
import numpy as np

model_path = 'car_models'

for i in range(1,100):
    path = os.path.join('data_network_SSMCO', str(i))
    
    f = open(os.path.join(path, 'ego.json'))
    data = json.load(f)
    
    mesh = o3d.io.read_point_cloud(os.path.join(path, 'dataA.ply'))
    points = np.asarray(mesh.points)
    egomesh = o3d.io.read_point_cloud(os.path.join(model_path, '{}.ply'.format(data['id'])))
    egopoints = np.asarray(egomesh.points)
    points = np.concatenate((points, egopoints), axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(os.path.join(path, "dataA.ply"), pcd)
    
    i = 1
    while i < 20:
        filename = os.path.join(path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        f = open(filename)
        data = json.load(f)
        
        mesh = o3d.io.read_point_cloud(os.path.join(path, 'data{}.ply'.format(chr(ord('A')+i))))
        points = np.asarray(mesh.points)
        egomesh = o3d.io.read_point_cloud(os.path.join(model_path, '{}.ply'.format(data['id'])))
        egopoints = np.asarray(egomesh.points)
        points = np.concatenate((points, egopoints), axis=0)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(os.path.join(path, "data{}.ply").format(chr(ord('A')+i)), pcd)
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
