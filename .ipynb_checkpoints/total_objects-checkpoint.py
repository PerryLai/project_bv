import open3d as o3d
import numpy as np
import copy
import os
import json
import math
import random
import argparse
from util import icp, rotate_source, count_walker, count_cyclist, count_vehicle
from algorithms import golden_sample, resolution, roi, accumulated_roi

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

root_path = "data"
n_v = 0
n_c = 0
n_w = 0


for i in range(100):
    path = os.path.join(root_path, str(i))
    cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

    vehicles = json.load(open(os.path.join(path, 'vehicles.json')))


    egoA = json.load(open(os.path.join(path, 'ego.json')))
    egoB = json.load(open(os.path.join(path, 'ego2.json')))

    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
    egoByaw = egoB['yaw'] if egoB['yaw'] > 0 else (egoB['yaw'] + 360)
    rotation_angle = (egoByaw - egoAyaw)*math.pi/180




    walkers = json.load(open(os.path.join(path, 'walkers.json')))

    center_x = egoA['x']
    center_y = egoA['y']

    found_walkers = []
    total_walkers = []

    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

    yaw = egoAyaw*math.pi/180 - math.pi/2

    rotation_matrix = np.array([[math.cos(yaw), math.sin(yaw)],
                                      [math.sin(yaw), -math.cos(yaw)]])

    total_walkers, found_walkers = count_walker(np.array([[0,0,0]]), walkers, rotation_matrix, (center_x, center_y), range=80)

    print("Total walkers: ", len(total_walkers))
    n_w += len(total_walkers)
    print("Found walkers: ", len(found_walkers))

    cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

    vehicles = json.load(open(os.path.join(path, 'vehicles.json')))

    cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]
    total_cyclists, found_cyclists = count_cyclist(np.array([[0,0,0]]), cyclists, rotation_matrix, (center_x, center_y), range=80)

    print("Total cyclists: ", len(total_cyclists))
    n_c += len(total_cyclists)
    print("Found cyclists: ", len(found_cyclists))

    vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]
    total_vehicles, found_vehicles = count_vehicle(np.array([[0,0,0]]), vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=80)

    print("Total vehicles: ", len(total_vehicles))
    n_v += len(total_vehicles)
    print("Found vehicles: ", len(found_vehicles))
    
print(n_v)
print(n_c)
print(n_w)