import os
import json
import math
import open3d as o3d
import numpy as np
from util import rotate_source, count_walker, count_cyclist, count_vehicle, icp


path = "data_network_SSMCO"

found_walkers_list = [0 for i in range(1,11)]
total_walkers_list = [0 for i in range(1,11)]
found_cyclists_list = [0 for i in range(1,11)]
total_cyclists_list = [0 for i in range(1,11)]
found_vehicles_list = [0 for i in range(1,11)]
total_vehicles_list = [0 for i in range(1,11)]

threshold = 0.2

for index in range(100):
    print("Processing: ", index)
    file_path = os.path.join(path, str(index))
    
    filename = os.path.join(file_path, 'ego.json')
    ego_info = json.load(open(filename))
    
    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    
    
    vehicle_pc_list = []
    vehicle_covpc_list = []
    vehicle_trans_list = []
    i = 1
    while i < 20:
        objects = {}
        filename = os.path.join(file_path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        file_pc_path = os.path.join(file_path, 'data{}.ply'.format(chr(ord('A')+i)))
        pc = o3d.io.read_point_cloud(file_pc_path)

        egoB = json.load(open(filename))

        egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
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
        trans = np.asarray([[1, 0, 0, -x], [0, 1, 0, -y], [0, 0, 1, z], [0, 0, 0, 1]])
        pc.transform(trans)
        
        result_points = np.array(pc.points)
        
        walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

        center_x = egoA['x']
        center_y = egoA['y']

        egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

        total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=100)
        
        objects['walkers'] = found_walkers

        cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

        vehicles = json.load(open(os.path.join(file_path, 'vehicles.json')))

        cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]
        total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=100)
        objects['cyclists'] = found_cyclists

        vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]
        total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=100)
        
        objects['vehicles'] = found_vehicles
        
        with open(os.path.join(file_path, 'ego{}_objects.json'.format(i+1)), "w") as f:
            json.dump(objects, f)
        
        i = i+1


    