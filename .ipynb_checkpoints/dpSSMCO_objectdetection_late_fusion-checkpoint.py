import os
import json
import math
import open3d as o3d
import numpy as np
import copy
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
    cov_path = os.path.join(file_path, 'COV')
    occ_path = os.path.join(file_path, 'OCC')
    
    filename = os.path.join(file_path, 'ego.json')
    ego_info = json.load(open(filename))
    
    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    
    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
    yaw = egoAyaw*math.pi/180 - math.pi/2

    rotation_matrix = np.array([[math.cos(yaw), math.sin(yaw)],
                              [math.sin(yaw), -math.cos(yaw)]])
    
    result_points = np.array(target.points)
        
    walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

    center_x = egoA['x']
    center_y = egoA['y']
    
    objects = {}

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
    
    vehicle_pc_list = []
    vehicle_covpc_list = []
    vehicle_trans_list = []
    
    filename = os.path.join(file_path, 'late_fusion_dp_icp_result.json')
    dp_result = json.load(open(filename))

    for i, items in enumerate(dp_result['items']):
        print(i, items)
        init_objects = copy.deepcopy(objects)
        for item in items:
            filename = os.path.join(file_path, 'ego{}_objects.json'.format(item+2))
            f = open(filename)
            objects_data = json.load(f)
            for a in objects_data['walkers']:
                duplicate = False
                for b in init_objects['walkers']:
                    if a['id'] == b['id']:
                        duplicate = True
                if not duplicate:
                    init_objects['walkers'].append(a)
                    
            for a in objects_data['cyclists']:
                duplicate = False
                for b in init_objects['cyclists']:
                    if a['id'] == b['id']:
                        duplicate = True
                if not duplicate:
                    init_objects['cyclists'].append(a)
                    
            for a in objects_data['vehicles']:
                duplicate = False
                for b in init_objects['vehicles']:
                    if a['id'] == b['id']:
                        duplicate = True
                if not duplicate:
                    init_objects['vehicles'].append(a)
            
        found_walkers_list[i] += len(init_objects['walkers'])
        total_walkers_list[i] += len(total_walkers)
        found_cyclists_list[i] += len(init_objects['cyclists'])
        total_cyclists_list[i] += len(total_cyclists)
        found_vehicles_list[i] += len(init_objects['vehicles'])
        total_vehicles_list[i] += len(total_vehicles)
        
print("Found walkers:", found_walkers_list)
print("Total walkers:", total_walkers_list)
print("Found walkers rate:", [found_walkers_list[i]/total_walkers_list[i] for i in range(10)])

print("Found cyclists:", found_cyclists_list)
print("Total cyclists:", total_cyclists_list)
print("Found cyclists rate:", [found_cyclists_list[i]/total_cyclists_list[i] for i in range(10)])

print("Found vehicles:", found_vehicles_list)
print("Total vehicles:", total_vehicles_list)
print("Found vehicles rate:", [found_vehicles_list[i]/total_vehicles_list[i] for i in range(10)])

print("Found objects rate:", [(found_walkers_list[i]+found_cyclists_list[i]+found_vehicles_list[i])/(total_walkers_list[i]+total_cyclists_list[i]+total_vehicles_list[i]) for i in range(10)])


    