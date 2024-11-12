import os
import json
import math
import open3d as o3d
import numpy as np
from util import rotate_source, count_walker, count_cyclist, count_vehicle


path = "data_network_SSMCO"

found_walkers_num = 0
total_walkers_num = 0
found_cyclists_num = 0
total_cyclists_num = 0
found_vehicles_num = 0
found_same_direction_vehicles_num = 0
found_oppo_direction_vehicles_num = 0
total_vehicles_num = 0

for index in range(100):
    print("Processing: ", index)
    file_path = os.path.join(path, str(index))
    filename = os.path.join(file_path, 'ego.json')
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
    yaw = egoAyaw*math.pi/180 - math.pi/2

    rotation_matrix = np.array([[math.cos(yaw), math.sin(yaw)],
                              [math.sin(yaw), -math.cos(yaw)]])
    
    filename = os.path.join(file_path, 'dp_result.json')
    dp_result = json.load(open(filename))
    result_path = os.path.join(file_path, 'result')
    length = len(dp_result['items'])
        
    pcd = o3d.io.read_point_cloud(os.path.join(result_path, "result{}.ply".format(length-1)))
    result_points = np.array(pcd.points)
        
    walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

    center_x = egoA['x']
    center_y = egoA['y']

    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

    total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=100)

    found_walkers_num += len(found_walkers)
    total_walkers_num += len(total_walkers)

    cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

    vehicles = json.load(open(os.path.join(file_path, 'vehicles.json')))

    cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]
    total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=100)

    found_cyclists_num += len(found_cyclists)
    total_cyclists_num += len(total_cyclists)

    vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]
    total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=100)

    for vehicle in found_vehicles:
        if abs(vehicle['yaw'] - egoA ['yaw']) < 5:
            found_same_direction_vehicles_num += 1
        if abs(abs(vehicle['yaw'] - egoA ['yaw'])-180) < 5:
            found_oppo_direction_vehicles_num += 1

    found_vehicles_num += len(found_vehicles)
    total_vehicles_num += len(total_vehicles)

    
print(found_walkers_num)
print(total_walkers_num)
print(found_cyclists_num)
print(total_cyclists_num)
print(found_vehicles_num)
print(found_same_direction_vehicles_num)
print(found_oppo_direction_vehicles_num)
print(total_vehicles_num)
# print("Found walkers:", found_walkers_list)
# print("Total walkers:", total_walkers_list)
# print("Found walkers rate:", [found_walkers_list[i]/total_walkers_list[i] for i in range(10)])

# print("Found cyclists:", found_cyclists_list)
# print("Total cyclists:", total_cyclists_list)
# print("Found cyclists rate:", [found_cyclists_list[i]/total_cyclists_list[i] for i in range(10)])

# print("Found vehicles:", found_vehicles_list)
# print("Total vehicles:", total_vehicles_list)
# print("Found vehicles rate:", [found_vehicles_list[i]/total_vehicles_list[i] for i in range(10)])

# print("Found objects rate:", [(found_walkers_list[i]+found_cyclists_list[i]+found_vehicles_list[i])/(total_walkers_list[i]+total_cyclists_list[i]+total_vehicles_list[i]) for i in range(10)])


    