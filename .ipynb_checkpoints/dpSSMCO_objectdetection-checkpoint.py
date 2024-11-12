import os
import json
import math
import open3d as o3d
import numpy as np
from util import rotate_source, count_walker, count_cyclist, count_vehicle


path = "data_network_SSMCO"

found_walkers_list = [0 for i in range(1,11)]
total_walkers_list = [0 for i in range(1,11)]
found_cyclists_list = [0 for i in range(1,11)]
total_cyclists_list = [0 for i in range(1,11)]
found_vehicles_list = [0 for i in range(1,11)]
total_vehicles_list = [0 for i in range(1,11)]

for index in range(100):
    print("Processing: ", index)
    file_path = os.path.join(path, str(index))
    cov_path = os.path.join(file_path, 'COV')
    occ_path = os.path.join(file_path, 'OCC')
    
    filename = os.path.join(file_path, 'ego.json')
    ego_info = json.load(open(filename))
    
    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    
    
    vehicle_pc_list = []
    vehicle_trans_list = []
    i = 1
    while i < 20:
        filename = os.path.join(file_path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        cov_pc_path = os.path.join(cov_path, 'data{}.ply'.format(chr(ord('A')+i)))
        cov_pc = o3d.io.read_point_cloud(cov_pc_path)
        cov_pc_points = cov_pc.points
        occ_pc_path = os.path.join(occ_path, 'data{}.ply'.format(chr(ord('A')+i)))
        occ_pc = o3d.io.read_point_cloud(occ_pc_path)
        occ_pc_points = occ_pc.points
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.concatenate((cov_pc_points, occ_pc_points), axis=0))

        
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
        pcd.transform(trans)
        vehicle_pc_list.append(pcd)
        i = i+1
    
    filename = os.path.join(file_path, 'dp_result.json')
    dp_result = json.load(open(filename))
    result_path = os.path.join(file_path, 'result')
    os.mkdir(result_path)

    for i, items in enumerate(dp_result['items']):
        result_points = np.array(target.points)
        for item in items:
            result_points = np.concatenate((result_points, vehicle_pc_list[item].points), axis=0)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(result_points)
        o3d.io.write_point_cloud(os.path.join(result_path, "result{}.ply".format(i)), pcd)
        
        
#         walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

#         center_x = egoA['x']
#         center_y = egoA['y']

#         egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

#         total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=100)
        
#         found_walkers_list[i] += len(found_walkers)
#         total_walkers_list[i] += len(total_walkers)

#         cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

#         vehicles = json.load(open(os.path.join(file_path, 'vehicles.json')))

#         cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]
#         total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=100)

#         found_cyclists_list[i] += len(found_cyclists)
#         total_cyclists_list[i] += len(total_cyclists)

#         vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]
#         total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=100)
        
#         found_vehicles_list[i] += len(found_vehicles)
#         total_vehicles_list[i] += len(total_vehicles)
        
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


    