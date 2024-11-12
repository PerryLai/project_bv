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
    cov_path = os.path.join(file_path, 'COV')
    occ_path = os.path.join(file_path, 'OCC')
    
    filename = os.path.join(file_path, 'ego.json')
    ego_info = json.load(open(filename))
    
    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    
    
    vehicle_pc_list = []
    vehicle_covpc_list = []
    vehicle_trans_list = []
    i = 1
    while i < 20:
        filename = os.path.join(file_path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        cov_pc_path = os.path.join(cov_path, 'data{}.ply'.format(chr(ord('A')+i)))
        cov_pc = o3d.io.read_point_cloud(cov_pc_path)
        cov_pc_points = cov_pc.points
        vehicle_covpc_list.append(cov_pc)
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
        vehicle_trans_list.append(trans)
        vehicle_pc_list.append(pcd)
        i = i+1
    
    cov_rate = json.load(open(os.path.join(cov_path, 'rate.json')))
    occ_rate = json.load(open(os.path.join(occ_path, 'rate.json')))
    rate_list = [(cov_rate[i]+occ_rate[i]) for i in range(len(cov_rate))]

    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target.points)
    total_rate = 0
    j = 0
    for i in range(10):
        while j < len(rate_list) and (total_rate + rate_list[j]) < 0.5*(i+1):
            total_rate = total_rate + rate_list[j]
            source_pcd = o3d.geometry.PointCloud()
            source_pcd.points = o3d.utility.Vector3dVector(vehicle_covpc_list[j].points)
            trans_final = icp(source_pcd, target_pcd)
            t_x = np.array(trans_final)[0, 3]
            t_y = np.array(trans_final)[1, 3]
            x = vehicle_trans_list[j][0, 3]
            y = vehicle_trans_list[j][1, 3]
            error = math.sqrt((t_x-x)**2+(t_y-y)**2)
            if error < threshold:
                print(j, "correct", error)
                source_pcd = o3d.geometry.PointCloud()
                source_pcd.points = o3d.utility.Vector3dVector(vehicle_pc_list[j].points)
                source_pcd.transform(trans_final)
                target_pcd.points = o3d.utility.Vector3dVector(np.concatenate((target_pcd.points, source_pcd.points), axis=0))
            else:
                print(j, "error", error)
            j = j+1
            
        result_points = np.array(target_pcd.points)
        
        walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

        center_x = egoA['x']
        center_y = egoA['y']

        egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

        total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=100)
        
        found_walkers_list[i] += len(found_walkers)
        total_walkers_list[i] += len(total_walkers)

        cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

        vehicles = json.load(open(os.path.join(file_path, 'vehicles.json')))

        cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]
        total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=100)

        found_cyclists_list[i] += len(found_cyclists)
        total_cyclists_list[i] += len(total_cyclists)

        vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]
        total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=100)
        
        found_vehicles_list[i] += len(found_vehicles)
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


    