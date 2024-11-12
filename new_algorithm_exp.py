import open3d as o3d
import os
import json
import random
import math
import numpy as np
from util import mergeply, draw_registration_result, prepare_dataset, execute_global_registration, rotate_source, icp, count_walker, count_cyclist, count_vehicle
from algorithms import golden_sample, resolution, roi, accumulated_roi, covisible_roi, filter_occlusion_points, extract_covisible_roi, array_resolution

# ALG = "Base"
ALG = "ROI"
path = "data"
threshold = 0.2
distance = 2
error_list = [[] for i in range(10)]
all_correct_list = [0 for i in range(10)]
partial_correct_list = [0 for i in range(10)]
all_reduced_size_list = [[] for i in range(10)]
partial_reduced_size_list = [[] for i in range(10)]
all_walkers_list = [[[0,0] for i in range(10)] for j in range(4)]
all_cyclists_list = [[[0,0] for i in range(10)] for j in range(4)]
all_vehicles_list = [[[0,0] for i in range(10)] for j in range(4)]
partial_walkers_list = [[[0,0] for i in range(10)] for j in range(4)]
partial_cyclists_list = [[[0,0] for i in range(10)] for j in range(4)]
partial_vehicles_list = [[[0,0] for i in range(10)] for j in range(4)]


for i in range(37, 38):
    print("Processing: ", i)
    file_path = os.path.join(path, str(i))

    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))

    source = o3d.io.read_point_cloud(os.path.join(file_path, 'dataB.ply'))
    egoB = json.load(open(os.path.join(file_path, 'ego2.json')))

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

    ## Preprocessing Algorithm
    original_size = len(source.points)
    occlusion_rate = 0

    estimated_target_position = [x + random.gauss(0, 1), y + random.gauss(0, 1), -1+random.gauss(0, 0.2)]

    partial_occlusion_points = filter_occlusion_points(source, estimated_target_position)
    # trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # draw_registration_result(pcd, target, trans_init)

    error = math.sqrt((estimated_target_position[0]-x)**2 + (estimated_target_position[1]-y)**2 + (estimated_target_position[2]+1)**2)

    covisible_points, all_occlusion_points = extract_covisible_roi(source, estimated_target_position, distance=distance)
    
    all_occlusion_points = o3d.utility.Vector3dVector(all_occlusion_points)
    all_occlusion_rate = len(all_occlusion_points)/original_size
    partial_occlusion_points = o3d.utility.Vector3dVector(partial_occlusion_points)
    partial_occlusion_rate = len(partial_occlusion_points)/original_size

    for k in range(10):
        resolution = (10-k)/10
        print("Resolution: ", resolution)
        resolution_covisible_points = array_resolution(covisible_points, resolution)
        source.points = o3d.utility.Vector3dVector(resolution_covisible_points)
        covisible_rate = len(source.points)/original_size
       
        all_reduced_size_list[k].append((covisible_rate, all_occlusion_rate))
        partial_reduced_size_list[k].append((covisible_rate, partial_occlusion_rate))

        ## End Preprocessing algorithm

        ## Visual
        # trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # draw_registration_result(source, target, trans_init)

        trans_final = icp(source, target)
        # draw_registration_result(pcd, target, trans_final)

        t_x = np.array(trans_final)[0, 3]
        t_y = np.array(trans_final)[1, 3]

        error = math.sqrt((t_x+x)**2+(t_y+y)**2)
        print(error)

        if error < threshold:
            error_list[k].append(error)
            all_correct_list[k] += 1
            partial_correct_list[k] += 1

            ## Objects counting
            all_points = np.concatenate((all_occlusion_points, source.points), axis=0)
            all_pcd = o3d.geometry.PointCloud()
            all_pcd.points = o3d.utility.Vector3dVector(all_points)
            all_pcd.transform(trans_final)
            all_result_points = np.concatenate((target.points, all_pcd.points), axis=0)

            partial_points = np.concatenate((partial_occlusion_points, source.points), axis=0)
            partial_pcd = o3d.geometry.PointCloud()
            partial_pcd.points = o3d.utility.Vector3dVector(partial_points)
            partial_pcd.transform(trans_final)
            partial_result_points = np.concatenate((target.points, partial_pcd.points), axis=0)


            ## count objects
            walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

            center_x = egoA['x']
            center_y = egoA['y']

            found_walkers = []
            total_walkers = []

            egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

            for index, r in enumerate([20,30,40,50]):
                total_walkers, found_walkers = count_walker(all_result_points, walkers, rotation_matrix, (center_x, center_y), range=r)

                all_walkers_list[index][k][0] += len(found_walkers)
                all_walkers_list[index][k][1] += len(total_walkers)

                total_walkers, found_walkers = count_walker(partial_result_points, walkers, rotation_matrix, (center_x, center_y), range=r)

                partial_walkers_list[index][k][0] += len(found_walkers)
                partial_walkers_list[index][k][1] += len(total_walkers)

            cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

            vehicles = json.load(open(os.path.join(file_path, 'vehicles.json')))

            cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]

            for index, r in enumerate([20,30,40,50]):
                total_cyclists, found_cyclists = count_cyclist(all_result_points, cyclists, rotation_matrix, (center_x, center_y), range=r)

                all_cyclists_list[index][k][0] += len(found_cyclists)
                all_cyclists_list[index][k][1] += len(total_cyclists)

                total_cyclists, found_cyclists = count_cyclist(partial_result_points, cyclists, rotation_matrix, (center_x, center_y), range=r)

                partial_cyclists_list[index][k][0] += len(found_cyclists)
                partial_cyclists_list[index][k][1] += len(total_cyclists)


            vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]

            for index, r in enumerate([20,30,40,50]):
                total_vehicles, found_vehicles = count_vehicle(all_result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=r)
                all_vehicles_list[index][k][0] += len(found_vehicles)
                all_vehicles_list[index][k][1] += len(total_vehicles)

                total_vehicles, found_vehicles = count_vehicle(partial_result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=r)
                partial_vehicles_list[index][k][0] += len(found_vehicles)
                partial_vehicles_list[index][k][1] += len(total_vehicles)

    print("Correct:", partial_correct_list)
    all_reduced_size_average_list = []
    for q in range(10):
        total = 0
        for element in all_reduced_size_list[q]:
            total += (element[0]+element[1])
        all_reduced_size_average_list.append(total/len(all_reduced_size_list[q]))

    print("All reduced size list:", all_reduced_size_average_list)

    partial_reduced_size_average_list = []
    for q in range(10):
        total = 0
        for element in partial_reduced_size_list[q]:
            total += (element[0]+element[1])
        partial_reduced_size_average_list.append(total/len(partial_reduced_size_list[q]))

    print("Partial reduced size list:", partial_reduced_size_average_list)

    info_object = {
        'processing': i,
        'error_list': error_list,
        'partial_correct_list': partial_correct_list,
        'all_correct_list': all_correct_list,
        'partial_correct_list': partial_correct_list,
        'all_reduced_size_list': all_reduced_size_list,
        'partial_reduced_size_list': partial_reduced_size_list,
        'all_walkers_list': all_walkers_list,
        'all_cyclists_list': all_cyclists_list,
        'all_vehicles_list': all_vehicles_list,
        'partial_walkers_list': partial_walkers_list,
        'partial_cyclists_list': partial_cyclists_list,
        'partial_vehicles_list': partial_vehicles_list,
    }
    jsonString = json.dumps(info_object)
    # jsonFile = open(os.path.join("checkpoints", "checkpoint_{}.json".format(distance)), "w")
    jsonFile.write(jsonString)
    jsonFile.close()

    
    
    