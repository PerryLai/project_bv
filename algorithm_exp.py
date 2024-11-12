import open3d as o3d
import os
import json
import random
import math
import numpy as np
from util import mergeply, draw_registration_result, prepare_dataset, execute_global_registration, rotate_source, icp, count_walker, count_cyclist, count_vehicle
from algorithms import golden_sample, resolution, roi, accumulated_roi, covisible_roi, filter_occlusion_points
# 自定義模組 util 和 algorithms 負責處理點雲合併、ROI 過濾、旋轉、ICP 對準等功能。

# ALG = "Base"                                      # 決定使用哪種預處理演算法 (ROI 或 Base)。
ALG = "ROI"
path = "data_network_SSMCO"                         # path = "data" # path 是資料目錄，其中包含 .ply 和 .json 檔案。
threshold = 0.2                                     # threshold 是 ICP 配準的誤差容忍度。
correct = 0
total_error = 0
error_list = []
total_occlusion_reduced_rate = 0
total_covisible_reduced_rate = 0
occlusion_size_list = []
reduced_size_list = []
total_estimated_position_error = 0
estimated_position_error_list = []
points_density = 1000                               # ROI 過濾後的點雲密度
reduced_rate_target = 0.15                          # ROI 過濾後的資料減少比例

total_walkers_count_80 = 0
total_cyclists_count_80 = 0
total_vehicles_count_80 = 0
found_walkers_num_80 = []
found_cyclists_num_80 = []
found_vehicles_num_80 = []

total_walkers_count_50 = 0
total_cyclists_count_50 = 0
total_vehicles_count_50 = 0
found_walkers_num_50 = []
found_cyclists_num_50 = []
found_vehicles_num_50 = []

total_walkers_count_20 = 0
total_cyclists_count_20 = 0
total_vehicles_count_20 = 0
found_walkers_num_20 = []
found_cyclists_num_20 = []
found_vehicles_num_20 = []


for i in range(100): # 迴圈遍歷 100 組資料，從檔案中讀取 點雲資料 (.ply) 和 車輛位置資訊 (.json)。
    print("=============================================================")
    print("Processing: ", i)
    file_path = os.path.join(path, str(i))

    target = o3d.io.read_point_cloud(os.path.join(file_path, 'dataA.ply'))
    egoA = json.load(open(os.path.join(file_path, 'ego.json')))
    result_points = target.points
    correct_check = 0
    # egoB_list = ['B', 'C', 'D', 'E']
    egoB_list = ['B']
    
    for index, name in enumerate(egoB_list):
        source = o3d.io.read_point_cloud(os.path.join(file_path, 'data{}.ply'.format(name)))
        egoB = json.load(open(os.path.join(file_path, 'ego{}.json'.format(index+2))))

        # 變數 egoA 和 egoB 分別代表目標和來源車輛的座標與方向。
        # 讀取車輛 B 的點雲資料並旋轉，使其對準車輛 A 的方向。
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

        if ALG == "ROI": # 估計目標物體的位置，使用 ROI 和 遮蔽檢測 過濾掉不必要的點雲以減少計算量。
            estimated_target_position = [x + random.gauss(0, 1), y + random.gauss(0, 1), -1+random.gauss(0, 0.2)]
            print('target_position:          ', [x, y, -1])
            print('estimated_target_position:', estimated_target_position)

            # filter_occlusion_points 濾掉被遮擋的點，covisible_roi 根據估計位置提取共視區域的點雲
            occlusion_points = filter_occlusion_points(source, estimated_target_position)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(occlusion_points)
            occlusion_rate = len(pcd.points)/original_size
            occlusion_size_list.append(occlusion_rate)
            print("occlusion rate:", occlusion_rate)
            trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            draw_registration_result(pcd, target, trans_init)

            error = math.sqrt((estimated_target_position[0]-x)**2 + (estimated_target_position[1]-y)**2 + (estimated_target_position[2]+1)**2)

            estimated_position_error_list.append(error)

            total_estimated_position_error += error
            
            source = covisible_roi(source, estimated_target_position, density=points_density, reduced_rate_target=reduced_rate_target)
            


        reduced_size = len(source.points)
        rate = reduced_size/original_size
        reduced_size_list.append(rate)
        print("reduced rate:", rate)

        ## End Preprocessing algorithm

        ## Visual
        # trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # draw_registration_result(source, target, trans_init)

        # 使用 ICP 對齊過濾後的點雲與目標點雲，並計算最終的誤差。
        trans_final = icp(source, target)
        # draw_registration_result(pcd, target, trans_final)

        t_x = np.array(trans_final)[0, 3]
        t_y = np.array(trans_final)[1, 3]
        print("Translation final: ", trans_final)


        error = math.sqrt((t_x+x)**2+(t_y+y)**2)
        print("error: ", error)

        error_list.append(error)
        if error < threshold:
            total_error += error
            total_covisible_reduced_rate += rate
            total_occlusion_reduced_rate += occlusion_rate
            correct_check += 1

            ## Objects counting
            if ALG == "ROI":
                points = np.concatenate((occlusion_points, source.points), axis=0)
                pcd.points = o3d.utility.Vector3dVector(points)
                pcd.transform(trans_final)
                result_points = np.concatenate((result_points, pcd.points), axis=0)
            else:
                source.transform(trans_final)
                result_points = np.concatenate((result_points, source.points), axis=0)
        
    if correct_check == len(egoB_list):
        correct += 1
        print("Correct: ", correct)
    else:
        print("Correct: ", correct)
        continue
    
    
    # pcd = o3d.geometry.PointCloud()
    # trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    # pcd.points = o3d.utility.Vector3dVector(result_points)
    # draw_registration_result(pcd, pcd, trans_init)
    
    # 透過 count_walker, count_cyclist 和 count_vehicle 來計算偵測到的物件數量。
    walkers = json.load(open(os.path.join(file_path, 'walkers.json')))

    center_x = egoA['x']
    center_y = egoA['y']

    found_walkers = []
    total_walkers = []

    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)

    total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=80)

    found_walkers_num_80.append(len(found_walkers))
    total_walkers_count_80 += len(total_walkers)
    
    total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=50)

    found_walkers_num_50.append(len(found_walkers))
    total_walkers_count_50 += len(total_walkers)
    
    total_walkers, found_walkers = count_walker(result_points, walkers, rotation_matrix, (center_x, center_y), range=20)

    found_walkers_num_20.append(len(found_walkers))
    total_walkers_count_20 += len(total_walkers)

    cyclies_id = ['vehicle.kawasaki.ninja', 'vehicle.diamondback.century', 'vehicle.vespa.zx125', 'vehicle.gazelle.omafiets', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.harley-davidson.low_rider']

    vehicles = json.load(open(os.path.join(file_path, 'vehicles.json')))

    cyclists = [vehicle for vehicle in vehicles if vehicle['id'] in cyclies_id]
    total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=80)

    found_cyclists_num_80.append(len(found_cyclists))
    total_cyclists_count_80 += len(total_cyclists)
    
    total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=50)

    found_cyclists_num_50.append(len(found_cyclists))
    total_cyclists_count_50 += len(total_cyclists)
    
    total_cyclists, found_cyclists = count_cyclist(result_points, cyclists, rotation_matrix, (center_x, center_y), range=20)

    found_cyclists_num_20.append(len(found_cyclists))
    total_cyclists_count_20 += len(total_cyclists)
    
    vehicles = [vehicle for vehicle in vehicles if vehicle['id'] not in cyclies_id]
    total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=80)

    found_vehicles_num_80.append(len(found_vehicles))
    total_vehicles_count_80 += len(total_vehicles)

    total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=50)

    found_vehicles_num_50.append(len(found_vehicles))
    total_vehicles_count_50 += len(total_vehicles)
    
    total_vehicles, found_vehicles = count_vehicle(result_points, vehicles, rotation_matrix, (center_x, center_y), egoAyaw, range=20)

    found_vehicles_num_20.append(len(found_vehicles))
    total_vehicles_count_20 += len(total_vehicles)
    
# 最後輸出整體的計算結果，包括 ICP 錯誤、減少率、物件檢測準確率。
print("With points density:", points_density) # 
print("With reduced rate target:", reduced_rate_target)
print("Final correct:", correct)
print("=============================================================")
print("Avarage icp error:", total_error/correct)
print("icp error std:", np.std(np.array(error_list)))
print("Average covisible reduced rate:", total_covisible_reduced_rate/correct)
print("Average occlusion reduced rate:", total_occlusion_reduced_rate/correct)
print("Average reduced rate:", total_covisible_reduced_rate/correct + total_occlusion_reduced_rate/correct)
print("Average estimated position error:", total_estimated_position_error/100)
print("=============================================================")
print("[Range80] Found walkers rate: ", sum(found_walkers_num_80)/total_walkers_count_80)
print("[Range80] Found cyclists rate: ", sum(found_cyclists_num_80)/total_cyclists_count_80)
print("[Range80] Found vehicles rate: ", sum(found_vehicles_num_80)/total_vehicles_count_80)
print("[Range50] Found walkers rate: ", sum(found_walkers_num_50)/total_walkers_count_50)
print("[Range50] Found cyclists rate: ", sum(found_cyclists_num_50)/total_cyclists_count_50)
print("[Range50] Found vehicles rate: ", sum(found_vehicles_num_50)/total_vehicles_count_50)
print("[Range20] Found walkers rate: ", sum(found_walkers_num_20)/total_walkers_count_20)
print("[Range20] Found cyclists rate: ", sum(found_cyclists_num_20)/total_cyclists_count_20)
print("[Range20] Found vehicles rate: ", sum(found_vehicles_num_20)/total_vehicles_count_20)
print("=============================================================")
print("=============================================================")
print("Final average reduced list:", reduced_size_list)
print("Final occlusion size list:", occlusion_size_list)
print("Final error list:", estimated_position_error_list)
print("=============================================================")
print("[Range80] Found walkers: ", found_walkers_num_80)
print("[Range80] Found cyclists: ", found_cyclists_num_80)
print("[Range80] Found vehicles: ", found_vehicles_num_80)
print("[Range50] Found walkers: ", found_walkers_num_50)
print("[Range50] Found cyclists: ", found_cyclists_num_50)
print("[Range50] Found vehicles: ", found_vehicles_num_50)
print("[Range20] Found walkers: ", found_walkers_num_20)
print("[Range20] Found cyclists: ", found_cyclists_num_20)
print("[Range20] Found vehicles: ", found_vehicles_num_20)