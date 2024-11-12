import os
import json
import math
import open3d as o3d
import numpy as np
from util import rotate_source


path = "data_network_SSMCO"

def get_area(x,y):
    area=0.5*( (x[0]*(y[1]-y[2])) + (x[1]*(y[2]-y[0])) + (x[2]*(y[0]-y[1])) )
    return int(area)

def extract_area(egoA, egoB, radius = 100):
    egoAyaw = egoA['yaw'] if egoA['yaw'] > 0 else (egoA['yaw'] + 360)
    egoByaw = egoB['yaw'] if egoB['yaw'] > 0 else (egoB['yaw'] + 360)
    rotation_angle = (egoAyaw - egoByaw)*math.pi/180

    w = egoB['box_y']/2
    l = egoB['box_x']/2
    corner = np.array([[w, l, 0],[-w, l, 0],[-w, -l, 0],[w, -l, 0]])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(corner)
    corner = rotate_source(pcd, rotation_angle)
    
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
    corner.transform(trans)
    
    corner_points = np.array(corner.points)
    
    angle_list = []
    min_angle = math.pi*2
    max_angle = 0
    for i in range(4):
        point = corner_points[i]
        angle = math.atan2(point[1], point[0])
        if angle < 0:
            angle += math.pi*2
        angle_list.append(angle)
        
        if angle > max_angle:
            max_angle = angle
        if angle < min_angle:
            min_angle = angle
        
    if (max_angle - min_angle) > math.pi:
        for i in range(4):
            if angle_list[i] < math.pi:
                angle_list[i] += 2*math.pi
    
    degree = max(angle_list)-min(angle_list)
    max_index = angle_list.index(max(angle_list))
    min_index = angle_list.index(min(angle_list))
    point1 = corner_points[max_index]
    point2 = corner_points[min_index]
    tri_area = get_area([0, point1[0], point2[0]], [0, point1[1], point1[1]])
    area = degree*radius*radius/2 - tri_area
    return area

def knapSack(W, wt, val): 
    n=len(val)
    table = [[0 for x in range(W + 1)] for x in range(n + 1)]
    items = [[[] for x in range(W + 1)] for x in range(n + 1)]
 
    for i in range(n + 1): 
        for j in range(W + 1): 
            if i == 0 or j == 0: 
                table[i][j] = 0
                items[i][j] = []
            elif wt[i-1] <= j:
                value = max(val[i-1] + table[i-1][j-wt[i-1]],  table[i-1][j])
                table[i][j] = value
                if value == (val[i-1] + table[i-1][j-wt[i-1]]):
                    items[i][j] = items[i-1][j-wt[i-1]].copy()
                    items[i][j].append(i-1)
                else:
                    items[i][j] = items[i-1][j]
            else: 
                table[i][j] = table[i-1][j]
                items[i][j] = items[i-1][j]
 
    return table[n], items[n]
    
for i in range(100):
    print("Processing: ", i)
    file_path = os.path.join(path, str(i))
    cov_path = os.path.join(file_path, 'COV')
    occ_path = os.path.join(file_path, 'OCC')
    cov_rate = json.load(open(os.path.join(cov_path, 'rate.json')))
    occ_rate = json.load(open(os.path.join(occ_path, 'rate.json')))
    # knapsack problem can only solve for integer weight
    rate_list = [int(100*(cov_rate[i]+occ_rate[i])) for i in range(len(cov_rate))]
    
    filename = os.path.join(file_path, 'ego.json')
    ego_info = json.load(open(filename))
    
    vehicle_info_list = []
    i = 1
    while i < 20:
        filename = os.path.join(file_path, 'ego{}.json'.format(i+1))
        if not os.path.exists(filename):
            break
        vehicle_info = json.load(open(filename))
        vehicle_info_list.append(vehicle_info)
        i = i+1
        
    vehicle_seen_area_list = []
    pro = [0.92, 0.8877551020408163, 0.7263157894736842, 0.6129032258064516, 0.5555555555555556, 0.5522388059701493, 0.48, 0.37142857142857144, 0.2857142857142857, 0.14285714285714285, 0.16666666666666666, 0.1, 0]
    for index, vehicle_info in enumerate(vehicle_info_list):
        area = extract_area(ego_info, vehicle_info)
        vehicle_seen_area_list.append(area*pro[index])
    
    area_table, items = knapSack(500, rate_list, vehicle_seen_area_list)
    area_table = [area_table[50*i] for i in range(1,11)]
    items = [items[50*i] for i in range(1,11)]
    print("Area", area_table)
    print("Item", items)
    with open(os.path.join(file_path,"dp_icp_result.json"), "w") as outfile:
        result = {'area': area_table, 'items': items}
        json_object = json.dumps(result)
        outfile.write(json_object)
    