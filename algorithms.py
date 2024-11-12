import numpy as np
import math
import random
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
from util import draw_registration_result
from sklearn.neighbors import NearestNeighbors
from scipy.linalg import svd

# Golden Sample 是一種點雲資料的篩選技術，用於從來源點雲 (source) 中選取與目標點雲 (target) 相對較接近的點。
def golden_sample(source, target, distance):
    translate = np.array([-distance[0], -distance[1], 1]) # 將 source 點雲平移，以便與 target 點雲對齊。
    source.translate(translate)
    neigh = NearestNeighbors(radius=0.1) # 建立鄰近搜尋器來搜尋 target 點雲中的鄰近點。半徑設為 0.1
    target_points = np.array(target.points)
    neigh.fit(target_points)
    source_points = np.array(source.points)
    rng = neigh.radius_neighbors(source_points)
    result = rng[1]
    result_boolean = []
    # 找出 source_points 中的每個點是否有鄰近的 target_points。若存在鄰近點，則保留該點，否則刪除。
    for i in range(result.size):
        result_boolean.append(True if result[i].size > 0 else False) 
    result_boolean = np.array(result_boolean)
    # 只保留在鄰近目標點內的 source 點雲
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(source_points[result_boolean])
    pcd.translate(-translate)
    return pcd

# 根據給定的解析度 (resolution) 隨機篩選點雲中的點，以降低點雲密度。
def resolution(source, resolution):
    booleans = [True, False]
    original_points = np.array(source.points)
    index = np.random.choice(booleans, int(original_points.size/3), p=[resolution, 1-resolution])
    source.points = o3d.utility.Vector3dVector(original_points[index])
    return source

# 與 resolution() 類似，但此函數處理的是點雲座標陣列（numpy array）而不是 Open3D 的點雲物件。
def array_resolution(points, resolution):
    booleans = [True, False]
    original_points = points
    index = np.random.choice(booleans, int(original_points.size/3), p=[resolution, 1-resolution])
    new_points = original_points[index]
    return new_points

# 功能：篩選出與目標物體方向一致的點雲，以減少資料量並集中處理與目標相關的區域。
def roi(source, estimated_target_position, threshold=0.99, point_threshold=5):

    new_points = source.points
    
    point_vector_list = []

    # 計算每個點到目標的向量
    for point in new_points:
        vector = np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])
        vector_length = np.sqrt(np.sum(vector**2))
        point_vector_list.append({'point': point, 'vector': vector/vector_length if vector_length > 0 else [0,0,0]})
        
    clusters = []

    # 分群基於方向向量相似度
    for point in point_vector_list:
        found = False
        for cluster in clusters:
            point_vector = point['vector']
            cluster_vector = cluster['vector']
            simularity = np.dot(point_vector, cluster_vector)
            if simularity > threshold:
                cluster['points'].append(point['point'])
                found = True
                break
        if not found:
            clusters.append({'vector': point['vector'], 'points': [point['point']]})
    
    final_points = []
    
    # 對每個群組內的點進行篩選，只保留距離最近的點
    for cluster in clusters:
        points = cluster['points']
        
        def vector_len(point):
            return np.sqrt(np.sum(np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])**2))
        
        points = sorted(points, key=lambda point: vector_len(point))
        
        min_len = vector_len(points[0])
        
        filtered_points = []
        for point in points:
            if abs(vector_len(point)-min_len) < point_threshold:
                filtered_points.append(point)

        # 這兩個print可以註解        
        print("points len:" , len(points))
        print("filtered_points len:" , len(filtered_points))
        
        final_points += filtered_points
        
    final_points = np.array(final_points)
    
    source.points = o3d.utility.Vector3dVector(final_points)
    
    # 這兩行可以註解
    trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    draw_registration_result(source, source, trans_init)
    
    return source

def filter_target_points(source, estimated_target_position):
    target_x = estimated_target_position[0]
    target_y = estimated_target_position[1]
    target_z = estimated_target_position[2]
    selected_points = []
    for i in range(1000):
        selected_points.append([target_x + random.gauss(0, 1), target_y + random.gauss(0, 1), target_z + random.gauss(0, 1)])
        selected_points.append([random.gauss(0, 1), random.gauss(0, 1), random.gauss(0, 1)])
    selected_points = np.array(selected_points)
    return selected_points

# 過濾掉遮蔽點，保留視線範圍內的點，以提高目標偵測的準確度。
def filter_occlusion_points(source, estimated_target_position):
    slope1 = 4/1.5
    slope2 = -4/1.5
    target_x = estimated_target_position[0]
    target_y = estimated_target_position[1]
    target_z = estimated_target_position[2]
    target_slope = target_y/target_x
    points = np.array(source.points)
        
    if target_slope > slope2 and target_slope < slope1:
        target_slope1 = (target_y-4)/target_x
        target_slope2 = (target_y+4)/target_x
        min_slope = min(target_slope1, target_slope2)
        max_slope = max(target_slope1, target_slope2)
        points_slope = (points[:, 1]-target_y)/(points[:, 0]-target_x)
        bool_array = np.logical_and(points_slope < max_slope, points_slope > min_slope)
        selected_points = points[bool_array]
        if target_x < 0:
            selected_points = selected_points[selected_points[:, 0] > 0]
        else:
            selected_points = selected_points[selected_points[:, 0] < 0]
    else:
        target_slope1 = target_y/(target_x-2)
        target_slope2 = target_y/(target_x+2)
        min_slope = min(target_slope1, target_slope2)
        max_slope = max(target_slope1, target_slope2)
        points_slope = (points[:, 1]-target_y)/(points[:, 0]-target_x)
        bool_array = np.logical_or(points_slope > max_slope, points_slope < min_slope)
        selected_points = points[bool_array]
        if target_y < 0:
            selected_points = selected_points[selected_points[:, 1] > 0]
        else:
            selected_points = selected_points[selected_points[:, 1] < 0]
    
    # 過濾掉超過最大坡度的點
    points_slope = (selected_points[:, 2]-target_z)/np.sqrt((selected_points[:, 0]-target_x)**2+(selected_points[:, 1]-target_y)**2)
    slope_limit = (-target_z)/math.sqrt((-target_x)**2+(-target_y)**2)
    selected_points = selected_points[points_slope < slope_limit]
    
    return selected_points

def crossProduct(a, b, o):
    return (b[0]-a[0])*(o[1]-a[1]) > (o[0]-a[0])*(b[1]-a[1])

def intersect(a, b, c, d):
    if ( max(a[0], b[0]) < min(c[0], d[0]) or max(a[1], b[1]) < min(c[1], d[1]) or max(c[0], d[0]) < min(a[0], b[0]) or max(c[1], d[1]) < min(a[1], b[1])):
        # print("False:", a, b, c, d)
        return False

    c1 = crossProduct(a, b, c)
    c2 = crossProduct(a, b, d)

    c3 = crossProduct(c, d, a)
    c4 = crossProduct(c, d, b)

    if ((c1 != c2) and (c3 != c4)):
        # print("True:", a, b, c, d, c1, c2, c3, c4)
        return True

    # print("False:", a, b, c, d)
    return False

# 過濾掉被遮蔽的點雲，基於四邊形範圍的可見性檢查。
def new_filter_occlusion_points(source, estimated_target_position, corner):
    target_x = estimated_target_position[0]
    target_y = estimated_target_position[1]
    target_z = estimated_target_position[2]
    target = np.array([target_x, target_y, target_z])
    nppoints = np.array(source.points)
    
    points = list(nppoints)
    corner_points = np.array(corner.points)

    # 使用四條邊界進行遮蔽檢查
    linecheck1 = lambda t: intersect(corner_points[0], corner_points[1], t, target)
    bool1 = np.array(list(map(linecheck1, points)))
    linecheck2 = lambda t: intersect(corner_points[1], corner_points[2], t, target)
    bool2 = np.array(list(map(linecheck2, points)))
    linecheck3 = lambda t: intersect(corner_points[2], corner_points[3], t, target)
    bool3 = np.array(list(map(linecheck3, points)))
    linecheck4 = lambda t: intersect(corner_points[3], corner_points[0], t, target)
    bool4 = np.array(list(map(linecheck4, points)))
    
    bool_array1 = np.logical_or(bool1, bool2)
    bool_array2 = np.logical_or(bool3, bool4)
    bool_array = np.logical_or(bool_array1, bool_array2)
    selected_points = nppoints[bool_array]
    
    points_slope = (selected_points[:, 2]-target_z)/np.sqrt((selected_points[:, 0]-target_x)**2+(selected_points[:, 1]-target_y)**2)
    slope_limit = (-target_z)/math.sqrt((-target_x)**2+(-target_y)**2)
    selected_points = selected_points[points_slope < slope_limit]
    
    return selected_points

# 適用於多視角資料
def accumulated_roi(source, estimated_target_position, threshold=0.98, density=20, reduced_rate_target=0.35):

    new_points = source.points
    original_size = np.array(new_points).shape[0]
    
    point_vector_list = []
    
    # 計算每個點相對於estimated_target_position的向量，將每個點的向量標準化，並儲存在 point_vector_list 中
    for point in new_points:
        vector = np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])
        vector_length = np.sqrt(np.sum(vector**2))
        point_vector_list.append({'point': point, 'vector': vector/vector_length if vector_length > 0 else [0,0,0]})
    
    # 使用 clusters_hash 將相似向量的點雲聚集在一起，以提高計算效率。
    # 基於向量的內積（即相似度）來決定點是否屬於某個群組。
    clusters_hash = [[] for i in range(1000)]
    for point in point_vector_list:
        found = False
        vector = point['vector']
        hash_key = int(vector[0]*vector[1]*vector[2]*1000)
        clusters = clusters_hash[hash_key]
        for cluster in clusters:
            point_vector = point['vector']
            cluster_vector = cluster['vector']
            simularity = np.dot(point_vector, cluster_vector)
            if simularity > threshold:
                cluster['points'].append(point['point'])
                found = True
                break
        if not found:
            clusters_hash[hash_key].append({'vector': point['vector'], 'points': [point['point']]})

    # 對每個群組進行密度控制和降采樣，定義不同距離範圍的體積，用於計算密度。
    final_points1 = None
    final_points2 = None
    final_points3 = None
    final_points4 = None
    final_points5 = None
    
    volume1 = 3*math.pi/4
    volume2 = 3*math.pi*(2**3-1)/4
    volume3 = 3*math.pi*(3**3-2**3)/4
    volume4 = 3*math.pi*(4**3-3**3)/4
    volume5 = 3*math.pi*(5**3-4**3)/4
    
    # 篩選與密度控制
    for clusters in clusters_hash:
        for cluster in clusters:
            points = cluster['points']

            def vector_len(point):
                return np.sqrt(np.sum(np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])**2))

            points_len = [vector_len(point) for point in points]

            min_len = min(points_len)

            filtered_points1 = []
            filtered_points2 = []
            filtered_points3 = []
            filtered_points4 = []
            filtered_points5 = []
            for point in points:
                diff = abs(vector_len(point)-min_len)
                if diff < 0.5:
                    filtered_points1.append(point)
                elif diff >= 0.5 and diff < 1:
                    filtered_points2.append(point)
                elif diff >= 1 and diff < 1.5:
                    filtered_points3.append(point)
                elif diff >= 1.5 and diff < 2:
                    filtered_points4.append(point)
                elif diff >= 2 and diff < 2.5:
                    filtered_points5.append(point)
                    
            filtered_points1 = np.array(filtered_points1)
            filtered_points2 = np.array(filtered_points2)
            filtered_points3 = np.array(filtered_points3)
            filtered_points4 = np.array(filtered_points4)
            filtered_points5 = np.array(filtered_points5)

            density1 = len(filtered_points1)/volume1
            if density1 > density:
                filtered_points1 = array_resolution(filtered_points1, density/density1)

            density2 = len(filtered_points2)/volume2
            if density2 > density:
                filtered_points2 = array_resolution(filtered_points2, density/density2)
            
            density3 = len(filtered_points3)/volume3
            if density3 > density:
                filtered_points3 = array_resolution(filtered_points3, density/density3)
            
            density4 = len(filtered_points4)/volume4
            if density4 > density:
                filtered_points4 = array_resolution(filtered_points4, density/density4)
            
            density5 = len(filtered_points5)/volume5
            if density5 > density:
                filtered_points5 = array_resolution(filtered_points5, density/density5)
            
            if final_points1 is None and filtered_points1.shape[0] != 0:
                final_points1 = filtered_points1
            elif filtered_points1.shape[0] != 0:
                final_points1 = np.concatenate((final_points1, filtered_points1), axis=0)
                
            if final_points2 is None and filtered_points2.shape[0] != 0:
                final_points2 = filtered_points2
            elif filtered_points2.shape[0] != 0:
                final_points2 = np.concatenate((final_points2, filtered_points2), axis=0)
                
            if final_points3 is None and filtered_points3.shape[0] != 0:
                final_points3 = filtered_points3
            elif filtered_points3.shape[0] != 0:
                final_points3 = np.concatenate((final_points3, filtered_points3), axis=0)
                
            if final_points4 is None and filtered_points4.shape[0] != 0:
                final_points4 = filtered_points4
            elif filtered_points4.shape[0] != 0:
                final_points4 = np.concatenate((final_points4, filtered_points4), axis=0)
                
            if final_points5 is None and filtered_points5.shape[0] != 0:
                final_points5 = filtered_points5
            elif filtered_points5.shape[0] != 0:
                final_points5 = np.concatenate((final_points5, filtered_points5), axis=0)

    # 合併結果並返回
    final_points = np.array(final_points1)
    print(final_points.shape)
    
    if final_points.shape[0]/original_size < reduced_rate_target and final_points2 is not None:
        final_points = np.append(final_points, final_points2, axis=0)
    
    if final_points.shape[0]/original_size < reduced_rate_target and final_points3 is not None:
        final_points = np.append(final_points, final_points3, axis=0)
    
    if final_points.shape[0]/original_size < reduced_rate_target and final_points4 is not None:
        final_points = np.append(final_points, final_points4, axis=0)
        
    if final_points.shape[0]/original_size < reduced_rate_target and final_points5 is not None:
        final_points = np.append(final_points, final_points5, axis=0)

    source.points = o3d.utility.Vector3dVector(final_points)
    
    trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    return source

# 主要目的是從點雲資料中篩選出covisible points
def covisible_roi(source, estimated_target_position, density=20, reduced_rate_target=0.35):
    print('density:', density)
    print(reduced_rate_target)

    new_points = source.points
    original_size = np.array(new_points).shape[0]
    
    groups = [[[] for x in range(180*2)] for y in range(360*2)] 
    
    for point in new_points:
        vector = np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])
        angle_index = int(math.degrees(math.atan(vector[1]/vector[0]))*2)
        elevation_index = int(math.degrees(math.atan(vector[2]/math.sqrt(vector[0]**2+vector[1]**2)))*2)+180
        groups[angle_index][elevation_index].append(point)
    
    final_points1 = None
    final_points2 = None
    final_points3 = None
    final_points4 = None
    final_points5 = None
    
    volume1 = 3*math.pi/4
    volume2 = 3*math.pi*(2**3-1)/4
    volume3 = 3*math.pi*(3**3-2**3)/4
    volume4 = 3*math.pi*(4**3-3**3)/4
    volume5 = 3*math.pi*(5**3-4**3)/4
    
    
    groups = np.array(groups, dtype=object)
    groups = groups.flatten()
    for group in groups:
        if len(group) == 0:
            continue
        points = group

        def vector_len(point):
            return np.sqrt(np.sum(np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])**2))

        points_len = [vector_len(point) for point in points]

        min_len = min(points_len)

        filtered_points1 = []
        filtered_points2 = []
        filtered_points3 = []
        filtered_points4 = []
        filtered_points5 = []
        for point in points:
            diff = abs(vector_len(point)-min_len)
            if diff < 0.5:
                filtered_points1.append(point)
            elif diff >= 0.5 and diff < 1:
                filtered_points2.append(point)
            elif diff >= 1 and diff < 1.5:
                filtered_points3.append(point)
            elif diff >= 1.5 and diff < 2:
                filtered_points4.append(point)
            elif diff >= 2 and diff < 2.5:
                filtered_points5.append(point)

        filtered_points1 = np.array(filtered_points1)
        filtered_points2 = np.array(filtered_points2)
        filtered_points3 = np.array(filtered_points3)
        filtered_points4 = np.array(filtered_points4)
        filtered_points5 = np.array(filtered_points5)

        density1 = len(filtered_points1)/volume1
        if density1 > density:
            filtered_points1 = array_resolution(filtered_points1, density/density1)

        density2 = len(filtered_points2)/volume2
        if density2 > density:
            filtered_points2 = array_resolution(filtered_points2, density/density2)

        density3 = len(filtered_points3)/volume3
        if density3 > density:
            filtered_points3 = array_resolution(filtered_points3, density/density3)

        density4 = len(filtered_points4)/volume4
        if density4 > density:
            filtered_points4 = array_resolution(filtered_points4, density/density4)

        density5 = len(filtered_points5)/volume5
        if density5 > density:
            filtered_points5 = array_resolution(filtered_points5, density/density5)

        if final_points1 is None and filtered_points1.shape[0] != 0:
            final_points1 = filtered_points1
        elif filtered_points1.shape[0] != 0:
            final_points1 = np.concatenate((final_points1, filtered_points1), axis=0)

        if final_points2 is None and filtered_points2.shape[0] != 0:
            final_points2 = filtered_points2
        elif filtered_points2.shape[0] != 0:
            final_points2 = np.concatenate((final_points2, filtered_points2), axis=0)

        if final_points3 is None and filtered_points3.shape[0] != 0:
            final_points3 = filtered_points3
        elif filtered_points3.shape[0] != 0:
            final_points3 = np.concatenate((final_points3, filtered_points3), axis=0)

        if final_points4 is None and filtered_points4.shape[0] != 0:
            final_points4 = filtered_points4
        elif filtered_points4.shape[0] != 0:
            final_points4 = np.concatenate((final_points4, filtered_points4), axis=0)

        if final_points5 is None and filtered_points5.shape[0] != 0:
            final_points5 = filtered_points5
        elif filtered_points5.shape[0] != 0:
            final_points5 = np.concatenate((final_points5, filtered_points5), axis=0)

    
    final_points = np.array(final_points1)
    print(final_points.shape)
    
    print(final_points.shape[0]/original_size)
    if final_points.shape[0]/original_size < reduced_rate_target and final_points2 is not None:
        final_points = np.append(final_points, final_points2, axis=0)
    print(final_points.shape[0]/original_size)
    
    if final_points.shape[0]/original_size < reduced_rate_target and final_points3 is not None:
        final_points = np.append(final_points, final_points3, axis=0)
    print(final_points.shape[0]/original_size)
    
    if final_points.shape[0]/original_size < reduced_rate_target and final_points4 is not None:
        final_points = np.append(final_points, final_points4, axis=0)
    print(final_points.shape[0]/original_size)
    
    if final_points.shape[0]/original_size < reduced_rate_target and final_points5 is not None:
        final_points = np.append(final_points, final_points5, axis=0)
    print(final_points.shape[0]/original_size)
    
    source.points = o3d.utility.Vector3dVector(final_points)
    
    trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    return source

# 目的是從來源點雲中提取與 estimated_target_position 共視的點，同時區分 occlusion points 和 covisible points
def extract_covisible_roi(source, estimated_target_position, distance=1):

    new_points = source.points
    original_size = np.array(new_points).shape[0]

    # 先初始化一個 360 x 180 的 2D 陣列 groups，用於根據水平角度和仰角對點進行分群。
    groups = [[[] for x in range(180*2)] for y in range(360*2)] 

    # 計算每個點到目標位置的向量，然後根據該向量的水平角度和仰角，將點分到不同的群組中。
    for point in new_points:
        vector = np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])
        angle_index = int(math.degrees(math.atan(vector[1]/vector[0]))*2)
        elevation_index = int(math.degrees(math.atan(vector[2]/math.sqrt(vector[0]**2+vector[1]**2)))*2)+180
        groups[angle_index][elevation_index].append(point)
    
    final_points1 = None
    
    volume1 = 3*math.pi/4
    
    # 對每個群組中的點，計算它們與目標的距離，並找出最接近目標的點（min_len）。
    # 如果點與 min_len 的距離小於指定的 distance，則認為它是共視點；否則為遮擋點。
    covisible_points = []
    occlusion_points = []
    
    groups = np.array(groups, dtype=object)
    groups = groups.flatten()
    for group in groups:
        if len(group) == 0:
            continue
        points = group

        def vector_len(point):
            return np.sqrt(np.sum(np.array([point[0]-estimated_target_position[0], point[1]-estimated_target_position[1], point[2]-estimated_target_position[2]])**2))

        points_len = [vector_len(point) for point in points]

        min_len = min(points_len)

        for point in points:
            diff = abs(vector_len(point)-min_len)
            if diff < distance:
                covisible_points.append(point)
            else:
                occlusion_points.append(point)

    covisible_points = np.array(covisible_points)
    occlusion_points = np.array(occlusion_points)
        
    # 返回共視點和遮擋點
    return covisible_points, occlusion_points

# 從來源點雲中篩除掉距離目標超過指定範圍 (threshold) 的點
def filter_out_of_range(source, target, threshold=80):
    points = np.array(source.points)
    # 使用歐氏距離計算每個點與目標的水平距離（僅考慮 x 和 y 軸）。
    distance = (points[:, 0]-target[0])**2 + (points[:, 1]-target[1])**2
    # 保留距離在 threshold 之內的點，並更新來源點雲。
    points = points[distance < threshold**2]
    
    source.points = o3d.utility.Vector3dVector(points)
    return source

# class CameraPoseEstimator:
#     def __init__(self, camera_matrix):
#         self.camera_matrix = camera_matrix
#         self.epnp = EPnP.EPnP()

#     def estimate_pose(self, world_points, image_points):
#         """
#         使用 EPnP 演算法估算相機姿態。

#         Parameters:
#         - world_points: ndarray (N, 3) - 3D 世界座標
#         - image_points: ndarray (N, 2) - 2D 影像像素座標
#         - camera_matrix: ndarray (3, 3) - 相機內參數矩陣

#         Returns:
#         - R: ndarray (3, 3) - 旋轉矩陣
#         - t: ndarray (3, 1) - 平移向量
#         - error: float - 重投影誤差
#         """
#         world_points = np.array(world_points)
#         image_points = np.array(image_points)

#         # 使用 EPnP 演算法進行相機姿態估計
#         error, Rt, Cc, Xc = self.epnp.efficient_pnp(world_points, image_points, self.camera_matrix)
        
#         # 拆分旋轉矩陣和平移向量
#         R = Rt[:, :3]
#         t = Rt[:, 3].reshape(-1, 1)
#         return R, t, error

#     def refine_pose_with_gauss(self, world_points, image_points):
#         """
#         使用 Gauss-Newton 優化 EPnP 演算法估算相機姿態。

#         Returns:
#         - R: ndarray (3, 3) - 旋轉矩陣
#         - t: ndarray (3, 1) - 平移向量
#         - error: float - 重投影誤差
#         """
#         world_points = np.array(world_points)
#         image_points = np.array(image_points)

#         error, Rt, Cc, Xc = self.epnp.efficient_pnp_gauss(world_points, image_points, self.camera_matrix)
        
#         R = Rt[:, :3]
#         t = Rt[:, 3].reshape(-1, 1)
#         return R, t, error









