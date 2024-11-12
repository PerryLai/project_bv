import os
import json
import math
import open3d as o3d
import numpy as np
from util import rotate_source, count_walker, count_cyclist, count_vehicle, icp


path = "data_network_SSMCO"

area_list = [0 for i in range(10)]

for index in range(100):
    print("Processing: ", index)
    file_path = os.path.join(path, str(index))
    
    filename = os.path.join(file_path, 'dp_icp_result.json')
    dp_result = json.load(open(filename))
    area = dp_result['area']
    for i in range(10):
        area_list[i] += area[i]
        

for i in range(10):
    area_list[i] /= 100

print(area_list)