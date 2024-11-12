import os
import json
import math
import numpy as np

path = "data"
threshold = 0.2
correct = [0 for i in range(12)]

for i in range(100):
    file_path = os.path.join(path, str(i))

    jsonA = os.path.join(file_path, 'ego.json')
    jsonB = os.path.join(file_path, 'ego2.json')
    # Opening JSON file
    f = open(jsonA)
    egoA = json.load(f)
    f = open(jsonB)
    egoB = json.load(f)

    yaw = egoB['yaw']
    if yaw < 2 and yaw > -2:
        y = egoB['x'] - egoA['x']
        x = egoB['y'] - egoA['y']
    elif yaw < 92 and yaw > 88:
        x = -(egoB['x'] - egoA['x'])
        y = egoB['y'] - egoA['y']
    elif yaw < -88 and yaw > -92:
        x = egoB['x'] - egoA['x']
        y = -(egoB['y'] - egoA['y'])
    elif (yaw < -178 and yaw > -180) or (yaw < 180 and yaw > 178):
        y = -(egoB['x'] - egoA['x'])
        x = -(egoB['y'] - egoA['y'])

    f = open(os.path.join(file_path, 'transformation_result.json'))
    data = json.load(f)

    for j in range(2):
        t_x = np.array(data[j])[0, 3]
        t_y = np.array(data[j])[1, 3]
        if abs(t_x-x) < threshold and abs(t_y-y) < threshold:
            correct[j] += 1

print([i/100 for i in correct])




