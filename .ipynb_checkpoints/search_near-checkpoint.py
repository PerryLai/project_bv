import os
import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument("--path",
                    help="path")
args = parser.parse_args()

path = args.path

vehicles_file = os.path.join(path, 'vehicles.json')
walkers_file = os.path.join(path, 'walkers.json')
x = -143.80743408203125
y = -91.61036682128906

with open(vehicles_file) as f:
    data = json.load(f)
    for i in data:
        w_x = i['x']
        w_y = i['y']
        if ((x-w_x)**2 + (y-w_y)**2) < 1600:
            print(i)
