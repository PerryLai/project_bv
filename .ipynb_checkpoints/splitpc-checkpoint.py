import open3d as o3d
import os
import math
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--start", type=int,
                    help="start angle")
parser.add_argument("--end", type=int,
                    help="end angle")
parser.add_argument("--path",
                    help="path")
args = parser.parse_args()

path = args.path
start = args.start*math.pi/180
end = args.end*math.pi/180

mesh = o3d.io.read_point_cloud(path)
points = np.asarray(mesh.points)


filter_points = []

for point in points:
    x = point[0]
    y = point[1]
    z = point[2]
    angle = math.atan2( y, x ) + math.pi
    if angle > start and angle < end:
        filter_points.append(point)
print(len(filter_points))
filter_points = np.asarray(filter_points)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(filter_points)
o3d.io.write_point_cloud(os.path.join(path[:-4] + str(args.start) + "_" + str(args.end) + ".ply"), pcd)


