"""
Docstring for curbs.py

This code detect curb points in LIDAR 3D point cloud. It takes input file with 
LIDAR 3D point cloud and write curb point cloud in .las file

Arguments is 
    - path to LIDAR 3d point cloud 
    - path to output file name of .las file with curbs points  
"""

import os
import math
import laspy
import sys
import datetime
import plyfile as plf
import numpy as np

if __name__ == "__main__":

    input_file = sys.argv[1]
    if (len(sys.argv) > 2):
        output_file = sys.argv[2]
    else:
        if os.name == 'nt':
            desktop = os.path.join(os.path.join(os.environ['USERPROFILE']),
                                   'Desktop')
        else:
            desktop = os.path.join(os.path.join(os.path.expanduser('~')),
                                   'Desktop')

        output_file = desktop + '\\borders_' + \
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+'.las'
    
    # read input file
    plydata = plf.PlyData.read(input_file)
    cloud = np.ndarray((len(plydata['vertex']), 3))

    start_p = plydata['vertex'][0]

    # transform cloud to local coordinates
    for i in range(len(plydata['vertex'])):
        v = plydata['vertex'][i]
        cloud[i] = [v['x'] - start_p['x'], v['y'] - start_p['y'],
                    v['z'] - start_p['z']]

    max_x = cloud[:, 0].max()
    min_x = cloud[:, 0].min()

    max_y = cloud[:, 1].max()
    min_y = cloud[:, 1].min()

    max_z = cloud[:, 2].max()
    min_z = cloud[:, 2].min()

    new_cloud = []
    for p in cloud:
        if (p[2] - min_z < 4):
            new_cloud.append(p)

    #fill the mesh, find max and min value of hight in each cell
    step_x = 0.1
    step_y = 0.1

    num_x = math.ceil((max_x - min_x)/step_x) + 1
    num_y = math.ceil((max_y - min_y)/step_y) + 1
    mesh = np.zeros((num_x, num_y, 2))

    for p in new_cloud:
        nx = math.ceil((p[0] - min_x)/step_x)
        ny = math.ceil((p[1] - min_y)/step_y)

        if (mesh[nx, ny, 0] == 0):
            mesh[nx, ny, 0] = p[2]
            mesh[nx, ny, 1] = p[2]

        if (mesh[nx, ny, 0] > p[2]):
            mesh[nx, ny, 0] = p[2]

        if (mesh[nx, ny, 1] < p[2]):
            mesh[nx, ny, 1] = p[2]

    #create and fill cloud with curbs points
    curb_cloud = []

    for p in new_cloud:
        nx = math.ceil((p[0] - min_x)/step_x)
        ny = math.ceil((p[1] - min_y)/step_y)

        if (mesh[nx, ny, 1] - mesh[nx, ny, 0] > 0.1) and
        (mesh[nx, ny, 1] - mesh[nx, ny, 0] < 0.2):
            curb_cloud.append((p[0] + start_p[0], p[1] + start_p[1],
                               p[2] + start_p[2]))

    #write output .las file
    header = laspy.LasHeader(point_format=3, version="1.2")

    las = laspy.LasData(header)
    las.x = np.array(curb_cloud)[:, 0]
    las.y = np.array(curb_cloud)[:, 1]
    las.z = np.array(curb_cloud)[:, 2]

    las.write(output_file)
