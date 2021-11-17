# find_curbs
This code detect curb points in LIDAR 3D point cloud. It takes input file with 
LIDAR 3D point cloud and write curb point cloud in .las file.

Arguments is

    - path to LIDAR 3d point cloud
    
    - path to output file name of .las file with curbs points

packages:

  os
  
  math
  
  laspy>=2.0.0
  
  sys
  
  datetime
  
  plyfile
  
  numpy
