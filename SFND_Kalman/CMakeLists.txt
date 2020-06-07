cmake_minimum_required (VERSION 3.0)
project (Lidar_and_Radar_Fusion)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Lidar_and_Radar_Fusion Lidar_and_Radar_Fusion.cpp)
target_link_libraries (Lidar_and_Radar_Fusion Eigen3::Eigen)
