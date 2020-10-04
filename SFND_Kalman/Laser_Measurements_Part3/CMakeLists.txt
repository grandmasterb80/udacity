cmake_minimum_required (VERSION 3.0)
project (LidarFusion)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (LidarFusion src/main.cpp src/kalman_filter.cpp src/tracking.cpp)
target_link_libraries (LidarFusion Eigen3::Eigen)
