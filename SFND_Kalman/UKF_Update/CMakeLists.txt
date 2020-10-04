cmake_minimum_required (VERSION 3.0)
project (UKF_Update)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (UKF_Update src/main.cpp src/ukf.cpp)
target_link_libraries (UKF_Update Eigen3::Eigen)
