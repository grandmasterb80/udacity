cmake_minimum_required (VERSION 3.0)
project (Generating_Sigma_Points_Assignment_1)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Generating_Sigma_Points_Assignment_1 src/main.cpp src/ukf.cpp)
target_link_libraries (Generating_Sigma_Points_Assignment_1 Eigen3::Eigen)
