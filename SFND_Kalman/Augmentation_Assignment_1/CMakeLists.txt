cmake_minimum_required (VERSION 3.0)
project (Augmentation_Assignment_1)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Augmentation_Assignment_1 src/main.cpp src/ukf.cpp)
target_link_libraries (Augmentation_Assignment_1 Eigen3::Eigen)
