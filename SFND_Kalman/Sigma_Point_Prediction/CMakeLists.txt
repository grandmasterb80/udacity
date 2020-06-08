cmake_minimum_required (VERSION 3.0)
project (Sigma_Point_Prediction)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Sigma_Point_Prediction src/main.cpp src/ukf.cpp)
target_link_libraries (Sigma_Point_Prediction Eigen3::Eigen)
