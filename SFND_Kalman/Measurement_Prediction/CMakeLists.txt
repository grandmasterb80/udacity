cmake_minimum_required (VERSION 3.0)
project (Measurement_Prediction)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Measurement_Prediction src/main.cpp src/ukf.cpp)
target_link_libraries (Measurement_Prediction Eigen3::Eigen)
