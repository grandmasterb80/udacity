cmake_minimum_required (VERSION 3.0)
project (Predict_Radar_Measurement)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Predict_Radar_Measurement src/main.cpp src/ukf.cpp)
target_link_libraries (Predict_Radar_Measurement Eigen3::Eigen)
