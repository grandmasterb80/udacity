cmake_minimum_required (VERSION 3.0)
project (Predicted_Mean_and_Covariance)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (Predicted_Mean_and_Covariance src/main.cpp src/ukf.cpp)
target_link_libraries (Predicted_Mean_and_Covariance Eigen3::Eigen)
