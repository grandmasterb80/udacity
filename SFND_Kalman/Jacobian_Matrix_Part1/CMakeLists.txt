cmake_minimum_required (VERSION 3.0)
project (JacobianMatrixPart1)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (JacobianMatrixPart1 src/main.cpp)
target_link_libraries (JacobianMatrixPart1 Eigen3::Eigen)
