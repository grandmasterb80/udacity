cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executables for exercise
add_executable (cornerness_harris src/cornerness_harris.cpp)
target_link_libraries (cornerness_harris ${OpenCV_LIBRARIES})

# Executables for exercise
add_executable (detect_keypoints src/detect_keypoints.cpp)
target_link_libraries (detect_keypoints ${OpenCV_LIBRARIES})

# Executables for exercise
add_executable (describe_keypoints src/describe_keypoints.cpp)
target_link_libraries (describe_keypoints ${OpenCV_LIBRARIES})

# Executables for exercise
#add_executable (SF_Camera_Examples src/SF_Camera_Examples.cpp)
#target_link_libraries (SF_Camera_Examples ${OpenCV_LIBRARIES})

# Executables for exercise
add_executable (descriptor_matching src/descriptor_matching.cpp src/structIO.cpp)
target_link_libraries (descriptor_matching ${OpenCV_LIBRARIES})

# Executables for exercise / solution
add_executable (descriptor_matching_solution src/descriptor_matching_solution.cpp src/structIO.cpp)
target_link_libraries (descriptor_matching_solution ${OpenCV_LIBRARIES})
