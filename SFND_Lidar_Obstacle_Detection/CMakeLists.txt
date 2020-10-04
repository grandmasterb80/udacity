cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

find_package(OpenMP)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/quiz/cluster/cluster_lib.cpp)


if(OpenMP_CXX_FOUND)
    target_link_libraries(environment PUBLIC ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
else()
    target_link_libraries (environment ${PCL_LIBRARIES})
endif()


