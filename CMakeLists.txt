cmake_minimum_required(VERSION 2.8.3)
project(loam_tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

find_package(catkin_simple REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_simple()

cs_add_executable(map_to_depth_node src/map_to_depth_node.cpp src/map_to_depth.cpp src/camera.cpp)
target_link_libraries(map_to_depth_node)

cs_install()
cs_export()
