cmake_minimum_required(VERSION 2.8.3)
project(distance_map)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

set(CXX_STANDARD c++17)

include_directories(/usr/include/eigen3)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src)

add_subdirectory(src)
