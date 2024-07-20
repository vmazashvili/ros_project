cmake_minimum_required(VERSION 3.0.2)
project(dmap_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmap_localization
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dmap_localizer src/dmap_localizer.cpp)

target_link_libraries(dmap_localizer
  ${catkin_LIBRARIES}
)

