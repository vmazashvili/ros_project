cmake_minimum_required(VERSION 3.0.2)
project(dmap_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmap_localization
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs tf pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dmap_localizer src/dmap_localizer.cpp)

target_link_libraries(dmap_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS dmap_localizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

