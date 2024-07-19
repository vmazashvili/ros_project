cmake_minimum_required(VERSION 3.0.2)
project(dmap_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

find_package(PCL REQUIRED)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the executable
add_executable(dmap_localizer src/dmap_localizer.cpp)

# Specify libraries to link against
target_link_libraries(dmap_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Add dependencies (if you have custom messages/services)
add_dependencies(dmap_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

