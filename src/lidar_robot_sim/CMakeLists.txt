cmake_minimum_required(VERSION 3.0.2)
project(lidar_robot_sim)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
)

find_package(Eigen3 REQUIRED)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_robot_sim
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs stage_ros std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIRS})


find_package(laser_geometry REQUIRED)

add_executable(distance_map_generator src/distance_map_generator.cpp)
target_link_libraries(distance_map_generator ${catkin_LIBRARIES})

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES} ${laser_geometry_LIBRARIES} ${EIGEN3_LIBRARIES})
