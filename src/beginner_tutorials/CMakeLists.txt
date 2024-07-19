cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

## Add cmake target dependencies of the executable
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against

