cmake_minimum_required(VERSION 2.8.3)
project(projecttassos)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  sensor_msgs
)

## Declare ROS messages and services
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/recognition.cpp
# )

## Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs tf sensor_msgs
)