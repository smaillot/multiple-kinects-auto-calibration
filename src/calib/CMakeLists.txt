cmake_minimum_required(VERSION 2.8.3)
project(calib)

find_package(catkin REQUIRED
    dynamic_reconfigure
    cmake_modules
)

################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    sensor_msgs
    pcl_ros
    geometry
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
)

add_library(Cloud
  include/calib/Cloud.h
  src/Cloud.cpp
)
add_dependencies(Cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Cloud
  ${catkin_LIBRARIES}
)