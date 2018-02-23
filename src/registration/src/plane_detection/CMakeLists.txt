cmake_minimum_required(VERSION 2.8.3)
project(plane_detection)

find_package(catkin REQUIRED)



################################################
## Declare ROS messages, services and actions ##
################################################


find_package(catkin 
  COMPONENTS
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
#   DEPENDS geometry
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(plane_detection_node src/plane_detection_node.cpp)
target_link_libraries(plane_detection_node 
  ${catkin_LIBRARIES}
)
add_dependencies(plane_detection_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)