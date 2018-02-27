cmake_minimum_required(VERSION 2.8.3)
project(plane_detection)

find_package(catkin REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin 
  REQUIRED 
    dynamic_reconfigure
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
  
generate_dynamic_reconfigure_options(
  cfg/PlaneDetection.cfg
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(PlaneDetector
  src/PlaneDetector.cpp
)
add_dependencies(PlaneDetector 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  geometry
)
target_link_libraries(PlaneDetector
  ${catkin_LIBRARIES}
)

add_executable(plane_detection_node src/plane_detection_node.cpp)
target_link_libraries(plane_detection_node 
  ${catkin_LIBRARIES}
  geometry
  PlaneDetector
)
add_dependencies(plane_detection_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)