cmake_minimum_required(VERSION 2.8.3)
project(registration)

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
  cfg/SubSampling.cfg
  cfg/Cutting.cfg
  cfg/RadiusFiltering.cfg
  cfg/PoseEstimation.cfg
)

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

add_executable(preprocessing_node src/preprocessing_node.cpp)
target_link_libraries(preprocessing_node 
  ${catkin_LIBRARIES}
  geometry
)
add_dependencies(preprocessing_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(line_matching_node src/line_matching_node.cpp)
target_link_libraries(line_matching_node 
  ${catkin_LIBRARIES}
  geometry
)
add_dependencies(line_matching_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(merging_node src/merging_node.cpp)
target_link_libraries(merging_node 
  ${catkin_LIBRARIES}
)
add_dependencies(merging_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(robust_pose_estimation_node src/robust_pose_estimation_node.cpp)
target_link_libraries(robust_pose_estimation_node 
  ${catkin_LIBRARIES}
)
add_dependencies(robust_pose_estimation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)