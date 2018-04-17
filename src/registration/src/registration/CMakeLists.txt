cmake_minimum_required(VERSION 2.8.3)
project(registration)

find_package(catkin REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin 
  REQUIRED 
    dynamic_reconfigure
    cmake_modules
  COMPONENTS
    roscpp
    pcl_conversions
    tf_conversions
    sensor_msgs
    pcl_ros
    geometry
    # plane_detection
)

find_package(Eigen REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/SubSampling.cfg
  cfg/Cutting.cfg
  cfg/RadiusFiltering.cfg
  cfg/OutliersRemoval.cfg
  cfg/Merging.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
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

add_executable(plane_matching_node src/plane_matching_node.cpp)
target_link_libraries(plane_matching_node 
  ${catkin_LIBRARIES}
  geometry
)
add_dependencies(plane_matching_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(merging_node src/merging_node.cpp)
target_link_libraries(merging_node 
  ${catkin_LIBRARIES}
  geometry
)
add_dependencies(merging_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(icp_node src/icp_node.cpp)
target_link_libraries(icp_node 
  ${catkin_LIBRARIES}
  geometry
)
add_dependencies(icp_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(result_saving_node src/result_saving_node.cpp)
target_link_libraries(result_saving_node 
  ${catkin_LIBRARIES}
)
add_dependencies(result_saving_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)