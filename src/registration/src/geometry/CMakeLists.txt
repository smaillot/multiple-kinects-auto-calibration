cmake_minimum_required(VERSION 2.8.3)
project(geometry)

################################################
## Declare ROS messages, services and actions ##
################################################

find_package(catkin 
  COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES geometry
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_conversions pcl_ros
)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(geometry
  src/Line.cpp
  src/Plane.cpp
  src/PointCloud.cpp
)
add_dependencies(geometry 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(geometry
  ${catkin_LIBRARIES}
)