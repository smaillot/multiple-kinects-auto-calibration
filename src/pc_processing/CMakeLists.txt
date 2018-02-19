cmake_minimum_required(VERSION 2.8.3)
project(pc_processing)

# add_definitions(-std=c++11)

####################################333
## packages

find_package(catkin REQUIRED dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  tf
)

find_package(OpenCV 3.3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/registration.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pc_processing
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs
#  DEPENDS system_lib
)


####################################333
## executables

add_executable(pc_processing_nodelet 
  src/pc_processing_nodelet.cpp
)

####################################333
## directories

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


####################################333
## libraries

add_library(PcProcessing src/PcProcessing.cpp)

target_link_libraries(PcProcessing ${catkin_LIBRARIES})
target_link_libraries(pc_processing_nodelet ${catkin_LIBRARIES} PcProcessing)

####################################333
## dependencies

add_dependencies(pc_processing_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)