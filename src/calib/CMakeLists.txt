cmake_minimum_required(VERSION 2.8.3)
project(calib)

#add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
    dynamic_reconfigure
    cmake_modules
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    sensor_msgs
    shape_msgs
    pcl_ros
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Planes.msg
  PlaneClouds.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  config/Registration.cfg
  config/Cloud.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   DEPENDS 
    Eigen
    libpcl-all
  CATKIN_DEPENDS
    message_runtime
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include/calib
)

add_library(Cloud
  include/calib/Cloud.h
  src/Cloud.cpp
)
add_dependencies(Cloud 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(Cloud
  ${catkin_LIBRARIES}
)

# add_library(Merging
#   include/calib/Merging.h
#   src/Merging.cpp
# )
# add_dependencies(Merging 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(Merging
#   ${catkin_LIBRARIES}
# )

add_executable(registration_node src/registration_node.cpp)
target_link_libraries(registration_node 
  ${catkin_LIBRARIES}
  Cloud
)
add_dependencies(registration_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)