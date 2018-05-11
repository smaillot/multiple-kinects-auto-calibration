cmake_minimum_required(VERSION 2.8.3)
project(calib)

#add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

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