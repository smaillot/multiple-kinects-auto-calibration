cmake_minimum_required(VERSION 2.8.3)
project(multiple_kinects)

# add_definitions(-std=c++11)

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

find_package(OpenCV REQUIRED)



generate_dynamic_reconfigure_options(
  cfg/subsampling.cfg
  cfg/line_detection.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multiple_kinects
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_node src/test_node.cpp)
add_executable(cv_node src/cv_node.cpp)

target_link_libraries(
    test_node
        ${catkin_LIBRARIES}
)

target_link_libraries(
    cv_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_dependencies(test_node ${PROJECT_NAME}_gencfg)
add_dependencies(cv_node ${PROJECT_NAME}_gencfg)
