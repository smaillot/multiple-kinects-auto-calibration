cmake_minimum_required(VERSION 2.8.3)
project(calib)

#add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)


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
    tf_conversions
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Planes.msg
  Matches.msg
  TF.msg
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
  config/Node.cfg
  config/Preprocessing.cfg
  config/Cloud.cfg
  config/Plane.cfg
  config/Matching.cfg
  config/Merging.cfg
  config/TransformEstimation.cfg
  config/TF.cfg
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include/calib
  ${PCL_INCLUDE_DIRS}
  /usr/local/cuda/include
)

# PCL
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_library(Preprocessing
  include/calib/Preprocessing.h
  src/Preprocessing.cpp
)
add_dependencies(Preprocessing 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(Preprocessing
  ${catkin_LIBRARIES}
  Cloud
  ${PCL_LIBRARIES}
)

add_library(PlaneDetector
  include/calib/PlaneDetector.h
  src/PlaneDetector.cpp
)
add_dependencies(PlaneDetector 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(PlaneDetector
  ${catkin_LIBRARIES}
  Cloud
  Preprocessing
  ${PCL_LIBRARIES}
)

add_library(Matching
  include/calib/Matching.h
  src/Matching.cpp
)
add_dependencies(Matching 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(Matching
  ${catkin_LIBRARIES}
  Cloud
)

add_library(TransformEstimator
  include/calib/TransformEstimator.h
  src/TransformEstimator.cpp
)
add_dependencies(TransformEstimator 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(TransformEstimator
  ${catkin_LIBRARIES}
  Cloud
)

add_executable(preprocessing_node src/preprocessing_node.cpp)
target_link_libraries(preprocessing_node 
  ${catkin_LIBRARIES}
  Cloud
  Preprocessing
  ${PCL_LIBRARIES}
)
add_dependencies(preprocessing_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(merging_node src/merging_node.cpp)
target_link_libraries(merging_node 
  ${catkin_LIBRARIES}
  Cloud
  ${PCL_LIBRARIES}
)
add_dependencies(merging_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(plane_detection_node src/plane_detection_node.cpp)
target_link_libraries(plane_detection_node 
  ${catkin_LIBRARIES}
  Cloud
  PlaneDetector
  ${PCL_LIBRARIES}
)
add_dependencies(plane_detection_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(matching_node src/matching_node.cpp)
target_link_libraries(matching_node 
  ${catkin_LIBRARIES}
  Cloud
  Matching
  ${PCL_LIBRARIES}
)
add_dependencies(matching_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(transform_estimation_node src/transform_estimation_node.cpp)
target_link_libraries(transform_estimation_node 
  ${catkin_LIBRARIES}
  Cloud
  Matching
  TransformEstimator
  ${PCL_LIBRARIES}
)
add_dependencies(transform_estimation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(tf_node src/tf_node.cpp)
target_link_libraries(tf_node 
  ${catkin_LIBRARIES}
  Cloud
  -lyaml-cpp
)
add_dependencies(tf_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)