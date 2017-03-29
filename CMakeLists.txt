cmake_minimum_required(VERSION 2.8.3)
project(merbots_pc_object_detection)

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  pluginlib
  cv_bridge
  image_transport
  camera_calibration_parsers
  sensor_msgs  
  tf_conversions
)


find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(PCL 1.7 REQUIRED COMPONENTS io registration segmentation visualization recognition)

catkin_package( CATKIN_DEPENDS roscpp INCLUDE_DIRS include
  LIBRARIES nodelet DEPENDS object_detection  sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#OBJECT DETECTION
add_library(objectDetection src/object_detection.cpp)
target_link_libraries(objectDetection ${catkin_LIBRARIES})
add_executable(objectDetection_node src/object_detection_node.cpp)
target_link_libraries(objectDetection_node objectDetection)
add_library(objectDetection_nodelet src/object_detection_nodelet.cpp)
target_link_libraries(objectDetection_nodelet objectDetection)
#Filter Points
add_executable(points2_filter src/thirdparty/points2_filter.cpp)
target_link_libraries (points2_filter ${PCL_LIBRARIES} ${catkin_LIBRARIES})



