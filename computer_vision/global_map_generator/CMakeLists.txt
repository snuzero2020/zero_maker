cmake_minimum_required(VERSION 3.0.2)
project(global_map_generator)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge
  DEPENDS
  opencv_highgui opencv_imgproc 
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(global_map_pub src/global_map.cpp ${HEADERS} ${SOURCES})
add_dependencies(global_map_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_map_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})
