cmake_minimum_required(VERSION 3.0.2)
project(path_planning)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
include/${PROJECT_NAME}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/planning.cpp src/planning_new.cpp src/rrt_ys.cpp src/rrt_ys.h)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

 target_link_libraries(${PROJECT_NAME}_node
   ${OpenCV_LIBS}
)
