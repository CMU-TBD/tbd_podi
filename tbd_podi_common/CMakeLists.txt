cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(tbd_podi_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  std_msgs
  sensor_msgs
  geometry_msgs
  joy
  tf
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
       roscpp
       roslib
       rosbag
       std_msgs
       sensor_msgs
       geometry_msgs
       joy
       tf
       actionlib_msgs
 )


 add_library(${PROJECT_NAME}
   src/control_robot.cpp
 )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(control_robot src/control_robot_node.cpp)
add_executable(laser_scan_frame_transform src/laser_scan_frame_transform.cpp)

target_link_libraries(control_robot ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(laser_scan_frame_transform ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/computer_state_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/handle_state_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})