cmake_minimum_required(VERSION 3.0.2)
project(tbd_podi_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PodiFaceInput.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  setHandleHeight.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tbd_podi_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)