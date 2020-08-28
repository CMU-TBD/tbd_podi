cmake_minimum_required(VERSION 3.0.2)
project(tbd_podi_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo REQUIRED)


## Define as a package
catkin_package(
)