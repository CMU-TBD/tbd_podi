# tbd_podi

This repository contains ROS packages used by our robot Podi. While all packages are released under MIT license, some files in `tbd_podi_description` originated from other repositiory licensed under MIT and BSD.

Here's a quick overview of each package.
# Packages:
## [tbd_podi_common](tbd_podi_common)
Package that interfaces with all the hardware component of Podi.

## [tbd_podi_description](tbd_podi_description)
Package that contains the URDF and meshes for Podi.

## [tbd_podi_gazebo](tbd_podi_gazebo)
Package that contains the launch files for starting Podi in Gazebo.

## [tbd_podi_face](tbd_podi_face)
Package that communicates with the android tablet which acts as the face for Podi.

## [tbd_podi_msgs](tbd_podi_msgs)
Package that consists of all `msg` and `action` definitions used by Podi.

## [tbd_podi_2dnav](tbd_podi_2dnav)
Package that contains the needed maps, configs, and launch files to run ROS Navigation on Podi.