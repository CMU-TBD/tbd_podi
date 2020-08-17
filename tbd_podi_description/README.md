
Released under MIT License. 

Some URDFs and Mesh files are sourced from other repositories. Here are the list and their respective licenses:
* The urdf of ZED was modified from code initially released by Stereolabs under MIT license ([link to original repository](https://github.com/stereolabs/zed-ros-wrapper)). 
* The original urdf of P3DX and its meshes were released by MobileRobots under the BSD license ([link to original repository](https://github.com/MobileRobots/amr-ros-config)). 
* The urdf for Azure Kinect was modified from code initially released by Microsoft under MIT license ([link to original repository](https://github.com/microsoft/Azure_Kinect_ROS_Driver)).

#### How to rebuild URDF?
After changing any `.xacro` code, run the following to regenerate the URDF files.
```
rosrun xacro xacro.py podi.urdf.xacro > podi.urdf
```