# TBD_podi_common
Copyright - Transportation, Bots, and Disability Lab, Carnegie Mellon University  
Released under MIT License  

To configure the computer, check out the [setup guide](https://github.com/CMU-TBD/TBD_podi_common/wiki/Podi-Setup-Guide). To run, check out the [run guide](https://github.com/CMU-TBD/TBD_podi_common/wiki/Podi-Running-Guide)

# Components (Nodes)

## control_robot_node
This node enables joystick commands (from `joy package`) for Podi. It also acts as an relay for other sources of control (`ros-navigation`, etc) to ensure safety and merging of commands from the joystick 

## laser_scan_frame_transform
As Podi's aluminimum structure gets in the way of the LIDAR, the laser gives out false readings. This node republishes the laser scanner reading and remove invalid readings. The current range is `pi * 5/8` to `-pi * 5/8`.

## handle_state_node
This node reads the encoder for the handler and publishes the current reading (0-1024) on the topic `handle_reading` (type: std_msgs/Int16). It also publishes the `sensor_msgs/JointState` messages for the `handle_rotation` and `handle_height` joints. 

## computer_state_node
This node reads the state of the laptop on board and publishes the state of the battery.

# joystick control mapping
TODO

# Quick Run Guide
1. Connect the USB-B and LiDAR's ethernet cable to the computer.
2. Run the following command: `roslaunch tbd_podi_common control_robot_laser.launch`.

# Past Contributors
- Amal Nanavati
