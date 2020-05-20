# Change log

<!-- ## [Unreleased]
### Fixed
- Fixed definition/structure files where `std_msgs::Float64` and `std_msgs::Int32` were using pointers in the struct instead of actual value.


## [0.2.0] - 2020-04-17 -->
<!-- ## TODOs
## [Unreleased] -->
## [Unreleased]
#### Added 
- Ability to set initial pose for ROS Navigation.

#### Changed
- Updated the Camera and LiDAR's Gazebo numbers to better match what the real system has. 

## [1.1.1] - 2020-04-30
#### Changed
- Incorrect topic publishing name for front camera changed from `podi/front_camera/podi/image_raw` to `podi/front_camera/image_raw`

## [1.1.0] - 2020-04-23
#### Added
- `tbd_podi_2dnav` that acts an interface for ROS Navigation packages. Migrated in old settings from other podi repositories.
- `maps` folder and multiple maps. Including `sim_NSH_04_2020.yaml` that describe the gazebo environment.
- added rosparam `vel_latch_duration` that specifies how long old velocities are published if no need velocities are received.

#### Changed
- Relied on TF_prefix and namespace to specify type of robot instead of encoding the name into the URDF.
- Moved the maps originally in `tbd_podi_common` into `tbd_podi_2dnav`.
- Refactor `control_robot_node` to improve readibility and control flow. 

#### Fixed
- Incorrect namespace caused Gazebo to crash.
- Incorrect Laser parameter in the Gazebo description of the front laser
- Incorrect calculation for the laser limiter node that ignores certain readings.
- Incorrect differential drive Gazebo setting causing wildly in-accurate URDF.
- Bug where empty twist will be sent to the robot if there is no new command at the current control loop. Added latch duration that republish old velocities.

#### Removed 
- unused launch files.

## [1.0.1] - 2020-04-21
#### Added
- This changelog.
#### Changed
- Have all topics and TF frame to be prefixed with `podi` to prevent collision in multi-robot situations.
- Changed the differential driver gazebo plugin to use the `podi` prefix
- Changed some arguments in roslaunch file to use lower_underscore_case instead of Camelcase.

## [1.0.0] - 2020-XX-XX
Initial Release with face, msgs, description, and control components.
