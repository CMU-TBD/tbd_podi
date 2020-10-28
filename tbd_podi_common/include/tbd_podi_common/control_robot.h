#include <iostream>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <ctime>
#include <boost/foreach.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "sensor_msgs/Joy.h"

namespace tbd_podi_common {
  class ControlRobot
  {
  public:
  	ControlRobot();
  	~ControlRobot();

	// Run the program
  	void spin();

  private:
  	// NodeHandles
  	ros::NodeHandle n, private_nh;

  	// Publishers / Subscribers
  	ros::Publisher cmd_vel_pub;
  	ros::Subscriber joy_sub;
  	ros::Subscriber ros_navigation_cmd_vel_sub;

  	// Recording / Playing Back Rosbags
  	std::string rosbagTopicName;
  	std::string playbackRosbagPath; // default name of the bag in the directory
	std::string rosbagRecordingDir; // default directory for rosbag
  	double proportionalGainFactor; // used for the P controller that ensures that the rosbag is playing back at the right rate
  	std::thread playbackBagThread;
  	bool isRecordingRosbagOpen;
  	std::mutex isRecordingRosbagOpenMutex;

	std::string joystickType_;

  	// Used for the functionality of the controlLoop
  	double controlLoopHz;

	// enabling switch
	bool enablingSwitchEnabled_;
	bool enablingSwitchState_;
	std::mutex enablingSwitchMutex_;

  	// concurrency literals for the estop
  	bool emergencyStop;
  	bool newEmergencyStop;
  	std::mutex emergencyStopMutex;

  	// concurrency literals for rosbag recording
  	bool recording;
  	bool newRecording;
  	std::mutex recordingMutex;

  	// concurrency literals for rosbag playback
  	bool playingBack;
  	bool newPlayingBack;
  	std::mutex playingBackMutex;

  	// concurrency literals for the joystick velocity
  	geometry_msgs::Twist joyVel;
  	std::mutex joyVelMutex;
  	bool newJoyVel;

  	// concurrency literals for the playback velocity
  	geometry_msgs::Twist playbackVel;
  	std::mutex playbackVelMutex;
  	bool newPlaybackVel;

  	// concurrency literals for the navigation velocity
  	geometry_msgs::Twist navigationVel;
  	std::mutex navigationVelMutex;
  	bool newNavigationVel;

  	// Concurrency literals for printing robot pose
  	std::mutex printingRobotPoseMutex;
  	bool arePrintingRobotPose;
  	geometry_msgs::Pose targetRobotPose;

  	double lin_min, ang_min, lin_max, ang_max;

  	tf::TransformListener tf_;

  	void stopRobot();
  	void playbackBagFn(std::string playbackRosbagPath, double p);
  	void publishVelocity(geometry_msgs::Twist &spd);
  	void messageCB(const std_msgs::Int16 &msg);
  	void navigationCB(const geometry_msgs::Twist &msg);
  	void joyCB(const sensor_msgs::Joy &msg);

	void startBagRecording();
	void stopBagRecording();
	rosbag::Bag *currentRecordingBag;

	ros::Duration velLatchDuration; // The amount of time a previously receive velocity should kept publishing

  };

} // end namespace tbd_podi_common
