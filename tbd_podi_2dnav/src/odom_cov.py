#!/usr/bin/env python

# output a custom covariance on the velocities

import rospy
import numpy as np
import sys

from nav_msgs.msg import Odometry

class odomRepublisher:
    def __init__(self, odom_topic, cov_param):
        self.cov_param = cov_param
        self.odom_topic = odom_topic
        rospy.loginfo(self.odom_topic)
        rospy.loginfo(self.cov_param)
        self.orig_odom = None
        self.odom = None

        rospy.Subscriber(self.odom_topic, Odometry, self.updateOdom)
        self.odomPub = rospy.Publisher(self.odom_topic + '_with_cov', Odometry, queue_size=10)

    def updateOdom(self, msg):
        self.orig_odom = msg

    def publishOdom(self):
        if(self.orig_odom is not None):
            self.odom = self.orig_odom
            self.cov = rospy.get_param(self.cov_param)
            self.odom.twist.covariance = np.diag([self.cov[0], 0, 0, 0, 0, self.cov[1]]).flatten('C')

            self.odomPub.publish(self.odom)

if __name__ == '__main__':
    rospy.init_node('odomRepublisher')
    myargv = rospy.myargv(argv = sys.argv)
    node = odomRepublisher(myargv[1], myargv[2])
    rate = rospy.Rate(50)            

    while(not rospy.is_shutdown()):
        node.publishOdom()
        rate.sleep()