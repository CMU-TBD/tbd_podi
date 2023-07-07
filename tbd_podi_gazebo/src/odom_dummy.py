#!/usr/bin/env python
import rospy
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates

class odomDummy:
    def __init__(self, model_name, topic_name, cov = None):
        self.model_name = model_name
        self.topic_name = topic_name
        self.currentModelStates = None
        self.currentOdomTrue = None
        self.currentPoseNoisy = None
        self.currentPoseRPY = None
        self.currentPoseRPYNoisy = None
        if(cov is None):
            self.covariance = np.diag([.1, .1, .1, 0, 0, 0])
        else:
            self.covariance = cov

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.updateMsgModelStates)

        self.odomPubNoisy = rospy.Publisher(topic_name, PoseWithCovarianceStamped, queue_size=10)
        self.odomPubTrue = rospy.Publisher(topic_name + '_gt', PoseStamped, queue_size=10)

    def updateMsgModelStates(self, msgModelStates):
        self.currentModelStates = msgModelStates
        msgTime = rospy.Time.now()
        self.getWorldPose(self.currentModelStates, msgTime)

    def getWorldPose(self, msgModelStates, msgTime):
        self.currentOdomTrue = PoseStamped()
        self.currentOdomTrue.header.stamp = msgTime
        self.currentOdomTrue.header.frame_id = self.model_name + "_base_link"
        try:
            idx = msgModelStates.name.index(self.model_name)
            self.currentOdomTrue.pose = msgModelStates.pose[idx]
            self.currentPoseRPY = np.hstack((np.array([self.currentOdomTrue.pose.position.x, self.currentOdomTrue.pose.position.y, self.currentOdomTrue.pose.position.z]),
                                         np.asarray(Rotation.from_quat([self.currentOdomTrue.pose.orientation.x, self.currentOdomTrue.pose.orientation.y,
                                                             self.currentOdomTrue.pose.orientation.z, self.currentOdomTrue.pose.orientation.w]).as_euler('xyz'))))
        except:
            self.currentOdomTrue = None
            return

    def addGaussianNoise(self, pose): # just to position
        self.currentPoseRPY_Noisy = np.random.default_rng().multivariate_normal(self.currentPoseRPY, self.covariance)
        poseNoisy = Pose()

        poseNoisy.position.x = self.currentPoseRPY_Noisy[0]
        poseNoisy.position.y = self.currentPoseRPY_Noisy[1]
        poseNoisy.position.z = self.currentPoseRPY_Noisy[2]

        orientation = np.asarray(Rotation.from_euler('xyz', self.currentPoseRPY_Noisy[3:]).as_quat())

        poseNoisy.orientation.x = orientation[0]
        poseNoisy.orientation.y = orientation[1]
        poseNoisy.orientation.z = orientation[2]
        poseNoisy.orientation.w = orientation[3]

        return poseNoisy                

    def publishOdom(self):
            if(self.currentOdomTrue is not None):
                msgOdom = PoseWithCovarianceStamped()
                msgOdom.header = self.currentOdomTrue.header
                msgOdom.pose.pose = self.addGaussianNoise(self.currentOdomTrue.pose)

                msgOdom.pose.covariance = self.covariance.flatten('C') #float64[36] row-major representation of 6x6 cov matrix

                self.odomPubNoisy.publish(msgOdom)
                self.odomPubTrue.publish(self.currentOdomTrue)

if __name__ == '__main__':
    rospy.init_node('odom_dummy')
    myargv = rospy.myargv(argv = sys.argv)

    node = odomDummy(myargv[1], myargv[2])
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        node.publishOdom()
        rate.sleep()