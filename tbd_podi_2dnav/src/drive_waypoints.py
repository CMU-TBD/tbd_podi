#!/usr/bin/env python3
import rospy, rospkg
from rospy.numpy_msg import numpy_msg
import numpy as np
import sys
import time
import yaml
import dtw
from scipy.spatial import distance
from scipy.spatial.transform import Rotation as Rot

from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, Path


class waypointSender:
    def __init__(self, waypoints, max_time, sim, model_name = None, out_path = '/../../tune_logs/planner/res/res.yaml'):
        self.waypoints = waypoints
        self.waypointIdx = 0
        self.currentStatusMsg = None
        self.goalDone = True
        self.pathDone = False
        self.goalUpdatedTime = rospy.get_rostime()
        self.startTime = rospy.get_time()
        self.timeout = False
        self.max_time = max_time
        self.sim = sim
        self.model_name = model_name
        self.msgGoal = None
        self.out_path = out_path

        self.global_plan = []
        self.odom = []
        self.odom_times = []
        self.last_odom_time = None

        rospy.Subscriber("move_base/status", GoalStatusArray, self.updateMsgStatus)
        rospy.Subscriber("move_base/SBPLLatticePlanner/plan", Path, self.updateGlobalPlan)
        rospy.Subscriber("odom", Odometry, self.updateOdom)
        self.waypointPub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=10)

    def updateGlobalPlan(self, msgPlan):
        for pose in msgPlan.poses:
            self.global_plan.append(self.getXYTh(pose.pose))

    def updateOdom(self, msgOdom):
        time_curr = msgOdom.header.stamp.to_sec()
        if(time_curr != self.last_odom_time):
            self.odom.append(self.getXYTh(msgOdom.pose.pose))
            self.odom_times.append(msgOdom.header.stamp.to_sec())
            self.last_odom_time = time_curr

    def updateMsgStatus(self, msgStatus):
        self.currentStatusMsg = msgStatus
        if(self.currentStatusMsg.status_list):
            if((self.currentStatusMsg.status_list[-1].status == 3) & ((rospy.get_rostime() - self.goalUpdatedTime).to_sec() > 1)):
                self.goalDone = True

    def getXYTh(self, pose):
        x = pose.position.x
        y = pose.position.y
        r = Rot.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        yaw = r.as_euler('zxy')[0]

        return [x, y, yaw]
    
    def getHandlePoses(self, poses, h_length):
        x = poses[:,0] - h_length*np.cos(poses[:,2])
        y = poses[:,1] - h_length*np.sin(poses[:,2])

        return np.stack((x, y)).T
    
    def get95Jerk(self, poses, times):
        # velocity
        dt = times[1:] - times[0:-1]
        vx = (poses[1:,0] - poses[0:-1,0])/dt
        vy = (poses[1:,1] - poses[0:-1,1])/dt

        # acceleration
        ax = (vx[1:] - vx[0:-1])/dt[1:]
        ay = (vy[1:] - vy[0:-1])/dt[1:]

        # jerk
        jx = (ax[1:] - ax[0:-1])/times[3:]/dt[2:]
        jy = (ay[1:] - ay[0:-1])/times[3:]/dt[2:]

        jt = np.sqrt(jx**2 + jy**2)

        return np.percentile(jt, 95)
    
    def publishWaypoint(self):
        time_elapsed = rospy.get_time() - self.startTime
        if(time_elapsed > self.max_time):
            rospy.loginfo("Timed out")
            self.timeout = True

        if(self.timeout or self.pathDone):
            res_dict = {}
            # spin down node

            ### for checking trajectory
            if(self.sim):
                #rospy.loginfo(self.global_plan)
                #rospy.loginfo(self.odom)
                # calculate handle position for global and actual
                h_length = 0.71

                global_handle = self.getHandlePoses(np.array(self.global_plan), h_length)
                actual_handle = self.getHandlePoses(np.array(self.odom), h_length)

                # calculate distance matrix between global plan and actual odometry
                dist_m = distance.cdist(global_handle, actual_handle)

                # feed to dtw
                dtw_res = dtw.dtw(dist_m)

                # endpoint error, total time taken, 
                end_err = np.linalg.norm(global_handle[-1, :] - actual_handle[-1, :])
                time_tot = self.max_time*2 if self.timeout else time_elapsed
                jerk_95 = self.get95Jerk(np.array(self.odom), np.array(self.odom_times))

                res_dict = {'traj_dist': float(dtw_res.normalizedDistance), 'end_err': float(end_err),
                            'time': float(time_tot), 'jerk': float(jerk_95)}#, 'global_plan': np.array(self.global_plan).tolist(), 'global_handle': global_handle.tolist()}

                # write out to file
                with open(rospkg.RosPack().get_path('tbd_podi_2dnav') + self.out_path, 'w') as file:
                     yaml.dump(res_dict, file, sort_keys = False)

            ### for checking localization
            # if(self.sim): # get Gazebo odom and send
            #     msgModelStates = rospy.wait_for_message('/gazebo/model_states', ModelStates)
            #     try:
            #         idx = msgModelStates.name.index(self.model_name)
            #         gt_pose = self.getXYTh(msgModelStates.pose[idx])
            #         res_dict['gt'] = {'x': float(gt_pose[0]), 'y': float(gt_pose[1]), 'th': float(gt_pose[2])}
            #     except:
            #         rospy.logwarn("Could not get ground truth pose from Gazebo!")
            # # regardless, send estimated location
            # odom = rospy.wait_for_message('odom', Odometry)
            # est_pose = self.getXYTh(odom.pose.pose)
            # res_dict['est'] = {'x': float(est_pose[0]), 'y': float(est_pose[1]), 'th': float(est_pose[2])}
            # res_dict['goal'] = {'x': self.waypoints[-1][0], 'y': self.waypoints[-1][1], 'th': self.waypoints[-1][2]}
            # with open(rospkg.RosPack().get_path('tbd_podi_2dnav') + self.out_path, 'w') as file:
            #     yaml.dump(res_dict, file, sort_keys = False)

            rospy.signal_shutdown("Waypoints done.")

        if(self.goalDone == False):
            return

        if(self.waypointIdx < len(self.waypoints)):
            nextWaypoint = self.waypoints[self.waypointIdx]
            rospy.loginfo("Moving to " + str(nextWaypoint))

            self.msgGoal = MoveBaseActionGoal()
            self.msgGoal.header.stamp = rospy.Time.now()
            self.msgGoal.goal.target_pose.header.frame_id = rospy.get_param("map_server/frame_id")
            self.msgGoal.goal.target_pose.pose.position.x = nextWaypoint[0]
            self.msgGoal.goal.target_pose.pose.position.y = nextWaypoint[1]
            self.msgGoal.goal.target_pose.pose.position.z = 0

            r = Rot.from_euler('z', [nextWaypoint[2]])
            r_quat = r.as_quat()

            self.msgGoal.goal.target_pose.pose.orientation.x = r_quat[0][0]
            self.msgGoal.goal.target_pose.pose.orientation.y = r_quat[0][1]
            self.msgGoal.goal.target_pose.pose.orientation.z = r_quat[0][2]
            self.msgGoal.goal.target_pose.pose.orientation.w = r_quat[0][3]
            self.waypointPub.publish(self.msgGoal)
            self.goalDone = False
            self.waypointIdx = self.waypointIdx + 1
        else:
            self.pathDone = True

if __name__ == '__main__':
    rospy.init_node('waypoints')

    myargv = rospy.myargv(argv = sys.argv)
    waypoints = [[2, 15, np.pi/2]]#[6, 2, 0], [5, 8, -np.pi/2], [2, 14, np.pi], [6, 14, np.pi/2]]

    node = waypointSender(waypoints, 120, myargv[1], myargv[2])
    rate = rospy.Rate(10)

    time.sleep(5)

    while not rospy.is_shutdown():
        node.publishWaypoint()
        rate.sleep()