#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import time
import copy
import numpy as np
import rospy
import tf
from autoware_planning_msgs.msg import Path, PathWithLaneId, Trajectory
from autoware_control_msgs.msg import ControlCommandStamped
from autoware_vehicle_msgs.msg import VehicleCommand
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import Bool, Float32, Header, Int32, Float32MultiArray


REF_LINK = "map"
SELF_LINK = "base_link"


LANE_CHANGE = 0
BEHAVIOR_VELOCITY = 1
OBSTACLE_AVOID = 2
OBSTACLE_STOP = 3
LAT_ACC = 4
VELOCITY_OPTIMIZE = 5
CONTROL_CMD = 6
VEHICLE_CMD = 7



class VelocityChecker:
    def __init__(self):

        self.autoware_engage = False
        self.external_vlim = 10000
        self.localization_twist = Twist()
        self.vehicle_twist = Twist()
        self.self_pose = Pose()
        self.v_arr = [0] * 8
        self.count = 0

        self.tfl = tf.TransformListener()  # for get self-position

        # planning path and trajectories
        self.sub_behavior_path_w_lid = rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", PathWithLaneId, self.CallBackBehaviorPathWLid, queue_size=1, tcp_nodelay=True
        )
        self.sub_behavior_velocity_path = rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path", Path, self.CallBackBehaviorPath, queue_size=1, tcp_nodelay=True
        )
        self.sub_avoid_trajectory = rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory", Trajectory, self.CallBackAvoidTrajectory, queue_size=1, tcp_nodelay=True
        )
        self.sub_lane_drive_trajectory = rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/trajectory", Trajectory, self.CallBackLaneDriveTrajectory, queue_size=1, tcp_nodelay=True
        )
        self.sub_latacc_trajectory = rospy.Subscriber(
            "/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_lateral_acc_filtered", Trajectory, self.CallBackLataccTrajectory, queue_size=1, tcp_nodelay=True
        )
        self.sub_trajectory = rospy.Subscriber(
            "/planning/scenario_planning/trajectory", Trajectory, self.CallBackScenarioTrajectory, queue_size=1, tcp_nodelay=True
        )

        # control commands
        self.sub_control_cmd = rospy.Subscriber(
            "/control/control_cmd", ControlCommandStamped, self.CallBackControlCmd, queue_size=1, tcp_nodelay=True
        )
        self.sub_vehicle_cmd = rospy.Subscriber(
            "/control/vehicle_cmd", VehicleCommand, self.CallBackVehicleCmd, queue_size=1, tcp_nodelay=True
        )

        # others related to velocity
        self.sub_autoware_engage = rospy.Subscriber(
            "/autoware/engage", Bool, self.CallBackAwEngage, queue_size=1, tcp_nodelay=True
        )
        self.sub_external_vlim = rospy.Subscriber(
            "/planning/scenario_planning/motion_velocity_optimizer/external_velocity_limit_mps", Bool, self.CallBackExternalVelLim, queue_size=1, tcp_nodelay=True
        )

        # self twist
        self.sub_localization_twist = rospy.Subscriber(
            "/localization/twist", TwistStamped, self.CallBackLocalizationTwist, queue_size=1, tcp_nodelay=True
        )
        self.sub_localization_twist = rospy.Subscriber(
            "/vehicle/twist", TwistStamped, self.CallBackVehicleTwist, queue_size=1, tcp_nodelay=True
        )

        # publish data
        self.pub_varr = rospy.Publisher("~closest_speeds", Float32MultiArray, queue_size=1)


        time.sleep(1.0)  # wait for ready to publish/subscribe

        # for publish traffic signal image
        rospy.Timer(rospy.Duration(1 / 10.0), self.timerCallback)


    def printInfo(self):
        v_arr_kmph = copy.copy(self.v_arr)
        for v in v_arr_kmph:
            v *= 3.6
        self.count = self.count % 30
        if self.count == 0:
            rospy.loginfo("")
            rospy.loginfo(
                "| Map Limit | Behavior | Obs Avoid | Obs Stop | External Lim | LatAcc Filtered | Optimized | Control Cmd | Vehicle Cmd | Engage | Localization Vel | Vehicle Vel | [km/h]")
        rospy.loginfo("| {0: 9.2f} | {1: 8.2f} | {2: 9.2f} | {3: 8.2f} | {4: 12.2f} | {5: 15.2f} | {6: 9.2f} | {7: 11.2f} | {8: 11.2f} | {9: 6d} | {10: 16.2f} | {11: 11.2f} |".format(
            v_arr_kmph[0], v_arr_kmph[1], v_arr_kmph[2], v_arr_kmph[3], self.external_vlim *
            3.6, v_arr_kmph[4], v_arr_kmph[5], v_arr_kmph[6], v_arr_kmph[7], self.autoware_engage,
            self.localization_twist.linear.x * 3.6, self.vehicle_twist.linear.x * 3.6))
        self.count += 1

    def timerCallback(self, t):
        # rospy.loginfo("timer called")
        self.updatePose(REF_LINK, SELF_LINK)
        self.pub_varr.publish(Float32MultiArray(data=self.v_arr))
        self.printInfo()

    def CallBackAwEngage(self, msg):
        self.autoware_engage = msg

    def CallBackExternalVelLim(self, msg):
        self.external_vlim = msg.data

    def CallBackLocalizationTwist(self, msg):
        self.localization_twist = msg.twist

    def CallBackVehicleTwist(self, msg):
        self.vehicle_twist = msg.twist

    def CallBackBehaviorPathWLid(self, msg):
        # rospy.loginfo("LANE_CHANGE called")
        closest = self.calcClosestPathWLid(msg)
        self.v_arr[LANE_CHANGE] = msg.points[closest].point.twist.linear.x
        return

    def CallBackBehaviorPath(self, msg):
        # rospy.loginfo("BEHAVIOR_VELOCITY called")
        closest = self.calcClosestPath(msg)
        self.v_arr[BEHAVIOR_VELOCITY] = msg.points[closest].twist.linear.x
        return

    def CallBackAvoidTrajectory(self, msg):
        # rospy.loginfo("OBSTACLE_AVOID called")
        closest = self.calcClosestTrajectory(msg)
        self.v_arr[OBSTACLE_AVOID] = msg.points[closest].twist.linear.x
        return

    def CallBackLaneDriveTrajectory(self, msg):
        # rospy.loginfo("OBSTACLE_STOP called")
        closest = self.calcClosestTrajectory(msg)
        self.v_arr[OBSTACLE_STOP] = msg.points[closest].twist.linear.x
        return

    def CallBackLataccTrajectory(self, msg):
        # rospy.loginfo("LAT_ACC called")
        closest = self.calcClosestTrajectory(msg)
        self.v_arr[LAT_ACC] = msg.points[closest].twist.linear.x
        return

    def CallBackScenarioTrajectory(self, msg):
        # rospy.loginfo("VELOCITY_OPTIMIZE called")
        closest = self.calcClosestTrajectory(msg)
        self.v_arr[VELOCITY_OPTIMIZE] = msg.points[closest].twist.linear.x
        return

    def CallBackControlCmd(self, msg):
        # rospy.loginfo("CONTROL_CMD called")
        self.v_arr[CONTROL_CMD] = msg.control.velocity
        return

    def CallBackVehicleCmd(self, msg):
        # rospy.loginfo("VEHICLE_CMD called")
        self.v_arr[VEHICLE_CMD] = msg.control.velocity
        return




    def calcClosestPath(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcClosestPathWLid(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].point.pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcClosestTrajectory(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcSquaredDist2d(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return dx * dx + dy * dy

    # def getSelfPos(self, from_link, to_link):
    #     trans, quat = self.getPose(from_link=from_link, to_link=to_link)
    #     rot = tf.transformations.euler_from_quaternion(quat)
        # self.self_x = trans[0]  # [m]
        # self.self_y = trans[1]  # [m]
        # self.self_th = np.rad2deg(rot[2])  # [deg]

    def updatePose(self, from_link, to_link):
        try:
            self.tfl.waitForTransform(from_link, to_link, rospy.Time(0), rospy.Duration(0.2))
            (trans, quat) = self.tfl.lookupTransform(from_link, to_link, rospy.Time(0))  # parent, child
            self.self_pose.position.x = trans[0]
            self.self_pose.position.y = trans[1]
            self.self_pose.position.z = trans[2]
            self.self_pose.orientation.x = quat[0]
            self.self_pose.orientation.y = quat[1]
            self.self_pose.orientation.z = quat[2]
            self.self_pose.orientation.w = quat[3]
            return
        except:
            trans = (0.0, 0.0, 0.0)
            quat = (0.0, 0.0, 0.0, 1.0)
            rospy.logwarn("cannot get position")
            return



def main():
    rospy.init_node("velocity_checker")
    VelocityChecker()
    rospy.spin()


if __name__ == "__main__":
    main()
