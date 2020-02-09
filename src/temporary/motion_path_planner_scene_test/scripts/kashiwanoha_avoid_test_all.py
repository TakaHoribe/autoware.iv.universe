#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import subprocess
import math
import subprocess
import sys
import time
import csv

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import Bool, Float32, Header, Int32

ROSBUG_ID_FIRST = 0
ROSBUG_ID_END = 5
ENABLE_COLLISION_CHECK = True

GOAL_DIST_THRESHOLD = 3.0
GOAL_YAW_THRESHOLD_RAD = 100
TIMEOUT_THR_SEC = 30.0
ROSBAG_PUB_DURATION_SEC = 15.0

REF_LINK = "map"
SELF_LINK = "base_link"

class Pos:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x     # [m]
        self.y = y     # [m]
        self.yaw = yaw # [rad]
        

class AvoidTest:
    def __init__(self):
        rospy.init_node("murakami_test")
        time.sleep(1.0)  # wait for rospy

        self.self_pose = Pos()

        self.finish_reasons = {"goal", "collision", "timeout"}
        self.current_scene_id = ROSBUG_ID_FIRST
        self.publish_rosbag = True
        self.current_start_pose = Pos()

        self.collision = False
        self.goal_dist_thr = GOAL_DIST_THRESHOLD
        self.goal_yaw_thr = GOAL_YAW_THRESHOLD_RAD
        self.rosbag_pub_duration_sec = ROSBAG_PUB_DURATION_SEC
        self.prev_pub_rosbag_time = 0.0
        rospy.Timer(rospy.Duration(1), self.timerCallback)

        self.tfl = tf.TransformListener()  # for get self-position
        

        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_engage = rospy.Publisher("/autoware/engage", Bool, queue_size=1)
        self.sub_collsion = rospy.Subscriber("/collsion_detection_result", Bool, self.callback_collision, queue_size=1)

        time.sleep(1.0)  # wait for ready to publish/subscribe

        self.report = open('./report.csv',  mode='w')
        self.report.write('rosbag id, result, detail\n')
        # self.report.save('report.csv')


        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        print('!!!!! start sscenario test all !!!!!')
        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n')

        self.run_test_all()

        # self.report.save('report.csv')
        self.report.close()


    def timerCallback(self, event):
        self.pub_rosbag_with_duration(self.rosbag_pub_duration_sec)
        return
    
    def pub_rosbag_with_duration(self, duration):
        if self.publish_rosbag:
            now_time = rospy.Time.now().to_sec()
            if now_time - self.prev_pub_rosbag_time > duration:
                rosbag_play = 'rosbag play ./scene/' + str(self.current_scene_id) + '.bag --wait-for-subscribers -r 100'
                # print('run : ' + rosbag_play)
                subprocess.check_output(rosbag_play.split())
                self.prev_pub_rosbag_time = now_time


    def run_test_all(self):

        for current_scene_id in range(ROSBUG_ID_FIRST, ROSBUG_ID_END + 1):
            self.current_scene_id = current_scene_id

            start_pose = Pos()
            goal_pose = Pos()
            if 0 <= current_scene_id < 2:
                start_pose = Pos(3698.361, 73761.133, 0.495)
                goal_pose = Pos(3800.940, 73813.547, 0.453)
            elif 2 <= current_scene_id < 6:
                start_pose = Pos(3788.597, 73807.820, 0.487)
                goal_pose = Pos(3812.777, 73770.305, -2.71)
            else:
                print('rosbag id is out of scope')

            print '----- start test. id : ', current_scene_id, ' -----'
            # sys.stdout.write("----- start test. id : " + str(current_scene_id) + " -----")
            
            self.current_start_pose = start_pose

            res = self.run_test(start_pose, goal_pose)


            if res == 'goal':
                print '===== test result id :', current_scene_id, ' =====> SUCCESS (goal)\n'
                self.report.write(str(current_scene_id) + ', success, goal\n')
            if res == 'collision':
                print '===== test result id :', current_scene_id, ' =====> FAIL (collision)\n'
                self.report.write(str(current_scene_id) + ', fail, collision\n')
            if res == 'timeout':
                print '===== test result id :', current_scene_id, ' =====> FAIL (timeout)\n'
                self.report.write(str(current_scene_id) + ', fail, timeout\n')

            time.sleep(0.1)  # wait for initialpose

        self.publish_rosbag = False
            

      

    def run_test(self, start_pose, goal_pose):

        self.reset_obstacles()
        time.sleep(0.5)  # wait for initialpose
        self.publish_initialize(start_pose)
        time.sleep(1.0)  # wait for initialpose
        self.publish_goal(goal_pose)
        self.publish_engage(True)
        self.prev_pub_rosbag_time = 0.0
        
        start_time = rospy.Time.now().to_sec()

        loop_rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update_self_pose()
            if (self.collision):
                reason = 'collision'
                break
            if (self.goal_judge(goal_pose)):
                reason = 'goal'
                break
            if self.timeout_judge(start_time, TIMEOUT_THR_SEC):
                reason = 'timeout'
                break
            loop_rate.sleep()

        return reason

    def outputResult(self):
        return
        

    def reset_obstacles(self):
        self.publish_initialize(Pos(10000, 10000, 0)) # reset
        time.sleep(0.5)
        self.publish_initialize(self.current_start_pose)     

    def publish_initialize(self, pose):
        posewcs = self.make_posestamped_with_cov(pose)
        # print('pub initial pose!!, x:', posewcs.pose.pose.position.x, ', y: ', posewcs.pose.pose.position.y)
        self.pub_initialpose.publish(posewcs)

    def publish_goal(self, goal):
        pose = self.make_posestamped(goal)
        self.pub_goal.publish(pose)

    def publish_engage(self, engage):
        engage_msg = Bool(engage)
        self.pub_engage.publish(engage_msg)

    def make_posestamped_with_cov(
        self, pose, xcov=0.25, ycov=0.25, yawcov=0.07, frame_id="map"
    ):  # pose: x[m], y[m], yaw[rad]
        posewcmsg = PoseWithCovarianceStamped()
        posewcmsg.header.stamp = rospy.Time.now()
        posewcmsg.header.frame_id = frame_id
        posewcmsg.pose.pose = self.make_pose(pose)
        posewcmsg.pose.covariance[0] = xcov  # cov of x,x
        posewcmsg.pose.covariance[7] = ycov  # cov of y, y
        posewcmsg.pose.covariance[35] = yawcov  # cov of rot_z, rot_z
        return posewcmsg

    def make_posestamped(self, pose, frame_id="map"):
        posemsg = PoseStamped()
        posemsg.header.stamp = rospy.Time.now()
        posemsg.header.frame_id = frame_id
        posemsg.pose = self.make_pose(pose)
        return posemsg

    def make_pose(self, pose):  # x[m], y[m], yaw[rad]
        posemsg = Pose()
        posemsg.position.x = pose.x
        posemsg.position.y = pose.y
        yaw_rad = pose.yaw
        q = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        posemsg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return posemsg


    def timeout_judge(self, start_time, give_up_time):
        now_time = rospy.Time.now().to_sec()
        scenario_time = now_time - start_time
        if scenario_time > give_up_time:
            return True
        else:
            return False


    def update_self_pose(self):
        trans, quat = self.get_pose(from_link=REF_LINK, to_link=SELF_LINK)
        rot = tf.transformations.euler_from_quaternion(quat)
        self.self_pose.x = trans[0]  # [m]
        self.self_pose.y = trans[1]  # [m]
        self.self_pose.yaw = rot[2]  # [rad]

    def get_pose(self, from_link, to_link):
        try:
            self.tfl.waitForTransform(from_link, to_link, rospy.Time(0), rospy.Duration(0.2))
            (trans, quat) = self.tfl.lookupTransform(from_link, to_link, rospy.Time(0))  # parent, child
            return trans, quat
        except:
            trans = (0.0, 0.0, 0.0)
            quat = (0.0, 0.0, 0.0, 1.0)
            rospy.logwarn("cannot get position. From %s, to %s", from_link, to_link)
            return trans, quat

    def goal_judge(self, goal_pose):
        now_dist, now_yaw_dist = self.calc_dist(self.self_pose, goal_pose)
        if now_dist <= self.goal_dist_thr and now_yaw_dist < self.goal_yaw_thr:
            return True
        else:
            return False

    def calc_dist(self, pos1, pos2):
        xy_dist = np.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)
        th_dist = np.abs(pos1.yaw - pos2.yaw) % 6.28
        return xy_dist, th_dist

    def callback_collision(self, colmsg):
        if not ENABLE_COLLISION_CHECK:
            return
        if colmsg.data:
            self.collision = colmsg.data

if __name__ == "__main__":

    avoid_test = AvoidTest()
    rospy.spin()
