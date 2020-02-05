#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import math
import subprocess
import sys
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from std_msgs.msg import Bool, Float32, Header, Int32


class Murakami:
    def __init__(self):
        rospy.init_node("murakami_test")

        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_engage = rospy.Publisher("/autoware/engage", Bool, queue_size=1)
        
        time.sleep(1.0)  # wait for ready to publish/subscribe
        self.publish_engage(False)
        self.publish_initialize(20000.259, 13.462, 0.477) # reset
        time.sleep(0.5)  # wait for ready to publish/subscribe
        self.publish_initialize(-138.259, 13.462, 0.477)

        time.sleep(0.5)  # wait for ready to publish/subscribe
        self.publish_goal(-58.820, -29.568, -1.116)


        self.play_rosbag()
        
        time.sleep(0.5)  # wait for ready to publish/subscribe
        self.publish_engage(False)

        print('done')
    def play_rosbag(self):
        args = sys.argv
        print(len(args))
        if len(args) > 1:
            scene_number = args[1]
            print('aaa')
            rosbag_play = 'rosbag play ./scene/' + scene_number + '.bag --wait-for-subscribers -i'
            os.system(rosbag_play)
        else:
            print('nosbag id is not set!!')

    def publish_initialize(self, x_s, y_s, th_s):
        posewcs = self.make_posstmp_with_cov((x_s, y_s, th_s))
        self.pub_initialpose.publish(posewcs)

    def publish_goal(self, x_g, y_g, th_g):
        pose = self.make_posstmp((x_g, y_g, th_g))
        self.pub_goal.publish(pose)

    def publish_engage(self, engage):
        engage_msg = Bool(engage)
        self.pub_engage.publish(engage_msg)

    def make_posstmp_with_cov(
        self, pose, xcov=0.25, ycov=0.25, thcov=0.07, frame_id="world"
    ):  # pose: x[m], y[m], th[deg]
        x = pose[0]
        y = pose[1]
        th = pose[2]
        posewcmsg = PoseWithCovarianceStamped()
        posewcmsg.header.stamp = rospy.Time.now()
        posewcmsg.header.frame_id = frame_id
        posewcmsg.pose.pose = self.make_pose(x, y, th)
        posewcmsg.pose.covariance[0] = xcov  # cov of x,x
        posewcmsg.pose.covariance[7] = ycov  # cov of y, y
        posewcmsg.pose.covariance[35] = thcov  # cov of rot_z, rot_z
        return posewcmsg

    def make_posstmp(self, pose, frame_id="world"):
        x = pose[0]
        y = pose[1]
        th = pose[2]
        posemsg = PoseStamped()
        posemsg.header.stamp = rospy.Time.now()
        posemsg.header.frame_id = frame_id
        posemsg.pose = self.make_pose(x, y, th)
        return posemsg

    def make_pose(self, x, y, th):  # x[m], y[m], th[deg]
        posemsg = Pose()
        posemsg.position.x = x
        posemsg.position.y = y
        th_rad = th
        q = tf.transformations.quaternion_from_euler(0, 0, th_rad)
        posemsg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return posemsg

if __name__ == "__main__":



    mk = Murakami()
    rospy.spin()
