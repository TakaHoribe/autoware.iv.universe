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

REF_LINK  = "viewer"
SELF_LINK = "base_link"

class Murakami:
    def __init__(self):
        rospy.init_node("murakami_test")

        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        # self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_engage = rospy.Publisher("/autoware/engage", Bool, queue_size=1)
            
        self.tfl = tf.TransformListener()
        time.sleep(1.0)  # wait for ready to publish/subscribe
        self.publish_engage(False)
        trans, quat = self.get_pose(from_link=REF_LINK, to_link=SELF_LINK)
        rot = tf.transformations.euler_from_quaternion(quat)
        self.publish_initialize(222222220, 0, 0) # reset obstacle
        time.sleep(0.5)  # wait for ready to publish/subscribe
        self.publish_initialize(trans[0], trans[1], rot[2])

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

    def publish_initialize(self, x_s, y_s, th_s):
        posewcs = self.make_posstmp_with_cov((x_s, y_s, th_s))
        self.pub_initialpose.publish(posewcs)

    def publish_engage(self, engage):
        engage_msg = Bool(engage)
        self.pub_engage.publish(engage_msg)

    def make_posstmp_with_cov(
        self, pose, xcov=0.25, ycov=0.25, thcov=0.07, frame_id=REF_LINK
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

    def make_posstmp(self, pose, frame_id=REF_LINK):
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
    # rospy.spin()
    # rospy.spi
