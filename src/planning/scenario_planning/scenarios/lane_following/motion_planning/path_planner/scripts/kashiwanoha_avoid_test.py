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


# def delete_rosbag(self, id, file_name):
#     # no error handling: TODO
#     rosbag_delete = "rm " + file_name + str(id) + ".bag"
#     subprocess.Popen(rosbag_delete, shell=True)

class Murakami(self):
    pass

    def __init__():
        rospy.init_node("horrible_murakami")
        # ScenarioMaker()
        # rospy.spin()
        os.system('rosbag play ./scene/1.bag')
        print('done')

        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.i(-138.259, 13.462, 0.477)
        self.g(-58.820, -29.568, -1.116)

    def i(self, x_s, y_s, th_s):
        posewcs = self.ake_posstmp_with_cov((x_s, y_s, th_s))
        self.pub_initialpose.publish(posewcs)

    def g(self, x_g, y_g, th_g):
        pose = self.make_pose(x_g, y_g, th_g)
        self.pub_goal.publish(pose)

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
