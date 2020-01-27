#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################
##<Warning>This is temporary file.##
####################################

import math

import numpy as np
import rospy
import tf
from autoware_msgs.msg import VehicleCmd
from autoware_vehicle_msgs.msg import Steering, VehicleCommandStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2

WHEEL_BASE = 2.9  # [m]
CAMERA_LINK = "traffic_light/camera_link"


class LgsvlConnectPublisher:
    def __init__(self):
        self.v_offset_rate = rospy.get_param("v_offset_rate", 1.0)  # [rate]
        self.w_offset_rate = rospy.get_param("w_offset_rate", 1.0)  # [rate]
        self.max_steering_velocity = rospy.get_param("max_steering_velocity", 1.0)  # [rad/s]

        self.v = 0.0
        self.last_steer = 0.0
        self.last_command_time = rospy.Time.now().to_sec()
        self.tfb = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()

        # for Problem1: publish /vehicle/status/twist
        self.subodom = rospy.Subscriber("/odom", Odometry, self.CallBackOdom, queue_size=1, tcp_nodelay=True)
        self.pubtwist = rospy.Publisher("/vehicle/status/twist", TwistStamped, queue_size=1)

        # for Problem2: rewrite timestamp of /points_raw
        self.subpcl = rospy.Subscriber("/points_raw", PointCloud2, self.CallBackPcl, queue_size=1, tcp_nodelay=True)
        self.pubpcl = rospy.Publisher("/points_raw_tmp", PointCloud2, queue_size=1)

        # for Problem3: publish cmd for LG-simulator
        self.pubcmd = rospy.Publisher("/vehicle_cmd", VehicleCmd, queue_size=1)
        self.subcmd = rospy.Subscriber(
            "/control/vehicle_cmd", VehicleCommandStamped, self.CallBackVehicleCmd, queue_size=1, tcp_nodelay=True
        )

        # for Problem4: publish /vehicle/status/steering
        self.pubsteering = rospy.Publisher("/vehicle/status/steering", Steering, queue_size=1)

        # for Problem5: rewrite timestamp and frame_id of /img
        self.subimg = rospy.Subscriber("/tmp_img", Image, self.CallBackImage, queue_size=1, tcp_nodelay=True)

        self.pubimg = rospy.Publisher("/tmp_img_lgsvl", Image, queue_size=1)

    def CallBackOdom(self, odmmsg):  # Problem 1: /vehicle/status/steering is not published.
        twistmsg = TwistStamped()
        twistmsg.header.stamp = rospy.Time.now()
        twistmsg.header.frame_id = "base_link"
        twistmsg.twist = odmmsg.twist.twist
        twistmsg.twist.linear.x *= self.v_offset_rate
        twistmsg.twist.angular.z *= self.w_offset_rate
        self.pubtwist.publish(twistmsg)
        self.PubVehicleSteering(v=odmmsg.twist.twist.linear.x, w=odmmsg.twist.twist.angular.z)
        self.v = odmmsg.twist.twist.linear.x

        """
        #temporary tf
        self.tfb.sendTransform((odmmsg.pose.pose.position.x, odmmsg.pose.pose.position.y, odmmsg.pose.pose.position.z),
                         (odmmsg.pose.pose.orientation.x, odmmsg.pose.pose.orientation.y, odmmsg.pose.pose.orientation.z, odmmsg.pose.pose.orientation.w),
                         rospy.Time.now(),
                         "tmp_base_link", #child
                         "tmp_gps") #parent
        (trans, quat) = self.tfl.lookupTransform('tmp_base_link','tmp_gps',rospy.Time(0)) #parent, child
        
        self.tfb.sendTransform(trans,
                               quat,
                               rospy.Time.now(),
                               "tmp_gps2",
                               "base_link")
        (trans2, quat2) = self.tfl.lookupTransform('map','tmp_gps2',rospy.Time(0)) #parent, child
        
        self.tfb.sendTransform(trans2,
                               quat2,
                               rospy.Time.now(),
                               "gps",
                               "map")
        """

    def CallBackPcl(self, pclmsg):  # Problem2: timestamp of /points_raw is too old
        pclmsg.header.stamp = rospy.Time.now()
        self.pubpcl.publish(pclmsg)

    def CallBackVehicleCmd(
        self, vcmdmsg
    ):  # Problem3: The output msg type of autoware is different from the input msg type of LG-simulator
        cmdmsg = VehicleCmd()
        cmdmsg.header.stamp = rospy.Time.now()
        cmdmsg.header.frame_id = "base_link"
        cmdmsg.twist_cmd.header.stamp = rospy.Time.now()
        # cmdmsg.twist_cmd.twist.linear.x = vcmdmsg.command.control.velocity#Velocity. Fine.
        cmdmsg.twist_cmd.twist.linear.x = vcmdmsg.command.control.acceleration / 4.0 + (
            self.v / self.v_offset_rate
        )  # accel + v ????????f**k! a/4.0????

        steer = self.cramp_steering(
            vcmdmsg.command.control.steering_angle
        )  # prevent from sudden change of steering in LG-simulator

        # Angular Velocity
        if self.v != 0:
            # cmdmsg.twist_cmd.twist.angular.z = math.tan(vcmdmsg.command.control.steering_angle)
            cmdmsg.twist_cmd.twist.angular.z = steer * 0.5 / self.w_offset_rate
        else:
            cmdmsg.twist_cmd.twist.angular.z = 0.0

        cmdmsg.ctrl_cmd.linear_velocity = vcmdmsg.command.control.velocity  # Velocity
        cmdmsg.ctrl_cmd.linear_acceleration = vcmdmsg.command.control.acceleration  # accel
        cmdmsg.ctrl_cmd.steering_angle = steer  # steer
        cmdmsg.emergency = vcmdmsg.command.emergency
        self.pubcmd.publish(cmdmsg)

    def cramp_steering(self, steer):
        dt = rospy.Time.now().to_sec() - self.last_command_time
        max_steer_delta = dt * self.max_steering_velocity
        if np.abs(steer - self.last_steer) > max_steer_delta:
            cramped_steer = self.last_steer + max_steer_delta * np.sign(steer - self.last_steer)
        else:
            cramped_steer = steer

        self.last_steer = cramped_steer
        self.last_command_time = rospy.Time.now().to_sec()
        return cramped_steer

    def PubVehicleSteering(self, v, w):  # Problem4: /vehicle/status/steering is not published
        strmsg = Steering()
        strmsg.header.stamp = rospy.Time.now()
        strmsg.header.frame_id = "base_link"
        if v != 0:
            v = max(0.5, v)  # very small v is bad effect for steer
            strmsg.data = math.atan(WHEEL_BASE * w / v)

        else:
            strmsg.data = 0.0
        self.pubsteering.publish(strmsg)

    def CallBackImage(self, imgmsg):
        imgmsg.header.stamp = rospy.Time.now()
        imgmsg.header.frame_id = CAMERA_LINK
        self.pubimg.publish(imgmsg)


def main():
    rospy.init_node("lgsvl_connect_publisher")
    LgsvlConnectPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
