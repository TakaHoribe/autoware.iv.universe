#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################
##<Warning>This is temporary file.##
####################################

import math
import os
import subprocess
import time
import numpy as np
import rospy
import roslib
import tf
import signal
from autoware_msgs.msg import VehicleCmd
from autoware_vehicle_msgs.msg import Steering, VehicleCommand, Shift
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2

WHEEL_BASE = 2.9  # [m]
CAMERA_LINK = "traffic_light/camera_link"


# geer
REVERSE = 2
DRIVE = 4


class LgsvlConnectPublisher:
    def __init__(self):
        self.v_offset_rate = rospy.get_param("v_offset_rate", 1.0)  # [rate] actual_velocity/velocity_from_simulator
        self.w_offset_rate = rospy.get_param(
            "w_offset_rate", 1.0
        )  # [rate] actual_angular_velocity/angular_velocity_from_simulator
        self.accel_command_offset_rate = rospy.get_param(
            "accel_offset_rate", 0.22
        )  # [rate] accel command/accel behavior in simulator
        self.steer_command_offset_rate = rospy.get_param(
            "steer_offset_rate", 0.28
        )  # [rate] steer command/steer behavior in simulator
        self.max_steering_velocity = rospy.get_param("max_steering_velocity", 2.0)  # [rad/s]
        self.throttle_brake_mode = rospy.get_param("throttle_brake_mode", True)  # [rad/s]
        self.is_warp_from_initialpose = rospy.get_param("is_wrap_from_initialpose", True)

        self.lg_process = None
        self.last_time_of_warp = None
        self.v = 0.0
        self.last_steer = 0.0  # last published steer
        self.last_received_steer = 0.0  # last received stere(state)
        self.last_command_time = rospy.Time.now().to_sec()
        self.shift = DRIVE
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
            "/control/vehicle_cmd", VehicleCommand, self.CallBackVehicleCmd, queue_size=1, tcp_nodelay=True
        )

        # for Problem4: publish /vehicle/status/steering
        self.pubsteering = rospy.Publisher("/vehicle/status/steering", Steering, queue_size=1)

        # for Problem5: rewrite timestamp and frame_id of /img
        self.subimg = rospy.Subscriber("/tmp_img", Image, self.CallBackImage, queue_size=1, tcp_nodelay=True)

        self.pubimg = rospy.Publisher("/tmp_img_lgsvl", Image, queue_size=1)

        # Warp from initialpose
        self.subpose = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.CallBackPose, queue_size=1, tcp_nodelay=True
        )
        self.pubpose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

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
        # temporary tf
        try:
            self.tfb.sendTransform(
                (odmmsg.pose.pose.position.x, odmmsg.pose.pose.position.y, odmmsg.pose.pose.position.z),
                (
                    odmmsg.pose.pose.orientation.x * 0.0,
                    odmmsg.pose.pose.orientation.z * 0.0,  # reverse
                    odmmsg.pose.pose.orientation.y,  # reverse
                    odmmsg.pose.pose.orientation.w,
                ),
                rospy.Time.now(),
                "tmp_base_link",  # child
                "tmp_gps",  # parent
            )
            (trans, quat) = self.tfl.lookupTransform("tmp_base_link", "tmp_gps", rospy.Time(0))  # parent, child

            self.tfb.sendTransform(trans, quat, rospy.Time.now(), "tmp_gps2", "base_link")
            (trans2, quat2) = self.tfl.lookupTransform("map", "tmp_gps2", rospy.Time(0))  # parent, child

            self.tfb.sendTransform(trans2, quat2, rospy.Time.now(), "gps", "map")
        except:
            pass
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
        # cmdmsg.twist_cmd.twist.linear.x = vcmdmsg.control.velocity#Velocity. Fine.
        cmdmsg.twist_cmd.twist.linear.x = vcmdmsg.control.acceleration * self.accel_command_offset_rate + (
            self.v / self.v_offset_rate
        )

        steer = self.cramp_steering(
            vcmdmsg.control.steering_angle
        )  # prevent from sudden change of steering in LG-simulator

        # Angular Velocity
        if self.v != 0:
            # cmdmsg.twist_cmd.twist.angular.z = math.tan(vcmdmsg.control.steering_angle)
            cmdmsg.twist_cmd.twist.angular.z = steer * (self.steer_command_offset_rate * 2.0) / self.w_offset_rate
        else:
            cmdmsg.twist_cmd.twist.angular.z = 0.0

        cmdmsg.ctrl_cmd.linear_velocity = vcmdmsg.control.velocity  # Velocity(no use)
        cmdmsg.ctrl_cmd.linear_acceleration = (
            vcmdmsg.control.acceleration * self.accel_command_offset_rate
        )  # accel
        cmdmsg.ctrl_cmd.steering_angle = -steer * self.steer_command_offset_rate  # steer
        cmdmsg.emergency = vcmdmsg.emergency

        if self.throttle_brake_mode:
            if vcmdmsg.shift == REVERSE:
                cmdmsg.gear = 63  # reverse
            else:
                cmdmsg.gear = 64  # drive

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
        if np.abs(v) > 0.1:
            strmsg.data = math.atan(WHEEL_BASE * w / v)
            self.last_received_steer = strmsg.data
        else:
            strmsg.data = self.last_received_steer
        self.pubsteering.publish(strmsg)

    def CallBackImage(self, imgmsg):
        imgmsg.header.stamp = rospy.Time.now()
        imgmsg.header.frame_id = CAMERA_LINK
        self.pubimg.publish(imgmsg)

    def CallBackPose(self, posemsg):
        if not self.is_warp_from_initialpose:
            return

        if self.last_time_of_warp is not None:
            now = rospy.Time.now().to_sec()
            if now - self.last_time_of_warp < 3.0:  # avoid frequent warp
                return

        if posemsg.header.frame_id == "map":
            x = posemsg.pose.pose.position.x
            y = posemsg.pose.pose.position.y
            q = posemsg.pose.pose.orientation
            rot_z = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            self.warp_in_lgsvl(x, y, rot_z)  # temporary
            self.last_time_of_warp = rospy.Time.now().to_sec()
            time.sleep(1)
            self.pubpose.publish(posemsg)  # reinitializaion after warp in simulation
        else:
            rospy.loginfo("frame_id of posemsg must be map")

    def warp_in_lgsvl(self, x, y, theta, map_name="kashiwanoha", vehicle_name="TierIVLexus"):  # temporary
        if self.lg_process is not None:
            killcmd = "kill -9 " + str(self.lg_process.pid)
            os.system(killcmd)
        pyfile = str(roslib.packages.get_pkg_dir("lgsvl_connector") + "/scripts/lg_generator.py")
        self_spawn_cmd = ["python3", pyfile, map_name, vehicle_name, str(x), str(y), str(theta)]
        self.lg_process = subprocess.Popen(self_spawn_cmd, shell=False)


def main():
    rospy.init_node("lgsvl_connect_publisher")
    LgsvlConnectPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
