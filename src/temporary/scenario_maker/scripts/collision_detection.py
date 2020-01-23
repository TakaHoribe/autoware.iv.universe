#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class CollisionDetection:
    def __init__(self):
        self.wheel_radius = rospy.get_param("wheel_radius", 0.39)
        self.wheel_width = rospy.get_param("wheel_width", 0.42)
        self.wheel_base = rospy.get_param("wheel_base", 2.79)  # between front wheel center and rear wheel center
        self.wheel_tread = rospy.get_param("wheel_tread", 1.63)  # between left wheel center and right wheel center
        self.front_overhang = rospy.get_param("front_overhang", 1.29)  # between front wheel center and vehicle front
        self.rear_overhang = rospy.get_param("rear_overhang", 1.1)  # between rear wheel center and vehicle rear
        self.vehicle_height = rospy.get_param("vehicle_height", 2.5)
        self.margin = rospy.get_param("margin", 0.1)  # [m]

        self.front_from_base = self.front_overhang + self.wheel_base + self.margin
        self.rear_from_base = -(self.rear_overhang + self.margin)
        self.left_from_base = self.wheel_tread / 2.0 + self.wheel_width / 2.0
        self.right_from_base = -self.wheel_tread / 2.0 + self.wheel_width / 2.0
        self.bottom_from_base = 0.0
        self.top_from_base = self.vehicle_height

        self.pcl_msg = None

        self.sub_pcl = rospy.Subscriber(
            "/sensing/preprocess/lidar/no_ground/pointcloud",
            PointCloud2,
            self.CallBackPointCloud,
            queue_size=1,
            tcp_nodelay=True,
        )  # for get pcl to detect collision

        self.pub_collision_detection_result = rospy.Publisher("/collsion_detection_result", Bool, queue_size=1)

        r = rospy.Rate(10)  # 10Hz
        # wait for subscribe pcl
        while self.pcl_msg is None:
            r.sleep()

        while not rospy.is_shutdown():
            collision = self.CollisionDetection()
            self.PubCollsionResult(collision)
            r.sleep()

    def CallBackPointCloud(self, pclmsg):
        self.pcl_msg = pclmsg

    def GetPcl(self):
        pointcloud = []
        for point in pc2.read_points(self.pcl_msg):
            x = point[0]
            y = point[1]
            z = point[2]
            pointcloud.append([x, y, z])
        return np.array(pointcloud)

    def CollisionDetection(self):
        pointcloud = self.GetPcl()
        if len(pointcloud) == 0:
            return False
        # collision detection ---> front >= point >= rear ---> true
        x_col = (self.front_from_base >= pointcloud[:, 0]) * (pointcloud[:, 0] >= self.rear_from_base)
        y_col = (self.left_from_base >= pointcloud[:, 1]) * (pointcloud[:, 1] >= self.right_from_base)
        z_col = (self.top_from_base >= pointcloud[:, 2]) * (pointcloud[:, 2] >= self.bottom_from_base)
        col = x_col * y_col * z_col  # x_col and y_col and z_col
        if np.sum(col) > 0:  # if pointcloud has collision points
            return True
        else:
            return False

    def PubCollsionResult(self, result):
        boolmsg = Bool()
        boolmsg.data = result
        self.pub_collision_detection_result.publish(result)


def main():
    rospy.init_node("collision_detection")
    CollisionDetection()


if __name__ == "__main__":
    main()
