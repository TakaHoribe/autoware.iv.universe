#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from autoware_planning_msgs.msg import Trajectory
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryVisualizer():

    def __init__(self):
        self.trajectory_external_velocity_limitted = Trajectory()
        self.trajectory_lateral_acc_filtered = Trajectory()
        self.trajectory_raw = Trajectory()
        self.trajectory_time_resamped = Trajectory()
        self.trajectory_final = Trajectory()

        self.substatus1 = rospy.Subscriber("/planning/motion_planning/motion_velocity_planner_osqp/debug/trajectory_external_velocity_limitted", Trajectory, self.CallBackTrajExVelLim, queue_size=1, tcp_nodelay=True)
        self.substatus2 = rospy.Subscriber("/planning/motion_planning/motion_velocity_planner_osqp/debug/trajectory_lateral_acc_filtered", Trajectory, self.CallBackTrajLatAccFiltered, queue_size=1, tcp_nodelay=True)
        self.substatus3 = rospy.Subscriber("/planning/motion_planning/motion_velocity_planner_osqp/debug/trajectory_raw", Trajectory, self.CallBackTrajRaw, queue_size=1, tcp_nodelay=True)
        self.substatus4 = rospy.Subscriber("/planning/motion_planning/motion_velocity_planner_osqp/debug/trajectory_time_resampled", Trajectory,self.CallBackTrajTimeResampled, queue_size=1, tcp_nodelay=True)
        self.substatus5 = rospy.Subscriber("/planning/motion_planning/trajectory", Trajectory,self.CallBackTrajFinal, queue_size=1, tcp_nodelay=True)

    def CallBackTrajExVelLim(self, cmd):
        self.trajectory_external_velocity_limitted = cmd

    def CallBackTrajLatAccFiltered(self, cmd):
        self.trajectory_lateral_acc_filtered = cmd

    def CallBackTrajRaw(self, cmd):
        self.trajectory_raw = cmd

    def CallBackTrajTimeResampled(self, cmd):
        self.trajectory_time_resamped = cmd

    def CallBackTrajFinal(self, cmd):
        self.trajectory_final = cmd
        self.plotTrajectory()

    def CalcArcLength(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def ToVelList(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.twist.linear.x)
        return v_list

    def CalcAcceleration(self, traj):
        a_arr = []
        for i in range(1, len(traj.points) - 1):
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v = 0.5 * (v1 + v0)
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            dt = ds / max(abs(v), 0.001)
            a = (v1 - v0) / dt
            a_arr.append(a)
        if len(traj.points) > 0:
            a_arr.append(0)
            a_arr.append(0)
        return a_arr

    def CalcJerk(self, traj):
        j_arr = []
        for i in range(1, len(traj.points) - 2):
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            p2 = traj.points[i+1]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v2 = p2.twist.linear.x

            dx0 = p1.pose.position.x - p0.pose.position.x
            dy0 = p1.pose.position.y - p0.pose.position.y
            ds0 = np.sqrt(dx0**2 + dy0**2)

            dx1 = p2.pose.position.x - p1.pose.position.x
            dy1 = p2.pose.position.y - p1.pose.position.y
            ds1 = np.sqrt(dx1**2 + dy1**2)

            dt0 = ds0 / max(abs(0.5*(v1+v0)), 0.001)
            dt1 = ds1 / max(abs(0.5*(v2+v1)), 0.001)

            a0 = (v1 - v0) / max(dt0, 0.001)
            a1 = (v2 - v1) / max(dt1, 0.001)
            j = (a1 - a0) / max(dt1, 0.001)
            j_arr.append(j)
        if len(traj.points) > 0:
            j_arr.append(0)
            j_arr.append(0)
            j_arr.append(0)
        return j_arr

    def plotTrajectory(self):
        plt.clf()
        ax1 = plt.subplot(3,1,1)#row, col, index(<raw*col)
        ax1.plot(self.CalcArcLength(self.trajectory_raw), self.ToVelList(self.trajectory_raw), label="0raw")
        ax1.plot(self.CalcArcLength(self.trajectory_external_velocity_limitted), self.ToVelList(self.trajectory_external_velocity_limitted), label="1external_velocity_limitted")
        ax1.plot(self.CalcArcLength(self.trajectory_lateral_acc_filtered), self.ToVelList(self.trajectory_lateral_acc_filtered), label="2lateral_acc_filtered")
        ax1.plot(self.CalcArcLength(self.trajectory_time_resamped), self.ToVelList(self.trajectory_time_resamped), label="3time_resamped")
        ax1.plot(self.CalcArcLength(self.trajectory_final), self.ToVelList(self.trajectory_final), label="4final")
        ax1.set_title("sample2")
        ax1.legend()

        ax2 = plt.subplot(3,1,2)
        ax2.plot(self.CalcArcLength(self.trajectory_final), self.CalcAcceleration(self.trajectory_final), label="final")
        # ax2.legend()

        ax3 = plt.subplot(3,1,3)
        ax3.plot(self.CalcArcLength(self.trajectory_final), self.CalcJerk(self.trajectory_final), label="final")
        # ax3.legend()

        #plt.show()
        plt.pause(.01)


def main():
    rospy.init_node("trajectory_visualizer")
    TrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()