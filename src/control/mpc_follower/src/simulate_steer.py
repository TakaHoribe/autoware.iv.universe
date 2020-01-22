#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from autoware_control_msgs.msg import VehicleCommandStamped
from autoware_vehicle_msgs.msg import VehicleStatusStamped

delay_compensation_time = 0.45 #TODO: ->rosparam (subscribe?)
steer_tau = 0.01 #TODO ->rosparam (subscribe?)

class SimulateSteer():

    def __init__(self):
        self.current_steer = None
        self.current_time = None
        self.command_array = []

        self.subcmd = rospy.Subscriber(
            "/control/vehicle_cmd", VehicleCommandStamped, self.CallBackVehicleCmd, queue_size=1, tcp_nodelay=True)

        self.substatus = rospy.Subscriber(
            "/vehicle/status", VehicleStatusStamped, self.CallBackVehicleStatus, queue_size=1, tcp_nodelay=True)

        self.pubstatus = rospy.Publisher(
            "/vehicle_debug/status", VehicleStatusStamped, queue_size=1)

    def CallBackVehicleCmd(self, cmdmsg):
        steercmd = cmdmsg.command.control.steering_angle
        self.StackCurrentSteer(steercmd, rospy.Time.now().to_sec())
        self.CalcCurrentSteer()

    def CallBackVehicleStatus(self, statusmsg):
        steer = statusmsg.status.steering_angle
        if self.current_time is None:
            self.current_steer = steer
            self.current_time = rospy.Time.now().to_sec()

    def StackCurrentSteer(self, steercmd, current_time):
        self.command_array.append([steercmd, current_time+delay_compensation_time])

        while(True):#remove unused command
            if len(self.command_array)<2 or self.current_time is None:
                break
            next_oldest_cmd = self.command_array[1]#oldest_cmd = self.command_array[0]
            cmd_time = next_oldest_cmd[1]
            if self.current_time  > cmd_time:
                self.command_array.pop(0)#pop oldest_cmd
            else:
                break

    def CalcCurrentSteer(self):
        if self.current_time is None:
            #no ready
            return False

        target_time = rospy.Time.now().to_sec()
        if target_time < self.current_time:
            #error case
            return False

        #calculation
        mpc_calc_time = self.current_time

        array_oldest_time = self.command_array[0][1]
        if mpc_calc_time < array_oldest_time:
            #not ready 
            self.current_time = None

        steer = self.current_steer
        for i in range(0, len(self.command_array)-1):
            command_steer = self.command_array[i][0]
            next_command_time = self.command_array[i][1]
            if next_command_time < mpc_calc_time:
                pass
            else:
                if next_command_time > target_time:
                    steer = self.OneStepForwardSteer(steer, command_steer, mpc_calc_time, target_time)
                    mpc_calc_time = target_time
                    break#finish calculation
                else:
                    steer = self.OneStepForwardSteer(steer, command_steer, mpc_calc_time, next_command_time)
                    mpc_calc_time =  next_command_time

        self.UpdateState(steer, target_time)
        self.PublishSteer()

    def OneStepForwardSteer(self, steer, cmd_steer, current_time, target_time):
        diff_time = target_time - current_time
        if diff_time < 0:
            print("Error")
            return steer
        else:
            #next_steer = steer + (cmd_steer - steer) * (diff_time / steer_tau) #1d approximation
            next_steer = steer + (cmd_steer - steer) * (1 - math.exp(-diff_time/steer_tau)) #exponential
            return next_steer

    def UpdateState(self, steer, steer_time):
        self.current_steer = steer
        self.current_time = steer_time

    def PublishSteer(self):
        vsmsg = VehicleStatusStamped()
        vsmsg.header.stamp = rospy.Time.now()#self.current_time?
        vsmsg.status.steering_angle = self.current_steer
        self.pubstatus.publish(vsmsg)

def main():
    rospy.init_node("simulate_mpc_steer")
    SimulateSteer()
    rospy.spin()


if __name__ == "__main__":
    main()