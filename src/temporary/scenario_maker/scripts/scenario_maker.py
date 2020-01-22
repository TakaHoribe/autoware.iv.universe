#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math, time
import numpy as np
import tf
from geometry_msgs.msg import Quaternion, Pose, Twist, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from std_msgs.msg import Bool, Header, Int32
from autoware_control_msgs.msg import VehicleStatusStamped

OBSTACLE_NUM = 10

class ScenarioMaker():

    def __init__(self):
        self.trial_num = 0

        self.start_time = 0
        self.obstacle_generated = np.zeros((OBSTACLE_NUM+1))
        self.obstacle_generated_time = np.zeros((OBSTACLE_NUM+1))

        self.self_x = 0.0
        self.self_y = 0.0
        self.self_th = 0.0
        self.self_vel = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_th = 0.0

        #TODO: ->rosparam
        self.goal_dist = 16.0#[m]
        self.goal_th = 180#[deg]
        self.goal_vel = 0.001#[m/s]
        self.give_up_time = 300#[s]

        self.tfl = tf.TransformListener() #for get self-position

        self.subvel = rospy.Subscriber(
            "/vehicle/status", VehicleStatusStamped, self.CallBackVehicleStatus, queue_size=1, tcp_nodelay=True)#for get self-velocity

        self.pubinitialpose = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.pubgoal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)

        self.pubcheckpoint = rospy.Publisher(
            "/checkpoint", PoseStamped, queue_size=1)

        self.pubengage = rospy.Publisher(
            "/autoware/engage", Bool, queue_size=1)

        self.pubpedestrianpose = rospy.Publisher(
            "/initial_pedestrian_pose", PoseStamped,  queue_size=1)

        self.pubpedestriantwist = rospy.Publisher(
            "/initial_pedestrian_twist", TwistStamped,  queue_size=1)

        self.pubcarpose = rospy.Publisher(
            "/initial_car_pose", PoseStamped,  queue_size=1)

        self.pubcartwist = rospy.Publisher(
            "/initial_car_twist", TwistStamped,  queue_size=1)

        self.pubobjectid = rospy.Publisher(
            "/object_id", Int32,  queue_size=1)

        self.pubresetobjectid = rospy.Publisher(
            "/reset_object_id", Int32,  queue_size=1)


        time.sleep(0.5)#wait for ready to publish/subscribe#TODO: fix this

        #set self-position and path
        self.scenario_path(only_initial_pose=True)
        time.sleep(1.0)
        self.reset_obstacle()
        time.sleep(1.0)
        self.scenario_obstacle()#obstacle is valid only when self-position is givenddd
        time.sleep(3.0)
        self.scenario_path()

        while not rospy.is_shutdown():
            self.GetSelfPos()
            if self.goal_judge(self.goal_dist, self.goal_th, self.goal_vel):
                print("Scenario Clear!", self.trial_num)
                self.retry_senario_path()
            elif self.give_up_judge(self.give_up_time):
                print("Failed...", self.trial_num)
                self.retry_senario_path()
            else:
                self.scenario_obstacle_manager()

    def retry_senario_path(self):
        self.pubengage.publish(False)#stop vehicle
        self.reset_obstacle()
        self.scenario_obstacle()
        time.sleep(5.0)
        self.trial_num += 1
        self.start_time = rospy.Time.now().to_sec()
        self.scenario_path()

    def scenario_path(self, only_initial_pose=False):
        self.pubengage.publish(False)#stop vehicle
        self.start_time = rospy.Time.now().to_sec()
        return self.scenario_path1(only_initial_pose)

    def scenario_obstacle(self):
        self.scenario_obstacle1()

    def scenario_obstacle_manager(self):
        self.scenario_obstacle_manager1()

    def scenario_path1(self, only_inital_pose=False):
        time.sleep(0.25)

        #publish Start Position
        self.pubinitialpose.publish(self.make_posstmp_with_cov(self.random_pose_maker(x=-139.1, y=-24.68, th=117.2, ver_sigma=0.5, lat_sigma=0.1, th_sigma=1.0)))
        time.sleep(0.25)
        if only_inital_pose:
            return

        #Publish Goal Position
        self.pubgoal.publish(self.make_posstmp(self.random_pose_maker(x=-117.8, y=4.553, th=117.2, ver_sigma=0.5, lat_sigma=0.1, th_sigma=1.0, goal_record=True)))
        time.sleep(0.25)

        #Publish Check Point
        self.pubcheckpoint.publish(self.make_posstmp(self.random_pose_maker(x=-60.3, y=-23.6, th=-62.8, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)))#checkpoint 1
        time.sleep(0.25)
        self.pubcheckpoint.publish(self.make_posstmp(self.random_pose_maker(x=1.4, y=5.4, th=117.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)))#checkpoint 2
        time.sleep(0.25)
        self.pubcheckpoint.publish(self.make_posstmp(self.random_pose_maker(x=-83.5, y=14.5, th=117.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)))#checkpoint 3
        time.sleep(0.25)
        self.pubcheckpoint.publish(self.make_posstmp(self.random_pose_maker(x=-43.5, y=64.2, th=27.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)))#checkpoint 4
        time.sleep(0.25)
        self.pubcheckpoint.publish(self.make_posstmp(self.random_pose_maker(x=-5.8, y=-19.8, th=-152.8, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)))#checkpoint 5
        time.sleep(0.25)
        self.pubcheckpoint.publish(self.make_posstmp(self.random_pose_maker(x=-119.1, y=-63.6, th=117.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)))#checkpoint 6
        time.sleep(0.25)

        #Publish Engage
        self.pubengage.publish(True)
        time.sleep(0.25)

    def scenario_obstacle1(self):
        ###obstacle 0: fixed pedestrian
        self.PubObjectId(0)
        time.sleep(0.25)
        self.PubObstacle(\
                        pose = self.random_pose_maker(x=-121.3, y=25.87, th=27.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0),\
                        vel = self.random_velocity_maker(v=0, v_sigma=0), \
                        obstacle_type="pedestrian", obstacle_id=0)

    def scenario_obstacle_manager1(self):
        #obstacle 1: obstacle car in lane change
        if self.obstacle_generate_judge_dist(-56.7, 57.3, 27.2, 18.0, 20.0):
            if self.obstacle_generated[1] == 0:
                self.PubObjectId(1)#TODO -> in PubObstacle
                time.sleep(0.25)#TODO -> in PubObstacle
                self.PubObstacle(\
                                pose = self.random_pose_maker(x=-56.7, y=57.3, th=27.2, ver_sigma=2.0, lat_sigma=0.0, th_sigma=0.0),\
                                vel = self.random_velocity_maker(v=5.0, v_sigma=1.5), \
                                obstacle_type="car", obstacle_id=1)
                self.obstacle_generated[1] = 1#TODO -> in PubObstacle
                self.obstacle_generated_time[1] = rospy.Time.now().to_sec()#TODO -> in PubObstacle
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[1] > 8.0:
                    self.reset_id_obstacle(1)
                    time.sleep(0.25)
                    self.obstacle_generated[1] = 1#no more publish(publish once)

        #obstacle 2: sudden pedestrian
        if self.obstacle_generate_judge_dist(-95.0, -67.0, -152.8, 25.0, 50.0):
            if self.obstacle_generated[2] == 0:
                self.PubObjectId(2)
                time.sleep(0.25)
                self.PubObstacle(\
                                pose = self.random_pose_maker(x=-95.0, y=-67.0, th=117.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=2.0),\
                                vel = self.random_velocity_maker(v=0.5, v_sigma=0.1), \
                                obstacle_type="pedestrian", obstacle_id=2)
                self.obstacle_generated[2] = 1
                self.obstacle_generated_time[2] = rospy.Time.now().to_sec()
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[2] > 12.0:
                    self.reset_id_obstacle(2)
                    time.sleep(0.25)
                    self.obstacle_generated[2] = 1#no more publish(publish once)

        #obstacle 3: crossing car 1(crossing with traffic light)
        if not self.obstacle_generate_judge_dist(-121.8, -25.6, 27.2, 45.0, 45.0):#if car is near the generate area, not genetate the obstacle
            if self.obstacle_generated[3] == 0:
                self.PubObjectId(3)
                time.sleep(0.25)
                self.PubObstacle(\
                                pose = self.random_pose_maker(x=-121.8, y=-25.6, th=27.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=0.0),\
                                vel = self.random_velocity_maker(v=8.0, v_sigma=1.0), \
                                obstacle_type="car", obstacle_id=3)
                self.obstacle_generated[3] = 1
                self.obstacle_generated_time[3] = rospy.Time.now().to_sec()
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[3] > 10.0:
                    self.reset_id_obstacle(3)
                    time.sleep(0.25)
                    self.PubObjectId(3)
                    time.sleep(0.25)
                    self.PubObstacle(\
                                    pose = self.random_pose_maker(x=-121.8, y=-25.6, th=27.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=0.0),\
                                    vel = self.random_velocity_maker(v=8.0, v_sigma=1.0), \
                                    obstacle_type="car", obstacle_id=3)
                    self.obstacle_generated[3] = 1
                    self.obstacle_generated_time[3] = rospy.Time.now().to_sec()

        #obstacle 4: crossing car 1(crossing without traffic light)
        if not self.obstacle_generate_judge_dist(-101.6, -65.2, 27.2, 20.0, 10.0):#if car is near the generate area, not genetate the obstacle
            if self.obstacle_generated[4] == 0:
                self.PubObjectId(4)
                time.sleep(0.25)
                self.PubObstacle(\
                                pose = self.random_pose_maker(x=-101.6, y=-65.2, th=27.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=0.0),\
                                vel = self.random_velocity_maker(v=8.0, v_sigma=1.0), \
                                obstacle_type="car", obstacle_id=4)
                self.obstacle_generated[4] = 1
                self.obstacle_generated_time[4] = rospy.Time.now().to_sec()
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[4] > 11.0:
                    self.reset_id_obstacle(4)
                    time.sleep(0.25)
                    self.PubObjectId(4)
                    time.sleep(0.25)
                    self.PubObstacle(\
                                    pose = self.random_pose_maker(x=-101.6, y=-65.2, th=27.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=0.0),\
                                    vel = self.random_velocity_maker(v=8.0, v_sigma=1.0), \
                                    obstacle_type="car", obstacle_id=4)
                    self.obstacle_generated[4] = 1
                    self.obstacle_generated_time[4] = rospy.Time.now().to_sec()

        #obstacle 5: crossing pedestrian(crossing with traffic light)
        if self.obstacle_generated[5] == 0:
            self.PubObjectId(5)
            time.sleep(0.25)
            self.PubObstacle(\
                            pose = self.random_pose_maker(x=-83.8, y=2.0, th=27.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=0.0),\
                            vel = self.random_velocity_maker(v=1.8, v_sigma=0.2), \
                            obstacle_type="pedestrian", obstacle_id=5)
            self.obstacle_generated[5] = 1
            self.obstacle_generated_time[5] = rospy.Time.now().to_sec()
        else:
            if rospy.Time.now().to_sec() - self.obstacle_generated_time[5] > 14.0:
                self.reset_id_obstacle(5)
                time.sleep(0.25)
                self.PubObjectId(5)
                time.sleep(0.25)
                self.PubObstacle(\
                                pose = self.random_pose_maker(x=-83.8, y=2.0, th=27.2, ver_sigma=0.1, lat_sigma=0.1, th_sigma=0.0),\
                                vel = self.random_velocity_maker(v=1.8, v_sigma=0.2), \
                                obstacle_type="pedestrian", obstacle_id=5)
                self.obstacle_generated[5] = 1
                self.obstacle_generated_time[5] = rospy.Time.now().to_sec()

        #obstacle 6: pause and vanish car
        if self.obstacle_generate_judge_dist(-33.1, -33.6, -152.8, 25.0, 20.0):
            if self.obstacle_generated[6] == 0:
                self.PubObjectId(6)
                time.sleep(0.25)
                self.PubObstacle(\
                                pose = self.random_pose_maker(x=-33.1, y=-33.6, th=-152.8, ver_sigma=0.1, lat_sigma=0.1, th_sigma=2.0),\
                                vel = self.random_velocity_maker(v=0.0, v_sigma=0.0), \
                                obstacle_type="car", obstacle_id=6)
                self.obstacle_generated[6] = 1
                self.obstacle_generated_time[6] = rospy.Time.now().to_sec()
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[6] > 10.0:
                    self.reset_id_obstacle(6)
                    time.sleep(0.25)
                    self.obstacle_generated[6] = 1#no more publish(publish once)


    def reset_id_obstacle(self, obs_id):
        self.obstacle_generated[obs_id] = 0
        self.obstacle_generated_time[obs_id] = 0
        self.PubResetObject(obs_id)

    def reset_obstacle(self):
        self.obstacle_generated = np.zeros((OBSTACLE_NUM+1))
        self.obstacle_generated_time = np.zeros((OBSTACLE_NUM+1))
        self.PubResetObject()#reset all object

    def collision_judge(self):
        pass#TODO

    def random_pose_maker(self, x, y, th, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0, goal_record=False):#x[m], y[m], th[deg]
        ver_error = np.random.randn()*ver_sigma
        lat_error = np.random.randn()*lat_sigma
        th_error = np.random.randn()*th_sigma
        th_rad = np.deg2rad(th)#deg2rad
        x_rand = x + ver_error*np.cos(th_rad) + lat_error*np.sin(th_rad)#add vertical error and lateral error
        y_rand = y + ver_error*np.sin(th_rad) + lat_error*np.cos(th_rad)#add vertical error and lateral error
        th_rand = th + th_error#add orientation error
        if goal_record:
            self.goal_x = x_rand
            self.goal_y = y_rand
            self.goal_th = th_rand
        return x_rand, y_rand, th_rand

    def random_velocity_maker(self, v, v_sigma=0.0):
        v_error = np.random.randn()*v_sigma
        v_rand = v + v_error
        return v_rand

    def make_posstmp(self, pose, frame_id="world"):
        x = pose[0]
        y = pose[1]
        th = pose[2]
        posemsg = PoseStamped()
        posemsg.header.stamp = rospy.Time.now()
        posemsg.header.frame_id =  frame_id
        posemsg.pose = self.make_pose(x, y, th)
        return posemsg


    def make_posstmp_with_cov(self, pose, xcov=0.25, ycov=0.25, thcov=0.07, frame_id="world"):#pose: x[m], y[m], th[deg]
        x = pose[0]
        y = pose[1]
        th = pose[2]
        posewcmsg = PoseWithCovarianceStamped()
        posewcmsg.header.stamp = rospy.Time.now()
        posewcmsg.header.frame_id =  frame_id
        posewcmsg.pose.pose =  self.make_pose(x, y, th)
        posewcmsg.pose.covariance[0] = xcov#cov of x,x
        posewcmsg.pose.covariance[7] = ycov#cov of y, y
        posewcmsg.pose.covariance[35] = thcov#cov of rot_z, rot_z
        return posewcmsg

    def make_pose_twist_stamped(self, pose, vel, frame_id="world"):
        x = pose[0]
        y = pose[1]
        th = pose[2]
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        psmsg = PoseStamped()
        psmsg.header = header
        psmsg.pose = self.make_pose(x, y, th)
        tsmsg = TwistStamped()
        tsmsg.header = header
        tsmsg.twist = self.make_twist(vel)
        return psmsg, tsmsg


    def make_pose(self, x, y, th):#x[m], y[m], th[deg]
        posemsg = Pose()
        posemsg.position.x = x
        posemsg.position.y = y
        th_rad = np.deg2rad(th)
        q = tf.transformations.quaternion_from_euler(0, 0, th_rad)
        posemsg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return posemsg

    def make_twist(self, v):
        twimsg = Twist()
        twimsg.linear.x = v
        return twimsg

    def calc_dist(self, x1, y1, th1, x2, y2, th2):
        xy_dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        th_dist = np.abs(th1 - th2) % 360
        return xy_dist, th_dist

    def obstacle_generate_judge_dist(self, x, y, th, dist_area, th_dist_area):
        now_dist, now_th_dist = self.calc_dist(self.self_x, self.self_y, self.self_th, x, y, th)
        if now_dist < dist_area and now_th_dist < th_dist_area:
            return True
        else:
            return False

    def goal_judge(self, goal_dist, goal_th, goal_vel):
        #now_dist = np.sqrt((self.self_x - self.goal_x)**2 + (self.self_y - self.goal_y)**2)
        #now_th_dist = np.abs(self.self_th - self.goal_th) % 360
        now_dist, now_th_dist = self.calc_dist(self.self_x, self.self_y, self.self_th, self.goal_x, self.goal_y, self.goal_th)
        now_vel_abs = np.abs(self.self_vel)

        if now_dist <= goal_dist and now_th_dist < goal_th and now_vel_abs < goal_vel:
            return True
        else:
            return False

    def give_up_judge(self, give_up_time):
        now_time = rospy.Time.now().to_sec()
        scenario_time = now_time - self.start_time
        if scenario_time > give_up_time:
            return True
        else:
            return False

    def GetSelfPos(self):
        trans, quat = self.get_pose(from_link="world", to_link="base_link")
        rot = tf.transformations.euler_from_quaternion(quat)
        self.self_x = trans[0]#[m]
        self.self_y = trans[1]#[m]
        self.self_th = np.rad2deg(rot[2])#[deg]

    def get_pose(self, from_link, to_link):
        try:
            self.tfl.waitForTransform(from_link, to_link, rospy.Time(0), rospy.Duration(0.2))
            (trans, quat) = self.tfl.lookupTransform(from_link, to_link, rospy.Time(0)) #parent, child
            return trans, quat
        except:
            trans = (0.0, 0.0, 0.0)
            quat = (0.0, 0.0, 0.0, 1.0)
            rospy.logwarn("cannot get position")
            return trans, quat

    def CallBackVehicleStatus(self, vsmsg):
        self.self_vel = vsmsg.status.velocity

    def PubObstacle(self, pose, vel, obstacle_type, obstacle_id, frame_id="world"):
        if obstacle_type == "pedestrian":
            pubpose = self.pubpedestrianpose
            pubtwist = self.pubpedestriantwist
        elif obstacle_type == "car":
            pubpose = self.pubcarpose
            pubtwist = self.pubcartwist
        else:
            rospy.logwarn("invalid obstacle type")
            return False
        pose, twist = self.make_pose_twist_stamped(pose, vel)
        pubpose.publish(pose)
        pubtwist.publish(twist)
        time.sleep(0.2)
        return True

    def PubObjectId(self, id=0):
        idmsg = Int32()
        idmsg.data = id
        self.pubobjectid.publish(idmsg)

    def PubResetObject(self, id=-1):#id=-1 -> reset all object
        idmsg = Int32()
        idmsg.data = id
        self.pubresetobjectid.publish(idmsg)


def main():
    rospy.init_node("scenario_maker")
    ScenarioMaker()
    rospy.spin()


if __name__ == "__main__":
    main()