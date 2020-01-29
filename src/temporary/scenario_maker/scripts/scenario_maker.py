#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import subprocess
import sys
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Header, Int32

OBSTACLE_NUM = 4096

# for dist judge
GENERATE_DIST_ALLWAYS = 0
GENERATE_DIST_INAREA = 1
GENERATE_DIST_OUTAREA = 2

# for traffic light judge
GENERATE_TRAFFIC_ALLWAYS = 0
GENERATE_TRAFFIC_GREEN = 1
GENERATE_TRAFFIC_RED = 2

# for traffic light
COLOR_GREEN = 0
COLOR_YELLOW = 1
COLOR_RED = 2
COLOR_BRACK = 3

# for alternate generate mode
BEFORE = 0
AFTER = 1

# fail pattern
TIMEOUT = 1
COLLISION = 2


class ScenarioMaker:
    def __init__(self):

        # TODO: ->rosparam
        self.goal_dist = rospy.get_param("~goal_dist", 1.0)  # Goal criteria(xy-distance) [m]
        self.goal_th = rospy.get_param("~goal_theta", 10)  # Goal criteria(theta-distance)[deg]
        self.goal_vel = rospy.get_param("~goal_velocity", 0.001)  # Goal criteria(velocity)[m/s]
        self.give_up_time = rospy.get_param("~give_up_time", 600)  # Goal criteria(time)[s]
        self.generate_obstacle = rospy.get_param("~generate_obstacle", True)  # Generate obstacle or not
        self.auto_engage = rospy.get_param("~auto_engage", True)  # Engage automaticlly or not
        self.retry_scenario = rospy.get_param("~retry_scenario", True)  # Retry scenario or not
        self.max_scenario_num = rospy.get_param("~max_scenario_num", 10)  # Numober of scenarios to try
        self.traffic_light_time = rospy.get_param("~traffic_light_time", 35)  # Time until the traffic light changes[s]
        self.initial_traffic_light = rospy.get_param(
            "~initial_traffic_light", "green"
        )  # initial traffic light. green or red.
        self.is_record_rosbag = rospy.get_param(
            "~record_rosbag", False
        )  # Record rosbag or not **Rosbag file is very large. Be careful!
        self.rosbag_node_name = rospy.get_param("~rosbag_node_name", "rosbag_node")  # The node name of rosbag-record
        self.delete_success_rosbag = rospy.get_param(
            "~delete_success_rosbag", True
        )  # delete rosbag of success scenario or not
        self.rosbag_file_name = rospy.get_param(
            "~rosbag_file_name", "/media/kimura/extra_ssd/rosbag/scenario"
        )  # dir/file name of rosbag file(**it must be absolute path ** )

        self.trial_num = 0
        self.start_time = 0
        self.traffic_light_start_time = rospy.Time.now().to_sec()
        if self.initial_traffic_light == "green":
            self.traffic_light = COLOR_GREEN
        elif self.initial_traffic_light == "red":
            self.traffic_light = COLOR_RED
        else:
            self.traffic_light = COLOR_GREEN  # exception

        self.obstacle_generated = np.zeros((OBSTACLE_NUM + 1))
        self.obstacle_generated_time = np.zeros((OBSTACLE_NUM + 1))
        self.obstacle_generated_count = np.zeros((OBSTACLE_NUM + 1))

        self.self_x = 0.0
        self.self_y = 0.0
        self.self_th = 0.0
        self.self_vel = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_th = 0.0

        self.image_green, self.image_yellow, self.image_red, self.image_black = self.MakeTrafficImage()

        self.fail_reason = 0
        self.collision = False

        self.camera_header = Header()

        self.tfl = tf.TransformListener()  # for get self-position

        self.sub_vel = rospy.Subscriber(
            "/vehicle/status/velocity", Float32, self.CallBackVehicleVelocity, queue_size=1, tcp_nodelay=True
        )

        # for publish time-sync image
        self.sub_camerainfo = rospy.Subscriber(
            "/sensing/camera/traffic_light/camera_info",
            CameraInfo,
            self.CallBackCameraInfoTime,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.sub_collsion_detection = rospy.Subscriber(
            "/collsion_detection_result", Bool, self.CallBackCollision, queue_size=1
        )

        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.pub_checkpoint = rospy.Publisher("/checkpoint", PoseStamped, queue_size=1)

        self.pub_engage = rospy.Publisher("/autoware/engage", Bool, queue_size=1)

        self.pub_pedestrianpose = rospy.Publisher("/initial_pedestrian_pose", PoseStamped, queue_size=1)

        self.pub_pedestriantwist = rospy.Publisher("/initial_pedestrian_twist", TwistStamped, queue_size=1)

        self.pub_carpose = rospy.Publisher("/initial_car_pose", PoseStamped, queue_size=1)

        self.pub_cartwist = rospy.Publisher("/initial_car_twist", TwistStamped, queue_size=1)

        self.pub_objectid = rospy.Publisher("/object_id", Int32, queue_size=1)

        self.pub_resetobjectid = rospy.Publisher("/reset_object_id", Int32, queue_size=1)

        self.pub_traffic_light_image = rospy.Publisher("/sensing/camera/traffic_light/image_raw", Image, queue_size=1)

        time.sleep(0.5)  # wait for ready to publish/subscribe#TODO: fix this

        ##################################################### main process start

        # set self-position and path
        if self.is_record_rosbag:
            self.record_rosbag(self.trial_num, self.rosbag_node_name, self.rosbag_file_name)
        self.scenario_path(only_initial_pose=True)
        time.sleep(1.0)
        self.reset_obstacle()
        time.sleep(1.0)
        if self.generate_obstacle:
            self.scenario_obstacle()  # obstacle is valid only when self-position is given
        time.sleep(3.0)
        self.scenario_path()

        while not rospy.is_shutdown():
            self.GetSelfPos()
            if self.goal_judge(self.goal_dist, self.goal_th, self.goal_vel):
                print("Scenario Clear!", self.trial_num, "Time:", rospy.Time.now().to_sec() - self.start_time)
                time.sleep(10.0)
                if self.is_record_rosbag:
                    self.end_rosbag(self.rosbag_node_name)
                    if self.delete_success_rosbag:
                        self.delete_rosbag(self.trial_num, self.rosbag_file_name)
                if self.retry_scenario:
                    self.retry_senario_path()
                else:
                    self.pub_engage.publish(False)  # stop vehicle
                    sys.exit()
            elif self.fail_judge(self.collision, self.give_up_time):
                print(
                    "Failed.",
                    self.trial_num,
                    "Time:",
                    rospy.Time.now().to_sec() - self.start_time,
                    "Fail Reason:",
                    self.fail_reason,
                )
                if self.is_record_rosbag:
                    self.end_rosbag(self.rosbag_node_name)
                if self.retry_scenario:
                    self.retry_senario_path()
                else:
                    self.pub_engage.publish(False)  # stop vehicle
                    sys.exit()
            else:
                if self.generate_obstacle:
                    self.scenario_obstacle_manager()
            self.traffic_light_manager()
            # self.traffic_light_publisher()

        ##################################################### main process end

    def retry_senario_path(self):
        self.trial_num += 1
        if self.trial_num > self.max_scenario_num:
            sys.exit()

        if self.is_record_rosbag:
            self.record_rosbag(self.trial_num, self.rosbag_node_name, self.rosbag_file_name)
        self.pub_engage.publish(False)  # stop vehicle
        self.collision = False
        self.fail_reason = 0
        self.reset_obstacle()
        self.scenario_obstacle()
        time.sleep(5.0)
        self.start_time = rospy.Time.now().to_sec()
        self.scenario_path()

    def scenario_path(self, only_initial_pose=False):
        self.pub_engage.publish(False)  # stop vehicle
        self.start_time = rospy.Time.now().to_sec()
        # Publish Scenario
        self.scenario_path1(only_initial_pose)
        # Publish Engage
        if self.auto_engage:
            self.pub_engage.publish(True)
            time.sleep(0.25)

    def scenario_obstacle(self):
        self.scenario_obstacle1()

    def scenario_obstacle_manager(self):
        self.scenario_obstacle_manager1()

    def scenario_path1(self, only_inital_pose=False):
        time.sleep(0.25)

        # publish Start Position
        self.pub_initialpose.publish(
            self.make_posstmp_with_cov(
                self.random_pose_maker(x=-139.1, y=-24.68, th=117.2, ver_sigma=0.5, lat_sigma=0.1, th_sigma=1.0)
            )
        )
        time.sleep(0.25)
        if only_inital_pose:
            return

        # Publish Goal Position
        self.pub_goal.publish(
            self.make_posstmp(
                self.random_pose_maker(
                    x=-117.8, y=4.553, th=117.2, ver_sigma=0.5, lat_sigma=0.1, th_sigma=1.0, goal_record=True
                )
            )
        )
        time.sleep(0.25)

        # Publish Check Point
        self.pub_checkpoint.publish(
            self.make_posstmp(
                self.random_pose_maker(x=-60.3, y=-23.6, th=-62.8, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)
            )
        )  # checkpoint 1
        time.sleep(0.25)
        self.pub_checkpoint.publish(
            self.make_posstmp(
                self.random_pose_maker(x=1.4, y=5.4, th=117.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)
            )
        )  # checkpoint 2
        time.sleep(0.25)
        self.pub_checkpoint.publish(
            self.make_posstmp(
                self.random_pose_maker(x=-83.5, y=14.5, th=117.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)
            )
        )  # checkpoint 3
        time.sleep(0.25)
        self.pub_checkpoint.publish(
            self.make_posstmp(
                self.random_pose_maker(x=-43.5, y=64.2, th=27.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)
            )
        )  # checkpoint 4
        time.sleep(0.25)
        self.pub_checkpoint.publish(
            self.make_posstmp(
                self.random_pose_maker(x=-5.8, y=-19.8, th=-152.8, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)
            )
        )  # checkpoint 5
        time.sleep(0.25)
        self.pub_checkpoint.publish(
            self.make_posstmp(
                self.random_pose_maker(x=-119.1, y=-63.6, th=117.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0)
            )
        )  # checkpoint 6
        time.sleep(0.25)

    def scenario_obstacle1(self):
        ###obstacle 0: fixed pedestrian
        self.PubObjectId(0)
        time.sleep(0.25)
        self.PubObstacle(
            pose=self.random_pose_maker(x=-121.3, y=25.87, th=27.2, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0),
            vel=self.random_velocity_maker(v=0, v_sigma=0),
            obstacle_type="pedestrian",
            obstacle_id=0,
        )

    def scenario_obstacle_manager1(self):
        # obstacle 1: obstacle car in lane change
        self.PubPatternedObstacle(
            pose=(-56.7, 57.3, 27.2),
            vel=3.0,
            ver_sigma=2.0,
            lat_sigma=0.0,
            th_sigma=0.0,
            vel_sigma=0.5,
            judge_pose=(-56.7, 57.3, 27.2),
            judge_dist_xy=40.0,
            judge_dist_th=45.0,
            generate_mode_dist=GENERATE_DIST_INAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_ALLWAYS,
            generate_once=False,
            generate_loop=20.0,
            obstacle_type="car",
            obstacle_id=1,
        )

        # obstacle 2: sudden pedestrian
        self.PubPatternedObstacle(
            pose=(-95.0, -67.0, 117.2),
            vel=0.5,
            ver_sigma=0.1,
            lat_sigma=0.1,
            th_sigma=1.0,
            vel_sigma=0.1,
            judge_pose=(-95.0, -67.0, -152.8),
            judge_dist_xy=35.0,
            judge_dist_th=45.0,
            generate_mode_dist=GENERATE_DIST_INAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_ALLWAYS,
            generate_once=False,
            generate_loop=12.0,
            obstacle_type="pedestrian",
            obstacle_id=2,
        )

        # obstacle 3: crossing car 1(crossing with traffic light)
        self.PubPatternedObstacle(
            pose=(-121.8, -25.6, 27.2),
            vel=8.0,
            ver_sigma=5.0,
            lat_sigma=0.1,
            th_sigma=0.0,
            vel_sigma=0.5,
            judge_pose=(-121.8, -25.6, 27.2),
            judge_dist_xy=30.0,
            judge_dist_th=30.0,
            generate_mode_dist=GENERATE_DIST_OUTAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_RED,
            generate_once=False,
            generate_loop=11.0,
            obstacle_type="car",
            obstacle_id=3,
        )

        # obstacle 4: crossing car 2(crossing without traffic light)
        self.PubPatternedObstacle(
            pose=(-101.6, -65.2, 27.2),
            vel=8.0,
            ver_sigma=5.0,
            lat_sigma=0.1,
            th_sigma=0.0,
            vel_sigma=1.0,
            judge_pose=(-101.6, -65.2, 27.2),
            judge_dist_xy=30.0,
            judge_dist_th=30.0,
            generate_mode_dist=GENERATE_DIST_OUTAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_ALLWAYS,
            generate_once=False,
            generate_loop=11.0,
            obstacle_type="car",
            obstacle_id=4,
        )

        # obstacle 5: crossing pedestrian(crossing with traffic light)
        self.PubPatternedObstacle(
            pose=(-83.8, 2.0, 27.2),
            vel=2.0,
            ver_sigma=0.5,
            lat_sigma=0.5,
            th_sigma=1.0,
            vel_sigma=0.2,
            judge_pose=(-83.8, 2.0, 27.2),
            judge_dist_xy=0,
            judge_dist_th=0,
            generate_mode_dist=GENERATE_DIST_ALLWAYS,
            generate_mode_traffic=GENERATE_TRAFFIC_RED,
            generate_once=False,
            generate_loop=14.0,
            obstacle_type="pedestrian",
            obstacle_id=5,
        )

        # obstacle 6: pause and vanish car
        self.PubPatternedObstacle(
            pose=(-33.1, -33.6, -152.8),
            vel=0.0,
            ver_sigma=0.1,
            lat_sigma=0.1,
            th_sigma=2.0,
            vel_sigma=0.0,
            judge_pose=(-33.1, -33.6, -152.8),
            judge_dist_xy=30.0,
            judge_dist_th=45.0,
            generate_mode_dist=GENERATE_DIST_INAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_ALLWAYS,
            generate_once=True,
            generate_loop=10.0,
            obstacle_type="car",
            obstacle_id=6,
        )

        # obstacle 7: crossing car 3(crossing with traffic light/stop to see red light) (alternative: obstacle 8)
        self.PubPatternedObstacle(
            pose=(-121.8, -25.6, 27.2),
            vel=4.5,
            ver_sigma=0.5,
            lat_sigma=0.1,
            th_sigma=0.0,
            vel_sigma=0.05,
            judge_pose=(-121.8, -25.6, 27.2),
            judge_dist_xy=30.0,
            judge_dist_th=30.0,
            generate_mode_dist=GENERATE_DIST_OUTAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_GREEN,
            generate_once=False,
            generate_loop=8.0,
            obstacle_type="car",
            obstacle_id=7,
            alternate_mode=True,
            alternate_timing=BEFORE,
        )

        # obstacle 8: stop car (alternative: obstacle 7)
        self.PubPatternedObstacle(
            pose=(-86.1, -7.5, 27.2),
            vel=0.0,
            ver_sigma=0.5,
            lat_sigma=0.1,
            th_sigma=0.0,
            vel_sigma=0.0,
            judge_pose=(-86.1, -7.5, 27.2),
            judge_dist_xy=30.0,
            judge_dist_th=30.0,
            generate_mode_dist=GENERATE_DIST_OUTAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_GREEN,
            generate_once=False,
            generate_loop=8.0,
            obstacle_type="car",
            obstacle_id=8,
            alternate_mode=True,
            alternate_timing=AFTER,
        )

        # obstacle 9:crossing pedestrian2(crossing with traffic light)
        self.PubPatternedObstacle(
            pose=(-81.8, -2.0, -62.8),
            vel=2.0,
            ver_sigma=0.5,
            lat_sigma=0.5,
            th_sigma=2.0,
            vel_sigma=0.1,
            judge_pose=(-81.8, -2.0, -62.8),
            judge_dist_xy=0.0,
            judge_dist_th=0.0,
            generate_mode_dist=GENERATE_DIST_OUTAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_GREEN,
            generate_once=False,
            generate_loop=4.0,
            obstacle_type="pedestrian",
            obstacle_id=9,
        )

        # obstacle 10:crossing pedestrian3(crossing with traffic light)
        self.PubPatternedObstacle(
            pose=(-81.8, -2.0, -62.8),
            vel=3.0,
            ver_sigma=0.5,
            lat_sigma=0.5,
            th_sigma=2.0,
            vel_sigma=0.1,
            judge_pose=(-81.8, -2.0, -62.8),
            judge_dist_xy=0.0,
            judge_dist_th=0.0,
            generate_mode_dist=GENERATE_DIST_OUTAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_GREEN,
            generate_once=False,
            generate_loop=3.0,
            obstacle_type="pedestrian",
            obstacle_id=10,
        )

        # obstacle 11:car for following
        self.PubPatternedObstacle(
            pose=(-20.4, 56.3, -62.8),
            vel=4.0,
            ver_sigma=0.5,
            lat_sigma=0.0,
            th_sigma=0.0,
            vel_sigma=1.0,
            judge_pose=(-20.4, 56.3, -62.8),
            judge_dist_xy=40.0,
            judge_dist_th=20.0,
            generate_mode_dist=GENERATE_DIST_INAREA,
            generate_mode_traffic=GENERATE_TRAFFIC_ALLWAYS,
            generate_once=True,
            generate_loop=7.0,
            obstacle_type="car",
            obstacle_id=11,
        )

    def traffic_light_manager(self):
        if rospy.Time.now().to_sec() - self.traffic_light_start_time > self.traffic_light_time:
            if self.traffic_light == COLOR_GREEN:
                self.traffic_light = COLOR_RED
            elif self.traffic_light == COLOR_RED:
                self.traffic_light = COLOR_GREEN
            else:
                pass  # no time to be yellow
            self.traffic_light_start_time = rospy.Time.now().to_sec()

    def traffic_light_publisher(self):
        self.PubTrafficLightImage(self.traffic_light_changer(self.traffic_light))

    def traffic_light_changer(self, traffic_light):  # according to self posture, change the traffic light to reference
        # temporary function!!! TODO: fix this
        if not self.judge_dist(-72.7, -2.31, 0, 45.0, 180.0):
            return COLOR_BRACK  # no traffic light

        if np.abs(np.cos((np.deg2rad(self.self_th - 117.2)))) > np.sqrt(2.0) / 2.0:
            return traffic_light
        else:
            if traffic_light == COLOR_RED:
                return COLOR_GREEN
            elif traffic_light == COLOR_GREEN:
                return COLOR_RED
            else:
                return traffic_light

    def reset_id_obstacle(self, obs_id):
        self.obstacle_generated[obs_id] = 0
        self.obstacle_generated_time[obs_id] = 0
        self.PubResetObject(obs_id)

    def reset_obstacle(self):
        self.obstacle_generated = np.zeros((OBSTACLE_NUM + 1))
        self.obstacle_generated_time = np.zeros((OBSTACLE_NUM + 1))
        self.obstacle_generated_count = np.zeros((OBSTACLE_NUM + 1))
        self.PubResetObject()  # reset all object

    def random_pose_maker(
        self, x, y, th, ver_sigma=0.0, lat_sigma=0.0, th_sigma=0.0, goal_record=False
    ):  # x[m], y[m], th[deg]
        ver_error = np.random.randn() * ver_sigma
        lat_error = np.random.randn() * lat_sigma
        th_error = np.random.randn() * th_sigma
        th_rad = np.deg2rad(th)  # deg2rad
        x_rand = x + ver_error * np.cos(th_rad) + lat_error * np.sin(th_rad)  # add vertical error and lateral error
        y_rand = y + ver_error * np.sin(th_rad) + lat_error * np.cos(th_rad)  # add vertical error and lateral error
        th_rand = th + th_error  # add orientation error
        if goal_record:
            self.goal_x = x_rand
            self.goal_y = y_rand
            self.goal_th = th_rand
        return x_rand, y_rand, th_rand

    def random_velocity_maker(self, v, v_sigma=0.0):
        v_error = np.random.randn() * v_sigma
        v_rand = v + v_error
        return v_rand

    def make_posstmp(self, pose, frame_id="world"):
        x = pose[0]
        y = pose[1]
        th = pose[2]
        posemsg = PoseStamped()
        posemsg.header.stamp = rospy.Time.now()
        posemsg.header.frame_id = frame_id
        posemsg.pose = self.make_pose(x, y, th)
        return posemsg

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

    def make_pose(self, x, y, th):  # x[m], y[m], th[deg]
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
        xy_dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        th_dist = np.abs(th1 - th2) % 360
        return xy_dist, th_dist

    def judge_dist(self, x, y, th, dist_area, th_dist_area):
        now_dist, now_th_dist = self.calc_dist(self.self_x, self.self_y, self.self_th, x, y, th)
        if now_dist < dist_area and now_th_dist < th_dist_area:
            return True
        else:
            return False

    def judge_alternate(self, alternate_mode, alternate_timing, obstacle_id):
        if alternate_mode:
            if alternate_timing == BEFORE and self.obstacle_generated_count[obstacle_id] % 2 == 0:
                return True
            elif alternate_timing == AFTER and self.obstacle_generated_count[obstacle_id] % 2 == 1:
                return True
            else:
                return False
        else:
            return True

    def goal_judge(self, goal_dist, goal_th, goal_vel):
        now_dist, now_th_dist = self.calc_dist(
            self.self_x, self.self_y, self.self_th, self.goal_x, self.goal_y, self.goal_th
        )
        now_vel_abs = np.abs(self.self_vel)

        if now_dist <= goal_dist and now_th_dist < goal_th and now_vel_abs < goal_vel:
            return True
        else:
            return False

    def fail_judge(self, collision, give_up_time):
        if self.collision_judge(collision):
            self.fail_reason = COLLISION
            return True
        elif self.time_out_judge(give_up_time):
            self.fail_reason = TIMEOUT
            return True

    def collision_judge(self, collision):
        if collision:
            return True
        else:
            return False

    def time_out_judge(self, give_up_time):
        now_time = rospy.Time.now().to_sec()
        scenario_time = now_time - self.start_time
        if scenario_time > give_up_time:
            return True
        else:
            return False

    def GetSelfPos(self):
        trans, quat = self.get_pose(from_link="world", to_link="base_link")
        rot = tf.transformations.euler_from_quaternion(quat)
        self.self_x = trans[0]  # [m]
        self.self_y = trans[1]  # [m]
        self.self_th = np.rad2deg(rot[2])  # [deg]

    def get_pose(self, from_link, to_link):
        try:
            self.tfl.waitForTransform(from_link, to_link, rospy.Time(0), rospy.Duration(0.2))
            (trans, quat) = self.tfl.lookupTransform(from_link, to_link, rospy.Time(0))  # parent, child
            return trans, quat
        except:
            trans = (0.0, 0.0, 0.0)
            quat = (0.0, 0.0, 0.0, 1.0)
            rospy.logwarn("cannot get position")
            return trans, quat

    def CallBackVehicleVelocity(self, vsmsg):
        self.self_vel = vsmsg.data

    def CallBackCameraInfoTime(self, cimsg):
        self.camera_header = cimsg.header
        self.traffic_light_publisher()

    def CallBackCollision(self, colmsg):
        if colmsg.data:
            self.collision = colmsg.data

    def PubPatternedObstacle(
        self,
        pose,
        vel,
        ver_sigma,
        lat_sigma,
        th_sigma,
        vel_sigma,
        judge_pose,
        judge_dist_xy,
        judge_dist_th,
        generate_mode_dist,
        generate_mode_traffic,
        generate_once,
        generate_loop,
        obstacle_type,
        obstacle_id,
        alternate_mode=False,
        alternate_timing=BEFORE,
        frame_id="world",
    ):
        x_obj, y_obj, th_obj = pose
        x_jdg, y_jdg, th_jdg = judge_pose
        if generate_mode_dist == GENERATE_DIST_ALLWAYS:
            generate_now_dist = True
        elif generate_mode_dist == GENERATE_DIST_INAREA:
            generate_now_dist = self.judge_dist(x_jdg, y_jdg, th_jdg, judge_dist_xy, judge_dist_th)
        elif generate_mode_dist == GENERATE_DIST_OUTAREA:
            generate_now_dist = not self.judge_dist(x_jdg, y_jdg, th_jdg, judge_dist_xy, judge_dist_th)
        else:
            # Error; rospy.logerror("invalid generate mode")
            generate_now_dist = False

        if generate_mode_traffic == GENERATE_TRAFFIC_ALLWAYS:
            generate_now_traffic = True
        elif generate_mode_traffic == GENERATE_TRAFFIC_GREEN:
            if self.traffic_light == COLOR_GREEN or self.traffic_light == COLOR_YELLOW:
                generate_now_traffic = True
            else:
                generate_now_traffic = False
        elif generate_mode_traffic == GENERATE_TRAFFIC_RED:
            if self.traffic_light == COLOR_RED:
                generate_now_traffic = True
            else:
                generate_now_traffic = False

        generate_now_alternate = self.judge_alternate(alternate_mode, alternate_timing, obstacle_id)

        if generate_now_dist and generate_now_traffic:
            if self.obstacle_generated[obstacle_id] == 0:
                self.PubObjectId(obstacle_id)
                time.sleep(0.25)
                if generate_now_alternate:
                    self.PubObstacle(
                        pose=self.random_pose_maker(
                            x=x_obj, y=y_obj, th=th_obj, ver_sigma=ver_sigma, lat_sigma=lat_sigma, th_sigma=th_sigma
                        ),
                        vel=self.random_velocity_maker(v=vel, v_sigma=vel_sigma),
                        obstacle_type=obstacle_type,
                        obstacle_id=obstacle_id,
                    )
                self.obstacle_generated[obstacle_id] = 1
                self.obstacle_generated_time[obstacle_id] = rospy.Time.now().to_sec()
                self.obstacle_generated_count[obstacle_id] += 1
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[obstacle_id] > generate_loop:
                    self.reset_id_obstacle(obstacle_id)
                    if generate_once:
                        self.obstacle_generated[obstacle_id] = 1  # no more generate(generate once)
                    else:
                        self.PubObjectId(obstacle_id)
                        time.sleep(0.25)
                        if generate_now_alternate:
                            self.PubObstacle(
                                pose=self.random_pose_maker(
                                    x=x_obj,
                                    y=y_obj,
                                    th=th_obj,
                                    ver_sigma=ver_sigma,
                                    lat_sigma=lat_sigma,
                                    th_sigma=th_sigma,
                                ),
                                vel=self.random_velocity_maker(v=vel, v_sigma=vel_sigma),
                                obstacle_type=obstacle_type,
                                obstacle_id=obstacle_id,
                            )
                        self.obstacle_generated[obstacle_id] = 1
                        self.obstacle_generated_time[obstacle_id] = rospy.Time.now().to_sec()
                        self.obstacle_generated_count[obstacle_id] += 1
                    time.sleep(0.25)
        else:
            self.reset_id_obstacle(obstacle_id)

    def PubObstacle(self, pose, vel, obstacle_type, obstacle_id, frame_id="world"):
        if obstacle_type == "pedestrian":
            pubpose = self.pub_pedestrianpose
            pubtwist = self.pub_pedestriantwist
        elif obstacle_type == "car":
            pubpose = self.pub_carpose
            pubtwist = self.pub_cartwist
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
        self.pub_objectid.publish(idmsg)

    def PubResetObject(self, id=-1):  # id=-1 -> reset all object
        idmsg = Int32()
        idmsg.data = id
        self.pub_resetobjectid.publish(idmsg)

    def PubTrafficLightImage(self, color):
        imgmsg = Image()
        imgmsg.header = self.camera_header  # newest header
        imgmsg.encoding = "bgr8"
        imgmsg.height = 1080
        imgmsg.width = 1920
        imgmsg.is_bigendian = False
        imgmsg.step = 3 * imgmsg.width
        imgsize = imgmsg.height * imgmsg.width * 3  # pixel*3(bgr)
        img = np.zeros((imgsize)).astype(np.uint8)
        if color == COLOR_GREEN:
            imgmsg.data = self.image_green
        elif color == COLOR_YELLOW:
            imgmsg.data = self.image_yellow
        elif color == COLOR_RED:
            imgmsg.data = self.image_red
        else:
            imgmsg.data = self.image_black  # black image
        self.pub_traffic_light_image.publish(imgmsg)

    def MakeTrafficImage(self):
        imgsize = 1080 * 1920 * 3  # pixel*3(bgr)
        img_green = np.zeros((imgsize)).astype(np.uint8)
        img_green[np.arange(0, imgsize, 3)] = 10  # brue
        img_green[np.arange(1, imgsize, 3)] = 255  # green
        img_green[np.arange(2, imgsize, 3)] = 10  # red
        img_yellow = np.zeros((imgsize)).astype(np.uint8)
        img_yellow[np.arange(2, imgsize, 3)] = 255  # red
        img_yellow[np.arange(1, imgsize, 3)] = 217  # green
        img_red = np.zeros((imgsize)).astype(np.uint8)
        img_red[np.arange(0, imgsize, 3)] = 10  # brue
        img_red[np.arange(1, imgsize, 3)] = 10  # green
        img_red[np.arange(2, imgsize, 3)] = 255  # red
        img_black = np.zeros((imgsize)).astype(np.uint8)
        return list(img_green), list(img_yellow), list(img_red), list(img_black)

    def record_rosbag(self, id, node_name, file_name):
        # no error handling: TODO
        rosbag_record = (
            "rosbag record -O "
            + file_name
            + str(id)
            + '.bag -a -x "/debug/(.*)|/sensing/(.*)|/traffic_light_classifier/(.*)|/rosout|/rosout_agg|(.*)/pcd|(.*)/costmap|(.*)/occupancy_grid|(.*)/drivable_area" __name:='
            + node_name
        )
        subprocess.Popen(rosbag_record, shell=True)

    def end_rosbag(self, node_name):
        # no error handling: TODO
        rosbag_end = "rosnode kill " + node_name
        subprocess.Popen(rosbag_end, shell=True)
        time.sleep(2.0)

    def delete_rosbag(self, id, file_name):
        # no error handling: TODO
        rosbag_delete = "rm " + file_name + str(id) + ".bag"
        subprocess.Popen(rosbag_delete, shell=True)


def main():
    rospy.init_node("scenario_maker")
    ScenarioMaker()
    rospy.spin()


if __name__ == "__main__":
    main()
