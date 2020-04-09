#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import subprocess
import sys
import time
import uuid, unique_id

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Header, Int32
from dummy_perception_publisher.msg import Object, InitialState
from autoware_perception_msgs.msg import Semantic, Shape

# from config.scenario_sanfrancisco_planning_simulator import *  # scenario #TODO: dynamically change import file


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
        self.scenario_name = rospy.get_param(
            "~scenario_name", "kashiwanoha"
        )  # Scenario Name ("kashiwanoha", "sanfrancisco", "sanfrancisco_lg")
        if self.scenario_name == "kashiwanoha":
            from config.scenario_kashiwanoha import *  # scenario
        elif self.scenario_name == "sanfrancisco":
            from config.scenario_sanfrancisco_planning_simulator import *  # scenario
        elif self.scenario_name == "sanfrancisco_lg":
            from config.scenario_sanfrancisco import *  # scenario
        else:
            rospy.logwarn("invalid scenario. load default(kashiwanoha) scenario")
            from config.scenario_kashiwanoha import *  # scenario

        self.initpos = s_initpos
        self.goalpos = s_goalpos
        self.checkpoint = s_checkpoint
        self.staticobstacle = s_staticobstacle
        self.dynamicobstacle = s_dynamicobstacle
        self.tlpos = s_tlpos

        self.goal_dist = rospy.get_param("~goal_dist", 1.0)  # Goal criteria(xy-distance) [m]
        self.goal_th = rospy.get_param("~goal_theta", 10)  # Goal criteria(theta-distance)[deg]
        self.goal_vel = rospy.get_param("~goal_velocity", 0.001)  # Goal criteria(velocity)[m/s]
        self.max_speed = rospy.get_param("~max_speed", 20.0)  # Max speed[m/s]
        self.give_up_time = rospy.get_param("~give_up_time", 600)  # Goal criteria(time)[s]
        self.generate_obstacle = rospy.get_param("~generate_obstacle", True)  # Generate obstacle or not
        self.auto_engage = rospy.get_param("~auto_engage", True)  # Engage automaticlly or not
        self.retry_scenario = rospy.get_param("~retry_scenario", True)  # Retry scenario or not
        self.is_retry_by_collision = rospy.get_param(
            "~is_retry_by_collision", False
        )  # Retry sceanrio by collison or not
        self.max_scenario_num = rospy.get_param("~max_scenario_num", 10)  # Numober of scenarios to try
        self.traffic_light_time = rospy.get_param("~traffic_light_time", 35)  # Time until the traffic light changes[s]
        self.is_pub_trafficimg = rospy.get_param("~is_pub_traffic_image", True)  # Publish Traffic Light or not
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
        self.use_ndt = rospy.get_param("~use_ndt", "false")

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

        self.fail_reason = 0
        self.collision = False

        self.camera_header = Header()
        self.camera_height = None
        self.camera_width = None

        self.tfl = tf.TransformListener()  # for get self-position

        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.pub_checkpoint = rospy.Publisher("/checkpoint", PoseStamped, queue_size=1)

        self.pub_engage = rospy.Publisher("/autoware/engage", Bool, queue_size=1, latch=True)

        """
        self.pub_pedestrianpose = rospy.Publisher("/initial_pedestrian_pose", PoseStamped, queue_size=1)

        self.pub_pedestriantwist = rospy.Publisher("/initial_pedestrian_twist", TwistStamped, queue_size=1)

        self.pub_carpose = rospy.Publisher("/initial_car_pose", PoseStamped, queue_size=1)

        self.pub_cartwist = rospy.Publisher("/initial_car_twist", TwistStamped, queue_size=1)

        self.pub_objectid = rospy.Publisher("/object_id", Int32, queue_size=1)

        self.pub_resetobjectid = rospy.Publisher("/reset_object_id", Int32, queue_size=1)
        """

        self.pub_objectinfo = rospy.Publisher(
            "/simulation/dummy_perception_publisher/object_info", Object, queue_size=1
        )

        self.pub_traffic_light_image = rospy.Publisher("/sensing/camera/traffic_light/image_raw", Image, queue_size=1)

        self.pub_max_speed = rospy.Publisher("/max_speed_mps", Float32, queue_size=1)

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

        self.image_green, self.image_yellow, self.image_red, self.image_black = self.MakeTrafficMsg()

        # for publish traffic signal image
        rospy.Timer(rospy.Duration(1 / 3.0), self.timerCallback)

        time.sleep(0.2)  # wait for ready to publish/subscribe
        time.sleep(3.0)  # wait for mission planner

        ##################################################### main process start

        # set self-position and path
        if self.is_record_rosbag:
            self.record_rosbag(self.trial_num, self.rosbag_node_name, self.rosbag_file_name)
        self.scenario_path(only_initial_pose=True, frame_id=REF_LINK)
        if self.use_ndt:
            time.sleep(5.0)  # wait to finish ndt matching
        self.reset_obstacle(frame_id=REF_LINK)
        time.sleep(1.0)
        if self.generate_obstacle:
            self.scenario_obstacle(frame_id=REF_LINK)  # obstacle is valid only when self-position is given
        time.sleep(2.0)  # wait for registration of obstacle
        self.scenario_path(frame_id=REF_LINK)

        # publish max speed
        self.pubMaxSpeed()

        r_time = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.GetSelfPos(REF_LINK, SELF_LINK)
            if self.goal_judge(self.goal_dist, self.goal_th, self.goal_vel):
                print("Scenario Clear!", self.trial_num, "Time:", rospy.Time.now().to_sec() - self.start_time)
                time.sleep(10.0)
                if self.is_record_rosbag:
                    self.end_rosbag(self.rosbag_node_name)
                    if self.delete_success_rosbag:
                        self.delete_rosbag(self.trial_num, self.rosbag_file_name)
                if self.retry_scenario:
                    self.retry_scenario_path(frame_id=REF_LINK)
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
                    self.retry_scenario_path(frame_id=REF_LINK)
                else:
                    self.pub_engage.publish(False)  # stop vehicle
                    sys.exit()
            else:
                if self.generate_obstacle:
                    self.scenario_obstacle_manager(frame_id=REF_LINK)
            self.traffic_light_manager()
            r_time.sleep()

        ##################################################### main process end

    def retry_scenario_path(self, frame_id):
        self.trial_num += 1
        if self.trial_num > self.max_scenario_num:
            sys.exit()

        if self.is_record_rosbag:
            self.record_rosbag(self.trial_num, self.rosbag_node_name, self.rosbag_file_name)
        self.pub_engage.publish(False)  # stop vehicle
        self.collision = False
        self.fail_reason = 0
        self.reset_obstacle(frame_id=frame_id)
        self.scenario_obstacle(frame_id=frame_id)
        time.sleep(5.0)
        self.start_time = rospy.Time.now().to_sec()
        self.scenario_path(frame_id=frame_id)

    def scenario_path(self, only_initial_pose=False, frame_id=""):
        self.pub_engage.publish(False)  # stop vehicle
        self.start_time = rospy.Time.now().to_sec()
        # Publish Scenario
        self.scenario_path1(only_initial_pose, frame_id)
        # Publish Engage
        if self.auto_engage:
            self.pub_engage.publish(True)
            time.sleep(0.2)

    def scenario_obstacle(self, frame_id):
        self.scenario_obstacle1(frame_id)

    def scenario_obstacle_manager(self, frame_id):
        self.scenario_obstacle_manager1(frame_id)

    def scenario_path1(self, only_inital_pose=False, frame_id=""):
        time.sleep(0.2)

        # publish Start Position
        self.pub_initialpose.publish(
            self.make_posstmp_with_cov(
                self.random_pose_maker(
                    x=self.initpos["x"],
                    y=self.initpos["y"],
                    th=self.initpos["th"],
                    ver_sigma=self.initpos["ver_sigma"],
                    lat_sigma=self.initpos["lat_sigma"],
                    th_sigma=self.initpos["th_sigma"],
                ),
                frame_id=frame_id,
            )
        )
        if self.use_ndt:
            time.sleep(5.0)  # wait to finish ndt matching
        if only_inital_pose:
            return

        # Publish Goal Position
        self.pub_goal.publish(
            self.make_posstmp(
                self.random_pose_maker(
                    x=self.goalpos["x"],
                    y=self.goalpos["y"],
                    th=self.goalpos["th"],
                    ver_sigma=self.goalpos["ver_sigma"],
                    lat_sigma=self.goalpos["lat_sigma"],
                    th_sigma=self.goalpos["th_sigma"],
                    goal_record=True,
                ),
                frame_id=frame_id,
            )
        )
        time.sleep(0.2)

        # Publish Check Point
        for cp in self.checkpoint:
            self.pub_checkpoint.publish(
                self.make_posstmp(
                    self.random_pose_maker(
                        x=cp["x"],
                        y=cp["y"],
                        th=cp["th"],
                        ver_sigma=cp["ver_sigma"],
                        lat_sigma=cp["lat_sigma"],
                        th_sigma=cp["th_sigma"],
                    ),
                    frame_id=frame_id,
                )
            )
            time.sleep(0.2)

    def scenario_obstacle1(self, frame_id):
        ###obstacle 0: fixed pedestrian
        for sb in self.staticobstacle:
            self.PubObject(
                pose=self.random_pose_maker(
                    x=sb["x"],
                    y=sb["y"],
                    th=sb["th"],
                    ver_sigma=sb["ver_sigma"],
                    lat_sigma=sb["lat_sigma"],
                    th_sigma=sb["th_sigma"],
                ),
                vel=self.random_velocity_maker(v=sb["v"], v_sigma=sb["v_sigma"]),
                obstacle_type=sb["obstacle_type"],
                obstacle_uuid=sb["obstacle_uuid"],
                frame_id=frame_id,
            )

    def scenario_obstacle_manager1(self, frame_id):
        for db in self.dynamicobstacle:
            self.PubPatternedObstacle(
                pose=(db["x"], db["y"], db["th"]),
                vel=db["v"],
                ver_sigma=db["v_sigma"],
                lat_sigma=db["lat_sigma"],
                th_sigma=db["th_sigma"],
                vel_sigma=db["v_sigma"],
                judge_pose=(db["judge_x"], db["judge_y"], db["judge_th"]),
                judge_dist_xy=db["judge_dist_xy"],
                judge_dist_th=db["judge_dist_th"],
                generate_mode_dist=db["generate_mode_dist"],
                generate_mode_traffic=db["generate_mode_traffic"],
                generate_once=db["generate_once"],
                generate_loop=db["generate_loop"],
                obstacle_type=db["obstacle_type"],
                obstacle_id=db["obstacle_id"],
                obstacle_uuid=db["obstacle_uuid"],
                alternate_mode=db["alternate_mode"],
                alternate_timing=db["alternate_timing"],
                frame_id=frame_id,
            )

    def pubMaxSpeed(self):
        if self.max_speed > 0:
            floatmsg = Float32()
            floatmsg.data = self.max_speed
            self.pub_max_speed.publish(floatmsg)

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
        if self.is_pub_trafficimg:
            self.PubTrafficLightImage(self.traffic_light_changer(self.traffic_light))

    def traffic_light_changer(self, traffic_light):  # according to self posture, change the traffic light to reference
        # temporary function!!! TODO: fix this
        if not self.judge_dist(
            self.tlpos["x"], self.tlpos["y"], self.tlpos["th"], self.tlpos["judge_dist_xy"], self.tlpos["judge_dist_th"]
        ):
            return COLOR_BRACK  # no traffic light

        if np.abs(np.cos((np.deg2rad(self.self_th - self.tlpos["turn_traffic_light_th"])))) > np.sqrt(2.0) / 2.0:
            return traffic_light
        else:
            if traffic_light == COLOR_RED:
                return COLOR_GREEN
            elif traffic_light == COLOR_GREEN:
                return COLOR_RED
            else:
                return traffic_light

    def reset_id_obstacle(self, obs_id, obs_uuid, frame_id):
        self.obstacle_generated[obs_id] = 0
        self.obstacle_generated_time[obs_id] = 0
        self.PubResetObject(obs_uuid, frame_id=frame_id)

    def reset_obstacle(self, frame_id):
        self.obstacle_generated = np.zeros((OBSTACLE_NUM + 1))
        self.obstacle_generated_time = np.zeros((OBSTACLE_NUM + 1))
        self.obstacle_generated_count = np.zeros((OBSTACLE_NUM + 1))
        self.PubResetObject(frame_id=frame_id)  # reset all object

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

    def make_posstmp(self, pose, frame_id):
        x = pose[0]
        y = pose[1]
        th = pose[2]
        posemsg = PoseStamped()
        posemsg.header.stamp = rospy.Time.now()
        posemsg.header.frame_id = frame_id
        posemsg.pose = self.make_pose(x, y, th)
        return posemsg

    def make_posstmp_with_cov(self, pose, xcov=0.25, ycov=0.25, thcov=0.07, frame_id=""):  # pose: x[m], y[m], th[deg]
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

    def make_pose_twist(self, pose, vel, frame_id):
        x = pose[0]
        y = pose[1]
        th = pose[2]
        psmsg = self.make_pose(x, y, th)
        tsmsg = self.make_twist(vel)
        return psmsg, tsmsg

    def make_pose_twist_stamped(self, pose, vel, frame_id):
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

    def GetSelfPos(self, from_link, to_link):
        trans, quat = self.get_pose(from_link=from_link, to_link=to_link)
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
        self.camera_height = cimsg.height
        self.camera_width = cimsg.width

    def timerCallback(self, event):
        self.traffic_light_publisher()

    def CallBackCollision(self, colmsg):
        if not self.is_retry_by_collision:
            return
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
        obstacle_uuid,
        alternate_mode=False,
        alternate_timing=BEFORE,
        frame_id="",
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
                if generate_now_alternate:
                    self.PubObject(
                        pose=self.random_pose_maker(
                            x=x_obj, y=y_obj, th=th_obj, ver_sigma=ver_sigma, lat_sigma=lat_sigma, th_sigma=th_sigma
                        ),
                        vel=self.random_velocity_maker(v=vel, v_sigma=vel_sigma),
                        obstacle_type=obstacle_type,
                        obstacle_uuid=obstacle_uuid,
                        frame_id=frame_id,
                    )
                self.obstacle_generated[obstacle_id] = 1
                self.obstacle_generated_time[obstacle_id] = rospy.Time.now().to_sec()
                self.obstacle_generated_count[obstacle_id] += 1
            else:
                if rospy.Time.now().to_sec() - self.obstacle_generated_time[obstacle_id] > generate_loop:
                    self.reset_id_obstacle(obstacle_id, obstacle_uuid, frame_id=frame_id)
                    if generate_once:
                        self.obstacle_generated[obstacle_id] = 1  # no more generate(generate once)
                    else:
                        if generate_now_alternate:
                            self.PubObject(
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
                                obstacle_uuid=obstacle_uuid,
                                frame_id=frame_id,
                            )
                        self.obstacle_generated[obstacle_id] = 1
                        self.obstacle_generated_time[obstacle_id] = rospy.Time.now().to_sec()
                        self.obstacle_generated_count[obstacle_id] += 1
                    time.sleep(0.1)
        else:
            self.reset_id_obstacle(obstacle_id, obstacle_uuid, frame_id=frame_id)

    def PubObject(self, pose, vel, obstacle_type, obstacle_uuid, frame_id):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        init_state = InitialState()
        pose, twist = self.make_pose_twist(pose, vel, frame_id)
        init_state.pose_covariance.pose = pose
        init_state.twist_covariance.twist = twist

        obj = Object()
        obj.header = header
        obj.action = Object.ADD
        obj.id = obstacle_uuid
        obj.initial_state = init_state
        if obstacle_type == "car":
            obj.semantic.type = Semantic.CAR
            obj.semantic.confidence = 1.0
            obj.shape.type = Shape.BOUNDING_BOX
            obj.shape.dimensions.x = 4.0
            obj.shape.dimensions.y = 1.8
            obj.shape.dimensions.z = 2.0
        elif obstacle_type == "pedestrian":
            obj.semantic.type = Semantic.PEDESTRIAN
            obj.semantic.confidence = 1.0
            obj.shape.type = Shape.CYLINDER
            obj.shape.dimensions.x = 0.8
            obj.shape.dimensions.y = 0.8
            obj.shape.dimensions.z = 2.0

        else:
            obj.semantic.type = Semantic.UNKNOWN
        self.pub_objectinfo.publish(obj)
        time.sleep(0.1)
        return True

    def PubResetObject(self, uuid=0, frame_id=""):  # id=0 -> reset all object
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        obj = Object()
        obj.header = header
        if uuid == 0:
            obj.id = unique_id.toMsg(unique_id.fromRandom())  # no mean
            obj.action = Object.DELETEALL
        else:
            obj.id = uuid
            obj.action = Object.DELETE
        self.pub_objectinfo.publish(obj)
        time.sleep(0.1)

    def PubTrafficLightImage(self, color):
        if color == COLOR_GREEN:
            imgmsg = self.image_green
        elif color == COLOR_YELLOW:
            imgmsg = self.image_yellow
        elif color == COLOR_RED:
            imgmsg = self.image_red
        else:
            imgmsg = self.image_black  # black image
        imgmsg.header = self.camera_header  # newest header
        self.pub_traffic_light_image.publish(imgmsg)

    def makeImageMsg(self, image, height, width):
        imgmsg = Image()
        imgmsg.encoding = "bgr8"
        imgmsg.height = height
        imgmsg.width = width
        imgmsg.is_bigendian = False
        imgmsg.step = 3 * imgmsg.width
        imgmsg.data = image
        return imgmsg

    def MakeTrafficMsg(self):
        while self.camera_height is None or self.camera_width is None:
            time.sleep(0.2)  # wait for subscrbing camera info
        imgsize = self.camera_height * self.camera_width * 3  # pixel*3(bgr)
        img_green = np.zeros((imgsize)).astype(np.uint8)
        img_green[np.arange(0, imgsize, 3)] = 200  # brue
        img_green[np.arange(1, imgsize, 3)] = 200  # green
        img_green[np.arange(2, imgsize, 3)] = 0  # red
        img_yellow = np.zeros((imgsize)).astype(np.uint8)
        img_yellow[np.arange(2, imgsize, 3)] = 255  # red
        img_yellow[np.arange(1, imgsize, 3)] = 217  # green
        img_red = np.zeros((imgsize)).astype(np.uint8)
        img_red[np.arange(0, imgsize, 3)] = 10  # brue
        img_red[np.arange(1, imgsize, 3)] = 10  # green
        img_red[np.arange(2, imgsize, 3)] = 255  # red
        img_black = np.zeros((imgsize)).astype(np.uint8)
        img_green_msg = self.makeImageMsg(list(img_green), self.camera_height, self.camera_width)
        img_yellow_msg = self.makeImageMsg(list(img_yellow), self.camera_height, self.camera_width)
        img_red_msg = self.makeImageMsg(list(img_red), self.camera_height, self.camera_width)
        img_black_msg = self.makeImageMsg(list(img_black), self.camera_height, self.camera_width)
        return img_green_msg, img_yellow_msg, img_red_msg, img_black_msg

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
