#!/bin/bash
python ../scripts/reset_obstacle.py
rosbag record -O $1 /initial_pedestrian_pose /initial_pedestrian_twist /initial_car_pose /initial_car_twist