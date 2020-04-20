# Autoware (Architecture Proposal)

![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)


# How to setup

## Requirement

### Hardware
 - x86 CPU (8 or more cores)
 - 16 GB or more of memory
 - Nvidia GPU (4GB or more of memory) : 

### Software
 - Ubuntu 18.04
 - Nvidia driver
 
If cuda or tensorRT is already installed, it is recommended to remove it.

## Autoware setup
1. setup ROS, CUDA 10.2, cuDNN7, TensorRT7, 
```
./setup_ubuntu18.04.sh
```
2. source build
```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Note that the computer need to be connected to Internet to download neural network weight files.

# How to Run

## Simulator
![sim](https://user-images.githubusercontent.com/8327598/79709776-0bd47b00-82fe-11ea-872e-d94ef25bc3bf.png)

### Rosbag
1. launch Autoware
```
$ roslaunch autoware_launch autoware.launch map_path:=[path] rosbag:=true
```
- sample map is [here](https://drive.google.com/a/public.tier4.jp/file/d/1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk/view?usp=sharing).
2. play rosbag
```
$ rosbag play --clock [rosbag file] -r 0.2
```
- sample rosbag is [here](https://drive.google.com/open?id=1BFcNjIBUVKwupPByATYczv2X4qZtdAeD).
  - Velodyne 128 (Top)
  - Velodyne 16 (Right)
  - Velodyne 16 (Left)
  - IMU (Tamagawa TAG300)
  - GNSS (Ublox F9P)
  - CAN data
  - ~~Camera x 7~~
    - Removed due to privacy concerns
    - Cannot run traffic light recognition
    - Decreased accuracy of object detection
 
`-r 0.2` means a 20% playback speed. Since it takes time for automatic initial position estimation, we recommend about 0.2.

### Planning Simulator
1. launch Autoware
```
$ roslaunch autoware_launch planning_simulator.launch map_path:=[path]
```
- sample map is [here](https://drive.google.com/a/public.tier4.jp/file/d/197kgRfSomZzaSbRrjWTx614le2qN-oxx/view?usp=sharing).
2. set initial pose
3. set goal pose
4. push engage button.
[autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html)
5. (optional) [webviz](https://webviz.io/app/)

## Real Car

### Prepare map
you need to prepare maps.
- lanelet2 map
- pointcloud map

### Setup hardware configuretion
TODO

### Run Autoware
1. launch Autoware
```
$ roslaunch autoware_launch autoware.launch map_path:=[path]
```
2. set goal pose
3. push engage button.
[autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html)
4. (optional) [webviz](https://webviz.io/app/)

# Videos
- [Scenario demo](https://youtu.be/kn2bIU_g0oY)
- [Obstacle avoidance in the same lane](https://youtu.be/s_4fBDixFJc)
- [Obstacle avoidance by lane change](https://youtu.be/SCIceXW9sqM)
- [Object recognition](https://youtu.be/uhhMIxe1zxQ)
- [Auto parking](https://youtu.be/e9R0F0ZJbWE)
- [360° FOV perception(Camera Lidar Fuison)](https://youtu.be/whzx-2RkVBA)
- [Robustness of localization](https://youtu.be/ydPxWB2jVnM)

