# How to use in a simulator

## How to use in Planning Simulator
---

Assuming already completed [install autoware](https://github.com/tier4/Autoware-T4B#install-autoware) and [prepare map](https://github.com/tier4/Autoware-T4B#prepare-map).


1. Launch Autoware with Planning Simulator
```
$ roslaunch autoware_launch planning_simulator.launch map_path:=[path]
```

2. Set initial position by using `2D Pose Estimate` in rviz.

3. Set goal position by using `2D Nav Goal` in rviz.

4. Engage vehicle.
    - a. Go to [autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html).
    - b. Push `Engage` button.

### Set dummy obstacles

* Set obstacles position by using `2D Dummy Pedestrina` or `2D Dummy Car` in rviz.
  * Shorcut keys `l` and `k` are assigned respectively.
  * Can adjust obstacles' postion error and velocity via `Tool Properties` in rviz.

## How to use in rosbag
---
Assuming already completed [install autoware](https://github.com/tier4/Autoware-T4B#install-autoware) and [prepare map](https://github.com/tier4/Autoware-T4B#prepare-map).

1. Download sample rosbag.
2. Launch Autoware
```
$ roslaunch autoware_launch autoware.launch map_path:=[path] rosbag:=true
```
3. Play sample rosbag.
```
$ rosbag play --clock -r 0.2　
```
