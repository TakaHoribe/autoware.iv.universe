# Simulation in Autoware
Autoware provides 2 types of simulations.Rosbag is used for testing/validation for `Sensing`, `Localization` and `Perception` stacks. Planning Simulator is mainly used for testing/validation for `Planning` stack by simulating traffic rules, interaction with dynamic objects and control command to vehicle.
![sim](https://user-images.githubusercontent.com/8327598/79709776-0bd47b00-82fe-11ea-872e-d94ef25bc3bf.png)


## How to use rosbag for simulation
---
Assuming already completed [Autoware setup](https://github.com/tier4/Autoware-T4B#autoware-setup).

1. Download sample map from [here](https://drive.google.com/a/public.tier4.jp/file/d/1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk/view?usp=sharing).
2. Download sample rosbag from [here](https://drive.google.com/a/public.tier4.jp/file/d/1ltuRryNn3EUBCZiS88Wnj54632JVeIVz/view?usp=sharing).

3. Launch Autoware with rosbag mode.
```
$ roslaunch autoware_launch autoware.launch map_path:=[path] rosbag:=true
```
4. Play sample rosbag.
```
$ rosbag play --clock -r 0.2　sample.bag
```

## How to use Planning Simulator
---

Assuming already completed [Autoware setup](https://github.com/tier4/Autoware-T4B#autoware-setup).

1. Download sample map from [here](https://drive.google.com/a/public.tier4.jp/file/d/1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk/view?usp=sharing).
2. Launch Autoware with Planning Simulator
```
$ roslaunch autoware_launch planning_simulator.launch map_path:=[path]
```

![launch_planning_simulator](https://user-images.githubusercontent.com/10920881/79715068-94a6e300-830d-11ea-9008-0ed311617c81.png)

3. Set initial position by using `2D Pose Estimate` in rviz.

![initial_pose](https://user-images.githubusercontent.com/10920881/79714203-0a5d7f80-830b-11ea-8ef2-90db71eb1f8d.png)

4. Set goal position by using `2D Nav Goal` in rviz.

![goal_pose](https://user-images.githubusercontent.com/10920881/79714208-0df10680-830b-11ea-9814-b4ee7be77610.png)

5. Engage vehicle.
    - a. Go to [autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html).
    - b. Push `Engage` button.

![engage](https://user-images.githubusercontent.com/10920881/79714298-4db7ee00-830b-11ea-9ac4-11e126d7a7c4.png)

### Set dummy obstacles

* Set obstacles position by using `2D Dummy Pedestria` or `2D Dummy Car` in rviz.
  * Shorcut keys `l` and `k` are assigned respectively.
  * Can adjust obstacles' postion error and velocity via `Tool Properties` in rviz.
  * Can delete all the objects by using `Delte All Objects` in rviz.
![Screenshot from 2020-04-20 13-42-03](https://user-images.githubusercontent.com/10920881/79714775-cbc8c480-830c-11ea-92e2-440200b13f99.png)
