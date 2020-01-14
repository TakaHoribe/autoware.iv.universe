![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)

# How to setup
## Software setup
### Install Autoware
1. install nvidia driver
2. setup ROS, Caffe, CUDA 10.0, cuDNN7, TensorRT7
- ubuntu 16.04
```
./setup_ubuntu16.04.sh
```
- ubuntu 18.04

```
./setup_ubuntu18.04.sh
```
**if you got cmake error in caffe build, please see [this](https://github.com/tier4/Autoware-T4B/wiki/Trouble-shooting)**.

### Set hardware configuration
In src/config/hardware/vehicle_description/config/vehicle_info.yaml
```
wheel_radius: 0.341
wheel_width: 0.42
wheel_base: 2.63 # between front wheel center and rear wheel center
wheel_tread: 1.55 # between left wheel center and right wheel center
front_overhang: 1.29 # between front wheel center and vehicle front
rear_overhang: 1.1 # between rear wheel center and vehicle rear 
vehicle_height: 2.0
```

### Run
1. launch Autoware
```
$ roslaunch autoware_launch autoware.launch
```
2. set goal pose
3. push engage button.
[autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html)
4. (optional) [webviz](https://webviz.io/app/)
# for developper (temporary)
## rule
- only use tf2
- base_link is rear wheel center
- global param list
```
wheel_radius: 0.341
wheel_width: 0.42
wheel_base: 2.63 # between front wheel center and rear wheel center
wheel_tread: 1.55 # between left wheel center and right wheel center
front_overhang: 1.29 # between front wheel center and vehicle front
rear_overhang: 1.1 # between rear wheel center and vehicle rear 
vehicle_height: 2.0
```
