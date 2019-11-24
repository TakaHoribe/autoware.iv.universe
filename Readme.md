![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)

# How to setup
## Software setup
### Install Autoware
1. install CUDA,TensorRT
2. ROS, Caffe setup

 ubuntu 16.04
```
./setup.sh
```

ubuntu 18.04

change `kinetic` in ansible/roles/ros/defaults/main.yml
```
./setup.sh
```
### Set hardware configuration
In src/config/hardware/vehicle_description/config/vehicle_info.yaml
```
wheel_radius: 0.341
wheel_width: 0.225
wheel_base: 2.950 # between front wheel center and rear wheel center
wheel_tread: 1.55 # between left wheel center and right wheel center
front_overhang: 1.0 # between front wheel center and vehicle front
rear_overhang: 0.5 # between rear wheel center and vehicle rear 
vehicle_length: 4.45 # rear_overhang + front_overhang + wheel_base
vehicle_width: 1.775 # wheel_tread + wheel_width
vehicle_height: 2.0
```


# for developper (temporary)
## rule
- only use tf2
- base_link is rear wheel center
- global param list
```
wheel_radius: 0.341
wheel_width: 0.225
wheel_base: 2.950 # between front wheel center and rear wheel center
wheel_tread: 1.55 # between left wheel center and right wheel center
front_overhang: 1.0 # between front wheel center and vehicle front
rear_overhang: 0.5 # between rear wheel center and vehicle rear 
vehicle_length: 4.45 # rear_overhang + front_overhang + wheel_base
vehicle_width: 1.775 # wheel_tread + wheel_width
vehicle_height: 2.0
```
