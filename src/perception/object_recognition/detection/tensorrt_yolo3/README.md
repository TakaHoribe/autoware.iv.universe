# ROS wrapper for TensorRT YOLOv3

## Referenced repositiory
Please check this repository for detail implementation.
Note that you are using the trained files provided by the repository.

https://github.com/lewes6369/TensorRT-Yolov3

Please note that above repository is under MIT license.
## How to use
1. Build this package.(Automatically download necessary files during build process)
2. `roslaunch tensorrt_yolo3 tensorrt_yolo3.launch`

## Interface
### Input topic type
  `sensor_msgs::Image`
### Output topic type
  `autoware_perception_msgs::DynamicObjectWithFeatureArray`
