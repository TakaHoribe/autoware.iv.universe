# Autoware (Architecture Proposal)

![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)

# What's this

This is the source code of the feasibility study for Autoware architecture proposal.

> **WARNING**: This source is solely for demonstrating an architecture proposal. It should not be used to drive cars. 

# How to setup

## Requirements

### Hardware
 - x86 CPU (8 or more cores)
 - 16 GB or more of memory
 - Nvidia GPU (4GB or more of memory) : 

### Software
 - Ubuntu 18.04
 - Nvidia driver
 
If cuda or tensorRT is already installed, it is recommended to remove it.

## Autoware setup
1. Clone this repository
```
git clone https://github.com/tier4/AutowareArchitectureProposal.git
cd AutowareArchitectureProposal/
```
2. Run the setup script
```
./setup_ubuntu18.04.sh
```
In this step, the following software are installed.
Please confirm their licenses before using them.

- [CMake](https://cmake.org/licensing/)
- [osqp](https://github.com/oxfordcontrol/osqp/blob/master/LICENSE)
- [ROS Melodic](https://github.com/ros/ros/blob/noetic-devel/LICENSE)
- [CUDA 10.2](https://docs.nvidia.com/cuda/eula/index.html)
- [cuDNN 7](https://docs.nvidia.com/deeplearning/sdk/cudnn-sla/index.html)
- [TensorRT 7](https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html)

> **info**: 
CMake will be upgraded from ROS's officially supported version.
To use the original version of CMake, run `sudo snap disable cmake`.
To use the upgraded version again, `sudo snap enable cmake`. <br>
Also, a couple of settings are written to your `.bashrc`. If you'd like to uninstall Autoware, remove them and purge apt packages in `ansible/roles/*.yml`. 

3. Build the source
```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Note that the computer need to be connected to Internet to download neural network weight files.

# How to run

How to run on a simulator is described [here](https://github.com/tier4/Autoware-T4B/blob/feature/add_how_to_use_doc/simulation.md).

# Videos
- [Scenario demo](https://youtu.be/kn2bIU_g0oY)
- [Obstacle avoidance in the same lane](https://youtu.be/s_4fBDixFJc)
- [Obstacle avoidance by lane change](https://youtu.be/SCIceXW9sqM)
- [Object recognition](https://youtu.be/uhhMIxe1zxQ)
- [Auto parking](https://youtu.be/e9R0F0ZJbWE)
- [360° FOV perception(Camera Lidar Fuison)](https://youtu.be/whzx-2RkVBA)
- [Robustness of localization](https://youtu.be/ydPxWB2jVnM)

# Note

Some pre-trained models provided by other repository are used in some packages.

 - tensorrt_yolo3 <br>
The pre-trained models are provided in the following repository. The trained file is automatically downloaded when you build. <br>
https://github.com/lewes6369/TensorRT-Yolov3 <br>
\[Original URL] <br>
Tranined file (416) : https://drive.google.com/drive/folders/18OxNcRrDrCUmoAMgngJlhEglQ1Hqk_NJ

- traffic_light_fine_detector <br>
A trained model in this package is based on the following .weights file and was fine-tuned with darknet by Tier IV. <br>
\[Original URL] <br>
https://pjreddie.com/media/files/yolov3.weights <br>
After fine-tuning, the trained model is converted to ONNX file with the following script. <br>
https://github.com/tier4/Autoware-T4B/blob/master/src/perception/traffic_light_recognition/traffic_light_fine_detector/scripts/yolov3_to_onnx.py <br>

- lidar_apollo_instance_segmentation <br>
This package makes use of three pre-trained models provided by apollo. These files are automatically downloaded when you build. <br>
\[Original URL] <br>
VLP-16 : https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel <br>
HDL-64 : https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel <br>
VLS-128 : https://github.com/ApolloAuto/apollo/raw/91844c80ee4bd0cc838b4de4c625852363c258b5/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel <br>

