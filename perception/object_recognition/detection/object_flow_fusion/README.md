# simple_lidar_optical_flow

## install
```
catkin build simple_lidar_optical_flow
```

## explanation

 - optical_flow_node
   - `~input_cloud` point cloud (sensor_msgs::PointCloud2)
   - `~output/flows` scene flows (autoware_perception_msgs::DynamicObjectWithFeatureArray)

 - object_flow_fusion_node
   - `~input_object` detected object(autoware_perception_msgs::DynamicObjectWithFeatureArray)
   - `~input_flow` scene flows (autoware_perception_msgs::DynamicObjectWithFeatureArray)
   - `~output` fusioned object (autoware_perception_msgs::DynamicObjectWithFeatureArray)

## launch
```
roslaunch simple_lidar_optical_flow simple_lidar_optical_flow.launch
```
