# map_file package

This package provides the features of loading various maps.

## pointcloud_map_filter

### Feature

pointcloud_map_filter_node subscribe pointcloud maps and current pose, the node extract pointcloud near to the current pose.

#### Subscribed Topics

- /pointcloud_map (sensor_msgs/PointCloud2) : Raw pointcloud map
- /current_pose (geometry_msgs/PoseStamped) : Current pose of the car

#### Published Topics

- /pointcloud_map/filtered (sensor_msgs/PointCloud2) : Filtered pointcloud submap

#### Parameters

- load_grid_size (double) : grid size of submap
- load_trigger_distance (double) : if the car moves load_trigger_distance(m), the map filter publish filtered submap

### How It Works

map_filter_node relay /pointcloud_map topic until it recieves /current_pose topic.  
Then, the /current_pose topic recieved, the map_filter_node publish submap.

### Demonstration

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/LpKIuI5b4DU/0.jpg)](http://www.youtube.com/watch?v=LpKIuI5b4DU)

---

## pointcloud_map_loader

### Feature

pointcloud_map_loader loads PointCloud file and publish the map data as sensor_msgs/PointCloud2 message.

### How to run

`rosrun map_file pointcloud_map_loader path/to/pointcloud1.pcd path/to/pointcloud2.pcd ...`

### Published Topics

- pointcloud_map (sensor_msgs/PointCloud2) : PointCloud Map

---

## lanelet2_map_loader

### Feature

lanelet2_map_loader loads Lanelet2 file and publish the map data as autoware_lanelet2_msgs/MapBin message.
The node projects lan/lon coordinates into MGRS coordinates.

### How to run

`rosrun map_file lanelet2_map_loader path/to/map.osm`

### Published Topics

- ~output/lanelet2_map (autoware_lanelet2_msgs/MapBin) : Binary data of loaded Lanelet2 Map

---

## lanelet2_map_visualization

### Feature

lanelet2_map_visualization visualizes autoware_lanelet2_msgs/MapBin messages into visualization_msgs/MarkerArray.

### How to Run

`rosrun map_file lanelet2_map_visualization`

### Subscribed Topics

- ~input/lanelet2_map (autoware_lanelet2_msgs/MapBin) : binary data of Lanelet2 Map

### Published Topics

- ~output/lanelet2_map_marker (visualization_msgs/MarkerArray) : visualization messages for RVIZ
