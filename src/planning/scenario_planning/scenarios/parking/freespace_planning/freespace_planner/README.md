# Freespace Planner

Freespace planner package provides the global planner nodes that plan waypoints in the space having static/dynamic obstacles.

## Freespace Planner - Astar Navi

`freespace_planner` is global path planner based on Hybrid A\* search algorithm in `astar_search` package. This node executes planning at a constant cycle and publish `lane_waypoints_array`.

_NOTE_ : You can use `Use Reverse Motion` option for generating path including k-turn, backward control is not enough unified in Autoware planners, currently.

Please see also: motion/packages/waypoint_planner/README.md

### How to launch

`$ roslaunch freespace_planner freespace_planner.launch`

### Parameters

Parameters can be set in both Launch file and Runtime manager:

| Parameter in RM      | Parameter in Launch  | Type     | Description                                   | Default                                      |
| -------------------- | -------------------- | -------- | --------------------------------------------- | -------------------------------------------- |
| `Use Reverse Motion` | `use_back`           | _Bool_   | Enable backward motion in Hybrid A\* search   | `true`                                       |
| `Costmap Topic`      | `costmap_topic`      | _String_ | Costmap topic for Hybrid-A\* search           | `semantics/costmap_generator/occupancy_grid` |
| `Waypoint Velocity`  | `waypoints_velocity` | _Double_ | Constant velocity on planned waypoints [km/h] | 5.0                                          |
| `Update Rate`        | `update_rate`        | _Double_ | Replanning and publishing rate [Hz]           | 1.0                                          |

### Subscriptions/Publications

```
Node [/astar_avoid]
Publications:
 * /safety_waypoints [autoware_msgs/Lane]

Subscriptions:
 * /base_waypoints [autoware_msgs/Lane]
 * /closest_waypoint [std_msgs/Int32]
 * /current_pose [geometry_msgs/PoseStamped]
 * /current_velocity [geometry_msgs/TwistStamped]
 * /semantics/costmap_generator/occupancy_grid [nav_msgs/OccupancyGrid]
 * /obstacle_waypoint [std_msgs/Int32]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]
```

### Demo videos

#### Freespace planning with static obstacle

[![Hybrid A*, freespace planning](https://img.youtube.com/vi/tXfexskIbrg/sddefault.jpg)](https://youtu.be/tXfexskIbrg)
