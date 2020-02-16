#include <behavior_velocity_planner/node.hpp>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_routing/Route.h>

#include <visualization_msgs/MarkerArray.h>

#include <utilization/path_utilization.hpp>

// Scene modules
#include <scene_module/blind_spot/manager.hpp>
#include <scene_module/crosswalk/manager.hpp>
#include <scene_module/intersection/manager.hpp>
#include <scene_module/momentary_stop/manager.hpp>
#include <scene_module/traffic_light/manager.hpp>

namespace {
template <class T>
T getParam(const ros::NodeHandle& nh, const std::string& key, const T& default_value) {
  T value;
  nh.param<T>(key, value, default_value);
  return value;
}

template <class T>
T waitForParam(const ros::NodeHandle& nh, const std::string& key) {
  T value;

  ros::Rate rate(1.0);
  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_INFO("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

geometry_msgs::TransformStamped waitForTransform(const tf2_ros::Buffer& tf_buffer, const std::string& from,
                                                 const std::string& to) {
  ros::Rate rate(1.0);
  while (ros::ok()) {
    try {
      const geometry_msgs::TransformStamped transform =
          tf_buffer.lookupTransform(from, to, ros::Time(0), ros::Duration(0.1));
      return transform;
    } catch (tf2::TransformException& ex) {
      ROS_INFO("waiting for transform from `%s` to `%s` ...", from.c_str(), to.c_str());
      rate.sleep();
    }
  }
}

geometry_msgs::PoseStamped transform2pose(const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}
}  // namespace

namespace behavior_planning {
BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
  // Subscribers
  path_with_lane_id_sub_ =
      pnh_.subscribe("input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::pathWithLaneIdCallback, this);
  perception_sub_ = pnh_.subscribe("input/perception", 1, &BehaviorVelocityPlannerNode::perceptionCallback, this);
  pointcloud_sub_ = pnh_.subscribe("input/pointcloud", 1, &BehaviorVelocityPlannerNode::pointcloudCallback, this);
  vehicle_velocity_sub_ =
      pnh_.subscribe("input/vehicle/velocity", 1, &BehaviorVelocityPlannerNode::velocityCallback, this);
  map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 10, &BehaviorVelocityPlannerNode::mapCallback, this);
  traffic_light_states_sub_ =
      pnh_.subscribe("input/traffic_light_states", 10, &BehaviorVelocityPlannerNode::trafficLightStatesCallback, this);

  // Publishers
  path_pub_ = pnh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/path", 1);

  // Parameters
  pnh_.param("foward_path_length", foward_path_length_, 1000.0);
  pnh_.param("backward_path_length", backward_path_length_, 5.0);

  // Vehicle Parameters
  planner_data_.wheel_base = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");
  planner_data_.front_overhang = waitForParam<double>(pnh_, "/vehicle_info/front_overhang");
  planner_data_.vehicle_width = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
  // TODO(Kenji Miyake): get from additional vehicle_info?
  planner_data_.base_link2front = planner_data_.front_overhang + planner_data_.wheel_base;

  // Initialize PlannerManager
  planner_manager_ptr_ = std::make_shared<BehaviorVelocityPlannerManager>();

  if (getParam<bool>(pnh_, "launch_momentary_stop", true))
    planner_manager_ptr_->launchSceneModule(std::make_shared<MomentaryStopModuleManager>());
  if (getParam<bool>(pnh_, "launch_crosswalk", true))
    planner_manager_ptr_->launchSceneModule(std::make_shared<CrosswalkModuleManager>());
  if (getParam<bool>(pnh_, "launch_traffic_light", true))
    planner_manager_ptr_->launchSceneModule(std::make_shared<TrafficLightModuleManager>());
  if (getParam<bool>(pnh_, "launch_intersection", true))
    planner_manager_ptr_->launchSceneModule(std::make_shared<IntersectionModuleManager>());
  if (getParam<bool>(pnh_, "launch_blind_spot", true))
    planner_manager_ptr_->launchSceneModule(std::make_shared<BlindSpotModuleManager>());

  // Wait for first current pose
  getCurrentPose();
}

geometry_msgs::PoseStamped BehaviorVelocityPlannerNode::getCurrentPose() {
  return transform2pose(waitForTransform(tf_buffer_, "map", "base_link"));
}

void BehaviorVelocityPlannerNode::perceptionCallback(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr& msg) {
  planner_data_.dynamic_objects = msg;
}

void BehaviorVelocityPlannerNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  planner_data_.no_ground_pointcloud_msg = msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_pointcloud;
  pcl::fromROSMsg(*msg, *no_ground_pointcloud);
  planner_data_.no_ground_pointcloud = no_ground_pointcloud;
}

void BehaviorVelocityPlannerNode::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  planner_data_.current_velocity = msg;
}

void BehaviorVelocityPlannerNode::mapCallback(const autoware_lanelet2_msgs::MapBin::ConstPtr& msg) {
  planner_data_.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, planner_data_.lanelet_map, &planner_data_.traffic_rules,
                                         &planner_data_.routing_graph);
}

void BehaviorVelocityPlannerNode::trafficLightStatesCallback(
    const autoware_traffic_light_msgs::TrafficLightStateArray::ConstPtr& msg) {
  for (const auto& state : msg->states) {
    planner_data_.traffic_light_id_map_[state.id] = {msg->header, state};
  }
}

void BehaviorVelocityPlannerNode::pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId& input_path_msg) {
  // Prepare planner data
  planner_data_.current_pose = getCurrentPose();
  const auto planner_data = std::make_shared<const PlannerData>(planner_data_);

  autoware_planning_msgs::PathWithLaneId veloctiy_planed_path;
  if (planner_manager_ptr_->callback(planner_data, input_path_msg, veloctiy_planed_path)) {
    // convert
    autoware_planning_msgs::Path path;
    for (const auto& path_point : veloctiy_planed_path.points) {
      path.points.push_back(path_point.point);
    }
    // screening
    autoware_planning_msgs::Path filtered_path;
    filterLitterPathPoint(path, filtered_path);
    // interpolation
    autoware_planning_msgs::Path interpolated_path_msg;
    interpolatePath(filtered_path, foward_path_length_, interpolated_path_msg);
    // check stop point
    autoware_planning_msgs::Path output_path_msg;
    filterStopPathPoint(interpolated_path_msg, output_path_msg);

    output_path_msg.header.frame_id = "map";
    output_path_msg.header.stamp = ros::Time::now();

    // TODO: This must be updated in each scene module, but copy from input message for now.
    output_path_msg.drivable_area = input_path_msg.drivable_area;

    path_pub_.publish(output_path_msg);
    publishDebugMarker(output_path_msg, debug_viz_pub_);
  }

  return;
};

void BehaviorVelocityPlannerNode::publishDebugMarker(const autoware_planning_msgs::Path& path,
                                                     const ros::Publisher& pub) {
  if (pub.getNumSubscribers() < 1) return;
  visualization_msgs::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header = path.header;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  pub.publish(output_msg);
}
}  // namespace behavior_planning
