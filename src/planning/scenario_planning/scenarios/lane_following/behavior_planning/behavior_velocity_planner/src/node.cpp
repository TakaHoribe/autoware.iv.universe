#include <behavior_velocity_planner/node.hpp>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_routing/Route.h>

#include <visualization_msgs/MarkerArray.h>

#include <utilization/path_utilization.hpp>

// Scene modules
#include <scene_module/blind_spot/blind_spot.hpp>
#include <scene_module/crosswalk/crosswalk.hpp>
#include <scene_module/intersection/manager.hpp>
#include <scene_module/momentary_stop/momentary_stop.hpp>
#include <scene_module/traffic_light/traffic_light.hpp>

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

    ROS_WARN("waiting for parameter `%s` ...", key);
    rate.sleep();
  }

  return {};
}
}  // namespace

namespace behavior_planning {
BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode() : nh_(), pnh_("~") {
  // Subscribers
  path_with_lane_id_sub_ =
      pnh_.subscribe("input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::pathWithLaneIdCallback, this);
  perception_sub_ = pnh_.subscribe("input/perception", 1, &SingletonDataManager::perceptionCallback,
                                   &SingletonDataManager::getInstance());
  pointcloud_sub_ = pnh_.subscribe("input/pointcloud", 1, &SingletonDataManager::pointcloudCallback,
                                   &SingletonDataManager::getInstance());
  vehicle_velocity_sub_ = pnh_.subscribe("input/vehicle/velocity", 1, &SingletonDataManager::velocityCallback,
                                         &SingletonDataManager::getInstance());
  map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 10, &SingletonDataManager::mapCallback,
                            &SingletonDataManager::getInstance());
  traffic_light_states_sub_ =
      pnh_.subscribe("input/traffic_light_states", 10, &SingletonDataManager::trafficLightStatesCallback,
                     &SingletonDataManager::getInstance());

  // Publishers
  path_pub_ = pnh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/path", 1);

  // Parameters
  pnh_.param("foward_path_length", foward_path_length_, 1000.0);
  pnh_.param("backward_path_length", backward_path_length_, 5.0);

  // Vehicle Parameters
  double wheel_base, front_overhang, vehicle_width;
  pnh_.param("/vehicle_info/wheel_base", wheel_base, 2.95);
  pnh_.param("/vehicle_info/front_overhang", front_overhang, 1.0);
  pnh_.param("/vehicle_info/vehicle_width", vehicle_width, 1.775);
  SingletonDataManager::getInstance().setWheelBase(wheel_base);
  SingletonDataManager::getInstance().setFrontOverhang(front_overhang);
  SingletonDataManager::getInstance().setVehicleWidth(vehicle_width);

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
}

void BehaviorVelocityPlannerNode::pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId& input_path_msg) {
  // if (path_pub_.getNumSubscribers() < 1)
  autoware_planning_msgs::PathWithLaneId veloctiy_planed_path;
  if (planner_manager_ptr_->callback(input_path_msg, veloctiy_planed_path)) {
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
    output_path_msg.drivable_area = input_path_msg.drivable_area;  // TODO: This must be updated in each scene module,
                                                                   // but copy from input message for now.
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
