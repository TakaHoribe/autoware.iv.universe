#include "scenario_selector_node.h"

#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>

namespace {

template <class T>
void onData(const T& data, T* buffer) {
  *buffer = data;
}

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
    const std::shared_ptr<lanelet::LaneletMap>& lanelet_map_ptr,
    const lanelet::BasicPoint2d& search_point) {
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
      lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, 1);

  if (nearest_lanelets.empty()) {
    return {};
  }

  const auto nearest_lanelet = nearest_lanelets.front().second;
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);

  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(nearest_lanelet, all_parking_lots,
                                                                 linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

bool isInParkingLot(const std::shared_ptr<lanelet::LaneletMap>& lanelet_map_ptr,
                    const geometry_msgs::Pose& current_pose) {
  const auto& p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
      findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

bool isNearTrajectoryEnd(const autoware_planning_msgs::Trajectory::ConstPtr& trajectory,
                         const geometry_msgs::Pose& current_pose, const double th_dist) {
  if (!trajectory || trajectory->points.empty()) {
    return false;
  }

  const auto& p1 = current_pose.position;
  const auto& p2 = trajectory->points.back().pose.position;

  const auto dist = std::hypot(p1.x - p2.x, p1.y - p2.y);

  return dist < th_dist;
}

}  // namespace

// TODO(Kenji Miyake): Manage states in mission_planner
autoware_planning_msgs::Scenario ScenarioSelectorNode::selectScenario() {
  autoware_planning_msgs::Scenario scenario;

  scenario.activating_scenarios.push_back(autoware_planning_msgs::Scenario::LaneFollowing);

  const auto is_in_parking_lot = isInParkingLot(lanelet_map_ptr_, current_pose_->pose);
  const auto is_near_trajectory_end =
      isNearTrajectoryEnd(input_lane_following_.buf_trajectory, current_pose_->pose, 0.5);

  if (is_in_parking_lot && is_near_trajectory_end) {
    current_scenario_ = autoware_planning_msgs::Scenario::Parking;
  }
  if (!is_in_parking_lot) {
    current_scenario_ = autoware_planning_msgs::Scenario::LaneFollowing;
  }

  if (current_scenario_ == autoware_planning_msgs::Scenario::Parking) {
    scenario.activating_scenarios.push_back(autoware_planning_msgs::Scenario::Parking);
  }

  scenario.current_scenario = current_scenario_;

  return scenario;
}

void ScenarioSelectorNode::onMap(const autoware_lanelet2_msgs::MapBin& msg) {
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_ptr_, &traffic_rules_ptr_,
                                         &routing_graph_ptr_);
}

void ScenarioSelectorNode::onRoute(const autoware_planning_msgs::Route::ConstPtr& msg) {
  route_ = msg;
}

void ScenarioSelectorNode::onTimer(const ros::TimerEvent& event) {
  // Check activation conditions
  if (!lanelet_map_ptr_ || !route_) {
    return;
  }

  // Get current pose
  {
    geometry_msgs::TransformStamped tf_current_pose;

    try {
      tf_current_pose =
          tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException ex) {
      ROS_ERROR("[scenario_selector] %s", ex.what());
    }

    geometry_msgs::PoseStamped::Ptr p(new geometry_msgs::PoseStamped);
    p->header = tf_current_pose.header;
    p->pose.orientation = tf_current_pose.transform.rotation;
    p->pose.position.x = tf_current_pose.transform.translation.x;
    p->pose.position.y = tf_current_pose.transform.translation.y;
    p->pose.position.z = tf_current_pose.transform.translation.z;
    current_pose_ = geometry_msgs::PoseStamped::ConstPtr(p);
  }

  if (!current_pose_) {
    return;
  }

  // Select scenario
  const auto scenario = selectScenario();
  output_.pub_scenario.publish(scenario);

  const auto& input = [&]() {
    if (scenario.current_scenario == autoware_planning_msgs::Scenario::LaneFollowing) {
      return input_lane_following_;
    }
    if (scenario.current_scenario == autoware_planning_msgs::Scenario::Parking) {
      return input_parking_;
    }
  }();

  if (!input.buf_trajectory) {
    return;
  }

  // Output
  constexpr double max_delay = 0.5;
  const auto now = ros::Time::now();
  if ((now - input.buf_trajectory->header.stamp).toSec() <= max_delay) {
    output_.pub_trajectory.publish(input.buf_trajectory);
  }
}

#define CALLBACK(buffer)                                       \
  static_cast<boost::function<void(const decltype(buffer)&)>>( \
      boost::bind(onData<decltype(buffer)>, _1, &buffer))

ScenarioSelectorNode::ScenarioSelectorNode()
    : nh_(""),
      private_nh_("~"),
      tf_listener_(tf_buffer_),
      current_scenario_(autoware_planning_msgs::Scenario::LaneFollowing) {
  // Input
  input_lane_following_.sub_trajectory = private_nh_.subscribe(
      "input/lane_following/trajectory", 1, CALLBACK(input_lane_following_.buf_trajectory));

  input_parking_.sub_trajectory =
      private_nh_.subscribe("input/parking/trajectory", 1, CALLBACK(input_parking_.buf_trajectory));

  sub_lanelet_map_ =
      private_nh_.subscribe("input/lanelet_map", 1, &ScenarioSelectorNode::onMap, this);

  sub_route_ = private_nh_.subscribe("input/route", 1, &ScenarioSelectorNode::onRoute, this);

  // Output
  output_.pub_scenario =
      private_nh_.advertise<autoware_planning_msgs::Scenario>("output/scenario", 1);
  output_.pub_trajectory =
      private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);

  // Timer Callback
  constexpr double update_rate = 10;
  timer_ = private_nh_.createTimer(ros::Rate(update_rate), &ScenarioSelectorNode::onTimer, this);
}
