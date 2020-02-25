#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <behavior_velocity_planner/api.hpp>
#include <cmath>
#include <scene_module/crosswalk/crosswalk.hpp>
namespace behavior_planning {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

CrosswalkModule::CrosswalkModule(CrosswalkModuleManager* manager_ptr, const lanelet::ConstLanelet& crosswalk,
                                 const int lane_id)
    : manager_ptr_(manager_ptr),
      state_(State::APPROARCH),
      crosswalk_(crosswalk),
      stop_margin_(1.0),
      stop_dynamic_object_prediction_time_margin_(3.0),
      slow_margin_(2.0),
      lane_id_(lane_id),
      task_id_(boost::uuids::random_generator()()) {
  if (manager_ptr_ != nullptr) manager_ptr_->registerTask(crosswalk_, task_id_);
}

bool CrosswalkModule::run(const autoware_planning_msgs::PathWithLaneId& input,
                          autoware_planning_msgs::PathWithLaneId& output) {
  output = input;

  // create polygon
  lanelet::CompoundPolygon3d lanelet_polygon = crosswalk_.polygon3d();
  Polygon polygon;
  for (const auto& lanelet_point : lanelet_polygon) {
    polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
  }
  polygon.outer().push_back(polygon.outer().front());

  // check state
  geometry_msgs::PoseStamped self_pose;
  if (!getCurrentSelfPose(self_pose)) return false;
  if (bg::within(Point(self_pose.pose.position.x, self_pose.pose.position.y), polygon))
    state_ = State::INSIDE;
  else if (state_ == State::INSIDE)
    state_ = State::GO_OUT;

  if (state_ == State::APPROARCH) {
    // check person in polygon
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr =
        std::make_shared<autoware_perception_msgs::DynamicObjectArray>();
    if (!getDynemicObjects(objects_ptr)) {
      ROS_ERROR_THROTTLE(1, "cannot recieve dynamic object. maybe don't stop at crosswalk");
      // return false;
    }

    // get no ground pointcloud (map frame)
    std::shared_ptr<sensor_msgs::PointCloud2 const> no_ground_ros_pointcloud_ptr= std::make_shared<sensor_msgs::PointCloud2 const>();
    if (!getNoGroundPointcloud(no_ground_ros_pointcloud_ptr)) {
      ROS_ERROR_THROTTLE(1, "cannot recieve pointcloud. maybe don't stop at crosswalk");
    }
    geometry_msgs::TransformStamped transform_stamped;
    if (!getTransform("map", no_ground_ros_pointcloud_ptr->header.frame_id, no_ground_ros_pointcloud_ptr->header.stamp,
                      ros::Duration(0.5), transform_stamped)) {
      ROS_ERROR_THROTTLE(1, "cannot transform pointcloud. don't stop at crosswalk");
      return false;
    }
    Eigen::Matrix4f affine_matrix = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    sensor_msgs::PointCloud2 transformed_ros_pointcloud;
    pcl_ros::transformPointCloud(affine_matrix, *no_ground_ros_pointcloud_ptr, transformed_ros_pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_ros_pointcloud, *no_ground_pointcloud_ptr);

    // check areas
    autoware_planning_msgs::PathWithLaneId slow_path, stop_path;
    // check slow area
    if (!checkSlowArea(input, polygon, objects_ptr, no_ground_pointcloud_ptr, slow_path)) {
      return false;
    }

    // check stop area
    if (!checkStopArea(slow_path, polygon, objects_ptr, no_ground_pointcloud_ptr, stop_path)) {
      return false;
    }
    // stop_path = slow_path;
    output = stop_path;
  }
  return true;
}

bool CrosswalkModule::endOfLife(const autoware_planning_msgs::PathWithLaneId& input) {
  bool is_end_of_life = false;

  bool found = false;
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      if (lane_id_ == input.points.at(i).lane_ids.at(j)) found = true;
    }
  }

  is_end_of_life = !found;
  if (is_end_of_life)
    if (manager_ptr_ != nullptr) manager_ptr_->unregisterTask(task_id_);
  return is_end_of_life;
}

bool CrosswalkModule::checkStopArea(
    const autoware_planning_msgs::PathWithLaneId& input,
    const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false>& crosswalk_polygon,
    const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const>& objects_ptr,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& no_ground_pointcloud_ptr,
    autoware_planning_msgs::PathWithLaneId& output) {
  output = input;
  bool pedestrian_found = false;
  bool object_found = false;
  ros::Time current_time = ros::Time::now();

  // create stop area
  std::vector<Point> path_collision_points;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    Line line = {{output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
                 {output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y}};
    std::vector<Point> line_collision_points;
    bg::intersection(crosswalk_polygon, line, line_collision_points);
    if (line_collision_points.empty()) continue;
    for (size_t j = 0; j < line_collision_points.size(); ++j) {
      path_collision_points.push_back(line_collision_points.at(j));
    }
  }
  if (path_collision_points.size() != 2) {
    ROS_ERROR_THROTTLE(1, "Must be 2. Size is %d", (int)path_collision_points.size());
    return false;
  }
  double width;
  if (!getVehicleWidth(width)) {
    ROS_ERROR("cannot get vehicle width");
    return false;
  }
  const double yaw = std::atan2(path_collision_points.at(1).y() - path_collision_points.at(0).y(),
                                path_collision_points.at(1).x() - path_collision_points.at(0).x()) +
                     M_PI_2;
  Polygon stop_polygon;
  const double extension_margin = 0.25;
  stop_polygon.outer().push_back(
      bg::make<Point>(path_collision_points.at(0).x() + std::cos(yaw) * ((width / 2.0) + extension_margin),
                      path_collision_points.at(0).y() + std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(
      bg::make<Point>(path_collision_points.at(0).x() - std::cos(yaw) * ((width / 2.0) + extension_margin),
                      path_collision_points.at(0).y() - std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(
      bg::make<Point>(path_collision_points.at(1).x() - std::cos(yaw) * ((width / 2.0) + extension_margin),
                      path_collision_points.at(1).y() - std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(
      bg::make<Point>(path_collision_points.at(1).x() + std::cos(yaw) * ((width / 2.0) + extension_margin),
                      path_collision_points.at(1).y() + std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(stop_polygon.outer().front());

  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < stop_polygon.outer().size(); ++i) {
    Eigen::Vector3d point;
    point << stop_polygon.outer().at(i).x(), stop_polygon.outer().at(i).y(), 0.0;
    points.push_back(point);
  }
  manager_ptr_->debuger.pushStopPolygon(points);
  // ----------------
  // check object pointcloud
  for (size_t i = 0; i < no_ground_pointcloud_ptr->size(); ++i) {
    Point point(no_ground_pointcloud_ptr->at(i).x, no_ground_pointcloud_ptr->at(i).y);
    if (bg::within(point, stop_polygon)) {
      object_found = true;
    }
  }

  // check pedestrian
  for (const auto& object : objects_ptr->objects) {
    if (object.semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN ||
        object.semantic.type == autoware_perception_msgs::Semantic::BICYCLE) {
      Point point(object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y);
      if (!bg::within(point, crosswalk_polygon)) continue;
      if (bg::within(point, stop_polygon)) {
        pedestrian_found = true;
      }
      for (const auto& object_path : object.state.predicted_paths) {
        for (size_t k = 0; k < object_path.path.size() - 1; ++k) {
          if ((current_time - object_path.path.at(k).header.stamp).toSec() <
              stop_dynamic_object_prediction_time_margin_) {
            Line line = {
                {object_path.path.at(k).pose.pose.position.x, object_path.path.at(k).pose.pose.position.y},
                {object_path.path.at(k + 1).pose.pose.position.x, object_path.path.at(k + 1).pose.pose.position.y}};
            std::vector<Point> line_collision_points;
            bg::intersection(stop_polygon, line, line_collision_points);
            if (!line_collision_points.empty()) pedestrian_found = true;
          }
        }
      }
    }
  }

  if (!pedestrian_found && !object_found) return true;

  // insert stop point
  if (!insertTargetVelocityPoint(input, crosswalk_polygon, stop_margin_, 0.0, output)) return false;
  return true;
}

bool CrosswalkModule::checkSlowArea(
    const autoware_planning_msgs::PathWithLaneId& input,
    const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false>& polygon,
    const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const>& objects_ptr,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& no_ground_pointcloud_ptr,
    autoware_planning_msgs::PathWithLaneId& output) {
  const double slow_velocity = 1.39;  // 5kmph
  output = input;
  bool pedestrian_found = false;
  for (size_t i = 0; i < objects_ptr->objects.size(); ++i) {
    if (objects_ptr->objects.at(i).semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN) {
      Point point(objects_ptr->objects.at(i).state.pose_covariance.pose.position.x,
                  objects_ptr->objects.at(i).state.pose_covariance.pose.position.y);
      if (bg::within(point, polygon)) {
        pedestrian_found = true;
      }
    }
  }
  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < polygon.outer().size(); ++i) {
    Eigen::Vector3d point;
    point << polygon.outer().at(i).x(), polygon.outer().at(i).y(), 0.0;
    points.push_back(point);
  }
  manager_ptr_->debuger.pushSlowPolygon(points);
  // ----------------

  if (!pedestrian_found) return true;

  // insert slow point
  if (!insertTargetVelocityPoint(input, polygon, slow_margin_, slow_velocity, output)) return false;
  return true;
}

bool CrosswalkModule::insertTargetVelocityPoint(
    const autoware_planning_msgs::PathWithLaneId& input,
    const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false>& polygon,
    const double& margin, const double& velocity, autoware_planning_msgs::PathWithLaneId& output) {
  output = input;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    Line line = {{output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
                 {output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y}};
    std::vector<Point> collision_points;
    bg::intersection(polygon, line, collision_points);

    if (collision_points.empty()) continue;
    // -- debug code --
    for (const auto& collision_point : collision_points) {
      Eigen::Vector3d point3d;
      point3d << collision_point.x(), collision_point.y(), 0;
      manager_ptr_->debuger.pushCollisionPoint(point3d);
    }
    std::vector<Eigen::Vector3d> line3d;
    Eigen::Vector3d point3d;
    point3d << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y,
        output.points.at(i).point.pose.position.z;
    line3d.push_back(point3d);
    point3d << output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y,
        output.points.at(i + 1).point.pose.position.z;
    line3d.push_back(point3d);
    manager_ptr_->debuger.pushCollisionLine(line3d);
    // ----------------

    // check nearest collision point
    Point nearest_collision_point;
    double min_dist;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist =
          bg::distance(Point(output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y),
                       collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
      }
    }

    // search target point index
    size_t insert_target_point_idx = 0;
    double base_link2front;
    double length_sum = 0;
    if (!getBaselink2FrontLength(base_link2front)) {
      ROS_ERROR("cannot get vehicle front to base_link");
      return false;
    }
    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    point1 << nearest_collision_point.x(), nearest_collision_point.y();
    point2 << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y;
    length_sum += (point2 - point1).norm();
    for (size_t j = i; 0 < j; --j) {
      if (target_length < length_sum) {
        insert_target_point_idx = j + 1;
        break;
      }
      point1 << output.points.at(j).point.pose.position.x, output.points.at(j).point.pose.position.y;
      point2 << output.points.at(j - 1).point.pose.position.x, output.points.at(j - 1).point.pose.position.y;
      length_sum += (point2 - point1).norm();
    }

    // create target point
    Eigen::Vector2d target_point;
    autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;
    getBackwordPointFromBasePoint(point2, point1, point2, length_sum - target_length, target_point);
    target_point_with_lane_id = output.points.at(std::max(int(insert_target_point_idx - 1), 0));
    target_point_with_lane_id.point.pose.position.x = target_point.x();
    target_point_with_lane_id.point.pose.position.y = target_point.y();
    target_point_with_lane_id.point.twist.linear.x = velocity;
    // -- debug code --
    if (velocity == 0.0)
      manager_ptr_->debuger.pushStopPose(target_point_with_lane_id.point.pose);
    else
      manager_ptr_->debuger.pushSlowPose(target_point_with_lane_id.point.pose);
    // ----------------

    // insert target point
    output.points.insert(output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

    // insert 0 velocity after target point
    for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
      output.points.at(j).point.twist.linear.x = std::min(velocity, output.points.at(j).point.twist.linear.x);
    return true;
  }
  return false;
}

bool CrosswalkModule::getBackwordPointFromBasePoint(const Eigen::Vector2d& line_point1,
                                                    const Eigen::Vector2d& line_point2,
                                                    const Eigen::Vector2d& base_point, const double backward_length,
                                                    Eigen::Vector2d& output_point) {
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

CrosswalkModuleManager::CrosswalkModuleManager() {
  lanelet::LaneletMapConstPtr lanelet_map_ptr;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
  while (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr) && ros::ok()) {
    ros::spinOnce();
  }
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::traffic_rules::TrafficRulesPtr pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
      lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
      lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ = std::make_shared<lanelet::routing::RoutingGraphContainer>(overall_graphs);
}

bool CrosswalkModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  geometry_msgs::PoseStamped self_pose;
  if (!getCurrentSelfPose(self_pose)) return false;
  lanelet::LaneletMapConstPtr lanelet_map_ptr;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
  if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr)) {
    return false;
  }
  if (overall_graphs_ptr_ == nullptr) return false;

  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      lanelet::ConstLanelet road_lanelet = lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j));
      std::vector<lanelet::ConstLanelet> crosswalks = overall_graphs_ptr_->conflictingInGraph(road_lanelet, 1);
      for (const auto& crosswalk : crosswalks) {
        if (!isRunning(crosswalk))
          v_module_ptr.push_back(std::make_shared<CrosswalkModule>(this, crosswalk, input.points.at(i).lane_ids.at(j)));
        // -- debug code --
        std::vector<Eigen::Vector3d> points;
        for (const auto& lanelet_point : crosswalk.polygon3d()) {
          Eigen::Vector3d point;
          point << lanelet_point.x(), lanelet_point.y(), lanelet_point.z();
          points.push_back(point);
        }
        debuger.pushCrosswalkPolygon(points);
        // ----------------
      }
    }
  }
  return true;
}

bool CrosswalkModuleManager::run(const autoware_planning_msgs::PathWithLaneId& input,
                                 autoware_planning_msgs::PathWithLaneId& output) {
  autoware_planning_msgs::PathWithLaneId input_path = input;
  for (size_t i = 0; i < scene_modules_ptr_.size(); ++i) {
    autoware_planning_msgs::PathWithLaneId output_path;
    if (scene_modules_ptr_.at(i)->run(input_path, output_path)) input_path = output_path;
  }
  debuger.publish();
  output = input_path;
  return true;
}

bool CrosswalkModuleManager::isRunning(const lanelet::ConstLanelet& crosswalk) {
  if (task_id_direct_map_.count(crosswalk) == 0) return false;
  return true;
}

bool CrosswalkModuleManager::registerTask(const lanelet::ConstLanelet& crosswalk, const boost::uuids::uuid& uuid) {
  ROS_INFO("Registered Crosswalk Task");
  task_id_direct_map_.emplace(crosswalk, boost::lexical_cast<std::string>(uuid));
  task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), crosswalk);
  return true;
}

bool CrosswalkModuleManager::unregisterTask(const boost::uuids::uuid& uuid) {
  ROS_INFO("Unregistered Crosswalk Task");
  task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
  task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
  return true;
}

CrosswalkDebugMarkersManager::CrosswalkDebugMarkersManager() : nh_(), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/crosswalk", 1);
}

void CrosswalkDebugMarkersManager::pushCollisionLine(const std::vector<Eigen::Vector3d>& line) {
  collision_lines_.push_back(std::vector<Eigen::Vector3d>(line));
}

void CrosswalkDebugMarkersManager::pushCollisionLine(const std::vector<Eigen::Vector2d>& line) {
  std::vector<Eigen::Vector3d> line3d;
  for (const auto& point : line) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0.0;
    line3d.push_back(point3d);
  }
  pushCollisionLine(line3d);
}

void CrosswalkDebugMarkersManager::pushCollisionPoint(const Eigen::Vector3d& point) {
  collision_points_.push_back(Eigen::Vector3d(point));
}

void CrosswalkDebugMarkersManager::pushCollisionPoint(const Eigen::Vector2d& point) {
  Eigen::Vector3d point3d;
  point3d << point.x(), point.y(), 0.0;
  pushCollisionPoint(point3d);
}

void CrosswalkDebugMarkersManager::pushStopPose(const geometry_msgs::Pose& pose) {
  stop_poses_.push_back(geometry_msgs::Pose(pose));
}

void CrosswalkDebugMarkersManager::pushSlowPose(const geometry_msgs::Pose& pose) {
  slow_poses_.push_back(geometry_msgs::Pose(pose));
}

void CrosswalkDebugMarkersManager::pushCrosswalkPolygon(const std::vector<Eigen::Vector3d>& polygon) {
  crosswalk_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushCrosswalkPolygon(const std::vector<Eigen::Vector2d>& polygon) {
  std::vector<Eigen::Vector3d> polygon3d;
  for (const auto& point : polygon) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0;
    polygon3d.push_back(point3d);
  }
  pushCrosswalkPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::pushStopPolygon(const std::vector<Eigen::Vector3d>& polygon) {
  stop_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushStopPolygon(const std::vector<Eigen::Vector2d>& polygon) {
  std::vector<Eigen::Vector3d> polygon3d;
  for (const auto& point : polygon) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0;
    polygon3d.push_back(point3d);
  }
  pushStopPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::pushSlowPolygon(const std::vector<Eigen::Vector3d>& polygon) {
  slow_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushSlowPolygon(const std::vector<Eigen::Vector2d>& polygon) {
  std::vector<Eigen::Vector3d> polygon3d;
  for (const auto& point : polygon) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0;
    polygon3d.push_back(point3d);
  }
  pushSlowPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::publish() {
  visualization_msgs::MarkerArray msg;
  ros::Time current_time = ros::Time::now();
  double base_link2front;
  getBaselink2FrontLength(base_link2front);
  tf2::Transform tf_base_link2front(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front, 0.0, 0.0));

  // Crosswalk polygons
  for (size_t i = 0; i < crosswalk_polygons_.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = crosswalk_polygons_.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = "crosswalk polygon line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);

    marker.ns = "crosswalk polygon point";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision line
  for (size_t i = 0; i < collision_lines_.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "collision line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < collision_lines_.at(i).size(); ++j) {
      geometry_msgs::Point point;
      point.x = collision_lines_.at(i).at(j).x();
      point.y = collision_lines_.at(i).at(j).y();
      point.z = collision_lines_.at(i).at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision point
  if (!collision_points_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "collision point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < collision_points_.size(); ++j) {
      geometry_msgs::Point point;
      point.x = collision_points_.at(j).x();
      point.y = collision_points_.at(j).y();
      point.z = collision_points_.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Slow polygon
  for (size_t i = 0; i < slow_polygons_.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = slow_polygons_.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = "slow polygon line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!slow_poses_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "slow point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < slow_poses_.size(); ++j) {
      geometry_msgs::Point point;
      point.x = slow_poses_.at(j).position.x;
      point.y = slow_poses_.at(j).position.y;
      point.z = slow_poses_.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Stop polygon
  for (size_t i = 0; i < stop_polygons_.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = stop_polygons_.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = "stop polygon line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Stop point
  if (!stop_poses_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < stop_poses_.size(); ++j) {
      geometry_msgs::Point point;
      point.x = stop_poses_.at(j).position.x;
      point.y = stop_poses_.at(j).position.y;
      point.z = stop_poses_.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Stop Geofence
  for (size_t j = 0; j < stop_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop geofence";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(stop_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Factor Text
  for (size_t j = 0; j < stop_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "factor text";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(stop_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "crosswalk";
    msg.markers.push_back(marker);
  }

  // Slow Geofence
  for (size_t j = 0; j < slow_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "slow geofence";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(slow_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Slow Factor Text
  for (size_t j = 0; j < slow_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "slow factor text";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(slow_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "crosswalk";
    msg.markers.push_back(marker);
  }


  debug_viz_pub_.publish(msg);
  collision_points_.clear();
  stop_poses_.clear();
  slow_poses_.clear();
  collision_lines_.clear();
  crosswalk_polygons_.clear();
  slow_polygons_.clear();
  stop_polygons_.clear();

  return;
}

}  // namespace behavior_planning