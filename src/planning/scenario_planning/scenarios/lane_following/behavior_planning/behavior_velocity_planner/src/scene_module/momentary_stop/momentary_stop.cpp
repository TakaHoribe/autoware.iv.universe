#include <behavior_velocity_planner/api.hpp>
#include <scene_module/momentary_stop/momentary_stop.hpp>

namespace behavior_planning {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

MomentaryStopModule::MomentaryStopModule(MomentaryStopModuleManager* manager_ptr,
                                         const lanelet::ConstLineString3d& stop_line, const int lane_id)
    : manager_ptr_(manager_ptr),
      state_(State::APPROARCH),
      stop_line_(stop_line),
      stop_margin_(0.0),
      lane_id_(lane_id),
      task_id_(boost::uuids::random_generator()()) {
  if (manager_ptr_ != nullptr) manager_ptr_->registerTask(stop_line, task_id_);
}

MomentaryStopModule::~MomentaryStopModule() {
  if (manager_ptr_ != nullptr) manager_ptr_->unregisterTask(task_id_);
}

bool MomentaryStopModule::run(const autoware_planning_msgs::PathWithLaneId& input,
                              autoware_planning_msgs::PathWithLaneId& output) {
  output = input;
  Eigen::Vector2d stop_point;
  bg::model::linestring<Point> stop_line = {{stop_line_[0].x(), stop_line_[0].y()},
                                            {stop_line_[1].x(), stop_line_[1].y()}};

  if (state_ == State::APPROARCH) {
    for (size_t i = 0; i < output.points.size() - 1; ++i) {
      bg::model::linestring<Point> line = {
          {output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
          {output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y}};
      std::vector<Point> collision_points;
      bg::intersection(stop_line, line, collision_points);
      if (collision_points.empty()) continue;

      // search stop point index
      size_t insert_stop_point_idx = 0;
      double base_link2front;
      double length_sum = 0;
      if (!getBaselink2FrontLength(base_link2front)) {
        ROS_ERROR("cannot get vehicle front to base_link");
        return false;
      }
      const double stop_length = stop_margin_ + base_link2front;
      Eigen::Vector2d point1, point2;
      point1 << collision_points.at(0).x(), collision_points.at(0).y();
      point2 << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y;
      length_sum += (point2 - point1).norm();
      for (size_t j = i; 0 < j; --j) {
        if (stop_length < length_sum) {
          insert_stop_point_idx = j + 1;
          break;
        }
        point1 << output.points.at(j).point.pose.position.x, output.points.at(j).point.pose.position.y;
        point2 << output.points.at(j - 1).point.pose.position.x, output.points.at(j - 1).point.pose.position.y;
        length_sum += (point2 - point1).norm();
      }

      // create stop point
      autoware_planning_msgs::PathPointWithLaneId stop_point_with_lane_id;
      getBackwordPointFromBasePoint(point2, point1, point2, length_sum - stop_length, stop_point);
      stop_point_with_lane_id = output.points.at(std::max(int(insert_stop_point_idx - 1), 0));
      stop_point_with_lane_id.point.pose.position.x = stop_point.x();
      stop_point_with_lane_id.point.pose.position.y = stop_point.y();
      stop_point_with_lane_id.point.twist.linear.x = 0.0;

      // insert stop point
      output.points.insert(output.points.begin() + insert_stop_point_idx, stop_point_with_lane_id);

      // insert 0 velocity after stop point
      for (size_t j = insert_stop_point_idx; j < output.points.size(); ++j)
        output.points.at(j).point.twist.linear.x = 0.0;
      break;
    }

    // update state
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose)) return true;
    const double x = stop_point.x() - self_pose.pose.position.x;
    const double y = stop_point.y() - self_pose.pose.position.y;
    const double dist = std::sqrt(x * x + y * y);
    if (dist < 2.0 && isVehicleStopping()) state_ = State::STOP;
    return true;
  } else if (state_ == State::STOP) {
    if (!isVehicleStopping()) state_ = State::START;
    return true;
  }
}
bool MomentaryStopModule::endOfLife(const autoware_planning_msgs::PathWithLaneId& input) {
  bool is_end_of_life = false;
  // geometry_msgs::PoseStamped self_pose;
  // if (!getCurrentSelfPose(self_pose))
  //     return false;
  // const double stop_point_x = (stop_line_[0].x() + stop_line_[1].x()) / 2.0;
  // const double stop_point_y = (stop_line_[0].y() + stop_line_[1].y()) / 2.0;
  // const double x = stop_point_x - self_pose.pose.position.x;
  // const double y = stop_point_y - self_pose.pose.position.y;
  // const double dist = std::sqrt(x * x + y * y);
  // if (state_ == State::START && 5.0 < dist)
  //     is_end_of_life = true;

  bool found = false;
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      if (lane_id_ == input.points.at(i).lane_ids.at(j)) found = true;
    }
  }

  // is_end_of_life = (!found && state_ == State::START);
  is_end_of_life = !found;
  return is_end_of_life;
}

bool MomentaryStopModule::getBackwordPointFromBasePoint(const Eigen::Vector2d& line_point1,
                                                        const Eigen::Vector2d& line_point2,
                                                        const Eigen::Vector2d& base_point, const double backward_length,
                                                        Eigen::Vector2d& output_point) {
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

bool MomentaryStopModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                                std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  geometry_msgs::PoseStamped self_pose;
  if (!getCurrentSelfPose(self_pose)) return false;
  lanelet::LaneletMapConstPtr lanelet_map_ptr;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
  if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr)) return false;
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems =
          (lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j)))
              .regulatoryElementsAs<const lanelet::TrafficSign>();
      if (traffic_sign_reg_elems.empty()) continue;
      std::shared_ptr<const lanelet::TrafficSign> traffic_sign = traffic_sign_reg_elems.front();
      if (traffic_sign->type() == "stop_sign") {
        lanelet::ConstLineStrings3d traffic_sign_stoplines = traffic_sign->refLines();
        for (const auto& traffic_sign_stopline : traffic_sign_stoplines) {
          if (!isRunning(traffic_sign_stopline)) {
            v_module_ptr.push_back(
                std::make_shared<MomentaryStopModule>(this, traffic_sign_stopline, input.points.at(i).lane_ids.at(j)));
          }
        }
      }
    }
  }
  return true;
}

bool MomentaryStopModuleManager::isRunning(const lanelet::ConstLineString3d& stop_line) {
  if (task_id_direct_map_.count(stop_line) == 0) return false;
  return true;
}

bool MomentaryStopModuleManager::registerTask(const lanelet::ConstLineString3d& stop_line,
                                              const boost::uuids::uuid& uuid) {
  ROS_INFO("Registered Momentary Stop Task");
  task_id_direct_map_.emplace(stop_line, boost::lexical_cast<std::string>(uuid));
  task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), stop_line);
  return true;
}
bool MomentaryStopModuleManager::unregisterTask(const boost::uuids::uuid& uuid) {
  ROS_INFO("Unregistered Momentary Stop Task");
  task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
  task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
  return true;
}

}  // namespace behavior_planning