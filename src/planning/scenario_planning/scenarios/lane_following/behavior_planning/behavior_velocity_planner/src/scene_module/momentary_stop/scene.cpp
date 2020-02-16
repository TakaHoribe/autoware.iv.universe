#include <scene_module/momentary_stop/scene.h>

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

MomentaryStopModule::MomentaryStopModule(const int64_t module_id, const lanelet::ConstLineString3d& stop_line)
    : SceneModuleInterface(module_id), stop_line_(stop_line), state_(State::APPROARCH) {}

bool MomentaryStopModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) {
  Eigen::Vector2d stop_point;
  bg::model::linestring<Point> stop_line = {{stop_line_[0].x(), stop_line_[0].y()},
                                            {stop_line_[1].x(), stop_line_[1].y()}};

  if (state_ == State::APPROARCH) {
    for (size_t i = 0; i < path->points.size() - 1; ++i) {
      bg::model::linestring<Point> line = {
          {path->points.at(i).point.pose.position.x, path->points.at(i).point.pose.position.y},
          {path->points.at(i + 1).point.pose.position.x, path->points.at(i + 1).point.pose.position.y}};
      std::vector<Point> collision_points;
      bg::intersection(stop_line, line, collision_points);
      if (collision_points.empty()) continue;

      // search stop point index
      size_t insert_stop_point_idx = 0;
      const double base_link2front = planner_data_->base_link2front;
      double length_sum = 0;

      const double stop_length = stop_margin_ + base_link2front;
      Eigen::Vector2d point1, point2;
      point1 << collision_points.at(0).x(), collision_points.at(0).y();
      point2 << path->points.at(i).point.pose.position.x, path->points.at(i).point.pose.position.y;
      length_sum += (point2 - point1).norm();
      for (size_t j = i; 0 < j; --j) {
        if (stop_length < length_sum) {
          insert_stop_point_idx = j + 1;
          break;
        }
        point1 << path->points.at(j).point.pose.position.x, path->points.at(j).point.pose.position.y;
        point2 << path->points.at(j - 1).point.pose.position.x, path->points.at(j - 1).point.pose.position.y;
        length_sum += (point2 - point1).norm();
      }

      // create stop point
      autoware_planning_msgs::PathPointWithLaneId stop_point_with_lane_id;
      getBackwordPointFromBasePoint(point2, point1, point2, length_sum - stop_length, stop_point);
      stop_point_with_lane_id = path->points.at(std::max(static_cast<int>(insert_stop_point_idx - 1), 0));
      stop_point_with_lane_id.point.pose.position.x = stop_point.x();
      stop_point_with_lane_id.point.pose.position.y = stop_point.y();
      stop_point_with_lane_id.point.twist.linear.x = 0.0;
      manager_ptr_->debuger.pushStopPose(stop_point_with_lane_id.point.pose);

      // insert stop point
      path->points.insert(path->points.begin() + insert_stop_point_idx, stop_point_with_lane_id);

      // insert 0 velocity after stop point
      for (size_t j = insert_stop_point_idx; j < path->points.size(); ++j)
        path->points.at(j).point.twist.linear.x = 0.0;
      break;
    }

    // update state
    geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;
    const double x = stop_point.x() - self_pose.pose.position.x;
    const double y = stop_point.y() - self_pose.pose.position.y;
    const double dist = std::sqrt(x * x + y * y);
    if (dist < 2.0 && planner_data_->isVehicleStopping()) state_ = State::STOP;
    return true;
  } else if (state_ == State::STOP) {
    if (!planner_data_->isVehicleStopping()) state_ = State::START;
    return true;
  }
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
