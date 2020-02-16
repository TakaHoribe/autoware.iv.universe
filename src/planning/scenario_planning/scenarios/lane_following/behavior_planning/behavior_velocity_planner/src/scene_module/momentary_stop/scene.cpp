#include <scene_module/momentary_stop/scene.h>

#include <scene_module/momentary_stop/manager.h>

namespace behavior_planning {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

MomentaryStopModule::MomentaryStopModule(MomentaryStopModuleManager* manager_ptr,
                                         const lanelet::ConstLineString3d& stop_line, const int lane_id)
    : manager_ptr_(manager_ptr), state_(State::APPROARCH), stop_line_(stop_line), stop_margin_(0.0), lane_id_(lane_id) {
  if (manager_ptr_ != nullptr) manager_ptr_->registerTask(stop_line);
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
      const double base_link2front = planner_data_->base_link2front;
      double length_sum = 0;

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
      stop_point_with_lane_id = output.points.at(std::max(static_cast<int>(insert_stop_point_idx - 1), 0));
      stop_point_with_lane_id.point.pose.position.x = stop_point.x();
      stop_point_with_lane_id.point.pose.position.y = stop_point.y();
      stop_point_with_lane_id.point.twist.linear.x = 0.0;
      manager_ptr_->debuger.pushStopPose(stop_point_with_lane_id.point.pose);

      // insert stop point
      output.points.insert(output.points.begin() + insert_stop_point_idx, stop_point_with_lane_id);

      // insert 0 velocity after stop point
      for (size_t j = insert_stop_point_idx; j < output.points.size(); ++j)
        output.points.at(j).point.twist.linear.x = 0.0;
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

}  // namespace behavior_planning
