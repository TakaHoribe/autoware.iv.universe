/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/utilities.h>

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <lanelet2_extension/utility/message_conversion.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <ros/ros.h>

#include <unordered_set>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

using autoware_planning_msgs::PathPointWithLaneId;
using autoware_planning_msgs::PathWithLaneId;
using lanelet::utils::to2D;
namespace
{
bool exists(const std::unordered_set<lanelet::Id>& set, const lanelet::Id& id);

template <typename T>
bool exists(const std::vector<T>& vectors, const T& item)
{
  for (const auto& i : vectors)
  {
    if (i == item)
    {
      return true;
    }
  }
  return false;
}
std::string toString(const geometry_msgs::Pose& pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

lanelet::Point3d toLaneletPoint(const geometry_msgs::Point point)
{
  return lanelet::Point3d(lanelet::InvalId, point.x, point.y, point.z);
}

double getAngleDifference(const double angle1, const double angle2)
{
  Eigen::Vector2d vec1, vec2;
  vec1 << std::cos(angle1), std::sin(angle1);
  vec2 << std::cos(angle2), std::sin(angle2);
  const double diff_angle = std::acos(vec1.dot(vec2));
  return std::fabs(diff_angle);
}

lanelet::ConstLineString3d getClosestSegment(const lanelet::BasicPoint2d& search_pt,
                                             const lanelet::ConstLineString3d& linestring)
{
  if (linestring.size() < 2)
  {
    return lanelet::LineString3d();
  }

  lanelet::ConstLineString3d closest_segment;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 1; i < linestring.size(); i++)
  {
    lanelet::BasicPoint3d prev_basic_pt = linestring[i - 1].basicPoint();
    lanelet::BasicPoint3d current_basic_pt = linestring[i].basicPoint();

    lanelet::Point3d prev_pt(lanelet::InvalId, prev_basic_pt.x(), prev_basic_pt.y(), prev_basic_pt.z());
    lanelet::Point3d current_pt(lanelet::InvalId, current_basic_pt.x(), current_basic_pt.y(), current_basic_pt.z());

    lanelet::LineString3d current_segment(lanelet::InvalId, { prev_pt, current_pt });
    double distance = lanelet::geometry::distance2d(to2D(current_segment).basicLineString(), search_pt);
    if (distance < min_distance)
    {
      closest_segment = current_segment;
      min_distance = distance;
    }
  }
  return closest_segment;
}

}  // namespace

namespace lane_change_planner
{
RouteHandler::RouteHandler() : is_route_msg_ready_(false), is_map_msg_ready_(false), is_handler_ready_(false)
{
}

void RouteHandler::mapCallback(const autoware_lanelet2_msgs::MapBin& map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setRouteLanelets();
}

void RouteHandler::routeCallback(const autoware_planning_msgs::Route& route_msg)
{
  route_msg_ = route_msg;
  is_route_msg_ready_ = true;
  is_handler_ready_ = false;

  setRouteLanelets();
}

bool RouteHandler::isHandlerReady()
{
  return is_handler_ready_;
}

void RouteHandler::setRouteLanelets()
{
  if (!is_route_msg_ready_ || !is_map_msg_ready_)
  {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  for (const auto& route_section : route_msg_.route_sections)
  {
    for (const auto& id : route_section.lane_ids)
    {
      const auto& llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_lane_id)
      {
        preferred_lanelets_.push_back(llt);
      }
    }
  }

  is_handler_ready_ = true;
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const
{
  return route_lanelets_;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(const lanelet::ConstLanelet& lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet))
  {
    return lanelet_sequence_forward;
  }

  lanelet_sequence_forward.push_back(lanelet);

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (true)
  {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet))
    {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(const lanelet::ConstLanelet& lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet))
  {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (true)
  {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet))
    {
      break;
    }
    lanelet_sequence_backward.push_back(prev_lanelet);
    current_lanelet = prev_lanelet;
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(const lanelet::ConstLanelet& lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence;
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet))
  {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getLaneletSequenceAfter(lanelet);
  lanelet_sequence_backward = getLaneletSequenceUpTo(lanelet);

  // loop check
  if (lanelet_sequence_forward.empty() > 1 && lanelet_sequence_backward.empty() > 1)
  {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id())
    {
      return lanelet_sequence_forward;
    }
  }

  lanelet_sequence.insert(lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.insert(lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getPreviousLaneletSequence(const lanelet::ConstLanelets& lanelet_sequence) const
{
  lanelet::ConstLanelets previous_lanelet_sequence;
  if (lanelet_sequence.empty())
    return previous_lanelet_sequence;

  auto first_lane = lanelet_sequence.front();
  if (exists(start_lanelets_, first_lane))
  {
    return previous_lanelet_sequence;
  }

  auto right_relations = lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, first_lane);
  for (const auto& right : right_relations)
  {
    previous_lanelet_sequence = getLaneletSequenceUpTo(right);
    if (!previous_lanelet_sequence.empty())
    {
      return previous_lanelet_sequence;
    }
  }

  auto left_relations = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, first_lane);
  for (const auto& left : left_relations)
  {
    previous_lanelet_sequence = getLaneletSequenceUpTo(left);
    if (!previous_lanelet_sequence.empty())
    {
      return previous_lanelet_sequence;
    }
  }
  return previous_lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getNeighborsWithinRoute(const lanelet::ConstLanelet& lanelet) const
{
  lanelet::ConstLanelets neighbor_lanelets = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, lanelet);
  lanelet::ConstLanelets neighbors_within_route;
  for (const auto& llt : neighbor_lanelets)
  {
    if (exists(route_lanelets_, llt))
    {
      neighbors_within_route.push_back(llt);
    }
  }
  return neighbors_within_route;
}

bool RouteHandler::getNextLaneletWithinRoute(const lanelet::ConstLanelet& lanelet,
                                             lanelet::ConstLanelet* next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet))
  {
    return false;
  }
  lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
  for (const auto& llt : following_lanelets)
  {
    if (exists(route_lanelets_, llt))
    {
      *next_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPreviousLaneletWithinRoute(const lanelet::ConstLanelet& lanelet,
                                                 lanelet::ConstLanelet* prev_lanelet) const
{
  if (exists(start_lanelets_, lanelet))
  {
    return false;
  }
  lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
  for (const auto& llt : previous_lanelets)
  {
    if (exists(route_lanelets_, llt))
    {
      *prev_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getLaneChangeTarget(const lanelet::ConstLanelet& lanelet,
                                       lanelet::ConstLanelet* target_lanelet) const
{
  int num = getNumLaneToPreferredLane(lanelet);
  if (num == 0)
  {
    *target_lanelet = lanelet;
    return false;
  }
  if (num < 0)
  {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet)) ? routing_graph_ptr_->right(lanelet) :
                                                                  routing_graph_ptr_->adjacentRight(lanelet);
    *target_lanelet = right_lanelet.get();
  }

  if (num > 0)
  {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet)) ? routing_graph_ptr_->left(lanelet) :
                                                                routing_graph_ptr_->adjacentLeft(lanelet);
    *target_lanelet = left_lanelet.get();
  }
  return true;
}

lanelet::ArcCoordinates getArcCoordinates(const lanelet::ConstLanelets& lanelet_sequence,
                                          const lanelet::ConstPoint2d& point)
{
  double min_distance = std::numeric_limits<double>::max();
  double length = 0;
  lanelet::ArcCoordinates arc_coordinates;
  for (const auto& llt : lanelet_sequence)
  {
    const auto& centerline_2d = lanelet::utils::to2D(llt.centerline());
    double distance = boost::geometry::comparable_distance(centerline_2d, lanelet::utils::to2D(point));
    if (distance < min_distance)
    {
      min_distance = distance;
      arc_coordinates = lanelet::geometry::toArcCoordinates(centerline_2d, lanelet::utils::to2D(point).basicPoint());
      arc_coordinates.length += length;
    }
    length += boost::geometry::length(centerline_2d);
  }
  return arc_coordinates;
}

bool RouteHandler::getClosestLaneletWithinRoute(const geometry_msgs::Pose& search_pose,
                                                lanelet::ConstLanelet* closest_lanelet, double distance_thresh) const
{
  if (closest_lanelet == nullptr)
  {
    return false;
  }

  lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);

  lanelet::ConstLanelets candidate_lanelets;
  double min_distance = std::numeric_limits<double>::max();
  for (const auto& llt : route_lanelets_)
  {
    double distance = boost::geometry::comparable_distance(llt.polygon2d().basicPolygon(), search_point);

    if (std::abs(distance - min_distance) <= std::numeric_limits<double>::epsilon())
    {
      candidate_lanelets.push_back(llt);
    }
    else if (distance < min_distance)
    {
      candidate_lanelets.clear();
      candidate_lanelets.push_back(llt);
      min_distance = distance;
    }
  }

  double min_angle = std::numeric_limits<double>::max();
  double pose_yaw = tf2::getYaw(search_pose.orientation);
  for (const auto& llt : candidate_lanelets)
  {
    lanelet::ConstLineString3d segment = getClosestSegment(search_point, llt.centerline());
    double segment_angle =
        std::atan2(segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
    double angle_diff = getAngleDifference(segment_angle, pose_yaw);
    if (angle_diff < min_angle)
    {
      min_angle = angle_diff;
      *closest_lanelet = llt;
    }
  }

  double actual_distance = boost::geometry::distance(closest_lanelet->polygon2d().basicPolygon(), search_point);

  return actual_distance < distance_thresh;
}

lanelet::ConstLanelets RouteHandler::getClosestLaneletSequence(const geometry_msgs::Pose& pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets empty_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet, std::numeric_limits<double>::epsilon()))
  {
    return empty_lanelets;
  }
  return getLaneletSequence(lanelet);
}

PathWithLaneId RouteHandler::getReferencePath(const geometry_msgs::Pose& pose, const double backward_path_length,
                                              const double forward_path_length) const
{
  PathWithLaneId reference_path;

  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose, &lanelet, std::numeric_limits<double>::epsilon()))
  {
    return reference_path;
  }
  const auto& lanelet_sequence = getLaneletSequence(lanelet);

  const auto arc_coordinates = getArcCoordinates(lanelet_sequence, lanelet::utils::to2D(toLaneletPoint(pose.position)));
  double s = arc_coordinates.length;
  double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;
  return getReferencePath(lanelet_sequence, s_backward, s_forward, false);
}

int RouteHandler::getNumLaneToPreferredLane(const lanelet::ConstLanelet& lanelet) const
{
  int num = 0;
  if (exists(preferred_lanelets_, lanelet))
  {
    return num;
  }
  const auto& right_lanes = lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
  for (const auto& right : right_lanes)
  {
    num--;
    if (exists(preferred_lanelets_, right))
    {
      return num;
    }
  }
  const auto& left_lanes = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
  num = 0;
  for (const auto& left : left_lanes)
  {
    num++;
    if (exists(preferred_lanelets_, left))
    {
      return num;
    }
  }
}

bool RouteHandler::isInPreferredLane(const geometry_msgs::PoseStamped& pose) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet, std::numeric_limits<double>::epsilon()))
  {
    return false;
  }
  return exists(preferred_lanelets_, lanelet);
}
bool RouteHandler::isInTargetLane(const geometry_msgs::PoseStamped& pose, const lanelet::ConstLanelets& target) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet, std::numeric_limits<double>::epsilon()))
  {
    return false;
  }
  return exists(target, lanelet);
}

PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());
  path.points.insert(path.points.end(), path2.points.begin(), path2.points.end());
  return path;
}

lanelet::ConstPoint3d get3DPointFrom2DArcLength(const lanelet::ConstLanelets& lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0;
  for (const auto& llt : lanelet_sequence)
  {
    const auto& centerline = llt.centerline();
    lanelet::ConstPoint3d prev_pt;
    if (!centerline.empty())
    {
      prev_pt = centerline.front();
    }
    for (const auto& pt : centerline)
    {
      double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s)
      {
        double ratio = (s - accumulated_distance2d) / distance2d;
        auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d(lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d();
}

PathWithLaneId RouteHandler::getReferencePath(const lanelet::ConstLanelets& lanelet_sequence, const double s_start,
                                              const double s_end, bool use_exact) const
{
  PathWithLaneId reference_path;
  double s = 0;

  for (const auto& llt : lanelet_sequence)
  {
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    const lanelet::ConstLineString3d centerline = llt.centerline();
    for (size_t i = 0; i < centerline.size(); i++)
    {
      const lanelet::ConstPoint3d pt = centerline[i];
      lanelet::ConstPoint3d next_pt = (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start)
      {
        if (use_exact)
        {
          const auto tmp_p = get3DPointFrom2DArcLength(lanelet_sequence, s_start);
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(tmp_p);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
        else
        {
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
      }

      if (s >= s_start && s <= s_end)
      {
        PathPointWithLaneId point;
        point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
        point.lane_ids.push_back(llt.id());
        point.point.twist.linear.x = limit.speedLimit.value();
        reference_path.points.push_back(point);
      }
      if (s < s_end && s + distance > s_end)
      {
        if (use_exact)
        {
          const auto tmp_p = get3DPointFrom2DArcLength(lanelet_sequence, s_end);
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(tmp_p);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
        else
        {
          PathPointWithLaneId point;
          point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(next_pt);
          point.lane_ids.push_back(llt.id());
          point.point.twist.linear.x = limit.speedLimit.value();
          reference_path.points.push_back(point);
        }
      }
      s += distance;
    }
  }

  reference_path = util::removeOverlappingPoints(reference_path);

  // set angle
  for (size_t i = 0; i < reference_path.points.size(); i++)
  {
    double angle = 0;
    auto& pt = reference_path.points.at(i);
    if (i + 1 < reference_path.points.size())
    {
      auto next_pt = reference_path.points.at(i + 1);
      angle = std::atan2(next_pt.point.pose.position.y - pt.point.pose.position.y,
                         next_pt.point.pose.position.x - pt.point.pose.position.x);
    }
    else if (i - 1 >= 0)
    {
      auto prev_pt = reference_path.points.at(i - 1);
      auto pt = reference_path.points.at(i);
      angle = std::atan2(pt.point.pose.position.y - prev_pt.point.pose.position.y,
                         pt.point.pose.position.x - prev_pt.point.pose.position.x);
    }
    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, angle);
    pt.point.pose.orientation = tf2::toMsg(yaw_quat);
  }

  return reference_path;
}

lanelet::ConstLanelets RouteHandler::getLaneChangeTarget(const geometry_msgs::Pose& pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets target_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet, std::numeric_limits<double>::epsilon()))
  {
    lanelet::ConstLanelets empty_lanelets;
    return empty_lanelets;
  }

  int num = getNumLaneToPreferredLane(lanelet);
  if (num < 0)
  {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet)) ? routing_graph_ptr_->right(lanelet) :
                                                                  routing_graph_ptr_->adjacentRight(lanelet);
    return getLaneletSequence(right_lanelet.get());
  }
  if (num > 0)
  {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet)) ? routing_graph_ptr_->left(lanelet) :
                                                                routing_graph_ptr_->adjacentLeft(lanelet);
    return getLaneletSequence(left_lanelet.get());
  }
  return getLaneletSequence(lanelet);
}

PathWithLaneId RouteHandler::getLaneChangePath(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist,
                                               const double backward_path_length,
                                               const double forward_path_length,
                                               const double lane_change_prepare_duration,
                                               const double lane_changing_duration) const
{
  PathWithLaneId reference_path;

  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose, &lanelet, std::numeric_limits<double>::epsilon()))
  {
    return reference_path;
  }

  double velocity =
      std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y + twist.linear.z * twist.linear.z);

  // saturation
  velocity = (velocity > 1.0) ? velocity : 1.0;

  auto lanelet_sequence = getLaneletSequence(lanelet);
  double straight_distance = velocity * lane_change_prepare_duration;
  double lane_change_distance = velocity * lane_changing_duration;

  lanelet::Point3d vehicle_position = toLaneletPoint(pose.position);

  auto arc_position = getArcCoordinates(lanelet_sequence, vehicle_position);
  double s = arc_position.length;

  double s_start = s - backward_path_length;
  double s_end = s + straight_distance;
  const auto reference_path1 = getReferencePath(lanelet_sequence, s_start, s_end);
  PathWithLaneId reference_path2;

  int num = getNumLaneToPreferredLane(lanelet);
  if (num < 0)
  {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet)) ? routing_graph_ptr_->right(lanelet) :
                                                                  routing_graph_ptr_->adjacentRight(lanelet);
    const auto& right_lanelet_sequence = getLaneletSequence(right_lanelet.get());
    auto arc_position = getArcCoordinates(right_lanelet_sequence, vehicle_position);
    s_start = arc_position.length + straight_distance + lane_change_distance;
    s_end = s_start + forward_path_length;
    reference_path2 = getReferencePath(right_lanelet_sequence, s_start, s_end);
  }

  if (num > 0)
  {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet)) ? routing_graph_ptr_->left(lanelet) :
                                                                routing_graph_ptr_->adjacentLeft(lanelet);
    const auto& left_lanelet_sequence = getLaneletSequence(left_lanelet.get());
    auto arc_position = getArcCoordinates(left_lanelet_sequence, vehicle_position);
    s_start = arc_position.length + straight_distance + lane_change_distance;
    s_end = s_start + forward_path_length;
    reference_path2 = getReferencePath(left_lanelet_sequence, s_start, s_end);
  }

  reference_path = combineReferencePath(reference_path1, reference_path2);

  return reference_path;
}

geometry_msgs::Pose RouteHandler::getGoalPose() const
{
  return route_msg_.goal_pose;
}

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (route_msg_.route_sections.empty())
  {
    return lanelet::InvalId;
  }
  else
  {
    return route_msg_.route_sections.back().preferred_lane_id;
  }
}

}  // namespace lane_change_planner
