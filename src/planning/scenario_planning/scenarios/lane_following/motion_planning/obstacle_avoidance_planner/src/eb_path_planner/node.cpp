
#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_system_msgs/AutowareState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "eb_path_planner/eb_path_smoother.h"
#include "eb_path_planner/modify_reference_path.h"
#include "eb_path_planner/node.h"
#include "eb_path_planner/spline_interpolate.h"
#include "eb_path_planner/util.h"

// clang-format off
#define DEBUG_INFO(...) { if (show_debug_info_) {ROS_INFO(__VA_ARGS__); } }
#define DEBUG_WARN(...) { if (show_debug_info_) {ROS_WARN(__VA_ARGS__); } }
// clang-format on

EBPathPlannerNode::EBPathPlannerNode() : nh_(), private_nh_("~") {
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  trajectory_pub_ = private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/path", 1);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("eb_path_planner_marker", 1, true);
  debug_clearance_map_in_occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("debug_clearance_map", 1, true);

  path_sub_ = private_nh_.subscribe("input/path", 1, &EBPathPlannerNode::pathCallback, this);
  objects_sub_ = private_nh_.subscribe("/perception/prediction/objects", 10, &EBPathPlannerNode::objectsCallback, this);
  monitored_state_sub_ = private_nh_.subscribe("/autoware/state", 10, &EBPathPlannerNode::monitoredStateCallback, this);
  enable_avoidance_sub_ =
      private_nh_.subscribe("enable_avoidance", 10, &EBPathPlannerNode::enableAvoidanceCallback, this);

  private_nh_.param<bool>("is_debug_clearance_map_mode", is_debug_clearance_map_mode_, true);
  private_nh_.param<bool>("is_debug_drivable_area_mode", is_debug_drivable_area_mode_, false);
  private_nh_.param<bool>("is_publishing_clearance_map_as_occupancy_grid",
                          is_publishing_clearance_map_as_occupancy_grid_, false);
  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, true);
  private_nh_.param<bool>("show_debug_info", show_debug_info_, false);
  private_nh_.param<int>("number_of_backward_path_points_for_detecting_objects",
                         number_of_backward_path_points_for_detecting_objects_, 5);
  private_nh_.param<double>("forward_fixing_distance", forward_fixing_distance_, 20.0);
  private_nh_.param<double>("backward_fixing_distance", backward_fixing_distance_, 10.0);
  private_nh_.param<double>("detecting_objects_radius_from_ego", detecting_objects_radius_from_ego_, 50.0);
  private_nh_.param<double>("detecting_objects_radius_around_path_point", detecting_objects_radius_around_path_point_,
                            2.5);
  private_nh_.param<double>("exploring_minumum_radius", exploring_minimum_radius_, 1.3);
  private_nh_.param<double>("delta_arc_length_for_path_smoothing", delta_arc_length_for_path_smoothing_, 1.0);
  private_nh_.param<double>("delta_arc_length_for_explored_points", delta_arc_length_for_explored_points_, 1.0);
  private_nh_.param<double>("max_avoiding_objects_velocity_ms", max_avoiding_objects_velocity_ms_, 0.1);
  private_nh_.param<double>("clearance_weight_when_exploring", clearance_weight_when_exploring_, 0.0);
  private_nh_.param<double>("exploring_goal_clearance_from_obstacle", exploring_goal_clearance_from_obstacle_,
                            exploring_minimum_radius_ * 3);
  private_nh_.param<double>("fixing_point_clearance_from_obstacle", fixing_point_clearance_from_obstacle_, 0.8);
  exploring_goal_clearance_from_obstacle_ =
      std::fmax(exploring_minimum_radius_ * 2, exploring_goal_clearance_from_obstacle_);
  private_nh_.param<double>("min_distance_threshold_when_switching_avoindance_to_path_following",
                            min_distance_threshold_when_switching_avoindance_to_path_following_, 0.1);
  private_nh_.param<double>("min_cos_similarity_when_switching_avoindance_to_path_following",
                            min_cos_similarity_when_switching_avoindance_to_path_following_, 0.95);
  private_nh_.param<double>("delta_yaw_threshold_for_closest_point", delta_yaw_threshold_for_closest_point_,
                            M_PI / 3.0);
  in_objects_ptr_ = std::make_unique<autoware_perception_msgs::DynamicObjectArray>();
  is_previously_avoidance_mode_ = false;
  previous_mode_ = Mode::LaneFollowing;
  initializing();
}

EBPathPlannerNode::~EBPathPlannerNode() {}

void EBPathPlannerNode::pathCallback(const autoware_planning_msgs::Path& msg) {
  if (msg.points.empty() || msg.drivable_area.data.empty()) {
    return;
  }
  current_ego_pose_ptr_ = getCurrentEgoPose();
  if (!current_ego_pose_ptr_) {
    return;
  }
  autoware_planning_msgs::Trajectory output_trajectory_msg = generateSmoothTrajectory(msg);
  trajectory_pub_.publish(output_trajectory_msg);
}

autoware_planning_msgs::Trajectory EBPathPlannerNode::generateSmoothTrajectory(
    const autoware_planning_msgs::Path& input_path) {
  std::shared_ptr<autoware_planning_msgs::Trajectory> smoothed_trajectory =
      generateSmoothTrajectoryFromExploredPoints(input_path);
  if (!smoothed_trajectory) {
    smoothed_trajectory = generateSmoothTrajectoryFromPath(input_path);
  }
  previous_optimized_points_ptr_ =
      std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(smoothed_trajectory->points);
  previous_path_points_ptr_ = std::make_unique<std::vector<autoware_planning_msgs::PathPoint>>(input_path.points);
  return *smoothed_trajectory;
}

std::shared_ptr<autoware_planning_msgs::Trajectory> EBPathPlannerNode::generateSmoothTrajectoryFromExploredPoints(
    const autoware_planning_msgs::Path& input_path) {
  bool is_avoidance_needed =
      isAvoidanceNeeded(input_path.points, *current_ego_pose_ptr_, previous_optimized_points_ptr_);
  if (!is_avoidance_needed || !enable_avoidance_) {
    return nullptr;
  }

  cv::Mat clearance_map;
  generateClearanceMap(input_path.drivable_area, in_objects_ptr_->objects, *current_ego_pose_ptr_, clearance_map);

  // 1 generateFixedExploredPoints(forward fix&& backward fix)
  std::vector<geometry_msgs::Point> previous_optimized_explored_points;
  if (previous_optimized_points_ptr_) {
    std::vector<geometry_msgs::Point> tmp_points;
    for (const auto& point : *previous_optimized_points_ptr_) {
      tmp_points.push_back(point.pose.position);
    }
    previous_optimized_explored_points = tmp_points;
  } else {
    std::vector<geometry_msgs::Point> tmp_points;
    for (const auto& point : input_path.points) {
      tmp_points.push_back(point.pose.position);
    }
    previous_optimized_explored_points = tmp_points;
  }

  cv::Mat only_objects_clearance_map =
      generateOnlyObjectsClearanceMap(clearance_map, in_objects_ptr_->objects, input_path.drivable_area.info);
  std::vector<geometry_msgs::Point> fixed_optimized_explored_points =
      generateFixedOptimizedExploredPoints(*current_ego_pose_ptr_, previous_optimized_explored_points, clearance_map,
                                           only_objects_clearance_map, input_path.drivable_area.info);
  // 2 generateExploredPoints(merged explored points)
  bool is_explore_needed = false;
  std::unique_ptr<std::vector<geometry_msgs::Point>> non_fixed_explored_points = generateNonFixedExploredPoints(
      input_path, fixed_optimized_explored_points, clearance_map, only_objects_clearance_map, is_explore_needed);
  std::vector<geometry_msgs::Point> tmp;
  if (non_fixed_explored_points) {
    tmp = *non_fixed_explored_points;
  }
  debugFixedNonFixedPointsMarker(fixed_optimized_explored_points, tmp);

  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  if (non_fixed_explored_points) {
    // ROS_WARN("explored optimization triggerred");
    std::vector<geometry_msgs::Point> debug_interpolated_points;
    std::vector<geometry_msgs::Point> debug_constrain_points;
    optimized_points = eb_path_smoother_ptr_->generateOptimizedExploredPoints(
        input_path.points, fixed_optimized_explored_points, *non_fixed_explored_points, *current_ego_pose_ptr_,
        clearance_map, input_path.drivable_area.info, debug_interpolated_points, debug_constrain_points);
    debugMarkers(debug_constrain_points, debug_interpolated_points, optimized_points);
  } else if (!is_explore_needed && previous_optimized_points_ptr_) {
    optimized_points = *previous_optimized_points_ptr_;
  } else {
    return nullptr;
  }

  std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points;
  generateFineOptimizedTrajectory(*current_ego_pose_ptr_, input_path.points, optimized_points, fine_optimized_points);
  std::shared_ptr<autoware_planning_msgs::Trajectory> output_smooth_trajectory =
      std::make_unique<autoware_planning_msgs::Trajectory>();
  output_smooth_trajectory->header = input_path.header;
  output_smooth_trajectory->points = fine_optimized_points;
  resettingPtrForLaneFollowing();
  is_previously_avoidance_mode_ = true;
  // ros::Duration(5.0).sleep();
  return output_smooth_trajectory;
}

bool EBPathPlannerNode::isAvoidanceNeeded(
    const std::vector<autoware_planning_msgs::PathPoint> in_path, const geometry_msgs::Pose self_pose,
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& previous_output_trajectory_points) {
  bool is_detecting_fixed_path_points = isDetectingFixedPathPoint(in_path);
  if (is_detecting_fixed_path_points) {
    return false;
  }

  bool is_objects_detected = detectAvoidingObjectsOnPath(self_pose, in_objects_ptr_->objects, in_path);

  if (is_objects_detected) {
    return true;
  }

  if (!is_previously_avoidance_mode_) {
    // ROS_WARN("previous mode was not avoidance && does not detect objects on path; relay path");
    return false;
  }

  geometry_msgs::Pose nearest_pose;
  if (!getNearestPose(self_pose, previous_output_trajectory_points, nearest_pose)) {
    // std::cout << "avoidance -> lanefollowing since ego_pose if close enough with path points" << std::endl;
    return false;
  }

  return isPoseCloseToPath(in_path, nearest_pose) ? false : true;
}

bool EBPathPlannerNode::isDetectingFixedPathPoint(const std::vector<autoware_planning_msgs::PathPoint>& path_points) {
  for (const auto path_point : path_points) {
    if (path_point.type == path_point.FIXED) {
      return true;
    }
  }
  return false;
}

bool EBPathPlannerNode::getNearestPose(
    const geometry_msgs::Pose self_pose,
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& trajectory_points,
    geometry_msgs::Pose& nearest_pose) {
  if (!trajectory_points) {
    return false;
  }
  double min_dist = 999999999;
  geometry_msgs::Pose tmp_nearest_pose;
  for (const auto& point : *trajectory_points) {
    double dx = point.pose.position.x - self_pose.position.x;
    double dy = point.pose.position.y - self_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      tmp_nearest_pose = point.pose;
    }
  }
  double thres = 3;
  if (min_dist < thres) {
    nearest_pose = tmp_nearest_pose;
    return true;
  } else {
    return false;
  }
}

bool EBPathPlannerNode::isPoseCloseToPath(const std::vector<autoware_planning_msgs::PathPoint> in_path,
                                          const geometry_msgs::Pose in_pose) {
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (size_t i = 0; i < in_path.size(); i++) {
    tmp_x.push_back(in_path[i].pose.position.x);
    tmp_y.push_back(in_path[i].pose.position.y);
  }
  double interval_dist = 0.1;
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, interval_dist, interpolated_points);

  int min_ind = -1;
  double min_dist = 99999999;
  for (int i = 0; i < interpolated_points.size(); i++) {
    double dx = interpolated_points[i].x - in_pose.position.x;
    double dy = interpolated_points[i].y - in_pose.position.y;
    double dist = (dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      min_ind = i;
    }
  }
  if (min_ind == -1) {
    return false;
  }
  double path_yaw;
  if (min_ind > 0) {
    double dx1 = interpolated_points[min_ind].x - interpolated_points[min_ind - 1].x;
    double dy1 = interpolated_points[min_ind].y - interpolated_points[min_ind - 1].y;
    path_yaw = std::atan2(dy1, dx1);
  } else {
    double dx1 = interpolated_points[min_ind + 1].x - interpolated_points[min_ind].x;
    double dy1 = interpolated_points[min_ind + 1].y - interpolated_points[min_ind].y;
    path_yaw = std::atan2(dy1, dx1);
  }
  double traj_yaw = tf2::getYaw(in_pose.orientation);
  double cos_similarity = std::cos(traj_yaw) * std::cos(path_yaw) + std::sin(traj_yaw) * std::sin(traj_yaw);
  if (min_dist < min_distance_threshold_when_switching_avoindance_to_path_following_ &&
      cos_similarity > min_cos_similarity_when_switching_avoindance_to_path_following_) {
    return true;
  } else {
    return false;
  }
}

void EBPathPlannerNode::initializing() {
  ROS_WARN("[EBPathPlanner] Resetting");
  ros::Duration(1.0).sleep();
  modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
      exploring_minimum_radius_, backward_fixing_distance_, clearance_weight_when_exploring_);
  eb_path_smoother_ptr_ =
      std::make_unique<EBPathSmoother>(exploring_minimum_radius_, backward_fixing_distance_, forward_fixing_distance_,
                                       delta_arc_length_for_path_smoothing_, delta_arc_length_for_explored_points_);
  is_previously_avoidance_mode_ = false;
  previous_optimized_points_ptr_ = nullptr;
  previous_path_points_ptr_ = nullptr;
}

void EBPathPlannerNode::resettingPtrForAvoidance() {
  if (is_previously_avoidance_mode_) {
    ROS_WARN("[EBPathPlanner] Avoidance->Lane Following");
  }
}

void EBPathPlannerNode::resettingPtrForLaneFollowing() {
  if (!is_previously_avoidance_mode_) {
    ROS_WARN("[EBPathPlanner] Lane Following->Avoidance");
  }
}

void EBPathPlannerNode::objectsCallback(const autoware_perception_msgs::DynamicObjectArray& msg) {
  in_objects_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(msg);
}

void EBPathPlannerNode::monitoredStateCallback(const autoware_system_msgs::AutowareState& msg) {
  if (msg.state == autoware_system_msgs::AutowareState::Planning) {
    initializing();
  }
}

void EBPathPlannerNode::enableAvoidanceCallback(const std_msgs::Bool& msg) { enable_avoidance_ = msg.data; }

bool EBPathPlannerNode::needResetPrevOptimizedExploredPoints(
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& previous_optimized_explored_points,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects) {
  bool is_need_reset = false;

  // check1
  if (previous_optimized_explored_points) {
    bool is_object_on_explored_points = detectAvoidingObjectsOnPoints(objects, *previous_optimized_explored_points);
    if (is_object_on_explored_points) {
      ROS_WARN("[EBPathPlanner] Reset eb path planner since objects on explored points");

      is_need_reset = true;
      return is_need_reset;
    }
  }

  return is_need_reset;
}

int EBPathPlannerNode::getNearestPathPointsIndFromPose(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, const geometry_msgs::Pose& pose) {
  double nearest_dist = std::numeric_limits<double>::max();
  int nearest_ind = 0;
  for (int i = 0; i < path_points.size(); i++) {
    double dx = path_points[i].pose.position.x - pose.position.x;
    double dy = path_points[i].pose.position.y - pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double path_point_yaw = 0;
    double path_dx = 0;
    double path_dy = 0;
    if (i > 0) {
      path_dx = path_points[i].pose.position.x - path_points[i - 1].pose.position.x;
      path_dy = path_points[i].pose.position.y - path_points[i - 1].pose.position.y;
    } else {
      path_dx = path_points[i + 1].pose.position.x - path_points[i].pose.position.x;
      path_dy = path_points[i + 1].pose.position.y - path_points[i].pose.position.y;
    }
    path_point_yaw = std::atan2(path_dy, path_dx);
    double pose_yaw = tf2::getYaw(pose.orientation);
    double delta_yaw = path_point_yaw - pose_yaw;
    double norm_delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
    if (dist < nearest_dist && std::fabs(norm_delta_yaw) < delta_yaw_threshold_for_closest_point_) {
      nearest_dist = dist;
      nearest_ind = i;
    }
  }
  return nearest_ind;
}

bool EBPathPlannerNode::needExprolation(const geometry_msgs::Pose& ego_pose,
                                        const autoware_planning_msgs::Path& in_path, const cv::Mat& clearance_map,
                                        const cv::Mat& only_objects_clearance_map,
                                        const std::vector<geometry_msgs::Point>& fixed_explored_points,
                                        geometry_msgs::Point& start_exploring_point,
                                        geometry_msgs::Point& goal_exploring_point) {
  if (fixed_explored_points.empty()) {
    int nearest_ind = getNearestPathPointsIndFromPose(in_path.points, ego_pose);
    // assuming delta_arc_length in path is about 1m
    const double delta_arc_length_for_path = 1;
    const double backward_distace_for_exploration = 5;
    int start_exploring_ind =
        std::max((int)(nearest_ind - backward_distace_for_exploration / delta_arc_length_for_path), 0);
    start_exploring_point = in_path.points[start_exploring_ind].pose.position;
  } else {
    start_exploring_point = fixed_explored_points.back();
  }

  int nearest_path_point_ind_from_start_exploring_point = 0;
  double min_dist2 = 999999999;
  for (int i = 0; i < in_path.points.size(); i++) {
    double dx = in_path.points[i].pose.position.x - start_exploring_point.x;
    double dy = in_path.points[i].pose.position.y - start_exploring_point.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist2) {
      min_dist2 = dist;
      nearest_path_point_ind_from_start_exploring_point = i;
    }
  }

  std::unique_ptr<geometry_msgs::Pose> exploring_goal_pose_in_map_ptr;
  double accum_dist2 = 0;
  for (int i = nearest_path_point_ind_from_start_exploring_point; i < in_path.points.size(); i++) {
    if (in_path.points[i].twist.linear.x < 1e-6 && i == in_path.points.size() - 1 && !fixed_explored_points.empty()) {
      double dx = in_path.points[i].pose.position.x - fixed_explored_points.back().x;
      double dy = in_path.points[i].pose.position.y - fixed_explored_points.back().y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < 1e-6) {
        // ROS_WARN_THROTTLE(10.0,"prevent redundant goal");
        // ROS_WARN("prevent redundant goal");
        return false;
      }
      exploring_goal_pose_in_map_ptr = std::make_unique<geometry_msgs::Pose>(in_path.points[i].pose);
      break;
    }
    if (i > nearest_path_point_ind_from_start_exploring_point) {
      double dx = in_path.points[i].pose.position.x - in_path.points[i - 1].pose.position.x;
      double dy = in_path.points[i].pose.position.y - in_path.points[i - 1].pose.position.y;
      accum_dist2 += std::sqrt(dx * dx + dy * dy);
    }
    geometry_msgs::Point image_point;
    if (util::transformMapToImage(in_path.points[i].pose.position, in_path.drivable_area.info, image_point)) {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = only_objects_clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x];
      if (clearance * in_path.drivable_area.info.resolution >= exploring_goal_clearance_from_obstacle_) {
        // if(accum_dist2 > exploring_minimum_radius_*10)
        // {
        exploring_goal_pose_in_map_ptr = std::make_unique<geometry_msgs::Pose>(in_path.points[i].pose);
        // }
        if (accum_dist2 > exploring_minimum_radius_ * 20) {
          break;
        }
      }
    } else {
      // ROS_WARN("not expect to call here");
      break;
    }
  }
  if (!exploring_goal_pose_in_map_ptr) {
    // ROS_WARN_THROTTLE(3.0, "[EBPathPlanner] Could not find appropriate goal");
    ROS_WARN_THROTTLE(2.0, "[EBPathPlanner] Could not find appropriate goal");
    return false;
  } else {
    if (previous_goal_point_for_exploration_ptr_) {
      double dx = exploring_goal_pose_in_map_ptr->position.x - previous_goal_point_for_exploration_ptr_->x;
      double dy = exploring_goal_pose_in_map_ptr->position.y - previous_goal_point_for_exploration_ptr_->y;
      double dist = std::sqrt(dx * dx + dy * dy);
      // std::cout << "idst "<<dist << std::endl;
      if (dist < 3) {
        // ROS_WARN( "[EBPathPlanner] Skip exploration current goal and previosu goal are close");
        return false;
      } else {
        goal_exploring_point = exploring_goal_pose_in_map_ptr->position;
        previous_goal_point_for_exploration_ptr_ = std::make_unique<geometry_msgs::Point>(goal_exploring_point);
        return true;
      }
      // ROS_WARN( "[EBPathPlanner] Skip exploration current goal and previosu goal are not close");
      // previous_goal_point_for_exploration_ptr_ =
      //   std::make_unique<geometry_msgs::Point>(goal_exploring_point);
    } else {
      goal_exploring_point = exploring_goal_pose_in_map_ptr->position;
      previous_goal_point_for_exploration_ptr_ = std::make_unique<geometry_msgs::Point>(goal_exploring_point);
      return true;
    }
  }
}

std::vector<geometry_msgs::Point> EBPathPlannerNode::generateFixedOptimizedExploredPoints(
    const geometry_msgs::Pose& ego_pose, const std::vector<geometry_msgs::Point>& explored_points,
    const cv::Mat& clearance_map, const cv::Mat& only_objects_clearance_map, const nav_msgs::MapMetaData& map_info) {
  double min_dist = 8888888;
  int min_ind = -1;
  for (int i = 0; i < explored_points.size(); i++) {
    double dx = explored_points[i].x - ego_pose.position.x;
    double dy = explored_points[i].y - ego_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      min_ind = i;
    }
  }
  if (min_ind == -1) {
    std::vector<geometry_msgs::Point> empty_fixed_points;
    return empty_fixed_points;
  }
  // TODO: more precise way to calculate backward fixing
  int backward_ind = std::max((int)(min_ind - backward_fixing_distance_ / 0.1), 0);
  int valid_backward_ind = 0;
  for (int i = backward_ind; i < explored_points.size(); i++) {
    geometry_msgs::Point image_point;
    if (util::transformMapToImage(explored_points[i], map_info, image_point)) {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x];
      if (clearance > 0) {
        valid_backward_ind = i;
        break;
      }
    }
  }
  double total_valid_accum_dist = 0;
  for (int i = valid_backward_ind; i < explored_points.size(); i++) {
    if (i > valid_backward_ind) {
      double dx = explored_points[i].x - explored_points[i - 1].x;
      double dy = explored_points[i].y - explored_points[i - 1].y;
      double dist = std::sqrt(dx * dx + dy * dy);
      total_valid_accum_dist += dist;
    }
  }
  // dont want to fix every single points from previous one
  double accum_dist_threshold =
      std::fmin(total_valid_accum_dist - 5, backward_fixing_distance_ + forward_fixing_distance_);
  // std::cout << "accum dist threshold "<< accum_dist_threshold << std::endl;
  int forward_ind = explored_points.size() - 1;
  double accum_dist = 0;
  for (int i = valid_backward_ind; i < explored_points.size(); i++) {
    if (i > valid_backward_ind) {
      double dx = explored_points[i].x - explored_points[i - 1].x;
      double dy = explored_points[i].y - explored_points[i - 1].y;
      double dist = std::sqrt(dx * dx + dy * dy);
      accum_dist += dist;
    }
    if (accum_dist > (accum_dist_threshold)) {
      forward_ind = i;
      break;
    }
  }

  std::vector<geometry_msgs::Point> fixed_explored_points;
  for (int i = valid_backward_ind; i <= forward_ind; i++) {
    fixed_explored_points.push_back(explored_points[i]);
  }
  if (fixed_explored_points.empty()) {
    return fixed_explored_points;
  }

  double min_dist1 = 9999999;
  int nearest_ind1 = 0;
  geometry_msgs::Point car_head;
  car_head.x = 5.0;
  car_head.y = 0;
  double car_yaw = tf2::getYaw(ego_pose.orientation);
  geometry_msgs::Point car_head_in_map;
  car_head_in_map.x = std::cos(car_yaw) * car_head.x + -1 * std::sin(car_yaw) * car_head.y;
  car_head_in_map.y = std::sin(car_yaw) * car_head.x + std::cos(car_yaw) * car_head.y;
  car_head_in_map.x += ego_pose.position.x;
  car_head_in_map.y += ego_pose.position.y;
  for (int i = 0; i < fixed_explored_points.size(); i++) {
    double dx = fixed_explored_points[i].x - car_head_in_map.x;
    double dy = fixed_explored_points[i].y - car_head_in_map.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist1) {
      min_dist1 = dist;
      nearest_ind1 = i;
    }
  }

  int valid_forward_ind = 0;
  for (int i = nearest_ind1; i < fixed_explored_points.size(); i++) {
    geometry_msgs::Point image_point;
    if (util::transformMapToImage(fixed_explored_points[i], map_info, image_point)) {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x];
      float objects_clearance = only_objects_clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x];
      if (clearance > 0 && objects_clearance * map_info.resolution >= fixing_point_clearance_from_obstacle_) {
        valid_forward_ind = i;
      } else {
        break;
      }
    }
  }
  for (int i = valid_forward_ind; i >= 0; i--) {
    geometry_msgs::Point image_point;
    if (util::transformMapToImage(fixed_explored_points[i], map_info, image_point)) {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x];
      float objects_clearance = only_objects_clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x];
      if (clearance > 0 && objects_clearance * map_info.resolution >= exploring_goal_clearance_from_obstacle_) {
        valid_forward_ind = i;
        break;
      }
    }
  }
  // std::cout << "fixed explored points size "<< fixed_explored_points.size() << std::endl;
  // std::cout << "valid forward ind "<< valid_forward_ind << std::endl;
  std::vector<geometry_msgs::Point> tmp_fixed_points;
  if (!fixed_explored_points.empty()) {
    for (int i = 0; i <= valid_forward_ind; i++) {
      tmp_fixed_points.push_back(fixed_explored_points[i]);
    }
    fixed_explored_points = tmp_fixed_points;
  }
  return fixed_explored_points;
}

std::unique_ptr<geometry_msgs::Pose> EBPathPlannerNode::getCurrentEgoPose() {
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer_ptr_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("[ObstacleAvoidancePlanner] %s", ex.what());
    return nullptr;
  }

  geometry_msgs::Pose p;
  p.orientation = tf_current_pose.transform.rotation;
  p.position.x = tf_current_pose.transform.translation.x;
  p.position.y = tf_current_pose.transform.translation.y;
  p.position.z = tf_current_pose.transform.translation.z;
  std::unique_ptr<geometry_msgs::Pose> p_ptr = std::make_unique<geometry_msgs::Pose>(p);
  return p_ptr;
}

std::unique_ptr<std::vector<geometry_msgs::Point>> EBPathPlannerNode::generateNonFixedExploredPoints(
    const autoware_planning_msgs::Path& input_path, const std::vector<geometry_msgs::Point>& fixed_explored_points,
    const cv::Mat& clearance_map, const cv::Mat& only_objects_clearance_map, bool& is_explore_needed) {
  geometry_msgs::Point start_exploring_point;
  geometry_msgs::Point goal_exploring_point;
  is_explore_needed = needExprolation(*current_ego_pose_ptr_, input_path, clearance_map, only_objects_clearance_map,
                                      fixed_explored_points, start_exploring_point, goal_exploring_point);
  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  if (is_explore_needed) {
    debugStartAndGoalMarkers(start_exploring_point, goal_exploring_point);
    std::vector<geometry_msgs::Point> explored_points;
    bool is_explore_success = modify_reference_path_ptr_->generateModifiedPath(
        *current_ego_pose_ptr_, start_exploring_point, goal_exploring_point, input_path.points,
        in_objects_ptr_->objects, explored_points, clearance_map, input_path.drivable_area.info);
    if (!is_explore_success) {
      return nullptr;
    }
    return std::make_unique<std::vector<geometry_msgs::Point>>(explored_points);
  }
  return nullptr;
}

std::shared_ptr<autoware_planning_msgs::Trajectory> EBPathPlannerNode::generateSmoothTrajectoryFromPath(
    const autoware_planning_msgs::Path& input_path) {
  std::vector<autoware_planning_msgs::TrajectoryPoint> smooth_trajectory_points;
  convertPathToSmoothTrajectory(*current_ego_pose_ptr_, input_path.points, smooth_trajectory_points);
  std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points;
  generateFineOptimizedTrajectory(*current_ego_pose_ptr_, input_path.points, smooth_trajectory_points,
                                  fine_optimized_points);
  autoware_planning_msgs::Trajectory output_trajectory;
  output_trajectory.header = input_path.header;
  output_trajectory.points = fine_optimized_points;
  std::shared_ptr<autoware_planning_msgs::Trajectory> output_trajectory_ptr =
      std::make_shared<autoware_planning_msgs::Trajectory>(output_trajectory);
  resettingPtrForAvoidance();
  is_previously_avoidance_mode_ = false;
  return output_trajectory_ptr;
}

bool EBPathPlannerNode::detectAvoidingObjectsOnPath(const geometry_msgs::Pose& ego_pose,
                                                    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
                                                    const std::vector<autoware_planning_msgs::PathPoint>& path_points) {
  std::vector<autoware_perception_msgs::DynamicObject> avoiding_objects;
  for (const auto& object : objects) {
    double dx = object.state.pose_covariance.pose.position.x - ego_pose.position.x;
    double dy = object.state.pose_covariance.pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (object.state.twist_covariance.twist.linear.x < max_avoiding_objects_velocity_ms_) {
      avoiding_objects.push_back(object);
    }
  }

  double min_dist = 1999999;
  int min_ind = 0;
  for (int i = 0; i < path_points.size(); i++) {
    double dx1 = path_points[i].pose.position.x - ego_pose.position.x;
    double dy1 = path_points[i].pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx1 * dx1 + dy1 * dy1);
    if (dist < min_dist) {
      min_dist = dist;
      min_ind = i;
    }
  }
  int search_start_idx = std::max((int)(min_ind - number_of_backward_path_points_for_detecting_objects_), 0);
  for (int i = search_start_idx; i < path_points.size(); i++) {
    for (const auto& avoiding_object : avoiding_objects) {
      double dist_from_path_point_to_obstalce;
      if (avoiding_object.semantic.type == avoiding_object.semantic.CAR ||
          avoiding_object.semantic.type == avoiding_object.semantic.BUS ||
          avoiding_object.semantic.type == avoiding_object.semantic.TRUCK) {
        double yaw = tf2::getYaw(avoiding_object.state.pose_covariance.pose.orientation);
        geometry_msgs::Point top_left;
        top_left.x = avoiding_object.shape.dimensions.x * 0.5;
        top_left.y = avoiding_object.shape.dimensions.y * 0.5;
        geometry_msgs::Point top_left_map;
        top_left_map.x = std::cos(yaw) * top_left.x + -1 * std::sin(yaw) * top_left.y;
        top_left_map.y = std::sin(yaw) * top_left.x + std::cos(yaw) * top_left.y;
        top_left_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        top_left_map.y += avoiding_object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point top_right;
        top_right.x = avoiding_object.shape.dimensions.x * 0.5;
        top_right.y = -1 * avoiding_object.shape.dimensions.y * 0.5;
        geometry_msgs::Point top_right_map;
        top_right_map.x = std::cos(yaw) * top_right.x + -1 * std::sin(yaw) * top_right.y;
        top_right_map.y = std::sin(yaw) * top_right.x + std::cos(yaw) * top_right.y;
        top_right_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        top_right_map.y += avoiding_object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point bottom_left;
        bottom_left.x = -1 * avoiding_object.shape.dimensions.x * 0.5;
        bottom_left.y = avoiding_object.shape.dimensions.y * 0.5;
        geometry_msgs::Point bottom_left_map;
        bottom_left_map.x = std::cos(yaw) * bottom_left.x + -1 * std::sin(yaw) * bottom_left.y;
        bottom_left_map.y = std::sin(yaw) * bottom_left.x + std::cos(yaw) * bottom_left.y;
        bottom_left_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        bottom_left_map.y += avoiding_object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point bottom_right;
        bottom_right.x = -1 * avoiding_object.shape.dimensions.x * 0.5;
        bottom_right.y = -1 * avoiding_object.shape.dimensions.y * 0.5;
        geometry_msgs::Point bottom_right_map;
        bottom_right_map.x = std::cos(yaw) * bottom_right.x + -1 * std::sin(yaw) * bottom_right.y;
        bottom_right_map.y = std::sin(yaw) * bottom_right.x + std::cos(yaw) * bottom_right.y;
        bottom_right_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        bottom_right_map.y += avoiding_object.state.pose_covariance.pose.position.y;

        double dx1 = path_points[i].pose.position.x - top_left_map.x;
        double dy1 = path_points[i].pose.position.y - top_left_map.y;
        double dx2 = path_points[i].pose.position.x - top_right_map.x;
        double dy2 = path_points[i].pose.position.y - top_right_map.y;
        double dx3 = path_points[i].pose.position.x - bottom_right_map.x;
        double dy3 = path_points[i].pose.position.y - bottom_right_map.y;
        double dx4 = path_points[i].pose.position.x - bottom_left_map.x;
        double dy4 = path_points[i].pose.position.y - bottom_left_map.y;
        double dist1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double dist2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        double dist3 = std::sqrt(dx3 * dx3 + dy3 * dy3);
        double dist4 = std::sqrt(dx4 * dx4 + dy4 * dy4);
        float min_dist = std::fmin(dist1, dist2);
        min_dist = std::fmin(min_dist, dist3);
        min_dist = std::fmin(min_dist, dist4);
        dist_from_path_point_to_obstalce = min_dist;
      } else {
        double dx1 = path_points[i].pose.position.x - avoiding_object.state.pose_covariance.pose.position.x;
        double dy1 = path_points[i].pose.position.y - avoiding_object.state.pose_covariance.pose.position.y;
        dist_from_path_point_to_obstalce = std::sqrt(dx1 * dx1 + dy1 * dy1);
      }
      double dx2 = ego_pose.position.x - avoiding_object.state.pose_covariance.pose.position.x;
      double dy2 = ego_pose.position.y - avoiding_object.state.pose_covariance.pose.position.y;
      double dist_from_ego_to_obstacle = std::sqrt(dx2 * dx2 + dy2 * dy2);

      if (dist_from_path_point_to_obstalce < detecting_objects_radius_around_path_point_ &&
          dist_from_ego_to_obstacle < detecting_objects_radius_from_ego_) {
        return true;
      }
    }
  }
  return false;
}

bool EBPathPlannerNode::detectAvoidingObjectsOnPoints(
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& prev_optimized_explored_points) {
  std::vector<autoware_perception_msgs::DynamicObject> avoiding_objects;
  for (const auto& object : objects) {
    if (object.state.twist_covariance.twist.linear.x < max_avoiding_objects_velocity_ms_) {
      avoiding_objects.push_back(object);
    }
  }
  for (const auto& point : prev_optimized_explored_points) {
    for (const auto& object : avoiding_objects) {
      double dx = point.pose.position.x - object.state.pose_covariance.pose.position.x;
      double dy = point.pose.position.y - object.state.pose_covariance.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < exploring_minimum_radius_) {
        return true;
      }
    }
  }
  return false;
}

bool EBPathPlannerNode::generateFineOptimizedTrajectory(
    const geometry_msgs::Pose& ego_pose, const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& fine_optimized_points) {
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
  if (path_points.empty()) {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose = ego_pose;
    tmp_point.twist.linear.x = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    fine_optimized_points.push_back(tmp_point);
    return true;
  }
  if (optimized_points.empty()) {
    for (const auto path_point : path_points) {
      autoware_planning_msgs::TrajectoryPoint tmp_point;
      tmp_point.pose = path_point.pose;
      tmp_point.twist.linear.x = 0;
      fine_optimized_points.push_back(tmp_point);
    }
    return true;
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (const auto& point : optimized_points) {
    tmp_x.push_back(point.pose.position.x);
    tmp_y.push_back(point.pose.position.y);
  }
  if (!optimized_points.empty() && !path_points.empty()) {
    double dx1 = optimized_points.back().pose.position.x - path_points.back().pose.position.x;
    double dy1 = optimized_points.back().pose.position.y - path_points.back().pose.position.y;
    double dist1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double yaw = tf2::getYaw(path_points.back().pose.orientation);
    double dx2 = std::cos(yaw);
    double dy2 = std::sin(yaw);
    double inner_product = dx1 * dx2 + dy1 * dy2;
    if (dist1 < 5.0 && inner_product < 0) {
      tmp_x.push_back(path_points.back().pose.position.x);
      tmp_y.push_back(path_points.back().pose.position.y);
    }
  }
  const double resolution = 0.1;
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, resolution, interpolated_points);
  int zero_velocity_path_points_ind = path_points.size() - 1;
  for (int i = 0; i < path_points.size(); i++) {
    if (path_points[i].twist.linear.x < 1e-6) {
      zero_velocity_path_points_ind = i;
      break;
    }
  }

  int previous_nearest_idx = 0;
  for (int i = 0; i < path_points.size(); i++) {
    double min_dist = 999999999999;
    int nearest_idx = 0;
    for (int j = previous_nearest_idx; j < interpolated_points.size(); j++) {
      double dx1 = interpolated_points[j].x - path_points[i].pose.position.x;
      double dy1 = interpolated_points[j].y - path_points[i].pose.position.y;
      double squared_dist = dx1 * dx1 + dy1 * dy1;
      double yaw = tf2::getYaw(path_points[i].pose.orientation);
      double dx2 = std::cos(yaw);
      double dy2 = std::sin(yaw);
      double ip = dx1 * dx2 + dy1 * dy2;
      ip = -1;
      if (squared_dist < min_dist && ip < 0) {
        min_dist = squared_dist;
        nearest_idx = j;
      }
    }
    for (size_t k = previous_nearest_idx; k < nearest_idx; k++) {
      autoware_planning_msgs::TrajectoryPoint tmp_traj_point;
      tmp_traj_point.pose.position = interpolated_points[k];
      tmp_traj_point.pose.position.z = path_points[i].pose.position.z;
      if (i >= zero_velocity_path_points_ind) {
        tmp_traj_point.twist.linear.x = 0.0;
      } else {
        tmp_traj_point.twist.linear.x = path_points[i].twist.linear.x;
      }
      fine_optimized_points.push_back(tmp_traj_point);
    }
    previous_nearest_idx = nearest_idx;
  }
  if (interpolated_points.empty() || fine_optimized_points.empty()) {
    for (const auto& point : optimized_points) {
      autoware_planning_msgs::TrajectoryPoint tmp_point;
      tmp_point.pose = point.pose;
      tmp_point.twist.linear.x = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      tf2::Quaternion quaternion;
      quaternion.setRPY(roll, pitch, yaw);
      tmp_point.pose.orientation = tf2::toMsg(quaternion);
      fine_optimized_points.push_back(tmp_point);
    }
    return true;
  }
  autoware_planning_msgs::TrajectoryPoint tmp_traj_point;
  tmp_traj_point.pose.position = interpolated_points.back();
  tmp_traj_point.pose.position.z = path_points.back().pose.position.z;
  tmp_traj_point.twist = fine_optimized_points.back().twist;
  fine_optimized_points.push_back(tmp_traj_point);

  if (zero_velocity_path_points_ind == path_points.size() - 1) {
    for (int i = 1; i < fine_optimized_points.size(); i++) {
      if (fine_optimized_points[i].twist.linear.x < 1e-5) {
        fine_optimized_points[i].twist.linear.x = fine_optimized_points[i - 1].twist.linear.x;
      }
    }
    fine_optimized_points.back().twist.linear.x = 0;
  }

  if (fine_optimized_points.size() == 1) {
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    fine_optimized_points.front().pose.orientation = tf2::toMsg(quaternion);
    return true;
  }

  for (size_t i = 0; i < fine_optimized_points.size(); i++) {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    double yaw = 0;
    if (i == fine_optimized_points.size() - 1) {
      double dx = fine_optimized_points[i].pose.position.x - fine_optimized_points[i - 1].pose.position.x;
      double dy = fine_optimized_points[i].pose.position.y - fine_optimized_points[i - 1].pose.position.y;
      yaw = std::atan2(dy, dx);
    } else {
      double dx = fine_optimized_points[i + 1].pose.position.x - fine_optimized_points[i].pose.position.x;
      double dy = fine_optimized_points[i + 1].pose.position.y - fine_optimized_points[i].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    double roll = 0;
    double pitch = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    fine_optimized_points[i].pose.orientation = tf2::toMsg(quaternion);
  }
}

void EBPathPlannerNode::getOccupancyGridValue(const nav_msgs::OccupancyGrid& og, const int i, const int j,
                                              unsigned char& value) {
  int i_flip = og.info.width - i - 1;
  int j_flip = og.info.height - j - 1;
  if (og.data[i_flip + j_flip * og.info.width] > 0) {
    value = 0;
  } else {
    value = 255;
  }
}

void EBPathPlannerNode::putOccupancyGridValue(nav_msgs::OccupancyGrid& og, const int i, const int j,
                                              const unsigned char& value) {
  int i_flip = og.info.width - i - 1;
  int j_flip = og.info.height - j - 1;
  og.data[i_flip + j_flip * og.info.width] = value;
}

bool EBPathPlannerNode::drawObstalcesOnImage(const std::vector<autoware_perception_msgs::DynamicObject>& objects,
                                             const nav_msgs::MapMetaData& map_info, cv::Mat& image) {
  for (const auto& object : objects) {
    if (object.state.twist_covariance.twist.linear.x < max_avoiding_objects_velocity_ms_) {
      if (object.semantic.type == object.semantic.CAR) {
        double yaw = tf2::getYaw(object.state.pose_covariance.pose.orientation);
        geometry_msgs::Point top_left;
        top_left.x = object.shape.dimensions.x * 0.5;
        top_left.y = object.shape.dimensions.y * 0.5;
        geometry_msgs::Point top_left_map;
        top_left_map.x = std::cos(yaw) * top_left.x + -1 * std::sin(yaw) * top_left.y;
        top_left_map.y = std::sin(yaw) * top_left.x + std::cos(yaw) * top_left.y;
        top_left_map.x += object.state.pose_covariance.pose.position.x;
        top_left_map.y += object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point top_right;
        top_right.x = object.shape.dimensions.x * 0.5;
        top_right.y = -1 * object.shape.dimensions.y * 0.5;
        geometry_msgs::Point top_right_map;
        top_right_map.x = std::cos(yaw) * top_right.x + -1 * std::sin(yaw) * top_right.y;
        top_right_map.y = std::sin(yaw) * top_right.x + std::cos(yaw) * top_right.y;
        top_right_map.x += object.state.pose_covariance.pose.position.x;
        top_right_map.y += object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point bottom_left;
        bottom_left.x = -1 * object.shape.dimensions.x * 0.5;
        bottom_left.y = object.shape.dimensions.y * 0.5;
        geometry_msgs::Point bottom_left_map;
        bottom_left_map.x = std::cos(yaw) * bottom_left.x + -1 * std::sin(yaw) * bottom_left.y;
        bottom_left_map.y = std::sin(yaw) * bottom_left.x + std::cos(yaw) * bottom_left.y;
        bottom_left_map.x += object.state.pose_covariance.pose.position.x;
        bottom_left_map.y += object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point bottom_right;
        bottom_right.x = -1 * object.shape.dimensions.x * 0.5;
        bottom_right.y = -1 * object.shape.dimensions.y * 0.5;
        geometry_msgs::Point bottom_right_map;
        bottom_right_map.x = std::cos(yaw) * bottom_right.x + -1 * std::sin(yaw) * bottom_right.y;
        bottom_right_map.y = std::sin(yaw) * bottom_right.x + std::cos(yaw) * bottom_right.y;
        bottom_right_map.x += object.state.pose_covariance.pose.position.x;
        bottom_right_map.y += object.state.pose_covariance.pose.position.y;

        geometry_msgs::Point top_left_map_in_image;
        geometry_msgs::Point top_right_map_in_image;
        geometry_msgs::Point bottom_left_map_in_image;
        geometry_msgs::Point bottom_right_map_in_image;
        util::transformMapToImage(top_left_map, map_info, top_left_map_in_image);
        util::transformMapToImage(top_right_map, map_info, top_right_map_in_image);
        util::transformMapToImage(bottom_left_map, map_info, bottom_left_map_in_image);
        util::transformMapToImage(bottom_right_map, map_info, bottom_right_map_in_image);
        cv::Point top_left_image_point = cv::Point(top_left_map_in_image.x, top_left_map_in_image.y);
        cv::Point top_right_image_point = cv::Point(top_right_map_in_image.x, top_right_map_in_image.y);
        cv::Point bottom_right_image_point = cv::Point(bottom_right_map_in_image.x, bottom_right_map_in_image.y);
        cv::Point bottom_left_image_point = cv::Point(bottom_left_map_in_image.x, bottom_left_map_in_image.y);
        cv::clipLine(cv::Rect(0, 0, image.size().width, image.size().height), top_left_image_point,
                     top_right_image_point);
        cv::clipLine(cv::Rect(0, 0, image.size().width, image.size().height), top_right_image_point,
                     bottom_right_image_point);
        cv::clipLine(cv::Rect(0, 0, image.size().width, image.size().height), bottom_right_image_point,
                     bottom_left_image_point);
        cv::clipLine(cv::Rect(0, 0, image.size().width, image.size().height), bottom_left_image_point,
                     top_left_image_point);

        cv::line(image, top_left_image_point, top_right_image_point, cv::Scalar(0), 10);
        cv::line(image, top_right_image_point, bottom_right_image_point, cv::Scalar(0), 10);
        cv::line(image, bottom_right_image_point, bottom_left_image_point, cv::Scalar(0), 10);
        cv::line(image, bottom_left_image_point, top_left_image_point, cv::Scalar(0), 10);
      } else {
        geometry_msgs::Point point_in_image;
        if (util::transformMapToImage(object.state.pose_covariance.pose.position, map_info, point_in_image)) {
          cv::circle(image, cv::Point(point_in_image.x, point_in_image.y), 15, cv::Scalar(0), -1);
        }
      }
    }
  }
}

cv::Mat EBPathPlannerNode::generateOnlyObjectsClearanceMap(
    const cv::Mat& clearance_map, const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const nav_msgs::MapMetaData& map_info) {
  cv::Mat objects_image = cv::Mat::ones(clearance_map.rows, clearance_map.cols, CV_8UC1) * 255;
  drawObstalcesOnImage(objects, map_info, objects_image);

  cv::Mat only_objects_clearance_map;
  cv::distanceTransform(objects_image, only_objects_clearance_map, cv::DIST_L2, 5);
  return only_objects_clearance_map;
}

bool EBPathPlannerNode::generateClearanceMap(const nav_msgs::OccupancyGrid& occupancy_grid,
                                             const std::vector<autoware_perception_msgs::DynamicObject>& objects,
                                             const geometry_msgs::Pose& debug_ego_pose, cv::Mat& clearance_map) {
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
  cv::Mat drivable_area = cv::Mat(occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1);

  drivable_area.forEach<unsigned char>([&](unsigned char& value, const int* position) -> void {
    getOccupancyGridValue(occupancy_grid, position[0], position[1], value);
  });

  drawObstalcesOnImage(objects, occupancy_grid.info, drivable_area);

  cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  // std::cout << " conversion & edt "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  if (is_debug_clearance_map_mode_) {
    cv::Mat tmp;
    clearance_map.copyTo(tmp);
    cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(10);
  }
  if (is_debug_drivable_area_mode_) {
    cv::Mat tmp;
    drivable_area.copyTo(tmp);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(10);
  }

  if (is_publishing_clearance_map_as_occupancy_grid_) {
    cv::Mat tmp;
    clearance_map.copyTo(tmp);
    cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    nav_msgs::OccupancyGrid clearance_map_in_og = occupancy_grid;
    tmp.forEach<unsigned char>([&](const unsigned char& value, const int* position) -> void {
      putOccupancyGridValue(clearance_map_in_og, position[0], position[1], value);
    });
    debug_clearance_map_in_occupancy_grid_pub_.publish(clearance_map_in_og);
  }
}

bool EBPathPlannerNode::needReplanForPathSmoothing(
    const geometry_msgs::Pose& ego_pose, const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& prev_optimized_path) {
  // check0 nullptr
  if (!prev_optimized_path) {
    return true;
  }

  // check1 exsting prev goal and in path points
  if (isPathShapeChanged(ego_pose, path_points, previous_path_points_ptr_)) {
    return true;
  }

  // check2 distance from ego to goal
  double min_dist = 999999999;
  double nearest_path_ind_from_ego;
  for (int i = 0; i < path_points.size(); i++) {
    double dx = path_points[i].pose.position.x - ego_pose.position.x;
    double dy = path_points[i].pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_path_ind_from_ego = i;
    }
  }
  double accum_dist = 0;
  for (int i = nearest_path_ind_from_ego; i < path_points.size(); i++) {
    if (i > nearest_path_ind_from_ego) {
      double dx = path_points[i].pose.position.x - path_points[i - 1].pose.position.x;
      double dy = path_points[i].pose.position.y - path_points[i - 1].pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      accum_dist += dist;
    }
  }

  double dx = path_points.back().pose.position.x - prev_optimized_path->back().pose.position.x;
  double dy = path_points.back().pose.position.y - prev_optimized_path->back().pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  // ROS_WARN("dist from goal %lf", dist);
  // ROS_WARN("accum dist  %lf", accum_dist);
  if (accum_dist < 50 && dist > delta_arc_length_for_path_smoothing_) {
    return true;
  }

  // check 3 distance from ego to prev end point
  double min_dist1 = 999999999;
  double nearest_prev_points_ind_from_ego = 0;
  for (int i = 0; i < prev_optimized_path->size(); i++) {
    double dx = prev_optimized_path->at(i).pose.position.x - ego_pose.position.x;
    double dy = prev_optimized_path->at(i).pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist1) {
      min_dist1 = dist;
      nearest_prev_points_ind_from_ego = i;
    }
  }
  double accum_dist1 = 0;
  for (int i = nearest_prev_points_ind_from_ego; i < prev_optimized_path->size(); i++) {
    if (i > nearest_prev_points_ind_from_ego) {
      double dx = prev_optimized_path->at(i).pose.position.x - prev_optimized_path->at(i - 1).pose.position.x;
      double dy = prev_optimized_path->at(i).pose.position.y - prev_optimized_path->at(i - 1).pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      accum_dist1 += dist;
    }
  }
  if (accum_dist1 < 50 && dist > delta_arc_length_for_path_smoothing_) {
    return true;
  }
  return false;
}

bool EBPathPlannerNode::calculateNewStartAndGoal(const geometry_msgs::Pose& ego_pose,
                                                 const std::vector<autoware_planning_msgs::PathPoint>& path_points,
                                                 geometry_msgs::Point& start_point, geometry_msgs::Point& goal_point) {
  // check2 distance from ego to goal
  int nearest_path_ind_from_ego = getNearestPathPointsIndFromPose(path_points, ego_pose);
  int start_ind = std::max((int)(nearest_path_ind_from_ego - backward_fixing_distance_), 0);
  start_point = path_points[start_ind].pose.position;

  double accum_dist = 0;
  int goal_ind = 0;
  for (int i = start_ind; i < path_points.size(); i++) {
    if (i > start_ind) {
      double dx = path_points[i].pose.position.x - path_points[i - 1].pose.position.x;
      double dy = path_points[i].pose.position.y - path_points[i - 1].pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      accum_dist += dist;
    }
    goal_ind = i;
    if (accum_dist > 80) {
      break;
    }
  }
  goal_point = path_points[goal_ind].pose.position;

  debugStartAndGoalMarkers(start_point, goal_point);
  // ROS_WARN("start point ind %d", start_ind);
  // ROS_WARN("goal point ind %d", goal_ind);
}

std::unique_ptr<std::vector<geometry_msgs::Pose>> EBPathPlannerNode::generateValidFixedPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<geometry_msgs::Pose>& fixed_path_points) {
  if (path_points.empty()) {
    std::unique_ptr<std::vector<geometry_msgs::Pose>> valid_fixed_path_points;
    valid_fixed_path_points = std::make_unique<std::vector<geometry_msgs::Pose>>(fixed_path_points);
    return valid_fixed_path_points;
  }
  // trim fixed path points
  double min_dist = 9999999999;
  int nearest_ind = 0;
  for (int i = 0; i < fixed_path_points.size(); i++) {
    double dx = fixed_path_points[i].position.x - path_points.front().pose.position.x;
    double dy = fixed_path_points[i].position.y - path_points.front().pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double yaw = tf2::getYaw(path_points.front().pose.orientation);
    double dx1 = std::cos(yaw);
    double dy1 = std::sin(yaw);
    double ip = dx * dx1 + dy * dy1;
    if (dist < min_dist && ip > 0) {
      min_dist = dist;
      nearest_ind = i;
    }
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (int i = 0; i < path_points.size(); i++) {
    tmp_x.push_back(path_points[i].pose.position.x);
    tmp_y.push_back(path_points[i].pose.position.y);
  }
  std::vector<geometry_msgs::Point> interpolated_path_points;
  const double sampling_resolution = 0.1;
  util::interpolate2DPoints(tmp_x, tmp_y, sampling_resolution, interpolated_path_points);
  std::vector<geometry_msgs::Pose> trimmed_fixed_path_points;
  for (int i = nearest_ind; i < fixed_path_points.size(); i++) {
    trimmed_fixed_path_points.push_back(fixed_path_points[i]);
  }

  int max_valid_idx = -1;
  for (const auto& fixed_path_point : trimmed_fixed_path_points) {
    bool is_one_fixed_point_valid = false;
    // 0.3 nearely equal 0.2(path smooth radius)*sqrt(2)
    const double max_valid_dist = 0.3;
    double min_dist = 99999999999;
    for (const auto& path_point : interpolated_path_points) {
      double dx = path_point.x - fixed_path_point.position.x;
      double dy = path_point.y - fixed_path_point.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
      }
      if (dist < max_valid_dist) {
        is_one_fixed_point_valid = true;
        break;
      }
    }
    if (!is_one_fixed_point_valid) {
      break;
    }
    max_valid_idx++;
  }
  if (max_valid_idx == -1) {
    return nullptr;
  }
  std::vector<geometry_msgs::Pose> valid_points;
  for (int i = 0; i <= max_valid_idx; i++) {
    valid_points.push_back(trimmed_fixed_path_points[i]);
  }
  return std::make_unique<std::vector<geometry_msgs::Pose>>(valid_points);
}

std::vector<geometry_msgs::Pose> EBPathPlannerNode::generateFixedOptimizedPathPoints(
    const geometry_msgs::Pose& ego_pose, const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const geometry_msgs::Point& start_point,
    std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& prev_optimized_path) {
  if (prev_optimized_path) {
    double min_dist = 9999999999;
    int nearest_prev_optimized_path_idx = 0;
    for (int i = 0; i < prev_optimized_path->size(); i++) {
      double dx = prev_optimized_path->at(i).pose.position.x - start_point.x;
      double dy = prev_optimized_path->at(i).pose.position.y - start_point.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_prev_optimized_path_idx = i;
      }
    }
    const double resolution = 0.1;
    int backward_fixing_ind =
        std::max((int)(nearest_prev_optimized_path_idx - backward_fixing_distance_ / resolution), 0);
    int forward_fixing_ind = std::min((int)(nearest_prev_optimized_path_idx + forward_fixing_distance_ / resolution),
                                      (int)(prev_optimized_path->size() - 1));
    std::vector<geometry_msgs::Pose> fixed_points;
    for (int i = backward_fixing_ind; i < forward_fixing_ind; i++) {
      fixed_points.push_back(prev_optimized_path->at(i).pose);
    }
    // path shape validation
    if (!isPathShapeChanged(ego_pose, path_points, previous_path_points_ptr_)) {
      return fixed_points;
    }
    // if valid pass fixed points
    // if not valid generate valid fixed path points
    std::unique_ptr<std::vector<geometry_msgs::Pose>> valid_fixed_points =
        generateValidFixedPathPoints(path_points, fixed_points);

    if (valid_fixed_points) {
      double min_dist = 9999999;
      for (const auto& point : *valid_fixed_points) {
        double dx = point.position.x - ego_pose.position.x;
        double dy = point.position.y - ego_pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
      if (min_dist > 1.5) {
        ROS_WARN("[EBPathPlanner] Could not fix points around base_link. Prepare for the sharp steering turn");
      }
      return *valid_fixed_points;
    } else {
      prev_optimized_path = nullptr;
      ROS_WARN("[EBPathPlanner] Could not fix points. Prepare for the sharp steering turn");
    }
  }

  if (prev_optimized_path == nullptr) {
    double min_dist = 9999999999;
    int nearest_path_idx = 0;
    for (int i = 0; i < path_points.size(); i++) {
      double dx = path_points[i].pose.position.x - start_point.x;
      double dy = path_points[i].pose.position.y - start_point.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_path_idx = i;
      }
    }
    int backward_fixing_ind =
        std::max((int)(nearest_path_idx - backward_fixing_distance_ / delta_arc_length_for_path_smoothing_), 0);
    int forward_fixing_ind =
        std::min((int)(nearest_path_idx + forward_fixing_distance_ / delta_arc_length_for_path_smoothing_),
                 (int)(path_points.size() - 1));
    std::vector<geometry_msgs::Pose> fixed_points;
    for (int i = backward_fixing_ind; i < forward_fixing_ind; i++) {
      fixed_points.push_back(path_points[i].pose);
    }
    return fixed_points;
  }
}

bool EBPathPlannerNode::isPathShapeChanged(
    const geometry_msgs::Pose& ego_pose, const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>>& prev_path_points) {
  if (!prev_path_points) {
    return true;
  }
  double min_dist1 = 99999999;
  int nearest_ind1 = 0;
  for (int i = 0; i < prev_path_points->size(); i++) {
    double dx = prev_path_points->at(i).pose.position.x - ego_pose.position.x;
    double dy = prev_path_points->at(i).pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist1) {
      min_dist1 = dist;
      nearest_ind1 = i;
    }
  }
  std::vector<double> prev_points_from_ego_x;
  std::vector<double> prev_points_from_ego_y;
  for (int i = nearest_ind1; i < prev_path_points->size(); i++) {
    // prev_points_from_ego.push_back(prev_path_points->at(i).pose.position);
    prev_points_from_ego_x.push_back(prev_path_points->at(i).pose.position.x);
    prev_points_from_ego_y.push_back(prev_path_points->at(i).pose.position.y);
  }
  double min_dist2 = 99999999;
  int nearest_ind2 = 0;
  for (int i = 0; i < path_points.size(); i++) {
    double dx = path_points[i].pose.position.x - ego_pose.position.x;
    double dy = path_points[i].pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist2) {
      min_dist2 = dist;
      nearest_ind2 = i;
    }
  }
  std::vector<double> points_from_ego_x;
  std::vector<double> points_from_ego_y;
  for (int i = nearest_ind2; i < path_points.size(); i++) {
    points_from_ego_x.push_back(path_points[i].pose.position.x);
    points_from_ego_y.push_back(path_points[i].pose.position.y);
    // points_from_ego.push_back(path_points[i].pose.position);
  }

  std::vector<geometry_msgs::Point> interpolated_prev_points_from_ego;
  std::vector<geometry_msgs::Point> interpolated_points_from_ego;
  util::interpolate2DPoints(prev_points_from_ego_x, prev_points_from_ego_y, 0.5, interpolated_prev_points_from_ego);
  util::interpolate2DPoints(points_from_ego_x, points_from_ego_y, 0.5, interpolated_points_from_ego);

  for (const auto& prev_point : interpolated_prev_points_from_ego) {
    double min_dist3 = 99999999;
    for (const auto& point : interpolated_points_from_ego) {
      double dx = point.x - prev_point.x;
      double dy = point.y - prev_point.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist3) {
        min_dist3 = dist;
      }
    }
    if (min_dist3 > 1) {
      return true;
    }
  }
  return false;
}

std::vector<geometry_msgs::Pose> EBPathPlannerNode::generateNonFixedPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<geometry_msgs::Pose>& fixed_points, const geometry_msgs::Point& goal_path_point) {
  // search start point for non-fixed points
  if (fixed_points.empty()) {
    std::vector<geometry_msgs::Pose> empty_non_fixed_points;
    return empty_non_fixed_points;
  }
  double last_fixed_points_yaw = tf2::getYaw(fixed_points.back().orientation);
  double min_dist = 99999999;
  int min_ind = -1;
  for (int i = 0; i < path_points.size(); i++) {
    double dx = path_points[i].pose.position.x - fixed_points.back().position.x;
    double dy = path_points[i].pose.position.y - fixed_points.back().position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double dx2 = std::cos(last_fixed_points_yaw);
    double dy2 = std::sin(last_fixed_points_yaw);
    double inner_product = dx * dx2 + dy * dy2;
    if (dist < min_dist && inner_product > 0) {
      min_dist = dist;
      min_ind = i;
    }
  }
  // std::cout << "min dist "<< min_dist << std::endl;
  // std::cout << "min ind "<<min_ind << std::endl;

  // check if goal is valid
  int goal_ind = -1;
  for (int i = min_ind; i < path_points.size(); i++) {
    double dx = path_points[i].pose.position.x - goal_path_point.x;
    double dy = path_points[i].pose.position.y - goal_path_point.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-5) {
      goal_ind = i;
    }
  }
  std::vector<geometry_msgs::Pose> non_fixed_points;
  if (goal_ind == -1) {
    return non_fixed_points;
  } else {
    for (int i = min_ind; i <= goal_ind; i++) {
      non_fixed_points.push_back(path_points[i].pose);
    }
    return non_fixed_points;
  }
}

bool EBPathPlannerNode::convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose, const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points) {
  if (!needReplanForPathSmoothing(ego_pose, path_points, previous_optimized_points_ptr_)) {
    // ROS_WARN("passing previous optimized path");
    // yes use previous optimized path
    smoothed_points = *previous_optimized_points_ptr_;
    return true;
  }

  // 1,update start and goal
  geometry_msgs::Point start_point;
  geometry_msgs::Point goal_point;
  calculateNewStartAndGoal(ego_pose, path_points, start_point, goal_point);

  // 2 generate fixed points for optimization
  std::vector<geometry_msgs::Pose> fixed_points =
      generateFixedOptimizedPathPoints(ego_pose, path_points, start_point, previous_optimized_points_ptr_);
  DEBUG_INFO("path points size %zu ", path_points.size());
  DEBUG_INFO("fixed points size %zu ", fixed_points.size());
  // 3 generate non fixed points for optimization
  std::vector<geometry_msgs::Pose> non_fixed_points;
  if (!fixed_points.empty()) {
    non_fixed_points = generateNonFixedPoints(path_points, fixed_points, goal_point);
  }
  DEBUG_INFO("non fixed points size %zu ", non_fixed_points.size());

  std::vector<geometry_msgs::Point> tmp_fixed;
  std::vector<geometry_msgs::Point> tmp_non_fixed;
  for (const auto point : fixed_points) {
    tmp_fixed.push_back(point.position);
  }
  for (const auto point : non_fixed_points) {
    tmp_non_fixed.push_back(point.position);
  }
  debugFixedNonFixedPointsMarker(tmp_fixed, tmp_non_fixed);

  // 4 generate optimized points
  std::vector<geometry_msgs::Point> debug_inter;
  std::vector<geometry_msgs::Point> debug_constrain;
  // std::cout << "fixed size " << fixed_points.size() << std::endl;
  smoothed_points = eb_path_smoother_ptr_->generateOptimizedPoints(path_points, fixed_points, non_fixed_points,
                                                                   debug_inter, debug_constrain);
  debugMarkers(debug_constrain, debug_inter, smoothed_points);
  return true;
}

void EBPathPlannerNode::debugFixedNonFixedPointsMarker(const std::vector<geometry_msgs::Point>& fixed,
                                                       const std::vector<geometry_msgs::Point>& non_fixed) {
  // debug; marker array
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;

  visualization_msgs::Marker fixed_marker;
  fixed_marker.lifetime = ros::Duration(20);
  fixed_marker.header.frame_id = "map";
  fixed_marker.header.stamp = ros::Time(0);
  fixed_marker.ns = std::string("fixed_points_marker");
  fixed_marker.action = visualization_msgs::Marker::ADD;
  fixed_marker.pose.orientation.w = 1.0;
  fixed_marker.id = unique_id;
  fixed_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  fixed_marker.scale.x = 1.0;
  fixed_marker.scale.y = 1.0;
  fixed_marker.scale.z = 1.0;
  fixed_marker.color.r = 1.0f;
  fixed_marker.color.a = 0.999;
  unique_id++;
  for (int i = 0; i < fixed.size(); i++) {
    fixed_marker.points.push_back(fixed[i]);
  }
  marker_array.markers.push_back(fixed_marker);

  visualization_msgs::Marker non_fixed_marker;
  non_fixed_marker.lifetime = ros::Duration(20);
  non_fixed_marker.header.frame_id = "map";
  non_fixed_marker.header.stamp = ros::Time(0);
  non_fixed_marker.ns = std::string("non_fixed_points_marker");
  non_fixed_marker.action = visualization_msgs::Marker::ADD;
  non_fixed_marker.pose.orientation.w = 1.0;
  non_fixed_marker.id = unique_id;
  non_fixed_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  non_fixed_marker.scale.x = 1.0;
  non_fixed_marker.scale.y = 1.0;
  non_fixed_marker.scale.z = 1.0;
  non_fixed_marker.color.g = 1.0f;
  non_fixed_marker.color.a = 0.99;
  unique_id++;
  for (int i = 0; i < non_fixed.size(); i++) {
    non_fixed_marker.points.push_back(non_fixed[i]);
  }
  if (!non_fixed_marker.points.empty()) {
    marker_array.markers.push_back(non_fixed_marker);
  }
  markers_pub_.publish(marker_array);
}

void EBPathPlannerNode::debugStartAndGoalMarkers(const geometry_msgs::Point& start_point,
                                                 const geometry_msgs::Point& goal_point) {
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;
  visualization_msgs::Marker start_marker;
  start_marker.lifetime = ros::Duration(2.0);
  start_marker.header.frame_id = "map";
  start_marker.header.stamp = ros::Time(0);
  start_marker.ns = std::string("start_point");
  start_marker.action = visualization_msgs::Marker::MODIFY;
  start_marker.pose.orientation.w = 1.0;
  start_marker.pose.position = start_point;
  start_marker.id = unique_id;
  start_marker.type = visualization_msgs::Marker::SPHERE;
  start_marker.scale.x = 1.0f;
  start_marker.scale.y = 1.0f;
  start_marker.scale.z = 1.0f;
  start_marker.color.r = 1.0f;
  start_marker.color.a = 1.0;
  unique_id++;
  marker_array.markers.push_back(start_marker);

  visualization_msgs::Marker goal_marker;
  goal_marker.lifetime = ros::Duration(2.0);
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = ros::Time(0);
  goal_marker.ns = std::string("goal_point");
  goal_marker.action = visualization_msgs::Marker::MODIFY;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.pose.position = goal_point;
  goal_marker.id = unique_id;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.scale.x = 1.0f;
  goal_marker.scale.y = 1.0f;
  goal_marker.scale.z = 1.0f;
  goal_marker.color.r = 1.0f;
  goal_marker.color.a = 1.0;
  unique_id++;
  marker_array.markers.push_back(goal_marker);
  markers_pub_.publish(marker_array);
}

void EBPathPlannerNode::debugMarkers(const std::vector<geometry_msgs::Point>& constrain_points,
                                     const std::vector<geometry_msgs::Point>& interpolated_points,
                                     const std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points) {
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;

  for (int i = 0; i < constrain_points.size(); i++) {
    visualization_msgs::Marker constrain_points_marker;
    constrain_points_marker.lifetime = ros::Duration(40.0);
    constrain_points_marker.header.frame_id = "map";
    constrain_points_marker.header.stamp = ros::Time(0);
    constrain_points_marker.ns = std::string("constrain_points_marker");
    constrain_points_marker.action = visualization_msgs::Marker::MODIFY;
    constrain_points_marker.pose.orientation.w = 1.0;
    constrain_points_marker.pose.position = constrain_points[i];
    constrain_points_marker.pose.position.z = current_ego_pose_ptr_->position.z;
    constrain_points_marker.id = unique_id;
    constrain_points_marker.type = visualization_msgs::Marker::SPHERE;
    constrain_points_marker.scale.x = std::abs(constrain_points[i].z * 2);
    constrain_points_marker.scale.y = std::abs(constrain_points[i].z * 2);
    constrain_points_marker.scale.z = 0.1;
    constrain_points_marker.color.r = 1.0f;
    constrain_points_marker.color.g = 0.5f;
    constrain_points_marker.color.a = 0.199;
    unique_id++;
    marker_array.markers.push_back(constrain_points_marker);
  }

  unique_id = 0;
  visualization_msgs::Marker interpolated_points_marker;
  interpolated_points_marker.lifetime = ros::Duration(40);
  interpolated_points_marker.header.frame_id = "map";
  interpolated_points_marker.header.stamp = ros::Time(0);
  interpolated_points_marker.ns = std::string("interpolated_points_marker");
  interpolated_points_marker.action = visualization_msgs::Marker::ADD;
  interpolated_points_marker.pose.orientation.w = 1.0;
  interpolated_points_marker.id = unique_id;
  interpolated_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  interpolated_points_marker.scale.x = 1.0;
  interpolated_points_marker.scale.y = 1.0;
  interpolated_points_marker.scale.z = 1.0;
  interpolated_points_marker.color.r = 1.0f;
  interpolated_points_marker.color.a = 0.99;
  unique_id++;
  for (int i = 0; i < interpolated_points.size(); i++) {
    interpolated_points_marker.points.push_back(interpolated_points[i]);
  }
  if (!interpolated_points_marker.points.empty()) {
    marker_array.markers.push_back(interpolated_points_marker);
  }

  visualization_msgs::Marker optimized_points_marker;
  optimized_points_marker.lifetime = ros::Duration(.1);
  optimized_points_marker.header.frame_id = "map";
  optimized_points_marker.header.stamp = ros::Time(0);
  optimized_points_marker.ns = std::string("optimized_points_marker");
  optimized_points_marker.action = visualization_msgs::Marker::ADD;
  optimized_points_marker.pose.orientation.w = 1.0;
  optimized_points_marker.id = unique_id;
  optimized_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  optimized_points_marker.scale.x = 1.0;
  optimized_points_marker.scale.y = 1.0;
  optimized_points_marker.scale.z = 1.0;
  optimized_points_marker.color.g = 1.0f;
  optimized_points_marker.color.a = 0.99;
  unique_id++;
  for (int i = 0; i < optimized_points.size(); i++) {
    optimized_points_marker.points.push_back(optimized_points[i].pose.position);
  }
  if (!optimized_points_marker.points.empty()) {
    marker_array.markers.push_back(optimized_points_marker);
  }

  unique_id = 0;
  for (int i = 0; i < optimized_points.size(); i++) {
    visualization_msgs::Marker optimized_points_text_marker;
    optimized_points_text_marker.lifetime = ros::Duration(40);
    optimized_points_text_marker.header.frame_id = "map";
    optimized_points_text_marker.header.stamp = ros::Time(0);
    optimized_points_text_marker.ns = std::string("optimized_points_text_marker");
    optimized_points_text_marker.action = visualization_msgs::Marker::ADD;
    optimized_points_text_marker.pose.orientation.w = 1.0;
    optimized_points_text_marker.id = unique_id;
    optimized_points_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    optimized_points_text_marker.pose.position = optimized_points[i].pose.position;
    optimized_points_text_marker.scale.x = 0.5;
    optimized_points_text_marker.scale.y = 0.5;
    optimized_points_text_marker.scale.z = 0.5;
    optimized_points_text_marker.color.g = 1.0f;
    optimized_points_text_marker.color.a = 0.99;
    optimized_points_text_marker.text = std::to_string(i);
    unique_id++;
    marker_array.markers.push_back(optimized_points_text_marker);
  }
  markers_pub_.publish(marker_array);
  // ros::Duration(10.0).sleep();
}