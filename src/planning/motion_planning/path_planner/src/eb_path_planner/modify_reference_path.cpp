
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>

#include <ros/console.h>
#include <tf2/utils.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <geometry_msgs/Pose.h>

#include "autoware_perception_msgs/DynamicObject.h"
#include "autoware_planning_msgs/PathPoint.h"
#include "autoware_planning_msgs/Route.h"
#include "eb_path_planner/modify_reference_path.h"


class Node
{
public:
  Node();
  ~Node();
  
  Eigen::Vector2d p;
  double r;
  double g;
  double h;
  double f;
  std::shared_ptr<Node> parent_node;
};

Node::Node(){}
Node::~Node(){}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point &point,
  const geometry_msgs::Pose &origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

double calculateEigen2DDistance(const Eigen::Vector2d& a, 
                                const Eigen::Vector2d& b)
{
  double dx = a(0)-b(0);
  double dy = a(1)-b(1);
  return std::sqrt(dx*dx+dy*dy);
}

bool transformMapToImage(const geometry_msgs::Point& map_point,
                         const geometry_msgs::Pose& map_ego_pose,
                         const int ego_costmap_x_length,
                         const int ego_costmap_y_width,
                         const double costmap_resolution,
                         geometry_msgs::Point& image_point)
{
  geometry_msgs::Point relative_p = 
    transformToRelativeCoordinate2D(map_point, map_ego_pose);
  double bottomleft_x = (relative_p.x+ego_costmap_x_length/2)/costmap_resolution;
  double bottomleft_y = (relative_p.y+ego_costmap_y_width/2)/costmap_resolution;
  double image_x = ego_costmap_y_width/costmap_resolution - bottomleft_y;
  double image_y = ego_costmap_x_length/costmap_resolution - bottomleft_x;
  if(image_x>=0 && 
     image_x<(int)ego_costmap_y_width/costmap_resolution &&
     image_y>=0 && 
     image_y<(int)ego_costmap_x_length/costmap_resolution)
  {
    image_point.x = image_x;
    image_point.y = image_y;
    return true;
  }
  else
  {
    return false;
  } 
}

bool transformImageToMap(const geometry_msgs::Point& image_point,
                         const geometry_msgs::Pose& map_ego_pose,
                         const int ego_costmap_x_length,
                         const int ego_costmap_y_width,
                         const double costmap_resolution,
                         geometry_msgs::Point& map_point)
{
  double bottomleft_x = ego_costmap_y_width/costmap_resolution - image_point.y;
  double bottomleft_y = ego_costmap_x_length/costmap_resolution - image_point.x;
  double relative_x = bottomleft_x*costmap_resolution - ego_costmap_x_length/2;
  double relative_y = bottomleft_y*costmap_resolution - ego_costmap_y_width/2;
  double yaw = tf2::getYaw(map_ego_pose.orientation);

  geometry_msgs::Point res;
  res.x = (cos(-yaw) * relative_x) + (sin(-yaw) * relative_y);
  res.y = ((-1) * sin(-yaw) * relative_x) + (cos(-yaw) * relative_y);
  
  map_point.x = res.x + map_ego_pose.position.x;
  map_point.y = res.y + map_ego_pose.position.y;
  map_point.z = map_ego_pose.position.z;
  return true;
}



ModifyReferencePath::ModifyReferencePath(
  int num_lookup_lanelet_for_driveable_area,
  double min_radius,
  double backward_distance):
is_fix_pose_mode_for_debug_(false),
is_debug_graph_a_star_mode_(false),
is_debug_each_iteration_mode_(false),
is_debug_driveable_area_mode_(false),
y_width_(100),
x_length_(100),
num_lookup_lanelet_for_drivealble_area_(num_lookup_lanelet_for_driveable_area),
resolution_(0.1),
time_limit_(200.0),
min_radius_(min_radius),
max_radius_(7.0),
backward_distance_(backward_distance),
static_objects_velocity_threshold_(0.2)
{
}

ModifyReferencePath::~ModifyReferencePath() 
{
}

bool ModifyReferencePath::expandNode(Node& parent_node, 
                             const cv::Mat& clearence_map,
                             const Node& goal_node,
                             const double min_r,
                             const double max_r,
                             std::vector<Node>& child_nodes,
                             Node& lowest_f_child_node,
                             int& lowest_f_child_node_index)
{ 
  //r min max
  double current_r;
  if(parent_node.r < min_r)
  {
    current_r = min_r;
  }
  else if(parent_node.r > max_r)
  {
    current_r = max_r;
  }
  else
  {
    current_r = parent_node.r;
  }
  
  if(is_debug_each_iteration_mode_)
  {
    cv::Mat tmp;
    clearence_map.copyTo(tmp);
    cv::rectangle(tmp, cv::Point(std::floor(parent_node.p(0))-(std::floor((current_r/resolution_)/std::sqrt(2))), 
                                std::floor(parent_node.p(1))-(std::floor((current_r/resolution_)/std::sqrt(2)))),
                      cv::Point(std::floor(parent_node.p(0))+(std::floor((current_r/resolution_)/std::sqrt(2))), 
                                std::floor(parent_node.p(1))+(std::floor((current_r/resolution_)/std::sqrt(2)))),
                      cv::Scalar(0), -1 ,CV_AA);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(0);
    cv::destroyAllWindows();
    
  }

  
  // std::vector<Node> child_nodes;
  Eigen::Vector2d delta_child_p;
  delta_child_p << current_r/resolution_,
                   0;
  double delta_theta = 2*M_PI/36.0;
  // double delta_theta = 2*M_PI/54.0;
  double lowest_f = 1000000000;
  int index = 0;
  for(double theta = 0; theta < 2*M_PI; theta += delta_theta)
  {
    Eigen::Matrix2d rotation;
    rotation << std::cos(theta), - std::sin(theta),
                std::sin(theta),   std::cos(theta);
    Eigen::Vector2d rotated_delta = rotation * delta_child_p;
    Node child_node;
    child_node.p = rotated_delta + parent_node.p;
    // std::cerr << "child node " << child_node.p(0)<<" "<<child_node.p(1) << std::endl;
    if(child_node.p(0)*resolution_ >=0 && 
       child_node.p(0)*resolution_ < y_width_ && 
       child_node.p(1)*resolution_ >=0 &&
       child_node.p(1)*resolution_ < x_length_)
    {
      double tmp_r = clearence_map.ptr<float>
                      ((int)child_node.p(1))[(int)child_node.p(0)]*resolution_;
      double r = std::min(tmp_r, max_r);
      if(r < min_r)
      {
        continue;
      }
      child_node.r = r;
    }
    else 
    {
      continue;
    }
    child_node.g = parent_node.g + current_r;
    child_node.h = calculateEigen2DDistance(child_node.p, goal_node.p);
    child_node.f = child_node.g + child_node.h;
    
    child_node.parent_node = std::make_shared<Node>(parent_node);
    if(child_node.f < lowest_f)
    {
      lowest_f = child_node.f;
      lowest_f_child_node = child_node;
      lowest_f_child_node_index = index;
    }
    child_nodes.push_back(child_node);
    index++;
    
  }
  return true;
}

bool ModifyReferencePath::isOverlap(Node& node, Node& goal_node)
{
  double image_distance = calculateEigen2DDistance(node.p, goal_node.p);
  if(image_distance*resolution_< node.r)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool ModifyReferencePath::nodeExistInClosedNodes(Node node, std::vector<Node> closed_nodes)
{
  for(const auto& closed_node: closed_nodes)
  {
    double image_distance = calculateEigen2DDistance(node.p, closed_node.p);
    if(image_distance*resolution_ <closed_node.r*0.5)
    {
      return true;
    }
  }
  return false;
}

bool ModifyReferencePath::solveGraphAStar(const geometry_msgs::Pose& ego_pose,
                     const geometry_msgs::Point& start_point_in_map,
                     const geometry_msgs::Point& goal_point_in_map,
                     const cv::Mat& clearance_map,
                     std::vector<geometry_msgs::Point>& explored_points)
{
  geometry_msgs::Point start_point_in_image;
  geometry_msgs::Point goal_point_in_image;
  if(transformMapToImage(start_point_in_map, ego_pose, x_length_, 
                      y_width_, resolution_, start_point_in_image) &&
     transformMapToImage(goal_point_in_map, ego_pose, x_length_, 
                      y_width_, resolution_, goal_point_in_image))
  {
    std::vector<Node> s_open;
    Node initial_node;
    double initial_r = clearance_map.ptr<float>((int)start_point_in_image.y)
                                                [(int)start_point_in_image.x]*resolution_;
    if(initial_r < min_radius_)
    {
      initial_r = min_radius_;
    }
    else if(initial_r > max_radius_)
    {
      initial_r = max_radius_;
    }
    initial_node.r = initial_r;
    initial_node.g = 0;
    Eigen::Vector2d initial_p(start_point_in_image.x, start_point_in_image.y);
    Eigen::Vector2d goal_p(goal_point_in_image.x, goal_point_in_image.y);
    
    initial_node.p = initial_p;
    initial_node.h = calculateEigen2DDistance(initial_node.p, goal_p);
    initial_node.f = initial_node.g + initial_node.h;
    initial_node.parent_node = nullptr;
    s_open.push_back(initial_node);
    
    Node goal_node;
    goal_node.p = goal_p;
    double goal_r = clearance_map.ptr<float>((int)goal_p(1))[(int)goal_p(0)]*resolution_;
    if(goal_r < min_radius_)
    {
      goal_r = min_radius_;
    }
    else if(goal_r > max_radius_)
    {
      goal_r = max_radius_;
    }
    goal_node.r = goal_r;
    goal_node.g = 0;
    goal_node.h = 0;
    goal_node.f = 0;  
    double f_goal = std::numeric_limits<double>::max();
    std::chrono::high_resolution_clock::time_point begin_explore= 
      std::chrono::high_resolution_clock::now();
    std::vector<Node> s_closed;
    Node lowest_f_node = s_open.front();
    bool is_explore_success = false;
    while(!s_open.empty())
    {
      std::chrono::high_resolution_clock::time_point current_explore= 
        std::chrono::high_resolution_clock::now();
      double current_accum_explroe_time = 
        std::chrono::duration_cast<std::chrono::nanoseconds>(current_explore - begin_explore).count()/(1000.0*1000.0);
      if(current_accum_explroe_time > time_limit_ && !is_debug_each_iteration_mode_)
      {
        ROS_ERROR("[EBPathPlanner] Exceed time limit of %lf [ms], Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
          time_limit_, ego_pose.position.x, ego_pose.position.y, ego_pose.position.z, 
          ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);
        return false;
      }
      if(f_goal < lowest_f_node.f)
      {
        break;
      }
      else if(nodeExistInClosedNodes(lowest_f_node, s_closed))
      {
        std::sort(s_open.begin(), s_open.end(), 
                [](const Node& i, const Node& j){return i.f < j.f;});
        lowest_f_node = s_open.front();s_open.erase(s_open.begin());
        continue;
      }
      else
      {
        if(isOverlap(lowest_f_node, goal_node))
        {
          s_closed.push_back(lowest_f_node);
          f_goal = lowest_f_node.f;
          is_explore_success = true;
          break;
        }
        std::vector<Node> child_nodes; 
        Node lowest_f_child_node;
        int lowest_f_child_node_index;
        expandNode(lowest_f_node, 
                    clearance_map,
                    goal_node,
                    min_radius_,
                    max_radius_,
                    child_nodes,
                    lowest_f_child_node,
                    lowest_f_child_node_index);
        s_closed.push_back(lowest_f_node);
        //update lowest_f_node
        if(child_nodes.size()==0)
        {
          std::sort(s_open.begin(), s_open.end(), 
                [](const Node& i, const Node& j){return i.f < j.f;});
          lowest_f_node = s_open.front();s_open.erase(s_open.begin());
        }
        else
        {
          lowest_f_node = lowest_f_child_node;
          child_nodes.erase(child_nodes.begin()+lowest_f_child_node_index);
        }
        s_open.insert(s_open.end(),
                      child_nodes.begin(),
                      child_nodes.end());
      }
    }
    if(!is_explore_success)
    {
      ROS_WARN("[EBPathPlanner] graph a star could not find path");
      return false;
    }
    //backtrack
    Node current_node = s_closed.back();
    if(current_node.parent_node == nullptr)
    {
      ROS_INFO("[EBPathPlanner] No node is explored, Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
        ego_pose.position.x, ego_pose.position.y, ego_pose.position.z, 
        ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);
      return false;
    }
    std::vector<geometry_msgs::Point> explored_image_points;
    do
    {
      geometry_msgs::Point explored_point;
      explored_point.x = current_node.p(0);
      explored_point.y = current_node.p(1);
      explored_point.z = 0;
      explored_image_points.insert(explored_image_points.begin(), explored_point);
      current_node = *current_node.parent_node;
    }while(current_node.parent_node != nullptr);
  
    for(const auto& point: explored_image_points)
    {
      geometry_msgs::Point map_point;
      transformImageToMap(point, ego_pose, x_length_, y_width_, resolution_,map_point);
      explored_points.push_back(map_point);
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool ModifyReferencePath::needExploration(
  const geometry_msgs::Pose& ego_pose,
  const geometry_msgs::Point& goal_point_in_map,
  const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
  std::unique_ptr<std::vector<geometry_msgs::Point>>& cached_explored_points_ptr,
  geometry_msgs::Point& start_point_in_map)
{
  
  if(!cached_explored_points_ptr)
  {
    cached_explored_points_ptr = std::make_unique<std::vector<geometry_msgs::Point>>();
    start_point_in_map = ego_pose.position; 
    return true;
  }
  else if(cached_explored_points_ptr->empty())
  {
    cached_explored_points_ptr->push_back(ego_pose.position);
  }
  // std::cout << "cached_points size "<< cached_explored_points_ptr->size() << std::endl;
  // std::cout << "path points size "<< path_points.size() << std::endl;
  // std::cout << "goal point "<< goal_point_in_map.x << " "<< goal_point_in_map.y << std::endl;
  // std::cout << "ego pose "<< ego_pose.position.x << " "<<ego_pose.position.y<< std::endl;
  double min_dist = 1000000000;
  geometry_msgs::Point last_explored_point = cached_explored_points_ptr->back();
  int nearest_path_point_idx_from_last_explored_point;
  for (int i = 0; i < path_points.size(); i++)
  {
    double dx = path_points[i].pose.position.x - last_explored_point.x;
    double dy = path_points[i].pose.position.y - last_explored_point.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_path_point_idx_from_last_explored_point = i;
    }
  }
  
  double min_dist1 = 1000000000;
  int nearest_path_point_idx_from_goal_point;
  for (int i = 0; i < path_points.size(); i++)
  {
    double dx = path_points[i].pose.position.x - goal_point_in_map.x;
    double dy = path_points[i].pose.position.y - goal_point_in_map.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_dist1)
    {
      min_dist1 = dist;
      nearest_path_point_idx_from_goal_point = i;
    }
  }
  if(nearest_path_point_idx_from_last_explored_point >= 
     nearest_path_point_idx_from_goal_point)
  {
    return false;
  }
  
  double accum_dist = 0;
  for (int i = nearest_path_point_idx_from_last_explored_point;
       i < nearest_path_point_idx_from_goal_point-1; i++)
  {
    double dx = path_points[i].pose.position.x-
                path_points[i+1].pose.position.x;
    double dy = path_points[i].pose.position.y-
                path_points[i+1].pose.position.y;
    accum_dist += std::sqrt(dx*dx+dy*dy);
    
  }
  // std::cout << "nearest path idx from last explored "<< nearest_path_point_idx_from_last_explored_point << std::endl;
  // std::cout << "nearest path idx from goal "<< nearest_path_point_idx_from_goal_point << std::endl;
  // std::cout << "accum dist "<< accum_dist << std::endl;
  if(accum_dist > min_radius_*3 )
  {
    double dx = goal_point_in_map.x - start_point_in_map.x;
    double dy = goal_point_in_map.y - start_point_in_map.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    // std::cout << "dist between goal and start "<<dist << std::endl;
    if(dist < min_radius_)
    {
      return false;
    }
    else
    {
      start_point_in_map = cached_explored_points_ptr->back();
      return true;
    }
  }
  return false;
}

bool ModifyReferencePath::generateModifiedPath(
  geometry_msgs::Pose& ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint>& path_points,
  const std::vector<autoware_perception_msgs::DynamicObject>& objects,
  const lanelet::routing::RoutingGraph& graph,
  lanelet::LaneletMap& map,
  const autoware_planning_msgs::Route& route,
  std::vector<geometry_msgs::Point>& debug_points,
  geometry_msgs::Point& debug_goal_point_in_map)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(!debug_fix_pose_&&is_fix_pose_mode_for_debug_)
  {
    debug_fix_pose_ = std::make_unique<geometry_msgs::Pose>(ego_pose);
  }
  else if(debug_fix_pose_&&is_fix_pose_mode_for_debug_)
  {
    ego_pose.position = debug_fix_pose_->position;
    ego_pose.orientation = debug_fix_pose_->orientation;
  }
  
  
  cv::Mat image = cv::Mat::zeros(
    (int)(2*y_width_/resolution_), 
    (int)(2*x_length_/resolution_),
    CV_8UC1);
    
  //use route to draw driveable area; should be deprecated in the future
  lanelet::BasicPoint2d ego_point(ego_pose.position.x, ego_pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
      lanelet::geometry::findNearest(map.laneletLayer, ego_point, 3);
  if(nearest_lanelets.empty())
  {
    ROS_ERROR("[EBPathPlanner] Ego vechicle is not in the lanelet ares");
    return false;
  }
  int nearest_route_secition_idx_from_ego_pose = 0; 
  for (int i = 0; i < route.route_sections.size(); i++)
  {
    for(const auto& llid: route.route_sections[i].lane_ids)
    {
      if(llid == nearest_lanelets.front().second.id())
      {
        nearest_route_secition_idx_from_ego_pose = i;
      }
    }
  }
  
  int exploring_route_section_id = 
    std::min(nearest_route_secition_idx_from_ego_pose+
              num_lookup_lanelet_for_drivealble_area_,
            (int)route.route_sections.size());
  for (int i = nearest_route_secition_idx_from_ego_pose;
       i < exploring_route_section_id; i++)
  {
    for(const auto& llid: route.route_sections[i].lane_ids)
    {
      lanelet::ConstLanelet llt = map.laneletLayer.get(llid);
      std::vector<cv::Point> cv_points;
      for(const auto& point: llt.polygon3d())
      {
        geometry_msgs::Point tmp;
        tmp.x = point.x();
        tmp.y = point.y();
        tmp.z = point.z();
        geometry_msgs::Point image_point;
        if(transformMapToImage(tmp, ego_pose,x_length_*2, y_width_*2, resolution_,image_point))
        {
          int pixel_x = image_point.x;
          int pixel_y = image_point.y;
          cv_points.emplace_back(cv::Point(pixel_x, pixel_y));
          geometry_msgs::Point point;
          point.x = pixel_x;
          point.y = pixel_y;
        }
      }
      cv::fillConvexPoly(image, cv_points.data(), cv_points.size(), cv::Scalar(255));
    }
  }
  //end route
  // cv::Rect rect = cv::Rect(left-top-x, left-top-y, width, height);
  cv::Rect rect = cv::Rect(y_width_/resolution_/2, x_length_/resolution_/2, 
                            y_width_/resolution_, x_length_/resolution_);
  image = image(rect);
  
  
  for(const auto& object: objects)
  {
    if(object.state.twist_covariance.twist.linear.x < static_objects_velocity_threshold_)
    {
      geometry_msgs::Point point_in_image;
      if(transformMapToImage(object.state.pose_covariance.pose.position, ego_pose, x_length_, 
                        y_width_, resolution_, point_in_image))
      {
        cv::rectangle(image,cv::Point(std::floor(point_in_image.x)-(std::floor((1.5*min_radius_/resolution_)/std::sqrt(2))), 
                                      std::floor(point_in_image.y)-(std::floor((1.5*min_radius_/resolution_)/std::sqrt(2)))),
                            cv::Point(std::floor(point_in_image.x)+(std::floor((1.5*min_radius_/resolution_)/std::sqrt(2))), 
                                      std::floor(point_in_image.y)+(std::floor((1.5*min_radius_/resolution_)/std::sqrt(2)))),
                            cv::Scalar(0), -1 ,CV_AA);
      } 
    }
  }
  //dummy object @ kahiwanoha
  // geometry_msgs::Point point_in_map;
  // point_in_map.x = 116.212295532;
  // point_in_map.y = -88.9616149902;
  // point_in_map.z =  0.0;

  // geometry_msgs::Point point_in_image;
  // if(transformMapToImage(point_in_map, ego_pose, x_length_, 
  //                   y_width_, resolution_, point_in_image))
  // {
  //   cv::rectangle(image,cv::Point(std::floor(point_in_image.x)-(std::floor((min_radius_/resolution_)/std::sqrt(2))), 
  //                               std::floor(point_in_image.y)-(std::floor((min_radius_/resolution_)/std::sqrt(2)))),
  //                     cv::Point(std::floor(point_in_image.x)+(std::floor((min_radius_/resolution_)/std::sqrt(2))), 
  //                               std::floor(point_in_image.y)+(std::floor((min_radius_/resolution_)/std::sqrt(2)))),
  //                     cv::Scalar(0), -1 ,CV_AA);
  // } 
  //end dummy object
  
  if(is_debug_driveable_area_mode_)
  {
    cv::Mat tmp;
    image.copyTo(tmp);
    // geometry_msgs::Point point_in_map;
    // point_in_map.x = 116.212295532;
    // point_in_map.y = -88.9616149902;
    // point_in_map.z =  0.0;

    // geometry_msgs::Point point_in_image;
    // if(transformMapToImage(point_in_map, ego_pose, x_length_, 
    //                   y_width_, resolution_, point_in_image))
    // {
    //   cv::rectangle(tmp,cv::Point(std::floor(point_in_image.x)-(std::floor((min_radius_/resolution_)/std::sqrt(2))), 
    //                               std::floor(point_in_image.y)-(std::floor((min_radius_/resolution_)/std::sqrt(2)))),
    //                     cv::Point(std::floor(point_in_image.x)+(std::floor((min_radius_/resolution_)/std::sqrt(2))), 
    //                               std::floor(point_in_image.y)+(std::floor((min_radius_/resolution_)/std::sqrt(2)))),
    //                     cv::Scalar(0), -1 ,CV_AA);
    // } 
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "lanelet find nearest & fill convex fill "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
  std::chrono::high_resolution_clock::time_point begin1= 
    std::chrono::high_resolution_clock::now();
  cv::Mat dst;
  cv::distanceTransform(image, dst, cv::DIST_L2, 5);
  geometry_msgs::Point start_point_in_map;
  std::chrono::high_resolution_clock::time_point end1= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time1 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - begin1);
  std::cout << " edt time: "<< time1.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
  
  std::chrono::high_resolution_clock::time_point begin2= 
    std::chrono::high_resolution_clock::now();
  int nearest_path_point_ind_from_prev_goal = 0;
  if(previous_exploring_goal_point_in_map_ptr_)
  {
    double min_dist = 999999999;
    for (int i = 0; i < path_points.size(); i++)
    {
      double dx = path_points[i].pose.position.x - 
                  previous_exploring_goal_point_in_map_ptr_->position.x;
      double dy = path_points[i].pose.position.y - 
                  previous_exploring_goal_point_in_map_ptr_->position.y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist < min_dist)
      {
        min_dist = dist;
        nearest_path_point_ind_from_prev_goal = i;
      }
    }
  }
  std::unique_ptr<geometry_msgs::Pose> exploring_goal_pose_in_map_ptr;
  double accum_dist = 0;
  for (int i = nearest_path_point_ind_from_prev_goal; 
           i < path_points.size(); i++)
  {
    if(path_points.size()<=1)
    {
      exploring_goal_pose_in_map_ptr = 
          std::make_unique<geometry_msgs::Pose>(ego_pose);
      break;
    }
    
    if(i!=nearest_path_point_ind_from_prev_goal)
    {
      double dx = path_points[i].pose.position.x - 
                  path_points[i-1].pose.position.x;
      double dy = path_points[i].pose.position.y - 
                  path_points[i-1].pose.position.y;
      accum_dist+= std::sqrt(dx*dx+dy*dy);
      
    }
    geometry_msgs::Point image_point;
    if(transformMapToImage(path_points[i].pose.position, 
                            ego_pose,
                            x_length_,
                            y_width_, 
                            resolution_,
                            image_point))
    {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = dst.ptr<float>((int)pixel_y)[(int)pixel_x]; 
      if(clearance>min_radius_/resolution_ )
      {
        if(accum_dist > max_radius_)
        {
          exploring_goal_pose_in_map_ptr = 
            std::make_unique<geometry_msgs::Pose>(path_points[i].pose);
          break;
        }
      }
    }
  }
  if(!exploring_goal_pose_in_map_ptr && previous_exploring_goal_point_in_map_ptr_)
  {
    exploring_goal_pose_in_map_ptr = 
      std::make_unique<geometry_msgs::Pose>(*previous_exploring_goal_point_in_map_ptr_);
  }
  else if(!exploring_goal_pose_in_map_ptr)
  {
    exploring_goal_pose_in_map_ptr = 
      std::make_unique<geometry_msgs::Pose>(ego_pose);
  }
  debug_goal_point_in_map = exploring_goal_pose_in_map_ptr->position;
  
  
  if(!needExploration(ego_pose, 
                      exploring_goal_pose_in_map_ptr->position, 
                      path_points,
                      cached_explored_points_ptr_, 
                      start_point_in_map) && 
     !is_debug_graph_a_star_mode_)
  {
    debug_points = *cached_explored_points_ptr_;
    std::chrono::high_resolution_clock::time_point end2= 
    std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time2 = 
      std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
    std::cout << "  explore goal + only decision logic "<< time2.count()/(1000.0*1000.0)<<" ms" <<std::endl;
    return true;
  }
  if(is_debug_graph_a_star_mode_)
  {
    solveGraphAStar(ego_pose, 
                    ego_pose.position, 
                    exploring_goal_pose_in_map_ptr->position, 
                    dst, 
                    debug_points);
  }
  else
  {
    std::vector<geometry_msgs::Point> explored_points;
    if(solveGraphAStar(ego_pose, 
                    start_point_in_map, 
                    exploring_goal_pose_in_map_ptr->position, 
                    dst, 
                    explored_points))
    {
      cached_explored_points_ptr_->insert(cached_explored_points_ptr_->end(),
                                          explored_points.begin(),
                                          explored_points.end());
    }
                    
    double min_dist = 100000000;
    int min_ind = 0;
    for (size_t i = 0; i < cached_explored_points_ptr_->size(); i++)
    {
      double dx = cached_explored_points_ptr_->at(i).x - ego_pose.position.x;
      double dy = cached_explored_points_ptr_->at(i).y - ego_pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      if(dist < min_dist)
      {
        min_dist = dist;
        min_ind = i;
      }
    }
    int num_backing_points = std::ceil(backward_distance_/min_radius_);
    min_ind = std::max(0, min_ind - num_backing_points);
    cached_explored_points_ptr_->erase(cached_explored_points_ptr_->begin(),
                                      cached_explored_points_ptr_->begin()+min_ind);
    debug_points = *cached_explored_points_ptr_;
  }
  
  previous_exploring_goal_point_in_map_ptr_ = 
    std::make_unique<geometry_msgs::Pose>(*exploring_goal_pose_in_map_ptr);
  std::chrono::high_resolution_clock::time_point end2= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time2 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  std::cout << "  explore goal + decision logic + explore path "<< time2.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  // cv::imwrite("/home/kosuke/aaa.jpg", dst);	
  return true;
}