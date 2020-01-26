
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
#include <nav_msgs/MapMetaData.h>

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

// bool transformMapToImage(const geometry_msgs::Point& map_point,
//                          const geometry_msgs::Pose& map_ego_pose,
//                          const int ego_costmap_x_length,
//                          const int ego_costmap_y_width,
//                          const double costmap_resolution,
//                          geometry_msgs::Point& image_point)
// {
//   geometry_msgs::Point relative_p = 
//     transformToRelativeCoordinate2D(map_point, map_ego_pose);
//   double bottomleft_x = (relative_p.x+ego_costmap_x_length/2)/costmap_resolution;
//   double bottomleft_y = (relative_p.y+ego_costmap_y_width/2)/costmap_resolution;
//   double image_x = ego_costmap_y_width/costmap_resolution - bottomleft_y;
//   double image_y = ego_costmap_x_length/costmap_resolution - bottomleft_x;
//   if(image_x>=0 && 
//      image_x<(int)ego_costmap_y_width/costmap_resolution &&
//      image_y>=0 && 
//      image_y<(int)ego_costmap_x_length/costmap_resolution)
//   {
//     image_point.x = image_x;
//     image_point.y = image_y;
//     return true;
//   }
//   else
//   {
//     return false;
//   } 
// }

// bool transformImageToMap(const geometry_msgs::Point& image_point,
//                          const geometry_msgs::Pose& map_ego_pose,
//                          const int ego_costmap_x_length,
//                          const int ego_costmap_y_width,
//                          const double costmap_resolution,
//                          geometry_msgs::Point& map_point)
// {
//   double bottomleft_x = ego_costmap_y_width/costmap_resolution - image_point.y;
//   double bottomleft_y = ego_costmap_x_length/costmap_resolution - image_point.x;
//   double relative_x = bottomleft_x*costmap_resolution - ego_costmap_x_length/2;
//   double relative_y = bottomleft_y*costmap_resolution - ego_costmap_y_width/2;
//   double yaw = tf2::getYaw(map_ego_pose.orientation);

//   geometry_msgs::Point res;
//   res.x = (cos(-yaw) * relative_x) + (sin(-yaw) * relative_y);
//   res.y = ((-1) * sin(-yaw) * relative_x) + (cos(-yaw) * relative_y);
  
//   map_point.x = res.x + map_ego_pose.position.x;
//   map_point.y = res.y + map_ego_pose.position.y;
//   map_point.z = map_ego_pose.position.z;
//   return true;
// }

bool transformMapToImage(const geometry_msgs::Point& map_point,
                         const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& image_point)
{
  geometry_msgs::Point relative_p = 
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  // std::cout << "relative p "<<relative_p.x <<" "<<relative_p.y << std::endl;
  // double bottomleft_x = (relative_p.x+ego_costmap_x_length/2)/costmap_resolution;
  // double bottomleft_y = (relative_p.y+ego_costmap_y_width/2)/costmap_resolution;
  // std::cout << "heifht "<< occupancy_grid_info.height << std::endl;
  // std::cout << "width "<< occupancy_grid_info.width << std::endl;
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x/resolution;
  double map_y_in_image_resolution = relative_p.y/resolution;
  double image_x = map_y_height - map_y_in_image_resolution;
  double image_y = map_x_width - map_x_in_image_resolution;
  // double bottomleft_x = (relative_p.x)/costmap_resolution;
  // double bottomleft_y = (relative_p.y)/costmap_resolution;
  // double image_x = ego_costmap_y_width/costmap_resolution - bottomleft_y;
  // double image_y = ego_costmap_x_length/costmap_resolution - bottomleft_x;
  // std::cout << "costmap width "<< ego_costmap_y_width << std::endl;
  // std::cout << "costmap length "<< ego_costmap_x_length << std::endl;
  // std::cout << "image x y "<< image_x << " "<< image_y << std::endl;
  if(image_x>=0 && 
     image_x<(int)map_y_height &&
     image_y>=0 && 
     image_y<(int)map_x_width)
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
                         const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& map_point)
{
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = map_x_width - image_point.y;
  double map_y_in_image_resolution = map_y_height - image_point.x;
  double relative_x = map_x_in_image_resolution*resolution;
  double relative_y = map_y_in_image_resolution*resolution;
  
  // double bottomleft_x = ego_costmap_y_width/costmap_resolution - image_point.y;
  // double bottomleft_y = ego_costmap_x_length/costmap_resolution - image_point.x;
  // double relative_x = bottomleft_x*costmap_resolution - ego_costmap_x_length/2;
  // double relative_y = bottomleft_y*costmap_resolution - ego_costmap_y_width/2;

  double yaw = tf2::getYaw(occupancy_grid_info.origin.orientation);
  geometry_msgs::Point res;
  res.x = (cos(-yaw) * relative_x) + (sin(-yaw) * relative_y);
  res.y = ((-1) * sin(-yaw) * relative_x) + (cos(-yaw) * relative_y);
  
  map_point.x = res.x + occupancy_grid_info.origin.position.x;
  map_point.y = res.y + occupancy_grid_info.origin.position.y;
  map_point.z = occupancy_grid_info.origin.position.z;
  return true;
}



ModifyReferencePath::ModifyReferencePath(
  int num_lookup_lanelet_for_driveable_area,
  double min_radius,
  double backward_distance):
is_fix_pose_mode_for_debug_(false),
is_debug_each_iteration_mode_(false),
is_debug_driveable_area_mode_(false),
is_debug_clearance_map_mode_(false),
clearance_map_y_width_(100),
clearance_map_x_length_(100),
num_lookup_lanelet_for_drivealble_area_(num_lookup_lanelet_for_driveable_area),
resolution_(0.1),
time_limit_(200.0),
min_radius_(min_radius),
max_radius_(min_radius),
backward_distance_(backward_distance),
static_objects_velocity_ms_threshold_(1.1),
loosing_clerance_for_explore_goal_threshold_(0.2),
heuristic_epsilon_(5)
{
}

ModifyReferencePath::~ModifyReferencePath() 
{
}

bool ModifyReferencePath::expandNode(Node& parent_node, 
                             const cv::Mat& clearance_map,
                             const nav_msgs::MapMetaData& map_info,
                             const Node& goal_node,
                             const double min_r,
                             const double max_r,
                             std::vector<Node>& child_nodes)
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
    clearance_map.copyTo(tmp);
    cv::circle(tmp, 
              cv::Point(goal_node.p(0), goal_node.p(1)), 
              10, 
              cv::Scalar(0),
              1);
    cv::rectangle(tmp, cv::Point(std::floor(parent_node.p(0))-(std::floor((current_r/resolution_)/std::sqrt(2))), 
                                std::floor(parent_node.p(1))-(std::floor((current_r/resolution_)/std::sqrt(2)))),
                      cv::Point(std::floor(parent_node.p(0))+(std::floor((current_r/resolution_)/std::sqrt(2))), 
                                std::floor(parent_node.p(1))+(std::floor((current_r/resolution_)/std::sqrt(2)))),
                      cv::Scalar(0), -1 ,CV_AA);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(20);
  }

  
  // std::vector<Node> child_nodes;
  Eigen::Vector2d delta_child_p;
  delta_child_p << current_r/resolution_,
                   0;
  // double delta_theta = 2*M_PI/36.0;
  // double delta_theta = 2*M_PI/54.0;
  double delta_theta = 2*M_PI/96.0;
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
       child_node.p(0) < map_info.height && 
       child_node.p(1)*resolution_ >=0 &&
       child_node.p(1) < map_info.width)
    {
      double tmp_r = clearance_map.ptr<float>
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
    //weighted a*: solution is epsiolon suboptimal
    child_node.h = calculateEigen2DDistance(child_node.p, goal_node.p)*resolution_*heuristic_epsilon_;
    child_node.f = child_node.g + child_node.h;
    
    child_node.parent_node = std::make_shared<Node>(parent_node);
    child_nodes.push_back(child_node);
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
                     const nav_msgs::MapMetaData& map_info,
                     std::vector<geometry_msgs::Point>& explored_points)
{
  geometry_msgs::Point start_point_in_image;
  geometry_msgs::Point goal_point_in_image;
  // if(transformMapToImage(start_point_in_map, ego_pose, clearance_map_x_length_, 
  //                     clearance_map_y_width_, resolution_, start_point_in_image) &&
  //    transformMapToImage(goal_point_in_map, ego_pose, clearance_map_x_length_, 
  //                     clearance_map_y_width_, resolution_, goal_point_in_image))
  if(transformMapToImage(start_point_in_map,
                         map_info,
                         start_point_in_image)&&
     transformMapToImage(goal_point_in_map, 
                         map_info,
                         goal_point_in_image))
  {
    std::cout << "aaa" << std::endl;
    std::vector<Node> s_open;
    Node initial_node;
    double initial_r = clearance_map.ptr<float>((int)start_point_in_image.y)
                                                [(int)start_point_in_image.x]*resolution_;
    if(initial_r <= min_radius_)
    {
      initial_r = min_radius_;
    }
    else if(initial_r >= max_radius_)
    {
      initial_r = max_radius_;
    }
    initial_node.r = initial_r;
    initial_node.g = 0;
    Eigen::Vector2d initial_p(start_point_in_image.x, start_point_in_image.y);
    Eigen::Vector2d goal_p(goal_point_in_image.x, goal_point_in_image.y);
    
    initial_node.p = initial_p;
    initial_node.h = calculateEigen2DDistance(initial_node.p, goal_p)*resolution_;
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
        ROS_WARN("[EBPathPlanner] Exceed time limit of %lf [ms], Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
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
        // std::cout << "node exist in clodes" << std::endl;
        std::sort(s_open.begin(), s_open.end(), 
                [](const Node& i, const Node& j){return i.f < j.f;});
        lowest_f_node = s_open.front();s_open.erase(s_open.begin());
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
                    map_info,
                    goal_node,
                    min_radius_,
                    max_radius_,
                    child_nodes);
        s_closed.push_back(lowest_f_node);
        s_open.insert(s_open.end(), child_nodes.begin(), child_nodes.end());
        std::sort(s_open.begin(), s_open.end(), 
                [](const Node& i, const Node& j){return i.f < j.f;});
        lowest_f_node = s_open.front();s_open.erase(s_open.begin());
      }
    }
    if(!is_explore_success)
    {
      ROS_WARN_THROTTLE(3.0, "[EBPathPlanner] graph a star could not find path");
      ROS_INFO("[EBPathPlanner] Start point: %lf %lf %lf ", 
        start_point_in_map.x, 
        start_point_in_map.y, 
        start_point_in_map.z);
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
    while(current_node.parent_node != nullptr)
    {
      geometry_msgs::Point explored_point;
      explored_point.x = current_node.p(0);
      explored_point.y = current_node.p(1);
      explored_point.z = 0;
      explored_image_points.insert(explored_image_points.begin(), explored_point);
      current_node = *current_node.parent_node;
    }
    //TODO: seek better way to write this part
    geometry_msgs::Point explored_point;
    explored_point.x = initial_node.p(0);
    explored_point.y = initial_node.p(1);
    explored_point.z = 0;
    explored_image_points.insert(explored_image_points.begin() ,explored_point);
  
    for(const auto& point: explored_image_points)
    {
      geometry_msgs::Point map_point;
      // transformImageToMap(point, ego_pose, clearance_map_x_length_, clearance_map_y_width_, resolution_,map_point);
      transformImageToMap(point,
                          map_info,
                          map_point);
      explored_points.push_back(map_point);
    }
    return true;
  }
  else
  {
    ROS_WARN_THROTTLE(2.0, "Strat point or/and goal point is outside of drivable area");
    return false;
  }
}

// bool ModifyReferencePath::arrangeExploredPointsBaseedOnClearance(
//     const cv::Mat& clearance_map,
//     const geometry_msgs::Pose& ego_pose,
//     std::vector<geometry_msgs::Point>& explored_points,
//     std::vector<geometry_msgs::Point>& debug_rearranged_points)
// {
//   // for(auto& point_in_map: explored_points)
//   for (int i = 1; i < explored_points.size()-2; i++)
//   {
//     geometry_msgs::Point point_in_map = explored_points[i];
//     geometry_msgs::Point point_in_image;
//     if(transformMapToImage(point_in_map, ego_pose, clearance_map_x_length_, 
//                     clearance_map_y_width_, resolution_, point_in_image))
//     {
//       Eigen::Vector2d eigen_point_in_image;
//       eigen_point_in_image << point_in_image.x, point_in_image.y; 
//       double current_r = clearance_map.ptr<float>
//         ((int)point_in_image.y)
//         [(int)point_in_image.x]*resolution_;
//       Eigen::Vector2d delta_p;
//       delta_p << current_r/resolution_ - resolution_*10,
//                       0;
//       double delta_theta = 2*M_PI/54.0;
//       double max_r = -1;
//       geometry_msgs::Point rearranged_point_in_image;
//       geometry_msgs::Point min_delta_p;
//       for(double theta = 0; theta < 2*M_PI; theta += delta_theta)
//       {
//         Eigen::Matrix2d rotation;
//         rotation << std::cos(theta), - std::sin(theta),
//                     std::sin(theta),   std::cos(theta);
//         Eigen::Vector2d rotated_delta = rotation * delta_p;
//         Eigen::Vector2d expanded_p;
//         expanded_p = rotated_delta + eigen_point_in_image;
//         if(expanded_p(0)*resolution_ >=0 && 
//            expanded_p(0)*resolution_ < clearance_map_y_width_ && 
//            expanded_p(1)*resolution_ >=0 &&
//            expanded_p(1)*resolution_ < clearance_map_x_length_)
//         {
//           double tmp_r = clearance_map.ptr<float>
//                           ((int)expanded_p(1))
//                           [(int)expanded_p(0)]*resolution_;
//           if(tmp_r > max_r)
//           {
//             max_r = tmp_r;
//             rearranged_point_in_image.x = expanded_p(0);
//             rearranged_point_in_image.y = expanded_p(1);
//             min_delta_p.x = rotated_delta(0);
//             min_delta_p.y = rotated_delta(1);
//           }
//         }
//         else 
//         {
//           continue;
//         }
//       }
//       geometry_msgs::Point rearranged_point_in_map;
//       transformImageToMap(rearranged_point_in_image, 
//                           ego_pose, 
//                           clearance_map_x_length_, 
//                           clearance_map_y_width_, 
//                           resolution_,
//                           rearranged_point_in_map);
//       debug_rearranged_points.push_back(rearranged_point_in_map);
//       if(max_r > current_r)
//       {
//         explored_points[i] = rearranged_point_in_map;
//       }        
      
//       // Eigen::Vector2d eigen_point_in_image;
//       // eigen_point_in_image << point_in_image.x, point_in_image.y; 
//       // double current_r = clearance_map.ptr<float>
//       //   ((int)point_in_image.y)
//       //   [(int)point_in_image.x]*resolution_;
//       // Eigen::Vector2d delta_p;
//       // delta_p << current_r/resolution_ - resolution_*10,
//       //                 0;
//       // double delta_theta = 2*M_PI/54.0;
//       // // double max_r = -1;
//       // double min_r = 10000000;
//       // geometry_msgs::Point rearranged_point_in_image;
//       // geometry_msgs::Point min_delta_p;
//       // for(double theta = 0; theta < 2*M_PI; theta += delta_theta)
//       // {
//       //   Eigen::Matrix2d rotation;
//       //   rotation << std::cos(theta), - std::sin(theta),
//       //               std::sin(theta),   std::cos(theta);
//       //   Eigen::Vector2d rotated_delta = rotation * delta_p;
//       //   Eigen::Vector2d expanded_p;
//       //   expanded_p = rotated_delta + eigen_point_in_image;
//       //   if(expanded_p(0)*resolution_ >=0 && 
//       //      expanded_p(0)*resolution_ < clearance_map_y_width_ && 
//       //      expanded_p(1)*resolution_ >=0 &&
//       //      expanded_p(1)*resolution_ < clearance_map_x_length_)
//       //   {
//       //     double tmp_r = clearance_map.ptr<float>
//       //                     ((int)expanded_p(1))
//       //                     [(int)expanded_p(0)]*resolution_;
//       //     // std::cout << "tmp r "<< tmp_r << std::endl;
//       //     if(tmp_r < min_r)
//       //     {
//       //       min_r = tmp_r;
//       //       rearranged_point_in_image.x = expanded_p(0);
//       //       rearranged_point_in_image.y = expanded_p(1);
//       //       min_delta_p.x = rotated_delta(0);
//       //       min_delta_p.y = rotated_delta(1);
//       //     }
//       //   }
//       //   else 
//       //   {
//       //     continue;
//       //   }
//       // }
//       // geometry_msgs::Point rearranged_point_in_map;
//       // transformImageToMap(rearranged_point_in_image, 
//       //                     ego_pose, 
//       //                     clearance_map_x_length_, 
//       //                     clearance_map_y_width_, 
//       //                     resolution_,
//       //                     rearranged_point_in_map);
//       // debug_rearranged_points.push_back(rearranged_point_in_map);
      
      
//       // // geometry_msgs::Point opposed_min_point;
//       // // geometry_msgs::Point opposed_min_point_in_map;
//       // // opposed_min_point.x = point_in_image.x - min_delta_p.x;
//       // // opposed_min_point.y = point_in_image.y - min_delta_p.y;
      
//       // // double opposed_min_point_r = clearance_map.ptr<float>
//       // //   ((int)opposed_min_point.y)
//       // //   [(int)opposed_min_point.x]*resolution_;
//       // // transformImageToMap(opposed_min_point, 
//       // //                     ego_pose, 
//       // //                     clearance_map_x_length_, 
//       // //                     clearance_map_y_width_, 
//       // //                     resolution_,
//       // //                     opposed_min_point_in_map);
//       // // debug_rearranged_points.push_back(opposed_min_point_in_map);
//       // // if(opposed_min_point_r > current_r)
//       // // {
//       // //   explored_points[i] = opposed_min_point_in_map;
//       // // }
      
//       // geometry_msgs::Point opposed_point_in_image;
//       // cv::LineIterator it(clearance_map, 
//       //                  cv::Point((int)point_in_image.x,
//       //                            (int)point_in_image.y),
//       //                  cv::Point((int)(point_in_image.x-2*min_delta_p.x),
//       //                            (int)(point_in_image.y-2*min_delta_p.y)),
//       //                  8);
//       // double max_r = -1;
//       // for (size_t i = 0; i < it.count; i++, ++it)
//       // {
//       //   double r_on_line = clearance_map.ptr<float>
//       //                       ((int)(it.pos().y))
//       //                       [(int)(it.pos().x)]*resolution_;
//       //   if(r_on_line > max_r)
//       //   {
//       //     max_r = r_on_line;
//       //     opposed_point_in_image.x = it.pos().x;
//       //     opposed_point_in_image.y = it.pos().y;
//       //   }
//       // }
//       // geometry_msgs::Point opposed_point_in_map;
//       // transformImageToMap(opposed_point_in_image, 
//       //                     ego_pose, 
//       //                     clearance_map_x_length_, 
//       //                     clearance_map_y_width_, 
//       //                     resolution_,
//       //                     opposed_point_in_map);
//       // debug_rearranged_points.push_back(opposed_point_in_map);
//       // std::cout << "max r "<< max_r << " current r "<<current_r << std::endl;
//       // if(max_r > current_r)
//       // {
//       //   explored_points[i] = opposed_point_in_map;
//       // }                    
//     }
//   }
//   return true;
// }

bool ModifyReferencePath::generateModifiedPath(
  geometry_msgs::Pose& ego_pose,
  const geometry_msgs::Pose& start_exploring_pose,
  const std::vector<autoware_planning_msgs::PathPoint>& path_points,
  const std::vector<autoware_perception_msgs::DynamicObject>& objects,
  const lanelet::routing::RoutingGraph& graph,
  lanelet::LaneletMap& map,
  const autoware_planning_msgs::Route& route,
  std::vector<geometry_msgs::Point>& explored_points,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  geometry_msgs::Point& debug_goal_point_in_map,
  std::vector<geometry_msgs::Point>& debug_rearrange_points)
{
  // std::chrono::high_resolution_clock::time_point begin= 
  //   std::chrono::high_resolution_clock::now();
  // if(!debug_fix_pose_&&is_fix_pose_mode_for_debug_)
  // {
  //   debug_fix_pose_ = std::make_unique<geometry_msgs::Pose>(ego_pose);
  // }
  // else if(debug_fix_pose_&&is_fix_pose_mode_for_debug_)
  // {
  //   ego_pose.position = debug_fix_pose_->position;
  //   ego_pose.orientation = debug_fix_pose_->orientation;
  // }
  
  
  // cv::Mat image = cv::Mat::zeros(
  //   (int)(2*clearance_map_y_width_/resolution_), 
  //   (int)(2*clearance_map_x_length_/resolution_),
  //   CV_8UC1);
    
  // //use route to draw driveable area; should be deprecated in the future
  // lanelet::BasicPoint2d ego_point(ego_pose.position.x, ego_pose.position.y);
  // std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
  //     lanelet::geometry::findNearest(map.laneletLayer, ego_point, 1);
  // if(nearest_lanelets.empty())
  // {
  //   ROS_WARN("[EBPathPlanner] Ego vechicle is not in the lanelet ares");
  //   return false;
  // }
  // int nearest_route_secition_idx_from_ego_pose = 0; 
  // for (int i = 0; i < route.route_sections.size(); i++)
  // {
  //   for(const auto& llid: route.route_sections[i].lane_ids)
  //   {
  //     for(const auto& nearest_lanelet: nearest_lanelets)
  //     {
  //       if(llid == nearest_lanelet.second.id())
  //       {
  //         nearest_route_secition_idx_from_ego_pose = i;
  //       }
  //     }
  //   }
  // }
  
  // int exploring_route_section_id = 
  //   std::min(nearest_route_secition_idx_from_ego_pose+
  //             num_lookup_lanelet_for_drivealble_area_,
  //           (int)route.route_sections.size());
  // for (int i = nearest_route_secition_idx_from_ego_pose;
  //      i < exploring_route_section_id; i++)
  // {
  //   for(const auto& llid: route.route_sections[i].lane_ids)
  //   {
  //     lanelet::ConstLanelet llt = map.laneletLayer.get(llid);
  //     std::vector<cv::Point> cv_points;
  //     for(const auto& point: llt.polygon3d())
  //     {
  //       geometry_msgs::Point tmp;
  //       tmp.x = point.x();
  //       tmp.y = point.y();
  //       tmp.z = point.z();
  //       geometry_msgs::Point image_point;
  //       if(transformMapToImage(tmp, ego_pose,clearance_map_x_length_*2, clearance_map_y_width_*2, resolution_,image_point))
  //       {
  //         int pixel_x = image_point.x;
  //         int pixel_y = image_point.y;
  //         cv_points.emplace_back(cv::Point(pixel_x, pixel_y));
  //         geometry_msgs::Point point;
  //         point.x = pixel_x;
  //         point.y = pixel_y;
  //       }
  //     }
  //     cv::fillConvexPoly(image, cv_points.data(), cv_points.size(), cv::Scalar(255));
  //   }
  // }
  // //end route
  // // cv::Rect rect = cv::Rect(left-top-x, left-top-y, width, height);
  // cv::Rect rect = cv::Rect(clearance_map_y_width_/resolution_/2, clearance_map_x_length_/resolution_/2, 
  //                           clearance_map_y_width_/resolution_, clearance_map_x_length_/resolution_);
  // image = image(rect);
  
  
  // for(const auto& object: objects)
  // {
  //   if(object.state.twist_covariance.twist.linear.x < static_objects_velocity_ms_threshold_)
  //   {
  //     geometry_msgs::Point point_in_image;
  //     if(transformMapToImage(object.state.pose_covariance.pose.position, ego_pose, clearance_map_x_length_, 
  //                       clearance_map_y_width_, resolution_, point_in_image))
  //     {
  //       cv::circle(image, 
  //                  cv::Point(point_in_image.x, point_in_image.y), 
  //                  15, 
  //                  cv::Scalar(0),
  //                  -1);
  //     } 
  //   }
  // }
  
  // if(is_debug_driveable_area_mode_)
  // {
  //   cv::Mat tmp;
  //   image.copyTo(tmp);
  //   cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  //   cv::imshow("image", tmp);
  //   cv::waitKey(10);
  //   // cv::destroyAllWindows();
  // }
  // std::chrono::high_resolution_clock::time_point end= 
  //   std::chrono::high_resolution_clock::now();
  // std::chrono::nanoseconds time = 
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  // std::cout << " lanelet find nearest & fill convex fill "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
  // std::chrono::high_resolution_clock::time_point begin1= 
  //   std::chrono::high_resolution_clock::now();
  // // cv::Mat clearance_map;
  // cv::distanceTransform(image, clearance_map, cv::DIST_L2, 5);
  // geometry_msgs::Point start_point_in_map;
  // std::chrono::high_resolution_clock::time_point end1= 
  //   std::chrono::high_resolution_clock::now();
  // std::chrono::nanoseconds time1 = 
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - begin1);
  // std::cout << " edt time: "<< time1.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
  std::chrono::high_resolution_clock::time_point begin2= 
    std::chrono::high_resolution_clock::now();
  if(is_debug_clearance_map_mode_)
  { 
    cv::Mat tmp;
    clearance_map.copyTo(tmp);
    cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(10);
  }
  
  int nearest_path_point_ind_from_start_exploring_point = 0;
  double min_dist = 999999999;
  for (int i = 0; i < path_points.size(); i++)
  {
    double dx = path_points[i].pose.position.x - 
                start_exploring_pose.position.x;
    double dy = path_points[i].pose.position.y - 
                start_exploring_pose.position.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_path_point_ind_from_start_exploring_point = i;
    }
  }
  
  std::unique_ptr<geometry_msgs::Pose> exploring_goal_pose_in_map_ptr;
  double accum_dist = 0;
  for (int i = nearest_path_point_ind_from_start_exploring_point; 
           i < path_points.size(); i++)
  {
    if(path_points.size()<=1)
    {
      exploring_goal_pose_in_map_ptr = 
          std::make_unique<geometry_msgs::Pose>(ego_pose);
      break;
    }
    
    if(i!=nearest_path_point_ind_from_start_exploring_point)
    {
      double dx = path_points[i].pose.position.x - 
                  path_points[i-1].pose.position.x;
      double dy = path_points[i].pose.position.y - 
                  path_points[i-1].pose.position.y;
      accum_dist+= std::sqrt(dx*dx+dy*dy);
      
    }
    geometry_msgs::Point image_point;
    // if(transformMapToImage(path_points[i].pose.position, 
    //                         ego_pose,
    //                         clearance_map_x_length_,
    //                         clearance_map_y_width_, 
    //                         resolution_,
    //                         image_point))
    if(transformMapToImage(path_points[i].pose.position, 
                           map_info,
                           image_point))
    {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x]; 
      if(clearance*resolution_>=min_radius_-loosing_clerance_for_explore_goal_threshold_)
      {
        exploring_goal_pose_in_map_ptr = 
          std::make_unique<geometry_msgs::Pose>(path_points[i].pose);
        if(accum_dist > max_radius_*15)
        {
          break;
        }
      }
    }
  }
  
  if(!exploring_goal_pose_in_map_ptr && previous_exploring_goal_pose_in_map_ptr_)
  {
    exploring_goal_pose_in_map_ptr = 
      std::make_unique<geometry_msgs::Pose>(*previous_exploring_goal_pose_in_map_ptr_);
  }
  else if(!exploring_goal_pose_in_map_ptr)
  {
    ROS_WARN("[EBPathPlanner] Explore from baselink positon."); 
    exploring_goal_pose_in_map_ptr = 
      std::make_unique<geometry_msgs::Pose>(ego_pose);
  }
  
  //for last stopping point 
  if(path_points.back().twist.linear.x < 1e-4)
  {
    double dx = exploring_goal_pose_in_map_ptr->position.x - 
                path_points.back().pose.position.x;
    double dy = exploring_goal_pose_in_map_ptr->position.y - 
                path_points.back().pose.position.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_radius_)
    {
      exploring_goal_pose_in_map_ptr = 
        std::make_unique<geometry_msgs::Pose>(path_points.back().pose);
    }
  }
  debug_goal_point_in_map = exploring_goal_pose_in_map_ptr->position;
  
  solveGraphAStar(ego_pose, 
                  start_exploring_pose.position, 
                  exploring_goal_pose_in_map_ptr->position, 
                  clearance_map, 
                  map_info,
                  explored_points);
  
  // arrangeExploredPointsBaseedOnClearance(clearance_map,
  //                                        ego_pose,
  //                                        explored_points,
  //                                        debug_rearrange_points);
  
  previous_exploring_goal_pose_in_map_ptr_ = 
    std::make_unique<geometry_msgs::Pose>(*exploring_goal_pose_in_map_ptr);
  // std::chrono::high_resolution_clock::time_point end2= 
  //   std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point end2= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time2 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  std::cout << " explore goal + decision logic + explore path "<< time2.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  // cv::imwrite("/home/kosuke/aaa.jpg", clearance_map);	
  return true;
}