
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>

#include <ros/console.h>
#include <tf2/utils.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

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

class GridNode
{
public:
  GridNode();
  ~GridNode();
  
  Eigen::Vector2i p;
  double r;
  double g;
  double h;
  double f;
  std::shared_ptr<GridNode> parent_node;
};
GridNode::GridNode(){}
GridNode::~GridNode(){}

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
  double min_radius,
  double backward_distance):
is_fix_pose_mode_for_debug_(false),
is_debug_each_iteration_mode_(false),
is_debug_driveable_area_mode_(false),
is_debug_clearance_map_mode_(false),
clearance_map_y_width_(100),
clearance_map_x_length_(100),
resolution_(0.1),
time_limit_millisecond_(200.0),
min_radius_(min_radius),
max_radius_(min_radius),
backward_distance_(backward_distance),
static_objects_velocity_ms_threshold_(1.1),
loosing_clerance_for_explore_goal_threshold_(0.2),
heuristic_epsilon_(1)
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
  // double delta_theta = 2*M_PI/96.0;
  double delta_theta = 2*M_PI/108.0;
  for(double theta = 0; theta < 2*M_PI; theta += delta_theta)
  {
    Eigen::Matrix2d rotation;
    rotation << std::cos(theta), - std::sin(theta),
                std::sin(theta),   std::cos(theta);
    Eigen::Vector2d rotated_delta = rotation * delta_child_p;
    Node child_node;
    child_node.p = rotated_delta + parent_node.p;
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
  if(transformMapToImage(start_point_in_map,
                         map_info,
                         start_point_in_image)&&
     transformMapToImage(goal_point_in_map, 
                         map_info,
                         goal_point_in_image))
  {
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
    std::priority_queue<
      Node, 
      std::vector<Node>, 
      std::function<bool(Node, Node)>>
      s_open([](Node a, Node b){return a.f > b.f;});
    s_open.push(initial_node);
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
    bool is_explore_success = false;
    while(!s_open.empty())
    {
      Node lowest_f_node = s_open.top();s_open.pop();
      std::chrono::high_resolution_clock::time_point current_explore= 
        std::chrono::high_resolution_clock::now();
      double current_accum_explroe_time = 
        std::chrono::duration_cast<std::chrono::nanoseconds>(current_explore - begin_explore).count()/(1000.0*1000.0);
      if(current_accum_explroe_time > time_limit_millisecond_ && !is_debug_each_iteration_mode_)
      {
        ROS_WARN("[EBPathPlanner] Exceed time limit of %lf [ms], Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
          time_limit_millisecond_, ego_pose.position.x, ego_pose.position.y, ego_pose.position.z, 
          ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);
        return false;
      }
      if(f_goal < lowest_f_node.f)
      {
        break;
      }
      else if(nodeExistInClosedNodes(lowest_f_node, s_closed))
      {
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
                    map_info,
                    goal_node,
                    min_radius_,
                    max_radius_,
                    child_nodes);
        s_closed.push_back(lowest_f_node);
        for(auto& node: child_nodes)
        {
          s_open.push(node);
        }
      }
    }
    if(!is_explore_success)
    {
      ROS_WARN_THROTTLE(3.0, "[EBPathPlanner] graph a star could not find path");
      ROS_INFO("[EBPathPlanner] Start point: %lf %lf %lf ", 
        start_point_in_map.x, 
        start_point_in_map.y, 
        start_point_in_map.z);
      ROS_INFO("[EBPathPlanner] A star faild. Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
        ego_pose.position.x, ego_pose.position.y, ego_pose.position.z, 
        ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);
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

// bool ModifyReferencePath::expandGridNode(
//   const GridNode& parent_node, 
//   const cv::Mat& clearance_map,
//   const nav_msgs::MapMetaData& map_info,
//   const GridNode& goal_node,
//   cv::Mat& visited_map,
//   std::vector<GridNode>& child_nodes)
// {
//   for(int delta_x_image  = -1; delta_x_image <= 1; delta_x_image++)
//   {
//     for(int delta_y_image = -1; delta_y_image <= 1; delta_y_image++)
//     {
//       int pix_x = parent_node.p(0)+delta_x_image;
//       int pix_y = parent_node.p(1)+delta_y_image;
//       int max_x_image = map_info.height;
//       int max_y_image = map_info.width;
//       if(pix_x >= 0 && 
//          pix_x <  max_x_image && 
//          pix_y >= 0 && 
//          pix_y < max_y_image)
//       {
//         float clearance = clearance_map.ptr<float>
//                             ((int)pix_y)
//                             [(int)pix_x];
//         int is_visited = visited_map.ptr<unsigned char>
//                             ((int)pix_y)
//                             [(int)pix_x];
//         if(clearance >= 14 && is_visited == 0)
//         {
//           GridNode child;
//           child.p = Eigen::Vector2i(pix_x, pix_y);
//           int abs_delta_x = std::abs(pix_x - goal_node.p(0));
//           int abs_delta_y = std::abs(pix_y - goal_node.p(1));
//           // std::cout << "goal xy "<< goal_node.p(0)<< " "<< goal_node.p(1) << std::endl;
//           // std::cout << "pix xy "<< pix_x<< " "<< pix_y << std::endl;
//           child.h = std::sqrt(abs_delta_x*abs_delta_x+abs_delta_y*abs_delta_y);
//           // std::cout << "h "<< child.h<< std::endl;
//           if(delta_x_image == 0 || delta_y_image == 0)
//           {
//             child.g = parent_node.g +  1;
//           }
//           else
//           {
//             child.g = parent_node.g + std::sqrt(2);
//             // child.g = parent_node.g + 10;
//           }
//           child.f = child.h + child.g;
//           child.parent_node = std::make_shared<GridNode>(parent_node);
          
//           child_nodes.push_back(child);
//           visited_map.ptr<unsigned char>
//                             (pix_y)
//                             [pix_x] = 255;
//         }
//       }
//     }
//   }
//   return true;
// }

// bool ModifyReferencePath::solveAStar(
//         const geometry_msgs::Point& start_point_in_map,
//         const geometry_msgs::Point& goal_point_in_map,
//         const cv::Mat& clearance_map,
//         const nav_msgs::MapMetaData& map_info,
//         std::vector<geometry_msgs::Point>& explored_points)
// {
  
//   std::chrono::high_resolution_clock::time_point begin3= 
//     std::chrono::high_resolution_clock::now();
  
//   geometry_msgs::Point start_point_in_image;
//   geometry_msgs::Point goal_point_in_image;
//   if(transformMapToImage(
//        start_point_in_map,
//        map_info,
//        start_point_in_image) && 
//      transformMapToImage(
//        goal_point_in_map,
//        map_info,
//        goal_point_in_image))
//   {
//     std::priority_queue<
//       GridNode, 
//       std::vector<GridNode>, 
//       std::function<bool(GridNode, GridNode)>>
//       s_open([](GridNode a, GridNode b){return a.f > b.f;});
    
//     // std::vector<GridNode> s_closed;
//     // cv::Mat visited_map = cv::Mat::zeros(clearance_map.rows, clearance_map.cols, CV_8UC1);
    
    
//     GridNode initial_node;
//     initial_node.g = 0;
//     Eigen::Vector2i initial_p((int)start_point_in_image.x, (int)start_point_in_image.y);
//     Eigen::Vector2i goal_p((int)goal_point_in_image.x, (int)goal_point_in_image.y);
    
//     initial_node.p = initial_p;
//     // int abs_delta_x = std::abs(goal_point_in_image.x-start_point_in_image.x);
//     // int abs_delta_y = std::abs(goal_point_in_image.y-start_point_in_image.y);
//     int abs_delta_x = std::abs(start_point_in_image.x-goal_point_in_image.x);
//     int abs_delta_y = std::abs(start_point_in_image.y-goal_point_in_image.y);
//     initial_node.h = std::sqrt(abs_delta_x*abs_delta_x+abs_delta_y*abs_delta_y);
//     initial_node.f = initial_node.g + initial_node.h;
//     initial_node.parent_node = nullptr;
//     s_open.push(initial_node);
    
//     GridNode goal_node;
//     goal_node.p = goal_p;
//     goal_node.g = 0;
//     goal_node.h = 0;
//     goal_node.f = 0;  
//     GridNode lowest_f_node;
//     s_open.push(lowest_f_node);
//     cv::Mat g_score_map = cv::Mat::ones(clearance_map.rows, clearance_map.cols, CV_32FC1)*100000000;
//     cv::Mat f_score_map = cv::Mat::ones(clearance_map.rows, clearance_map.cols, CV_32FC1)*100000000;
//     cv::Mat openset_map = 
//         cv::Mat::zeros(clearance_map.rows, clearance_map.cols, CV_8UC1);
//     openset_map.ptr<int>((int)initial_p(1))
//                           [(int)initial_p(0)] = 255;
//     g_score_map.ptr<float>((int)initial_p(1))
//                           [(int)initial_p(0)] = 0;
//     f_score_map.ptr<float>((int)initial_p(1))
//                           [(int)initial_p(0)] = initial_node.h;
//     bool is_explore_success = false;
//     std::chrono::high_resolution_clock::time_point begin_explore= 
//       std::chrono::high_resolution_clock::now();
//     GridNode goal_grid_node;
//     while(!s_open.empty())
//     {
//       lowest_f_node = s_open.top(); s_open.pop();
//       openset_map.ptr<int>((int)lowest_f_node.p(1))
//                           [(int)lowest_f_node.p(0)] = 0;
//       // cv::Mat tmp;
//       // openset_map.copyTo(tmp);
//       // cv::circle(tmp, 
//       //         cv::Point(lowest_f_node.p(0), lowest_f_node.p(1)), 
//       //         10, 
//       //         cv::Scalar(255),
//       //         1);
//       // cv::circle(tmp, 
//       //         cv::Point(goal_node.p(0), goal_node.p(1)), 
//       //         1, 
//       //         cv::Scalar(255),
//       //         1);
//       // cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
//       // cv::imshow("image", tmp);
//       // cv::waitKey(10);
      
//       std::chrono::high_resolution_clock::time_point current_explore= 
//         std::chrono::high_resolution_clock::now();
//       double current_accum_explroe_time = 
//         std::chrono::duration_cast<std::chrono::nanoseconds>
//         (current_explore - begin_explore).count()/(1000.0*1000.0);
//       if(current_accum_explroe_time > time_limit_millisecond_ && !is_debug_each_iteration_mode_)
//       {
//         ROS_WARN("[EBPathPlanner] Exceed time limit of %lf [ms] ", time_limit_millisecond_);
//         return false;
//       }
//       else
//       {
//         if(lowest_f_node.h == 0)
//         {
//           std::cout << "reach goal!!!!" << std::endl;
//           goal_grid_node  = lowest_f_node;
//           is_explore_success = true;
//           break;
//         }
//         for(int delta_x_image  = -1; delta_x_image <= 1; delta_x_image++)
//         {
//           for(int delta_y_image = -1; delta_y_image <= 1; delta_y_image++)
//           {
//             int pix_x = lowest_f_node.p(0)+delta_x_image;
//             int pix_y = lowest_f_node.p(1)+delta_y_image;
//             int max_x_image = map_info.height;
//             int max_y_image = map_info.width;
//             std::cout << "pic xy "<< pix_x << " "<< pix_y << std::endl;
//             if(pix_x >= 0 && 
//               pix_x <  max_x_image && 
//               pix_y >= 0 && 
//               pix_y < max_y_image)
//             {
//               float clearance = clearance_map.ptr<float>
//                                   ((int)pix_y)
//                                   [(int)pix_x];
                   
//               if(clearance >= 10)
//               {
//                 // GridNode child;
//                 // child.p = Eigen::Vector2i(pix_x, pix_y);
//                 double tmp_g; 
//                 if(delta_x_image == 0 || delta_y_image == 0)
//                 {
//                   tmp_g = lowest_f_node.g +  1;
//                 }
//                 else
//                 {
//                   // continue;
//                   tmp_g = lowest_f_node.g + std::sqrt(2);
//                   tmp_g = lowest_f_node.g + 1;
//                 }
//                 float current_g_score = g_score_map.ptr<float>
//                                              ((int)pix_y)
//                                              [(int)pix_x];
//                 if(tmp_g < current_g_score)
//                 {
//                   g_score_map.ptr<float>
//                               ((int)pix_y)
//                                 [(int)pix_x] = tmp_g;
//                   float abs_delta_x = std::abs(pix_x - goal_node.p(0));
//                   float abs_delta_y = std::abs(pix_y - goal_node.p(1));
//                   float huristics = std::sqrt(abs_delta_x*abs_delta_x+abs_delta_y*abs_delta_y);
//                   f_score_map.ptr<float>
//                               ((int)pix_y)
//                               [(int)pix_x] = tmp_g + huristics;
//                   if(openset_map.ptr<int>((int)pix_y)[(int)pix_x]== 0)
//                   {
//                     openset_map.ptr<int>((int)pix_y)[(int)pix_x] = 255;
//                     GridNode child;
//                     child.g = tmp_g;
//                     child.h = huristics;
//                     child.p = Eigen::Vector2i(pix_x, pix_y);
//                     child.f = tmp_g + huristics;
//                     child.parent_node = std::make_shared<GridNode>(lowest_f_node);
//                     s_open.push(child);
//                   }
//                 }
//               }
//             }
//           }
//         }
//       }
//     }
//     if(!is_explore_success)
//     {
//       ROS_WARN( "[EBPathPlanner] graph a star could not find path");
//       return false;
//     }
//     //backtrack
//     GridNode current_node = goal_grid_node;
//     if(current_node.parent_node == nullptr)
//     {
//       return false;
//     }
//     std::vector<geometry_msgs::Point> explored_image_points;
//     while(current_node.parent_node != nullptr)
//     {
//       geometry_msgs::Point explored_point;
//       explored_point.x = current_node.p(0);
//       explored_point.y = current_node.p(1);
//       explored_point.z = 0;
//       explored_image_points.insert(explored_image_points.begin(), explored_point);
//       current_node = *current_node.parent_node;
//     }
//     for(const auto& point: explored_image_points)
//     {
//       geometry_msgs::Point map_point;
//       transformImageToMap(point,
//                           map_info,
//                           map_point);
//       explored_points.push_back(map_point);
//     }
//     return true;
//   }
// }



bool ModifyReferencePath::generateModifiedPath(
  geometry_msgs::Pose& ego_pose,
  const geometry_msgs::Pose& start_exploring_pose,
  const std::vector<autoware_planning_msgs::PathPoint>& path_points,
  const std::vector<autoware_perception_msgs::DynamicObject>& objects,
  std::vector<geometry_msgs::Point>& explored_points,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  geometry_msgs::Point& debug_goal_point_in_map,
  std::vector<geometry_msgs::Point>& debug_rearrange_points)
{
  
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
        if(accum_dist > max_radius_*12)
        {
          break;
        }
      }
    }
    else
    {
      break;
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
  
  // std::cout << "goal "<< exploring_goal_pose_in_map_ptr->position.x << " "<< exploring_goal_pose_in_map_ptr->position.y << std::endl;
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
  
  bool is_explore_success = solveGraphAStar(ego_pose, 
                          start_exploring_pose.position, 
                          exploring_goal_pose_in_map_ptr->position, 
                          clearance_map, 
                          map_info,
                          explored_points);
  if(!is_explore_success)
  {
    return is_explore_success;
  }                
  // solveAStar(start_exploring_pose.position, 
  //            exploring_goal_pose_in_map_ptr->position, 
  //            clearance_map, 
  //            map_info,
  //            explored_points);
  
  //experimental: swap last explored points with actual explored goal
  if(!explored_points.empty())
  {
    explored_points.erase(explored_points.end());
    explored_points.push_back(exploring_goal_pose_in_map_ptr->position);
  }
  
  previous_exploring_goal_pose_in_map_ptr_ = 
    std::make_unique<geometry_msgs::Pose>(*exploring_goal_pose_in_map_ptr);
  std::chrono::high_resolution_clock::time_point end2= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time2 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  std::cout << " explore goal + decision logic + explore path "<< time2.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  return true;
}