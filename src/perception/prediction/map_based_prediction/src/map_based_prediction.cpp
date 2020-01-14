#include <chrono>

#include <unique_id/unique_id.h>

#include <tf2/utils.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>

#include "cubic_spline.hpp"

#include "map_based_prediction.h"

MapBasedPrediction::MapBasedPrediction(
  double interpolating_resolution,
  double time_horizon,
  double sampling_delta_time
):
interpolating_resolution_(interpolating_resolution),
time_horizon_(time_horizon),
sampling_delta_time_(sampling_delta_time)
{
}


double calculateEuclideanDistance(
  const geometry_msgs::Point& point1,
  const geometry_msgs::Point& point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double distance = std::sqrt(dx*dx+dy*dy);
  return distance;
}


bool getNearestPoint(
  const std::vector<geometry_msgs::Point>& points, 
  const geometry_msgs::Point point,
  geometry_msgs::Point& nearest_point)
{
  double min_dist = 1e+10;
  bool flag = false;
  for(const auto& tmp_point : points)
  {
    double distance = calculateEuclideanDistance(point, tmp_point);
    if(distance < min_dist)
    {
      min_dist = distance;
      nearest_point = point;
      flag = true;
    }
  }
  return flag;
}

bool getNearestPointIdx(
  const std::vector<geometry_msgs::Point>& points, 
  const geometry_msgs::Point point,
  geometry_msgs::Point& nearest_point,
  size_t& nearest_index)
{
  double min_dist = 10000000;
  bool flag = false;
  size_t index = 0;
  for(const auto& tmp_point : points)
  {
    double distance = calculateEuclideanDistance(point, tmp_point);
    if(distance < min_dist)
    {
      min_dist = distance;
      nearest_point = tmp_point;
      flag = true;
      nearest_index = index;
    }
    index ++;
  }
  return flag;
}

bool MapBasedPrediction::doPrediction(
  const DynamicObjectWithLanesArray& in_objects, 
  std::vector<autoware_perception_msgs::DynamicObject>& out_objects,
  std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin= 
  std::chrono::high_resolution_clock::now();
  for (auto& object_with_lanes: in_objects.objects)
  {
    if(object_with_lanes.object.semantic.type == autoware_perception_msgs::Semantic::CAR||
       object_with_lanes.object.semantic.type == autoware_perception_msgs::Semantic::BUS||
       object_with_lanes.object.semantic.type == autoware_perception_msgs::Semantic::TRUCK)
    {
      const double abs_velo = 
          std::sqrt(std::pow(object_with_lanes.object.state.twist_covariance.twist.linear.x, 2)+
                    std::pow(object_with_lanes.object.state.twist_covariance.twist.linear.y,2));
      double minimum_velocity_threshold = 0.5;
      if(abs_velo < minimum_velocity_threshold)
      {
        out_objects.push_back(object_with_lanes.object);
        continue;
      }
      
      autoware_perception_msgs::DynamicObject tmp_object;
      tmp_object = object_with_lanes.object;
      for(const auto& path: object_with_lanes.lanes)
      {
        
        std::vector<double> tmp_x;
        std::vector<double> tmp_y;
        std::vector<geometry_msgs::Pose> geometry_points = path;
        for(size_t i = 0; i< path.size(); i++)
        {
          if(i>0)
          {
            double dist = calculateEuclideanDistance(
              geometry_points[i].position,
              geometry_points[i-1].position);
            if(dist < interpolating_resolution_)
            {
              continue;
            }
          }
          tmp_x.push_back(geometry_points[i].position.x);
          tmp_y.push_back(geometry_points[i].position.y);
        }
        
        Spline2D spline2d(tmp_x, tmp_y);
        std::vector<geometry_msgs::Point> interpolated_points;
        std::vector<double> interpolated_yaws;
        for(float s=0.0; s<spline2d.s.back(); s+=interpolating_resolution_)
        {
            std::array<double, 2> point1 = spline2d.calc_position(s);
            geometry_msgs::Point g_point;
            g_point.x = point1[0];
            g_point.y = point1[1];
            g_point.z = object_with_lanes.object.state.pose_covariance.pose.position.z;
            interpolated_points.push_back(g_point);
            interpolated_yaws.push_back(spline2d.calc_yaw(s));
        }
        debug_interpolated_points = interpolated_points;
        
        geometry_msgs::Point object_point = 
        object_with_lanes.object.state.pose_covariance.pose.position;
        geometry_msgs::Point nearest_point;
        size_t nearest_point_idx;
        if(getNearestPointIdx(interpolated_points, object_point, nearest_point, nearest_point_idx))
        {
          // calculate initial position in Frenet coordinate
          // Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
          // Path Planning for Highly Automated Driving on Embedded GPUs
          double current_s_position = interpolating_resolution_ 
                                    * static_cast<double>(nearest_point_idx);
          double current_d_position = calculateEuclideanDistance(nearest_point, object_point);
          
          double lane_yaw = spline2d.calc_yaw(current_s_position);
          std::vector<double> origin_v = {std::cos(lane_yaw), 
                                          std::sin(lane_yaw)};
          std::vector<double> object_v = {object_point.x-nearest_point.x, 
                                          object_point.y-nearest_point.y};
          double cross2d = object_v[0]*origin_v[1] - object_v[1]*origin_v[0]; 
          if(cross2d<0)
          {
            current_d_position*= -1;
          }
          
          double current_d_velocity = 0;
          double current_s_velocity = object_with_lanes.object.state.twist_covariance.twist.linear.x;
          double target_s_position = std::min(spline2d.s.back(), current_s_position+10);
          autoware_perception_msgs::PredictedPath path;

          // for ego point
          geometry_msgs::PoseWithCovarianceStamped point;
          point.pose.pose.position = object_point;
          getPredictedPath(object_point.z,
                          current_d_position,
                          current_d_velocity,
                          current_s_position,
                          current_s_velocity,
                          target_s_position,
                          in_objects.header,
                          spline2d,
                          path);
          tmp_object.state.predicted_paths.push_back(path);
        }
        else
        {
          // std::cerr << "could not find nearest point"  << std::endl;
          continue;
        }
      }
      normalizeLikelyhood(tmp_object.state.predicted_paths);
      out_objects.push_back(tmp_object);
    }
    else
    {
      geometry_msgs::Pose object_pose = object_with_lanes.object.state.pose_covariance.pose;
      double object_linear_velocity = object_with_lanes.object.state.twist_covariance.twist.linear.x;
      autoware_perception_msgs::PredictedPath path;
      getLinearPredictedPath(object_pose, 
                             object_linear_velocity,
                             in_objects.header,
                             path);
      autoware_perception_msgs::DynamicObject tmp_object;
      tmp_object = object_with_lanes.object;
      tmp_object.state.predicted_paths.push_back(path);
      out_objects.push_back(tmp_object);
    }
  }
  // 3. 現在日時を再度取得
  std::chrono::high_resolution_clock::time_point end = 
  std::chrono::high_resolution_clock::now();
  // 経過時間を取得
  std::chrono::nanoseconds time = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  // std::cerr <<"prediction time " <<time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  return true;
}

bool MapBasedPrediction::doLinearPrediction(
  const autoware_perception_msgs::DynamicObjectArray& in_objects, 
  std::vector<autoware_perception_msgs::DynamicObject>& out_objects)
{
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin= 
  std::chrono::high_resolution_clock::now();

  for(const auto object: in_objects.objects)
  {
    geometry_msgs::Pose object_pose = object.state.pose_covariance.pose;
    double object_linear_velocity = object.state.twist_covariance.twist.linear.x;
    autoware_perception_msgs::PredictedPath path;
    getLinearPredictedPath(object_pose, 
                            object_linear_velocity,
                            in_objects.header,
                            path);
    autoware_perception_msgs::DynamicObject tmp_object;
    tmp_object = object;
    tmp_object.state.predicted_paths.push_back(path);
    out_objects.push_back(tmp_object);
  }
     

  // 3. 現在日時を再度取得
  std::chrono::high_resolution_clock::time_point end = 
  std::chrono::high_resolution_clock::now();
  // 経過時間を取得
  std::chrono::nanoseconds time = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  // std::cerr <<"prediction time " <<time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  return true;
}
                  

bool MapBasedPrediction::normalizeLikelyhood(std::vector<autoware_perception_msgs::PredictedPath>& paths)
{
  // TODO: is could not be the smartest way
  double sum_confidence = 0;
  for (const auto& path: paths)
  {
    sum_confidence += 1/path.confidence;
  }
  
  for (auto& path: paths)
  {
    path.confidence =  (1/path.confidence)/sum_confidence;
  }
}

bool MapBasedPrediction::getPredictedPath(
  const double height,
  const double current_d_position,
  const double current_d_velocity,
  const double current_s_position,
  const double current_s_velocity,
  const double target_s_position,
  const std_msgs::Header& origin_header,
  Spline2D& spline2d,
  autoware_perception_msgs::PredictedPath& path)
{
  //Quintic polynominal for d
  // A = np.array([[T**3, T**4, T**5],
  //               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
  //               [6 * T, 12 * T ** 2, 20 * T ** 3]])
  // A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
  //                    [-15/(T**4), 7/(T**3), -1/(T**2)],
  //                    [6/(T**5), -3/(T**4),  1/(2*T**3)]])
  // b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],
  //                [vxe - self.a1 - 2 * self.a2 * T],
  //                [axe - 2 * self.a2]])
  double target_d_position = 0;
  
  double t = time_horizon_;
  Eigen::Matrix3d a_3_inv;
  a_3_inv << 10/std::pow(t,3), -4/std::pow(t,2), 1/(2*t),
          -15/std::pow(t,4), 7/std::pow(t,3), -1/std::pow(t,2),
           6/std::pow(t,5), -3/std::pow(t,4), 1/(2*std::pow(t,3));
           
  double target_d_velocity = current_d_velocity;
  double target_d_accerelation = 0;
  Eigen::Vector3d b_3;
  b_3 << target_d_position - current_d_position - current_d_velocity * t,
         target_d_velocity - current_d_velocity,
         target_d_accerelation;
         
  Eigen::Vector3d x_3;
  x_3 = a_3_inv * b_3;
  
         
  // Quatric polynominal
  // A_inv = np.matrix([[1/(T**2), -1/(3*T)],
  //                         [-1/(2*T**3), 1/(4*T**2)]])
  // b = np.matrix([[vxe - self.a1 - 2 * self.a2 * T],
  //               [axe - 2 * self.a2]])
  Eigen::Matrix2d a_2_inv;
  a_2_inv << 1/std::pow(t,2), -1/(3*t),
             -1/(2*std::pow(t,3)), 1/(4*std::pow(t,2));
  double target_s_velocity = current_s_velocity;
  Eigen::Vector2d b_2;
  b_2 << target_s_velocity - current_s_velocity,
         0;
  Eigen::Vector2d x_2;
  x_2 = a_2_inv*b_2;
  
  // sampling points from calculated path
  double dt = sampling_delta_time_;
  std::vector<double> d_vec;
  double calculated_d, calculated_s;
  for(double i = 0; i < t; i+=dt)
  {
    calculated_d = current_d_position + 
                          current_d_velocity * i +
                          0*2*i*i + 
                          x_3(0) * i*i*i+
                          x_3(1)*i*i*i*i+
                          x_3(2)*i*i*i*i*i;
    calculated_s = current_s_position + 
                          current_s_velocity*i + 
                          2*0*i*i +
                          x_2(0) * i*i*i +
                          x_2(1) *i*i*i*i;
    
    geometry_msgs::PoseWithCovarianceStamped tmp_point;
    if(calculated_s> spline2d.s.back())
    {
      break;
    }
    std::array<double, 2> p = spline2d.calc_position(calculated_s);
    double yaw = spline2d.calc_yaw(calculated_s);
    tmp_point.pose.pose.position.x = p[0]+ std::cos(yaw-M_PI/2.0)*calculated_d;
    tmp_point.pose.pose.position.y = p[1]+ std::sin(yaw-M_PI/2.0)*calculated_d;
    tmp_point.pose.pose.position.z = height;
    tmp_point.header = origin_header;
    tmp_point.header.stamp = origin_header.stamp + ros::Duration(i);
    path.path.push_back(tmp_point);
  }
  path.confidence = calculateLikelyhood(current_d_position);
  
  return false;
}

bool MapBasedPrediction::getLinearPredictedPath(
  const geometry_msgs::Pose& object_pose,
  const double linear_velocity,
  const std_msgs::Header& origin_header,
  autoware_perception_msgs::PredictedPath& path)
{
  double yaw = tf2::getYaw(object_pose.orientation);
  double dt = sampling_delta_time_;
  double time_horizon = time_horizon_;
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose  = object_pose;
  for(double i = 0; i < time_horizon; i+=dt)
  {
    double next_x = pose.pose.pose.position.x + std::cos(yaw)*linear_velocity*dt;
    double next_y = pose.pose.pose.position.y + std::sin(yaw)*linear_velocity*dt;
    pose.pose.pose.position.x = next_x;
    pose.pose.pose.position.y = next_y;
    pose.header = origin_header;
    pose.header.stamp = origin_header.stamp + ros::Duration(i);
    path.path.push_back(pose);
  }
  path.confidence = 1.0;
}

double MapBasedPrediction::calculateLikelyhood(const double current_d)
{
  double d_std = 0.5;
  double likelyhood = std::abs(current_d)/d_std;
  return likelyhood;
}
