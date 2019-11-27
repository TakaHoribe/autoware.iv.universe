/*
 *  Copyright (c) 2019, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include "pointcloud_preprocessor/ground_filter/ray_ground_filter_nodelet.h"


namespace pointcloud_preprocessor
{
  RayGroundFilterNodelet::RayGroundFilterNodelet()
  {
    grid_width_ = 1000;
    grid_height_ = 1000;
    grid_precision_ = 0.2;
    ray_ground_filter::generateColors(colors_, color_num_);
  }
  void RayGroundFilterNodelet::ConvertXYZIToRTZColor(const pcl::PointCloud<PointType_>::Ptr in_cloud,
                                                     PointCloudXYZIRTColor &out_organized_points,
                                                     std::vector <pcl::PointIndices> &out_radial_divided_indices,
                                                     std::vector <PointCloudXYZIRTColor> &out_radial_ordered_clouds)
  {
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
      PointXYZIRTColor new_point;
      auto radius = (float) sqrt(
        in_cloud->points[i].x * in_cloud->points[i].x
        + in_cloud->points[i].y * in_cloud->points[i].y
      );
      auto theta = (float) atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
      if (theta < 0)
      {
        theta += 360;
      }

      auto radial_div = (size_t) floor(theta / radial_divider_angle_);
      auto concentric_div = (size_t) floor(fabs(radius / concentric_divider_distance_));

      new_point.point.x = in_cloud->points[i].x;
      new_point.point.y = in_cloud->points[i].y;
      new_point.point.z = in_cloud->points[i].z;
      //new_point.ring = in_cloud->points[i].ring;
      new_point.radius = radius;
      new_point.theta = theta;
      new_point.radial_div = radial_div;
      new_point.concentric_div = concentric_div;
      new_point.red = (size_t) colors_[new_point.radial_div % color_num_].val[0];
      new_point.green = (size_t) colors_[new_point.radial_div % color_num_].val[1];
      new_point.blue = (size_t) colors_[new_point.radial_div % color_num_].val[2];
      new_point.original_index = i;

      out_organized_points[i] = new_point;

      //radial divisions
      out_radial_divided_indices[radial_div].indices.push_back(i);

      out_radial_ordered_clouds[radial_div].push_back(new_point);

    }//end for

    //order radial points on each division
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
      std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                [](const PointXYZIRTColor &a, const PointXYZIRTColor &b)
                {
                  return a.radius < b.radius;
                });
    }
  }


  void RayGroundFilterNodelet::ClassifyPointCloud(std::vector <PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                                                  pcl::PointIndices &out_ground_indices,
                                                  pcl::PointIndices &out_no_ground_indices)
  {
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++)//sweep through each radial division
    {
      float prev_radius = 0.f;
      float prev_height = -sensor_height_;
      bool prev_ground = false;
      bool current_ground = false;
      for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++)//loop through each point in the radial div
      {
        float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
        float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
        float current_height = in_radial_ordered_clouds[i][j].point.z;
        float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

        //for points which are very close causing the height threshold to be tiny, set a minimum value
        if (height_threshold < min_height_threshold_)
        {
          height_threshold = min_height_threshold_;
        }
        //only check points which radius is larger than the concentric_divider
        if (points_distance < concentric_divider_distance_)
        {
          current_ground = prev_ground;
        } else
        {
          //check current point height against the LOCAL threshold (previous point)
          if (current_height <= (prev_height + height_threshold)
              && current_height >= (prev_height - height_threshold)
            )
          {
            //Check again using general geometry (radius from origin) if previous points wasn't ground
            if (!prev_ground)
            {
              if (current_height <= (-sensor_height_ + general_height_threshold)
                  && current_height >= (-sensor_height_ - general_height_threshold))
              {
                current_ground = true;
              } else
              {
                current_ground = false;
              }
            } else
            {
              current_ground = true;
            }
          } else
          {
            //check if previous point is too far from previous one, if so classify again
            if (points_distance > reclass_distance_threshold_ &&
                (current_height <= (-sensor_height_ + height_threshold)
                 && current_height >= (-sensor_height_ - height_threshold))
              )
            {
              current_ground = true;
            } else
            {
              current_ground = false;
            }
          }
        }//end larger than concentric_divider

        if (current_ground)
        {
          out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
          prev_ground = true;
        } else
        {
          out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
          prev_ground = false;
        }

        prev_radius = in_radial_ordered_clouds[i][j].radius;
        prev_height = in_radial_ordered_clouds[i][j].point.z;
      }
    }
  }


  void RayGroundFilterNodelet::ClipCloud(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr,
                                  double in_clip_height,
                                  pcl::PointCloud<PointType_>::Ptr out_clipped_cloud_ptr)
  {
    pcl::ExtractIndices <PointType_> extractor;
    extractor.setInputCloud(in_cloud_ptr);
    pcl::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      if (in_cloud_ptr->points[i].z > in_clip_height)
      {
        indices.indices.push_back(i);
      }
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true);//true removes the indices, false leaves only the indices
    extractor.filter(*out_clipped_cloud_ptr);
  }

  void RayGroundFilterNodelet::RemovePointsUpTo(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr,
                                         double in_min_distance,
                                         pcl::PointCloud<PointType_>::Ptr out_filtered_cloud_ptr)
  {
    pcl::ExtractIndices <PointType_> extractor;
    extractor.setInputCloud(in_cloud_ptr);
    pcl::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      if (sqrt(in_cloud_ptr->points[i].x * in_cloud_ptr->points[i].x +
               in_cloud_ptr->points[i].y * in_cloud_ptr->points[i].y)
          < in_min_distance)
      {
        indices.indices.push_back(i);
      }
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true);//true removes the indices, false leaves only the indices
    extractor.filter(*out_filtered_cloud_ptr);
  }

  pcl::PointCloud<RayGroundFilterNodelet::PointType_>::Ptr
  RayGroundFilterNodelet::MatchClouds(const pcl::PointCloud<PointType_>::Ptr &in_previous_scan,
                               pcl::PointCloud<PointType_>::Ptr in_current_scan)
  {
    if (in_previous_scan->points.size() > min_icp_points_
    && in_current_scan->points.size() > min_icp_points_)
    {

      pcl::PointCloud<PointType_>::Ptr matched_cloud(new pcl::PointCloud<PointType_>);
      pcl::IterativeClosestPoint<PointType_, PointType_> icp;
      icp.setInputSource(in_previous_scan);
      icp.setInputTarget(in_current_scan);

      icp.align(*matched_cloud);

      if (icp.hasConverged())
      {
        return matched_cloud;
      }
    }
    return in_current_scan;
  }


  template<typename T>
  cv::Point2i
  RayGroundFilterNodelet::ProjectPointToGrid(const T &in_point, size_t in_grid_w, size_t in_grid_h, double in_precision)
  {
    size_t u, v;
    try
    {
      u = -in_point.x / in_precision + in_grid_w / 2;
      v = -in_point.y / in_precision + in_grid_h / 2;
    }
    catch (std::exception &e)
    {
      ROS_ERROR("[Ground Filter] precision set to 0. %s", e.what());
      throw;
    }
    return cv::Point2i(v, u);
  }

  template<typename T>
  cv::Mat
  RayGroundFilterNodelet::CreateOccupancyFromCloud(const T &in_cloud_ptr)
  {
    cv::Mat grid(grid_height_, grid_width_, CV_8UC1, cv::Scalar(0));
    for (const auto &point: in_cloud_ptr->points)
    {
      cv::Point2i pixel = ProjectPointToGrid(point, grid_width_, grid_height_, grid_precision_);
      if (pixel.x >= 0 && pixel.x < grid_width_
          && pixel.y >= 0 && pixel.y < grid_height_)
      {
        cv::circle(grid, pixel, 2, cv::Scalar(1), CV_FILLED);
      }
    }
    return grid;
  }

  pcl::PointCloud<RayGroundFilterNodelet::PointType_>::Ptr
  RayGroundFilterNodelet::ValidatePointCloudWithOccupancyGrid(const pcl::PointCloud<PointType_>::Ptr &in_cloud,
                                                       cv::Mat in_out_grid, size_t in_threshold)
  {
    pcl::PointCloud<PointType_>::Ptr validated_cloud_ptr(new pcl::PointCloud <PointType_>);

    for (const auto &point: in_cloud->points)
    {
      cv::Point2i pixel = ProjectPointToGrid(point, grid_width_, grid_height_, grid_precision_);
      if (pixel.x >= 0 && pixel.x < (int)grid_width_
          && pixel.y >= 0 && pixel.y < (int)grid_height_)
      {
        if (in_out_grid.at<char>(pixel) >= (int)in_threshold)
        {
          validated_cloud_ptr->points.push_back(point);
        } else
        {
          if (in_out_grid.at<char>(pixel) > 0)
          {
            in_out_grid.at<char>(pixel) -= 1;
          }
        }
      }
    }
    return validated_cloud_ptr;
  }

  bool
  RayGroundFilterNodelet::child_init(ros::NodeHandle &nh, bool &has_service)
  {
    // Enable the dynamic reconfigure service
    has_service = true;
    srv_ = boost::make_shared < dynamic_reconfigure::Server < pointcloud_preprocessor::RayGroundFilterConfig > > (nh);
    dynamic_reconfigure::Server<pointcloud_preprocessor::RayGroundFilterConfig>::CallbackType f = boost::bind(
      &RayGroundFilterNodelet::config_callback, this, _1, _2);
    srv_->setCallback(f);
    return (true);
  }

  void
  RayGroundFilterNodelet::ExtractPointsIndices(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr,
                                             const pcl::PointIndices& in_indices,
                                             pcl::PointCloud<PointType_>::Ptr out_only_indices_cloud_ptr,
                                             pcl::PointCloud<PointType_>::Ptr out_removed_indices_cloud_ptr)
  {
    pcl::ExtractIndices<PointType_> extract_ground;
    extract_ground.setInputCloud (in_cloud_ptr);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(in_indices));

    extract_ground.setNegative(false);//true removes the indices, false leaves only the indices
    extract_ground.filter(*out_only_indices_cloud_ptr);

    extract_ground.setNegative(true);//true removes the indices, false leaves only the indices
    extract_ground.filter(*out_removed_indices_cloud_ptr);
  }

  template<typename T>
  void
  RayGroundFilterNodelet::publish_cloud(const ros::Publisher &in_publisher,
                                      const T &in_cloud_to_publish_ptr,
                                      const std_msgs::Header &in_header)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
  }


  pcl::PointCloud<RayGroundFilterNodelet::PointType_>::Ptr
  RayGroundFilterNodelet::Voxelize(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr)
  {
    pcl::PointCloud<PointType_>::Ptr voxelized_cloud_ptr(new pcl::PointCloud <PointType_>);
    // Create the filtering object
    pcl::VoxelGrid<PointType_> voxel_filter;
    voxel_filter.setInputCloud (in_cloud_ptr);
    voxel_filter.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter (*voxelized_cloud_ptr);
    return voxelized_cloud_ptr;
  }

  void
  RayGroundFilterNodelet::filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices,
                                      PointCloud2 &output)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<PointType_>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud <PointType_>);
    pcl::fromROSMsg(*input, *current_sensor_cloud_ptr);

    pcl::PointCloud<PointType_>::Ptr clipped_cloud_ptr(new pcl::PointCloud <PointType_>);

    // auto t1 = std::chrono::high_resolution_clock::now();

    //remove points above certain point
    ClipCloud(current_sensor_cloud_ptr, clipping_height_, clipped_cloud_ptr);

    //remove closer points than a threshold
    pcl::PointCloud<PointType_>::Ptr close_cloud_ptr(new pcl::PointCloud <PointType_>);
    RemovePointsUpTo(clipped_cloud_ptr, min_point_distance_, close_cloud_ptr);

    //apply voxel if point cloud is larger than a threshold
    pcl::PointCloud<PointType_>::Ptr filtered_cloud_ptr(new pcl::PointCloud <PointType_>);
    if (close_cloud_ptr->points.size() > max_cloud_size_)
    {
      filtered_cloud_ptr = Voxelize(close_cloud_ptr);
    } else
    {
      filtered_cloud_ptr = close_cloud_ptr;
    }

    PointCloudXYZIRTColor organized_points;
    std::vector <pcl::PointIndices> radial_division_indices;
    std::vector <pcl::PointIndices> closest_indices;
    std::vector <PointCloudXYZIRTColor> radial_ordered_clouds;

    radial_dividers_num_ = ceil(360 / radial_divider_angle_);

    ConvertXYZIToRTZColor(filtered_cloud_ptr,
                          organized_points,
                          radial_division_indices,
                          radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    ClassifyPointCloud(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::PointCloud<PointType_>::Ptr ground_cloud_ptr(new pcl::PointCloud <PointType_>);
    pcl::PointCloud<PointType_>::Ptr no_ground_cloud_ptr(new pcl::PointCloud <PointType_>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr radials_cloud_ptr(new pcl::PointCloud <pcl::PointXYZRGB>);

    ExtractPointsIndices(filtered_cloud_ptr, ground_indices, ground_cloud_ptr, no_ground_cloud_ptr);

    pcl::toROSMsg(*no_ground_cloud_ptr, output);
    output.header = input->header;
  }

  void
  RayGroundFilterNodelet::subscribe()
  {
    Filter::subscribe();
  }

  void
  RayGroundFilterNodelet::unsubscribe()
  {
    Filter::unsubscribe();
  }

  void
  RayGroundFilterNodelet::config_callback(pointcloud_preprocessor::RayGroundFilterConfig &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (sensor_height_ != config.sensor_height)
    {
      sensor_height_ = config.sensor_height;
      NODELET_DEBUG("[%s::config_callback] Setting sensor_height to: %f.", getName().c_str(), config.sensor_height);
    }
    if (general_max_slope_ != config.general_max_slope)
    {
      general_max_slope_ = config.general_max_slope;
      NODELET_DEBUG("[%s::config_callback] Setting general_max_slope to: %f.", getName().c_str(),
                    config.general_max_slope);
    }
    if (local_max_slope_ != config.local_max_slope)
    {
      local_max_slope_ = config.local_max_slope;
      NODELET_DEBUG("[%s::config_callback] Setting local_max_slope to: %f.", getName().c_str(), config.local_max_slope);
    }
    if (radial_divider_angle_ != config.radial_divider_angle)
    {
      radial_divider_angle_ = config.radial_divider_angle;
      NODELET_DEBUG("[%s::config_callback] Setting radial_divider_angle to: %f.", getName().c_str(),
                    config.radial_divider_angle);
    }
    if (concentric_divider_distance_ != config.concentric_divider_distance)
    {
      concentric_divider_distance_ = config.concentric_divider_distance;
      NODELET_DEBUG("[%s::config_callback] Setting concentric_divider_distance to: %f.", getName().c_str(),
                    config.concentric_divider_distance);
    }
    if (min_height_threshold_ != config.min_height_threshold)
    {
      min_height_threshold_ = config.min_height_threshold;
      NODELET_DEBUG("[%s::config_callback] Setting min_height_threshold_ to: %f.", getName().c_str(),
                    config.min_height_threshold);
    }
    if (clipping_height_ != config.clipping_height)
    {
      clipping_height_ = config.clipping_height;
      NODELET_DEBUG("[%s::config_callback] Setting clipping_height to: %f.", getName().c_str(), config.clipping_height);
    }
    if (min_point_distance_ != config.min_point_distance)
    {
      min_point_distance_ = config.min_point_distance;
      NODELET_DEBUG("[%s::config_callback] Setting min_point_distance_ to: %f.", getName().c_str(),
                    config.min_point_distance);
    }
    if (reclass_distance_threshold_ != config.reclass_distance_threshold)
    {
      reclass_distance_threshold_ = config.reclass_distance_threshold;
      NODELET_DEBUG("[%s::config_callback] Setting reclass_distance_threshold to: %f.", getName().c_str(),
                    config.reclass_distance_threshold);
    }


  }

} // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::RayGroundFilterNodelet, nodelet::Nodelet
);