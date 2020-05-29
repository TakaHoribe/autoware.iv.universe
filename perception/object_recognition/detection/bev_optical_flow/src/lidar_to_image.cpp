/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include <bev_optical_flow/lidar_to_image.h>

namespace bev_optical_flow
{
  LidarToBEVImage::LidarToBEVImage() : nh_(""), pnh_("~") {
    pnh_.param<float>("grid_size", grid_size_, 0.25);
    pnh_.param<float>("point_radius", point_radius_, 50);
    pnh_.param<float>("z_max", z_max_, 3.0);
    pnh_.param<float>("z_min", z_min_, 0.0);
    pnh_.param<int>("depth_max", depth_max_, 255);
    pnh_.param<int>("depth_min", depth_min_, 0);

    image_size_ = 2 * static_cast<int>(point_radius_ / grid_size_);
    utils_ = std::make_shared<Utils>();
  }

  float LidarToBEVImage::pointToPixel
  (const pcl::PointXYZ& point, cv::Point2d& px) {
    // affine transform base_link coords to image coords
    Eigen::Affine2f base2image =
      Eigen::Translation<float, 2>(point_radius_, point_radius_) *
      Eigen::Rotation2Df(180 * M_PI / 180).toRotationMatrix();
    Eigen::Vector2f transformed_p =
      (base2image * Eigen::Vector2f(point.x, point.y)) / grid_size_;
    px.x = transformed_p[1];
    px.y = transformed_p[0];

    float intensity = (point.z + std::abs(z_min_)) / (z_max_ - z_min_);
    return std::round((depth_max_ - depth_min_) * intensity);
  }

  void LidarToBEVImage::getBEVImage(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                    cv::Mat& bev_image) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    bev_image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);
    cv::Mat px_cnt = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_32FC1);
    cv::Mat depth_sum = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_32FC1);

    for (int i=0; i<cloud->points.size(); i++) {
      auto p = cloud->points.at(i);
      if (p.x < -point_radius_ || point_radius_ < p.x ||
          p.y < -point_radius_ || point_radius_ < p.y ||
          p.z < z_min_ || z_max_ < p.z) {
        continue;
      }
      cv::Point2d px;
      float depth = pointToPixel(p, px);
      px_cnt.at<float>(px.y, px.x) += 1;
      depth_sum.at<float>(px.y, px.x) += depth;
    }

    cv::Mat depth_ave = depth_sum / px_cnt;
    for (int i=0; i<depth_ave.cols; i++) {
      for (int j=0; j<depth_ave.rows; j++) {
        cv::circle(bev_image, cv::Point2d(i,j), 0,
                   static_cast<int>(depth_ave.at<float>(j,i)), -1);
      }
    }

    // rotate bev image to fixing on map coords
    float map2base_angle = utils_->getMap2BaseAngle(cloud_msg->header.stamp);
    cv::Mat affine;
    cv::getRotationMatrix2D(cv::Point2f(static_cast<int>(bev_image.cols * 0.5),
                                        static_cast<int>(bev_image.rows * 0.5)),
                            (map2base_angle * 180 / M_PI),
                            1).copyTo(affine);
    cv::warpAffine(bev_image, bev_image, affine, bev_image.size(),
                   cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  }

} // bev_optical_flow
