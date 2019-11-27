#pragma once

#include <chrono>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "pointcloud_preprocessor/filter.h"
#include "pointcloud_preprocessor/RayGroundFilterConfig.h"
#include "pointcloud_preprocessor/ground_filter/gencolors.hpp"

namespace pointcloud_preprocessor
{
  class RayGroundFilterNodelet : public pointcloud_preprocessor::Filter
  {
  protected:
    boost::shared_ptr <dynamic_reconfigure::Server<pointcloud_preprocessor::RayGroundFilterConfig>> srv_;

    void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output) override;

    void subscribe() override;

    void unsubscribe() override;

    bool child_init(ros::NodeHandle &nh, bool &has_service) override;

    void config_callback(pointcloud_preprocessor::RayGroundFilterConfig &config, uint32_t level);

  private:
    double sensor_height_;//sensor height from the ground in meters
    double general_max_slope_;//degrees
    double local_max_slope_;//degrees
    double radial_divider_angle_;//distance in rads between dividers
    double concentric_divider_distance_;//distance in meters between concentric divisions
    double min_height_threshold_;//minimum height threshold regardless the slope, useful for close points
    double clipping_height_;//the points higher than this will be removed from the input cloud.
    double min_point_distance_;//minimum distance from the origin to consider a point as valid
    double reclass_distance_threshold_;//distance between points at which re classification will occur
    const size_t max_cloud_size_ = 30000;
    const float voxel_size_ = 0.08f;

    //typedef velodyne_pointcloud::PointXYZIR VeloPointType_;
    typedef pcl::PointXYZI PointType_;

    size_t radial_dividers_num_;
    size_t concentric_dividers_num_;

    size_t grid_width_;
    size_t grid_height_;
    double grid_precision_;
    cv::Mat previous_occupancy_mat_;
    cv::Mat accumulated_occupancy_mat_;
    const size_t min_icp_points_ = 10;//min num points to allow icp to start

    std::vector <cv::Scalar> colors_;
    const size_t color_num_ = 10;//different number of color to generate
    pcl::PointCloud<PointType_>::Ptr previous_cloud_ptr_;//holds the previous groundless result of ground classification

    struct PointXYZIRTColor
    {
      pcl::PointXYZI point;

      size_t ring;        //ring number if available

      float radius;       //cylindric coords on XY Plane
      float theta;        //angle deg on XY plane

      size_t radial_div;  //index of the radial divsion to which this point belongs to
      size_t concentric_div;//index of the concentric division to which this points belongs to

      size_t red;         //Red component  [0-255]
      size_t green;       //Green Component[0-255]
      size_t blue;        //Blue component [0-255]

      size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector <PointXYZIRTColor> PointCloudXYZIRTColor;

    /*!
     * Publishes a PointCloud on the specified publisher with a header
     * @tparam T PointCloud type
     * @param in_publisher publisher to use
     * @param in_cloud_to_publish_ptr point cloud to be published
     * @param in_header header to use in the message
     */
    template<typename T>
    void publish_cloud(const ros::Publisher &in_publisher,
                       const T &in_cloud_to_publish_ptr,
                       const std_msgs::Header &in_header);

    /*!
     * Validates a pointcloud with the specified occupancy grid according to certain threshold.
     * The points are checked against an occupancy grid, if the points are matched between frames n number of times
     * (in_threshold), then the points is considered as valid.
     * @param in_cloud pointcloud to validate
     * @param in_out_grid occupancy grid to use to validate in_cloud
     * @param in_threshold the number of times a point should have been visited before accepting it as valid
     * @return A point cloud with only the valid points.
     */
    pcl::PointCloud<PointType_>::Ptr
    ValidatePointCloudWithOccupancyGrid(const pcl::PointCloud<PointType_>::Ptr &in_cloud,
                                        cv::Mat in_out_grid, size_t in_threshold);

    /*!
     * Projects a 3D point to a 2D cv::Mat occupancy grid with the specified dimensions.
     * @tparam T 3D PointType
     * @param in_point 3D points to project
     * @param in_grid_w grid width in pixels
     * @param in_grid_h grid height in pixels
     * @param in_precision pixels/meter
     * @return returns the corresponding 2D point in the occupancy grid
     */
    template<typename T>
    cv::Point2i
    ProjectPointToGrid(const T &in_point, size_t in_grid_w, size_t in_grid_h, double in_precision);

    /*!
     * Creates an occupancy grid with the specified dimensions from a 3D pointcloud in the XY plane
     * @tparam T 3D PointType
     * @return Occupancy grid in cv::Mat
     */
    template<typename T>
    cv::Mat
    CreateOccupancyFromCloud(const T &);

    /*!
     * Match input point clouds using the ICP matching algorithm.
     * @param in_previous_scan
     * @param in_current_scan
     * @return The previous scan transformed to current coordinate frame.
     */
    pcl::PointCloud<PointType_>::Ptr
    MatchClouds(const pcl::PointCloud<PointType_>::Ptr &in_previous_scan,
                pcl::PointCloud<PointType_>::Ptr in_current_scan);

    /*!
     *
     * @param[in] in_cloud Input Point Cloud to be organized in radial segments
     * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
     * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
     * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
     */
    void ConvertXYZIToRTZColor(const pcl::PointCloud<PointType_>::Ptr in_cloud,
                               PointCloudXYZIRTColor &out_organized_points,
                               std::vector <pcl::PointIndices> &out_radial_divided_indices,
                               std::vector <PointCloudXYZIRTColor> &out_radial_ordered_clouds);


    /*!
     * Classifies Points in the PointCoud as Ground and Not Ground
     * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
     * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
     * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
     */
    void ClassifyPointCloud(std::vector <PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                            pcl::PointIndices &out_ground_indices,
                            pcl::PointIndices &out_no_ground_indices);


    /*!
     * Removes the points higher than a threshold
     * @param in_cloud_ptr PointCloud to perform Clipping
     * @param in_clip_height Maximum allowed height in the cloud
     * @param out_clipped_cloud_ptr Resultung PointCloud with the points removed
     */
    void ClipCloud(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr,
                   double in_clip_height,
                   pcl::PointCloud<PointType_>::Ptr out_clipped_cloud_ptr);


    /*!
     * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
     * in the indices
     * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
     * @param in_indices Indices of the points to be both removed and kept
     * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
     * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
     */
    void ExtractPointsIndices(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr,
                              const pcl::PointIndices &in_indices,
                              pcl::PointCloud<PointType_>::Ptr out_only_indices_cloud_ptr,
                              pcl::PointCloud<PointType_>::Ptr out_removed_indices_cloud_ptr);

    /*!
     * Removes points up to a certain distance in the XY Plane
     * @param in_cloud_ptr Input PointCloud
     * @param in_min_distance Minimum valid distance, points closer than this will be removed.
     * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
     */
    void RemovePointsUpTo(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr,
                          double in_min_distance,
                          pcl::PointCloud<PointType_>::Ptr out_filtered_cloud_ptr);

    pcl::PointCloud<PointType_>::Ptr
    Voxelize(const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RayGroundFilterNodelet();
  };
} // namespace pointcloud_preprocessor
