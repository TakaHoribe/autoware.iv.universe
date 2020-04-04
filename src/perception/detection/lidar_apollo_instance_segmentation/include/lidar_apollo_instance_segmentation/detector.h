#pragma once
#include "lidar_apollo_instance_segmentation/node.h"
#include "TrtNet.h"
#include "cluster2d.h"
// #include "data_reader.h"
#include "feature_generator.h"

#include <memory>

class LidarApolloInstanceSegmentation : public LidarInstanceSegmentationInterface {
 public:
  LidarApolloInstanceSegmentation();
  ~LidarApolloInstanceSegmentation(){};
  bool detectDynamicObjects(const sensor_msgs::PointCloud2& input,
                            autoware_perception_msgs::DynamicObjectWithFeatureArray& output) override;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<Tn::trtNet> net_ptr_;
  std::shared_ptr<Cluster2D> cluster2d_;
  std::shared_ptr<FeatureGenerator> feature_generator_;
  float score_threshold_;
};