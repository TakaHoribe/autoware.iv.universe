#pragma once
#include <ros/ros.h>
#include <autoware_planning_msgs/Trajectory.h>

class TrajectoryLoaderNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  using csv = std::vector<std::vector<std::string>>;
  using association = std::map<std::string /* label */, int /* row */>;

  bool publish(const std_msgs::Header &header, const association &label_row_association_map, const csv &file_data);
  bool loadData(const std::string &file_name, association &label_row_association_map, csv &file_data);
  std::vector<std::string> split(const std::string &input, char delimiter);
  void deleteHeadSpace(std::string &string);
  void deleteUnit(std::string &string);

public:
  TrajectoryLoaderNode();
  ~TrajectoryLoaderNode(){};
};